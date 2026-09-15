"""薄封装 curbsvm1 算法链路: 抓帧 -> 提平面 -> 算台阶高度/距离/角度。

仅在 vision.enabled=true 时被 import, 避免无相机环境下崩溃。
依赖: pyrealsense2, polylidar, fastgac, surfacedetector (curbsvm1_package).
"""

import logging
import os
import sys
from dataclasses import dataclass
from typing import Optional

# 把 curbsvm1_package 暴露的 surfacedetector 包加入 sys.path
_CURBSVM1_DEFAULT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "curbsvm1_package")
)


def _ensure_curbsvm1_on_path(extra_path: Optional[str] = None) -> None:
    candidates = [p for p in (extra_path, _CURBSVM1_DEFAULT) if p]
    for p in candidates:
        if p and os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


log = logging.getLogger(__name__)


@dataclass
class PipelineFrame:
    curb_height_m: float = 0.0
    distance_m: Optional[float] = None
    theta_deg: Optional[float] = None
    found_planes: bool = False


class CurbPipeline:
    """封装 curbsvm1.curbsvm1.get_polygon + analyze_planes + hplane 调用链。"""

    def __init__(self, realsense_yaml: Optional[str] = None,
                 curbsvm1_path: Optional[str] = None,
                 camera_to_front_edge_m: float = 0.30,
                 show_window: bool = False):
        _ensure_curbsvm1_on_path(curbsvm1_path)

        # 延迟 import: 这些库较重, 且在无相机环境会出错
        from surfacedetector.curbsvm1 import (  # type: ignore
            create_pipeline, get_frames, get_polygon, valid_frames, resolve_package_path,
            colorize_images_open_cv,
        )
        from surfacedetector.utility.helper_wheelchair_svm import (  # type: ignore
            analyze_planes, hplane, get_theta_and_distance,
        )
        from surfacedetector.utility.helper import plot_planes_and_obstacles  # type: ignore
        from polylidar import Polylidar3D  # type: ignore
        from fastgac import GaussianAccumulatorS2, IcoCharts  # type: ignore
        import yaml

        self._create_pipeline = create_pipeline
        self._get_frames = get_frames
        self._get_polygon = get_polygon
        self._valid_frames = valid_frames
        self._analyze_planes = analyze_planes
        self._hplane = hplane
        self._get_theta_and_distance = get_theta_and_distance
        self._resolve = resolve_package_path
        self._colorize = colorize_images_open_cv
        self._plot_planes = plot_planes_and_obstacles

        if realsense_yaml is None:
            realsense_yaml = os.path.join(_CURBSVM1_DEFAULT, "surfacedetector/config/default.yaml")
        with open(realsense_yaml, "r", encoding="utf-8") as f:
            self._config = yaml.safe_load(f)

        # 我们直接调用 get_polygon/analyze_planes, 不走 capture() 主循环,
        # 所以 curbsvm1 自己的 imshow 路径必须关闭, 由主线程统一弹窗。
        self._config["show_images"] = False
        self._config["show_polygon"] = True
        if "tracking" in self._config:
            self._config["tracking"]["enabled"] = False
        self.show_window = bool(show_window)

        pipeline, process_modules, filters, proj_mat, t265_device = self._create_pipeline(self._config)
        self._pipeline = pipeline
        self._process_modules = process_modules
        self._filters = filters
        self._proj_mat = proj_mat
        self._t265_pipeline = t265_device.get("pipeline") if t265_device else None

        self._ll_objects = {
            "pl": Polylidar3D(**self._config["polylidar"]),
            "ga": GaussianAccumulatorS2(level=self._config["fastga"]["level"]),
            "ico": IcoCharts(level=self._config["fastga"]["level"]),
        }
        self.camera_to_front_edge_m = float(camera_to_front_edge_m)

        import threading as _th
        self._dbg_lock = _th.Lock()
        self._latest_debug_image = None

    def close(self) -> None:
        try:
            self._pipeline.stop()
        except Exception:
            pass

    def step(self) -> Optional[PipelineFrame]:
        """单步: 抓一帧, 返回最新台阶估计。"""
        color_image, depth_image, meta = self._get_frames(
            self._pipeline, self._t265_pipeline, self._process_modules, self._filters, self._config,
        )
        if color_image is None or depth_image is None:
            return None
        if not self._valid_frames(color_image, depth_image, **self._config["polygon"]["frameskip"]):
            return None

        _planes, _obs, geometric_planes, _timings = self._get_polygon(
            depth_image, self._config, self._ll_objects, **meta
        )

        curb_height, first_plane, second_plane = self._analyze_planes(geometric_planes)
        result = PipelineFrame(curb_height_m=float(curb_height))

        if curb_height > 0.0 and first_plane is not None and second_plane is not None:
            try:
                _square, normal_svm, center = self._hplane(first_plane, second_plane)
                dist, theta = self._get_theta_and_distance(
                    normal_svm, center, first_plane["normal_ransac"]
                )
                result.distance_m = float(dist)
                result.theta_deg = float(theta)
                result.found_planes = True
            except Exception as exc:  # SVM 偶发数值问题不应让线程退出
                log.debug("hplane/get_theta_and_distance failed: %s", exc)

        # 渲染 debug 图 (在 worker 线程内生成 numpy BGR, 不调用 cv2.imshow)
        if self.show_window:
            try:
                import numpy as np
                import cv2
                color_cv, depth_cv = self._colorize(color_image, depth_image, self._config)
                try:
                    self._plot_planes(_planes, _obs, self._proj_mat, None, color_cv, self._config)
                except Exception:
                    pass
                images = np.hstack((color_cv, depth_cv))

                # 与原版 curbsvm1.py 完全一致的状态文字
                theta_val = result.theta_deg
                dist_val = result.distance_m
                theta_bad = False
                dist_bad = False
                if result.found_planes:
                    theta_bad = not ((-65 <= (theta_val or 0) <= 65) or (-180 <= (theta_val or 0) <= -120))
                    dist_bad = (dist_val or 0) < 0.15
                    status_text = "Status: UNRELIABLE" if (theta_bad or dist_bad) else "Status: RELIABLE"
                else:
                    status_text = "Status: NO CURB DETECTED"

                ground_dist = None
                if dist_val is not None:
                    ground_dist = max(dist_val - self.camera_to_front_edge_m, 0.0)

                cv2.putText(images, status_text, (10, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(images, "Curb Height: {:.2f} m".format(result.curb_height_m),
                            (10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                if ground_dist is not None:
                    cv2.putText(images, "Ground Distance: {:.2f} m".format(ground_dist),
                                (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                if dist_val is not None:
                    cv2.putText(images, "Debug Plane Distance: {:.2f} m".format(dist_val),
                                (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                if theta_val is not None:
                    cv2.putText(images, "Angle to the Curb: {:.2f} deg".format(theta_val),
                                (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                with self._dbg_lock:
                    self._latest_debug_image = images
            except Exception as exc:
                log.debug("debug image render failed: %s", exc)

        return result

    def pop_debug_image(self):
        with self._dbg_lock:
            img = self._latest_debug_image
            self._latest_debug_image = None
            return img

    def ground_distance_from_front(self, dist_m: float) -> float:
        return max(dist_m - self.camera_to_front_edge_m, 0.0)
