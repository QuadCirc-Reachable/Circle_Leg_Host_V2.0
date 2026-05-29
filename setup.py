from setuptools import setup, find_packages

setup(
    name="circle_leg_host",
    version="2.0.0",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    install_requires=[
        "pygame>=2.5",
        "pyserial==3.5",
        "pyyaml>=6.0",
        "numpy>=1.21",
    ],
    extras_require={
        # 视觉依赖：仅在启用视觉功能时安装
        "vision": [
            "pyrealsense2",
            "open3d",
            "scipy>=1.4.0",
            "opencv-python",
            "shapely",
            "scikit-learn",
            "matplotlib",
        ],
    },
    python_requires=">=3.8",
)
