# MCU 集成指南 — Vision UART 通讯 (`Circle_Leg_Host_V2.0`)

> 目标：在不动现有 0xFF / 0xFE 链路、不改 RosComm/RosManager 任何文件的前提下，
> 通过一个开关宏 `ENABLE_VISION_COMM`，让 MCU 能够：
> 1. **发送** `0xFD` (4B) — 启停视觉 + 最小台阶高度阈值；
> 2. **接收** `0xFC` (10B) — 台阶高度、距离、角度等结果。
>
> 关闭宏后，编译产物与 V1.0 **完全一致**（不引入新的全局对象 / 新任务 / 新回调）。

---

## 0. 协议常量（与上位机一一对应）

所有多字节字段均为 **小端 (little-endian)**。帧格式仍是
`SOF(0xAA, 1B) + LEN(1B) + ID(1B) + HeaderCRC16(2B LE) + Payload(NB) + PacketCRC16(2B LE)`。

| ID    | 方向    | Payload 长度 | 用途                  |
| ----- | ------- | ------------ | --------------------- |
| 0xFF  | PC→MCU  | 14           | 手柄 (已存在)         |
| 0xFE  | MCU→PC  | 40           | Reachable_Msg (已存在)|
| **0xFD** | **MCU→PC** | **4**  | **VisionCtrl (新增)** |
| **0xFC** | **PC→MCU** | **10** | **VisionResult (新增)** |

### 0.1 `0xFD VisionCtrl` (4 字节)

| 偏移 | 字段          | 类型    | 说明                            |
| ---- | ------------- | ------- | ------------------------------- |
| 0    | enable        | uint8   | 0=关闭视觉 1=开启视觉           |
| 1    | min_height_mm | uint16  | 最小台阶高度阈值 (单位 mm)      |
| 3    | flags         | uint8   | bit0=RESET, bit1=VERBOSE        |

### 0.2 `0xFC VisionResult` (10 字节)

| 偏移 | 字段           | 类型    | 说明                                                  |
| ---- | -------------- | ------- | ----------------------------------------------------- |
| 0    | seq            | uint8   | 上位机自增序号 (0..255 循环)                          |
| 1    | status         | uint8   | bit0=DETECTED, bit1=RELIABLE, bit2=DISABLED, bit3=FAULT |
| 2    | curb_height_mm | uint16  | 台阶高度，单位 mm (经低通)                            |
| 4    | distance_mm    | uint16  | 车前缘到台阶的水平距离，单位 mm                       |
| 6    | theta_cdeg     | int16   | 朝向角，单位 0.01°                                    |
| 8    | confidence     | uint8   | 0..100 置信度                                         |
| 9    | flags          | uint8   | bit0=THETA_BAD,bit1=DIST_BAD,bit2=NO_PLANES,bit3=STALE|

> 解析时应当：先检查 `status & VSTATUS_FAULT/DISABLED`；再检查 `status & VSTATUS_DETECTED`；
> 决策依据使用 `(detected) && (reliable) && !(flags & STALE)`。

---

## 1. 集成总开关 (`AppConfig.h`)

在 `Core/Inc/AppConfig.h` 末尾追加：

```c
/* ===== Vision UART Comm (host: Circle_Leg_Host_V2.0) ===== */
#ifndef ENABLE_VISION_COMM
#define ENABLE_VISION_COMM 0   /* 0=完全编译关闭, 与 V1.0 等价 */
#endif

#define VISION_COMM_TX_PERIOD_MS    100   /* 0xFD 心跳周期 */
#define VISION_COMM_RESULT_TIMEOUT  500   /* 超过此毫秒未收到 0xFC 视为陈旧 */
```

> 关闭时（默认）：下面新增的 .cpp 文件里所有内容都在 `#if ENABLE_VISION_COMM` 内，
> 链接器实际不引入任何 vision 相关符号。

---

## 2. 新增文件：`Applications/Vision_Comm.hpp`

```cpp
#pragma once
#include "AppConfig.h"
#if ENABLE_VISION_COMM

#include <cstdint>
#include "Comm_Msg.hpp"  // 复用现有 namespace 风格

namespace Protocol
{
/* 0xFD: MCU -> PC, 4 bytes */
struct VisionCtrl_Msg
{
    uint8_t  enable;          // 0/1
    uint16_t min_height_mm;   // mm
    uint8_t  flags;           // bit0=RESET bit1=VERBOSE
} __attribute__((packed));
static_assert(sizeof(VisionCtrl_Msg) == 4, "VisionCtrl_Msg must be 4 bytes");

/* 0xFC: PC -> MCU, 10 bytes */
struct VisionResult_Msg
{
    uint8_t  seq;
    uint8_t  status;          // bit0 DETECTED, bit1 RELIABLE, bit2 DISABLED, bit3 FAULT
    uint16_t curb_height_mm;
    uint16_t distance_mm;
    int16_t  theta_cdeg;
    uint8_t  confidence;      // 0..100
    uint8_t  flags;           // bit0 THETA_BAD, bit1 DIST_BAD, bit2 NO_PLANES, bit3 STALE
} __attribute__((packed));
static_assert(sizeof(VisionResult_Msg) == 10, "VisionResult_Msg must be 10 bytes");

/* 位定义 */
#define VSTATUS_DETECTED   (1u << 0)
#define VSTATUS_RELIABLE   (1u << 1)
#define VSTATUS_DISABLED   (1u << 2)
#define VSTATUS_FAULT      (1u << 3)

#define VFLAG_THETA_BAD    (1u << 0)
#define VFLAG_DIST_BAD     (1u << 1)
#define VFLAG_NO_PLANES    (1u << 2)
#define VFLAG_STALE        (1u << 3)

#define VCTRL_FLAG_RESET   (1u << 0)
#define VCTRL_FLAG_VERBOSE (1u << 1)
}  // namespace Protocol

namespace Applications::Vision_Comm
{
/* 由 UserTask.cpp 调用一次, 内部 xTaskCreateStatic 启动后台任务 */
void init();

/* 上层逻辑接口 (供 Chassis_Task 等读取/写入) */
void   SetEnable(bool enable);
void   SetMinHeightMm(uint16_t mm);
bool   GetEnable();
bool   IsResultFresh();
Protocol::VisionResult_Msg GetLatestResult();   // 拷贝返回, 调用方临界区由内部保证
}  // namespace Applications::Vision_Comm

#endif  // ENABLE_VISION_COMM
```

---

## 3. 新增文件：`Applications/Vision_Comm.cpp`

> **关键：完全不修改 `PC_Comm.cpp`。** 仅复用 `RosManager::managers[0]` 现成的 UART 与
> `registerFrameCallback(id, cb)` 接口（PC_Comm.cpp 已注册 0xFF；这里注册 0xFC，互不冲突）。

```cpp
#include "Vision_Comm.hpp"
#if ENABLE_VISION_COMM

#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "RosComm.hpp"        // 工程内已有
#include "RosManager.hpp"     // 工程内已有

namespace Applications::Vision_Comm
{
using namespace Core::Communication::RosComm;

/* ----- 内部状态 (用临界区, 体量很小) ----- */
static Protocol::VisionCtrl_Msg   g_ctrl   = { 0, 0, 0 };
static Protocol::VisionResult_Msg g_result = {};
static volatile TickType_t        g_last_result_tick = 0;
static volatile bool              g_has_result       = false;

/* ----- 0xFC 接收回调 ----- */
static void VisionRxCallback(uint8_t *data, uint16_t len, UART_HandleTypeDef *handle)
{
    if (handle != RosManager::managers[0].getUARTHandle()) return;
    if (len != sizeof(Protocol::VisionResult_Msg))         return;

    ATOMIC_ENTER_CRITICAL();
    memcpy((void *)&g_result, data, sizeof(g_result));
    g_last_result_tick = xPortIsInsideInterrupt() ? xTaskGetTickCountFromISR()
                                                  : xTaskGetTickCount();
    g_has_result = true;
    ATOMIC_EXIT_CRITICAL();
}

/* ----- 公共 setter / getter ----- */
void SetEnable(bool enable)
{
    ATOMIC_ENTER_CRITICAL();
    g_ctrl.enable = enable ? 1 : 0;
    ATOMIC_EXIT_CRITICAL();
}
void SetMinHeightMm(uint16_t mm)
{
    ATOMIC_ENTER_CRITICAL();
    g_ctrl.min_height_mm = mm;
    ATOMIC_EXIT_CRITICAL();
}
bool GetEnable()
{
    return g_ctrl.enable != 0;
}
bool IsResultFresh()
{
    if (!g_has_result) return false;
    return (xTaskGetTickCount() - g_last_result_tick) <= pdMS_TO_TICKS(VISION_COMM_RESULT_TIMEOUT);
}
Protocol::VisionResult_Msg GetLatestResult()
{
    Protocol::VisionResult_Msg out{};
    ATOMIC_ENTER_CRITICAL();
    out = g_result;
    ATOMIC_EXIT_CRITICAL();
    return out;
}

/* ----- 心跳任务: 100ms 发一次 0xFD ----- */
static void VisionCommTask(void *)
{
    /* 注册 0xFC 接收 */
    RosManager::managers[0].registerFrameCallback(0xFC, VisionRxCallback);

    FrameHeader tx_hdr;
    tx_hdr.sof        = START_BYTE;          // 0xAA
    tx_hdr.protocolID = 0xFD;
    tx_hdr.dataLen    = sizeof(Protocol::VisionCtrl_Msg);

    /* 初始默认: 上电关闭, 阈值 20mm */
    SetEnable(false);
    SetMinHeightMm(20);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(VISION_COMM_TX_PERIOD_MS);

    while (true)
    {
        if (RosManager::managers[0].isInitialized())
        {
            Protocol::VisionCtrl_Msg snapshot;
            ATOMIC_ENTER_CRITICAL();
            snapshot = g_ctrl;
            ATOMIC_EXIT_CRITICAL();
            RosManager::managers[0].transmit(tx_hdr, (uint8_t *)&snapshot);
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

static StackType_t  uxVisionStack[1024];
static StaticTask_t xVisionTCB;
void init()
{
    xTaskCreateStatic(VisionCommTask, "Vision_Comm",
                      1024, NULL, 0,
                      uxVisionStack, &xVisionTCB);
}
}  // namespace Applications::Vision_Comm

#endif  // ENABLE_VISION_COMM
```

---

## 4. `Core/Src/UserTask.cpp` — **唯一**一处需要修改的现有文件

在已有的初始化序列末尾，加一段宏保护的调用即可（**不修改其它任何代码**）：

```cpp
#include "AppConfig.h"
// ... 已有 include ...

#if ENABLE_VISION_COMM
#include "Vision_Comm.hpp"
#endif

extern "C" void UserTask(void *)
{
    // ... 原有所有初始化保持不变 ...

#if ENABLE_VISION_COMM
    Applications::Vision_Comm::init();
#endif

    // ... 原有循环保持不变 ...
}
```

> 关闭 `ENABLE_VISION_COMM` 时，预处理直接剥离这两段，等价于 V1.0。

---

## 5. 构建系统 — `Core.mk`

在 `Core.mk` 里 `Applications/*.cpp` 已经被 wildcard 匹配的话，**无需改动**。
若是逐文件列举，只需追加：

```make
CPP_SOURCES += Applications/Vision_Comm.cpp
```

---

## 6. 上层逻辑如何使用（示例：`Chassis_Task.cpp`）

```cpp
#if ENABLE_VISION_COMM
#include "Vision_Comm.hpp"
#endif

void some_decision_step()
{
#if ENABLE_VISION_COMM
    // 你的开关决策, 比如某个状态进入 "上台阶模式" 时开启视觉
    Applications::Vision_Comm::SetEnable(true);
    Applications::Vision_Comm::SetMinHeightMm(30);

    if (Applications::Vision_Comm::IsResultFresh())
    {
        auto r = Applications::Vision_Comm::GetLatestResult();
        if ((r.status & VSTATUS_DETECTED) && (r.status & VSTATUS_RELIABLE)
            && !(r.flags & VFLAG_STALE))
        {
            uint16_t h_mm    = r.curb_height_mm;
            uint16_t dist_mm = r.distance_mm;
            // ... 触发抬腿规划 ...
        }
    }
#endif
}
```

---

## 7. 回归验证清单

| 项目                                                  | 期望结果                                  |
| ----------------------------------------------------- | ----------------------------------------- |
| `ENABLE_VISION_COMM = 0` 编译                         | 与 V1.0 二进制等价（map 文件无新符号）    |
| 上位机 `vision.enabled=false` 启动                    | 串口字节流与 V1.0 完全一致 (仅 0xFF/0xFE) |
| `pytest tests/`                                       | 7 项全过；CRC 表与 V1.0 字节级一致         |
| `ENABLE_VISION_COMM = 1` 编译, 上位机不开 vision       | MCU 心跳 0xFD 仍发出，上位机用解析器可吃掉；不会因 0xFC 缺失而崩溃 |
| 上位机开 vision (`--vision`)                           | MCU 端 `Vision_Comm::IsResultFresh()` 返回 true，`GetLatestResult().status` 出现 DETECTED |
| 拔掉相机                                              | `status & VSTATUS_FAULT`；MCU 端可降级    |

---

## 8. 设计要点小结（为什么这样最简）

1. **不动 PC_Comm**：复用同一个 `UART_HandleTypeDef`、同一份 `RosManager::managers[0]`、同一组 CRC，
   仅借助现成的 `registerFrameCallback(id, cb)` 注册新 ID，避免一切并发改造。
2. **单文件 + 单宏**：`Vision_Comm.{hpp,cpp}` 全部内容在 `#if ENABLE_VISION_COMM` 内；
   `UserTask.cpp` 只追加 2 行 init 调用。关闭后 0 副作用。
3. **100 ms 心跳**：与上位机 `ctrl_timeout_ms=1500` 留 15× 容差；
   即便 MCU 卡顿，上位机也不会立刻断开视觉。
4. **临界区拷贝**：所有 setter / getter 用 `ATOMIC_ENTER/EXIT_CRITICAL()`，
   保证与 ISR 回调的并发安全；不引入 mutex/queue。
