# HardFault 调试记录

## 概述

本项目在调试过程中遇到了两次 HardFault 异常，分别由不同的根因导致。本文档记录两次故障的现象、寄存器分析、根因定位及修复方案。

---

## 第一次 HardFault：INVSTATE（用法错误）

### 现象

| 寄存器 | 值 | 含义 |
|--------|------|------|
| HFSR | 0x40000000 | FORCED=1 — 故障提升为 HardFault |
| CFSR | 0x00000200 | **INVSTATE=1** (bit 1) — 尝试执行无效指令 |

### 根因 1：sscanf 类型不匹配导致内存越界

**文件：** `My_lib/commuction.c`

**原代码：**
```c
sscanf((char *)servo_rx_data, "#%03uP%04u!",
       (int*) &g_servo_id, (int*)&g_servo_pwm);
```

| 变量 | 实际类型 | 占用 | sscanf 写入大小 | 越界 |
|------|----------|------|------------------|------|
| `g_servo_id` | `uint8_t` | 1 字节 | `%u` + `(int*)` = 4 字节 | **越界 3 字节** |
| `g_servo_pwm` | `uint16_t` | 2 字节 | `%u` + `(int*)` = 4 字节 | **越界 2 字节** |

连续内存越界踩坏了相邻变量（函数返回地址、任务控制块等），导致 CPU 跳转到非法地址执行。

**修复：** 使用临时 `unsigned int` 变量匹配 `%u` 格式，再安全转型。

```c
unsigned int tmp_id, tmp_pwm;
sscanf((char *)servo_rx_data, "#%03uP%04u!", &tmp_id, &tmp_pwm);
g_servo_id = (uint8_t)tmp_id;
g_servo_pwm = (uint16_t)tmp_pwm;
```

### 根因 2：IK_3D 中反复调用 ServoBus_Start_Receive()

**文件：** `My_lib/drive.c`

**原代码：**
```c
// 当前关节1角度
ServoBus_Start_Receive();    // ← 运动学计算中反复触发串口DMA操作
float theta1_current_deg = (robot_arm->motor[0].motor_rx_pos-500)/7.04;
```

每次 `move_to()` → `IK_3D()` 都会调用 `ServoBus_Start_Receive()`，该函数内部又嵌套调用 `HAL_UART_AbortReceive`、`HAL_UARTEx_ReceiveToIdle_DMA`、以及 3 次 `ServoBus_ReadAngle`（DMA 发送），形成很深的调用栈，且频繁打断串口通信。

**修复：** 移除该调用，直接使用已有的反馈值。

### 根因 3：任务栈过小

**文件：** `My_task/task_init.c`

`requirement_2` 任务栈仅 **256 words**（1024 字节），而 `ServoBus_Move_Many` 中 `char cmd[256]` 本身占用 256 字节，加上多层函数调用和数学函数栈使用，极易栈溢出。

**修复：** 将任务栈增大至 512 words。

---

## 第二次 HardFault：IMPRECISERR（不精确总线错误）

### 现象

| 寄存器 | 值 | 含义 |
|--------|------|------|
| HFSR | 0x40000000 | FORCED=1 — 故障提升为 HardFault |
| CFSR | 0x00000400 | **IMPRECISERR=1** (bit 10) — 不精确总线错误 |
| BFARVALID | 1 | BFAR 地址有效 |

IMPRECISERR 是 Bus Fault 的一种。CPU 将总线写入请求放到写缓冲区后继续执行，但该写入在总线上实际执行时失败。由于 CPU 已经执行到后面指令，PC/LR 不直接指向出错点，调试难度更高。

### 根因 1：DMA 读取栈上已释放的局部缓冲区（最致命）

**文件：** `My_lib/commuction.c`，调用链：

```c
ServoBus_Move_Many(&arm, 3)              // task_init.c
  → char cmd[256];                       // 栈上分配
  → ServoBus_SendCmd(cmd)                // 传入栈地址
    → HAL_UART_Transmit_DMA(&huart2, (uint8_t*)cmd, len);  // DMA 异步读取
  → return;                              // 栈帧释放，DMA 还在读！
```

`HAL_UART_Transmit_DMA` 是**异步非阻塞**的，它配置好 DMA 后立即返回。DMA 控制器在后台通过系统总线读取 `cmd` 所在的内存。当 `ServoBus_Move_Many` 返回后，`cmd[256]` 所在的栈帧被回收，后续函数调用会覆盖这片内存：

- DMA 读到被覆盖的垃圾数据 → 发送错误指令
- 如果内存内容被覆盖为非法地址 → DMA 总线访问失败 → **IMPRECISERR**

同样的问题也存在于 `ServoBus_ReadAngle`（`char cmd[16]`）、`ServoBus_Move_One`（`char cmd[32]`）等所有通过 `ServoBus_SendCmd` 发送的命令。

| 调用链 | 栈上缓冲区 | DMA 是否安全 |
|--------|-----------|-------------|
| `ServoBus_Move_Many` → `SendCmd` | `cmd[256]` | ❌ 函数返回后栈释放 |
| `ServoBus_ReadAngle` → `SendCmd` | `cmd[16]` | ❌ 函数返回后栈释放 |
| `ServoBus_Move_One` → `SendCmd` | `cmd[32]` | ❌ 函数返回后栈释放 |

**修复：** 在 `commuction.c` 中新增全局 DMA 安全缓冲区 `dma_tx_buf[256]`，`ServoBus_SendCmd` 先将指令内容拷贝到全局缓冲区，再启动 DMA 发送：

```c
static char dma_tx_buf[256];       // 全局，DMA 安全

HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd)
{
    ...
    memcpy(dma_tx_buf, cmd, len);           // 先拷贝到全局缓冲区
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dma_tx_buf, len);  // DMA 读全局变量
    ...
}
```

### 根因 2：HAL_UART_ErrorCallback 在 ISR 中操作 DMA

**文件：** `My_lib/commuction.c`

**原代码：**
```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2) {
        ...
        ServoBus_Start_Receive();    // ← 在中断上下文中调用！
    }
}
```

`HAL_UART_ErrorCallback` 是**中断回调**（ISR context），其内部 `ServoBus_Start_Receive()` 又会调用：
1. `HAL_UART_AbortReceive(&huart2)` — **阻塞轮询**等待 DMA 停止（中断中阻塞 = 大忌）
2. `HAL_UARTEx_ReceiveToIdle_DMA(...)` — 配置 RX DMA
3. 3 次 `ServoBus_ReadAngle()` → `HAL_UART_Transmit_DMA(...)` — 操作 TX DMA

在 UART 中断中同时操作同一个 UART 的 RX/TX DMA 通道，HAL 状态机被打乱，寄存器写入混乱 → 总线写入失败 → **IMPRECISERR**。

**修复：** 中断中只设标志位，恢复操作推迟到任务上下文：

```c
// 中断中 — 只设置标志
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2) {
        servo_tx_busy = 0;
        servo_error_pending = 1;     // 只设标志，不调任何HAL DMA函数
        error_cnt++;
    }
}

// 任务上下文中 — 安全恢复
void ServoBus_ErrorRecovery(void)
{
    if(!servo_error_pending) return;
    servo_error_pending = 0;
    HAL_UART_Abort(&huart2);         // 安全：在任务上下文中调用
    ...  // 复位缓冲区、重启接收
}
```

### 根因 3：ServoBus_Start_Receive 职责过杂

原 `ServoBus_Start_Receive()` 在启动 DMA 接收后立即发送 3 个读角度命令。由于 `ServoBus_ReadAngle` 的 `cmd[16]` 也在栈上，DMA 发送时可能读到被后续调用覆盖的数据。

**修复：** `ServoBus_Start_Receive` 职责简化，只启动接收。读角度命令由调用方在其他地方按需发送。

---

## 修改文件清单

| 文件 | 修改内容 |
|------|---------|
| `My_lib/commuction.c` | ① 新增全局 `dma_tx_buf[256]`；② `SendCmd` 先拷贝到全局缓冲区再启动 DMA；③ `ErrorCallback` 中只设标志；④ 新增 `ServoBus_ErrorRecovery()`；⑤ 简化 `ServoBus_Start_Receive()` |
| `My_lib/commuction.h` | 声明 `servo_error_pending`、`ServoBus_ErrorRecovery()` |
| `My_lib/drive.c` | 移除 `IK_3D` 中的 `ServoBus_Start_Receive()` 调用 |
| `My_task/task_init.c` | `requirement_2` 任务栈 256 → 512 words |
| `My_task/start_task.c` | 任务入口调用 `ServoBus_ErrorRecovery()` |

---

## 经验教训

1. **DMA 缓冲区必须生命周期长于 DMA 传输** — 永远不要用栈上局部变量作为 DMA 源/目标缓冲区
2. **中断中不要调用 HAL 阻塞函数** — `HAL_UART_AbortReceive`、`HAL_UART_Abort` 等会轮询等待，不能在 ISR 中调用
3. **中断中不要操作 DMA** — 中断中只设标志位，实际 DMA 操作推迟到任务上下文
4. **sscanf 格式字符串必须匹配变量实际类型** — 强制类型转换 `(int*)&uint8_t` 会导致内存越界
5. **FreeRTOS 任务栈大小需要评估** — 包含大局部数组（`cmd[256]`）的任务至少需要 512 words
