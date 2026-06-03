# robot_arm_3d

STM32F407 三轴机械臂工程，包含舵机串口控制、简单逆运动学和 K230 视觉坐标接入。

## 当前整理结果

- 已从构建中移除未被引用的旧模块：`My_lib/motors520.*`、`My_lib/PID_old.*`。
- 已删除业务代码中的大段注释块、废弃任务入口和调试残留变量。
- 详细设计说明集中保留在本文档；源码内只保留短注释和必要命名。
- `Core/`、`Drivers/`、`Middlewares/` 主要是 CubeMX 或第三方代码，未做激进删改，避免后续重新生成和升级时冲突。

## 硬件与工具

| 项目 | 说明 |
|------|------|
| 主控 | STM32F407VG |
| 执行机构 | 众灵串口舵机 |
| 视觉模块 | K230 |
| 调试器 | J-Link V9 |
| 构建 | CMake + arm-none-eabi-gcc |
| 烧录 | OpenOCD |

## 目录说明

| 路径 | 作用 |
|------|------|
| `My_task/` | FreeRTOS 任务入口和上层动作逻辑 |
| `My_lib/commuction.*` | 舵机总线通信、DMA 收发、同步等待回复 |
| `My_lib/drive.*` | 机械臂角度/PWM换算、2D/3D 逆运动学 |
| `My_lib/k230.*` | K230 串口接收和目标坐标解析 |
| `My_lib/crc_ccitt.*` | K230 帧校验 |
| `Core/` | STM32CubeMX 生成代码 |
| `Drivers/` | HAL/CMSIS 驱动 |
| `Middlewares/` | FreeRTOS 中间件 |

## 当前任务模型

`MX_FREERTOS_Init()` 会创建 `defaultTask`，它只负责调用 `task_init()`。

`task_init()` 当前只启动一个任务：

- `mot_rece`
  负责初始化舵机通信，并周期读取 1/2/3 号舵机当前位置。

这意味着当前固件默认行为是“舵机总线初始化 + 周期位置轮询”。  
`start_task.c` 里保留了三个备用动作流程，但默认不会自动启动：

- `requirement1()`：单关节动作演示。
- `requirement2()`：笛卡尔矩形轨迹插补。
- `requirement3()`：等待 K230 目标坐标并移动到目标点。

如果后续要启用这些流程，建议在 `task_init()` 里显式创建独立任务，而不是把多个无限循环函数串行调用。

## 舵机通信设计

### 数据结构

`ServoBus_t` 是全局机械臂对象，包含：

- `target_pos`：目标末端坐标。
- `state_pos`：预留的状态坐标。
- `target_time`：本次动作时间。
- `motor[3]`：三个舵机的 ID、发送位置、接收位置。

### 串口同步机制

舵机使用 UART2 + DMA，通信层把异步中断包装成同步调用：

- `servo_tx_sem`
  表示 DMA 发送完成。
- `servo_rx_sem`
  表示收到数据或出现错误，供轮询式接收流程使用。
- `servo_rx_reply_sem`
  表示一问一答式命令已经拿到完整回复。

`ServoBus_SendAndWaitReply()` 的流程：

1. 清掉上一次残留信号量。
2. 发送串口命令。
3. 等待 DMA 发送完成。
4. 等待接收回调给出回复完成信号。
5. 读取 `g_servo_id`、`g_servo_pwm` 和 `arm.motor[].motor_rx_pos`。

### 关键接口

- `ServoBus_Start_Receive()`
  启动 UART2 的 `ReceiveToIdle DMA`。
- `ServoBus_ReadAngle(id)`
  发送 `#xxxPRAD!` 并阻塞等待当前角度回复。
- `ServoBus_Move_Many(&arm, 3)`
  把三个舵机命令拼成一帧群发。
- `ServoBus_ErrorRecovery()`
  在 UART 异常后中止并重启接收 DMA。

### 角度与 PWM 约定

- 关节 1、2：`0° -> 500`，比例系数约 `7.407`。
- 关节 3：`0° -> 1500`，支持 `-135° ~ 135°`。
- `set_angles()` 会先把角度换算成 PWM，再调用 `ServoBus_Move_Many()`。

## 运动学说明

### 连杆参数

定义在 `My_lib/drive.c`：

- `L1 = 0.0555 m`
- `L2 = 0.0700 m`
- `L3 = 0.1040 m`

### 逆运动学

- `IK_3D()`
  先根据 `(x, y)` 计算底座旋转角，再调用 `IK_2D()` 处理竖直平面。
- `IK_2D()`
  把目标点投影到 2D 平面，用余弦定理解出第二、第三关节角。

### 直线插补

`line_interp()` 会：

1. 根据起点/终点生成 `x(t)`、`y(t)`、`z(t)` 三条一次函数。
2. 按固定步长采样目标点。
3. 每个采样点调用 `move_to()`，再由逆运动学换算关节角。

当前实现更偏向“命令序列生成”。如果后续要做平滑轨迹，建议补上严格的时间同步和反馈闭环。

## K230 协议说明

K230 走 UART3 + DMA 空闲中断，当前约定帧格式：

```text
[HEAD][LEN][DATA...][CRC_H][CRC_L][TAIL]
```

- `HEAD = 0xAA`
- `TAIL = 0x55`
- `LEN` 是 `DATA` 长度
- 当前 `DATA` 为 3 字节：`x, y, z`
- 总帧长应为 `LEN + 5`

`K230_ParseFrame()` 会做三步校验：

1. 校验帧头和帧尾。
2. 校验长度字段与实际帧长是否一致。
3. 校验 CRC-CCITT。

校验通过后更新：

- `k230_target_pos.x`
- `k230_target_pos.y`
- `k230_target_pos.z`
- `k230_comm_status = K230_RECEIVED_OK`

## 编译与烧录

```bash
cmake --preset Debug
cmake --build --preset Debug
```

```bash
openocd -f openocd.cfg -c "program build/Debug/robot_arm_3d.elf verify reset exit"
```

首次安装依赖：

```bash
bash tools/setup_ubuntu.sh
source ~/.bashrc
```

## 维护建议

- `Core/`、`Drivers/`、`Middlewares/` 优先通过 CubeMX 或官方升级维护，不建议手工做大规模注释清理。
- 业务说明优先写在 `README.md`，源码里只保留“做什么”的短注释，不写大段背景解释。
- 如果要新增任务，优先在 `task_init()` 做明确的任务创建，不要继续保留被注释掉的 `xTaskCreate()` 模板。
