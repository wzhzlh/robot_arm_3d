# robot_arm_3d

三轴机械臂（配合 K230 模块实现视觉追踪；纯位控）

## 硬件平台

| 项目 | 参数 |
|------|------|
| **主控** | STM32F407VG (Cortex-M4F @ 168MHz) |
| **舵机** | 众灵舵机（串口控制） |
| **视觉模块** | K230 |
| **调试器** | J-Link V9 (SWD 接口) |

## 开发环境

### 工具链

| 层级 | 技术栈 |
|------|--------|
| **IDE** | VS Code |
| **构建系统** | CMake 3.22+ / Ninja |
| **编译器** | arm-none-eabi-gcc (主) / starm-clang (备选) |
| **烧录** | OpenOCD 0.11.0 + JLink SWD |
| **调试** | VSCode (cortex-debug) 或 Ozone v3.04i |

### 首次安装

```bash
# 一键安装所有依赖（arm-gcc / OpenOCD / JLink / udev 规则 / SVD 文件）
bash tools/setup_ubuntu.sh
source ~/.bashrc
```

### 编译

```bash
# Debug 版（含调试符号，-O0）
cmake --preset Debug && cmake --build --preset Debug

# Release 版（优化体积，-Os）
cmake --preset Release && cmake --build --preset Release
```

### 烧录

```bash
# Release 版
openocd -f openocd.cfg -c "program build/Release/robot_arm_3d.elf verify reset exit"

# Debug 版
openocd -f openocd.cfg -c "program build/Debug/robot_arm_3d.elf verify reset exit"
```

### 调试

| 方式 | 启动 |
|------|------|
| **VSCode + cortex-debug** | 打开项目，`F5` → 选择配置文件 |
| **VSCode + cppdbg + OpenOCD** | `F5` → `Debug with OpenOCD + GDB` |
| **Ozone v3.04i** | `ozone -j debug/robot_arm_3d.jdebug` |

> Ozone 直接通过 JLink 连接目标，无需启动 OpenOCD。

### VS Code 快捷任务

| 操作 | 任务名 |
|------|--------|
| 编译 Debug | `Build Debug` |
| 编译 Release | `Build Release` |
| 烧录 Release | `Flash with OpenOCD` |
| 烧录 Debug | `Flash with OpenOCD (Debug)` |
| 启动 GDB 服务器 | `OpenOCD Server` |
| 启动 Ozone | `Open Ozone` |
| 一键烧录+调试 | `Flash & Debug` |

## 项目结构

```
robot_arm_3d/
├── CMakeLists.txt              # 主构建文件
├── CMakePresets.json           # CMake 配置预设
├── openocd.cfg                 # OpenOCD 烧录配置
├── .clangd                     # Clangd LSP 配置
├── cmake/
│   ├── gcc-arm-none-eabi.cmake # ARM GCC 工具链文件
│   ├── starm-clang.cmake       # LLVM/Clang 备选工具链
│   └── stm32cubemx/            # STM32CubeMX 构建规则
├── Core/                       # HAL 核心代码
├── Drivers/                    # STM32 HAL 驱动
├── Middlewares/                 # FreeRTOS 中间件
├── My_lib/                     # 自定义库（舵机控制、PID、协议等）
├── My_task/                    # 应用任务
├── debug/
│   └── robot_arm_3d.jdebug     # Ozone 调试配置
├── tools/
│   └── setup_ubuntu.sh         # Ubuntu 环境一键安装
└── .vscode/
    ├── launch.json             # 调试配置
    ├── tasks.json              # 构建/烧录/调试任务
    ├── settings.json           # 编辑器设置
    └── extensions.json         # 推荐openocd -f openocd.cfg -c "program build/Debug/robot_arm_3d.elf verify reset exit"扩展
```

## 固件内存占用

| 版本 | Flash | RAM |
|------|-------|-----|
| Debug | ~8.7% (44.5 KB) | ~14.4% (18.9 KB) |
| Release | ~6.5% (33.5 KB) | ~14.4% (18.9 KB) |
