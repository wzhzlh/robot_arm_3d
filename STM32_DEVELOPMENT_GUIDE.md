# STM32 CMake 开发完全指南

本指南用于快速设置 STM32 微控制器项目的编译、烧录和调试环境（使用 J-Link）。

## 目录
1. [环境配置](#环境配置)
2. [项目结构](#项目结构)
3. [CMake 配置](#cmake-配置)
4. [编译流程](#编译流程)
5. [烧录和调试](#烧录和调试)
6. [快捷脚本](#快捷脚本)
7. [常见问题](#常见问题)

---
新 MCU 需要调整
cmake/gcc-arm-none-eabi.cmake - 改 -mcpu=cortex-mX
链接脚本 - 改文件名和路径
OpenOCD 命令 - 改 stm32fXx.cfg
CMakeLists.txt - 改源文件和库路径



# 1️⃣ 编译
cmake --build --preset Debug

# 2️⃣ 启动 OpenOCD（一个终端）
openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg

# 3️⃣ 烧录或调试（另一个终端）
./flash.sh    # 烧录
./debug.sh    # 调试

## 环境配置

### 必需工具

| 工具 | 用途 | 安装方式 |
|------|------|--------|
| `gcc-arm-none-eabi` | ARM 交叉编译器 | `apt install gcc-arm-none-eabi` |
| `cmake` | 构建系统 | `apt install cmake` |
| `ninja-build` | 构建工具 | `apt install ninja-build` |
| `gdb-multiarch` | 调试器 | `apt install gdb-multiarch` |
| `openocd` | 烧录/调试服务器 | `apt install openocd` |

### 验证安装

```bash
arm-none-eabi-gcc --version   # 10.3.1 或更高
cmake --version               # 3.22 或更高
ninja --version               # 1.10 或更高
gdb-multiarch --version       # 12.1 或更高
openocd --version             # 0.11 或更高
```

### 权限配置（USB 设备访问）

```bash
# 添加用户到 dialout 和 plugdev 组
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

# 添加 udev 规则
sudo tee /etc/udev/rules.d/99-jlink.rules > /dev/null <<EOF
# J-Link
ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0101", MODE="0666"
ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0102", MODE="0666"
ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0103", MODE="0666"
ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0104", MODE="0666"
EOF

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 重新登录或执行
newgrp dialout
```

---

## 项目结构

```
project_root/
├── CMakeLists.txt                    # 主 CMake 配置
├── CMakePresets.json                 # CMake 预设（Debug/Release）
├── STM32_DEVELOPMENT_GUIDE.md        # 本指南
├── debug.sh                          # 快捷调试脚本
├── flash.sh                          # 快捷烧录脚本
├── STM32F407XX_FLASH.ld              # 链接脚本
├── cmake/
│   ├── gcc-arm-none-eabi.cmake       # ARM 工具链配置
│   └── stm32cubemx/
│       ├── CMakeLists.txt
│       └── stm32cubemx.ioc           # CubeMX 项目
├── Core/
│   ├── Inc/                          # HAL 头文件
│   └── Src/                          # HAL 源文件
├── Drivers/                          # CMSIS/HAL 库
├── My_task/                          # 用户任务代码
├── My_lib/                           # 用户库代码
└── build/
    └── Debug/
        ├── robot_arm_3d.elf          # 可执行文件（含符号）
        ├── robot_arm_3d.bin          # 二进制文件（烧录用）
        └── robot_arm_3d.map          # 符号表
```

---

## CMake 配置

### CMakeLists.txt 关键部分

```cmake
# 最小 CMake 版本
cmake_minimum_required(VERSION 3.22)

# 编译选项
set(CMAKE_C_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")  # Debug 或 Release
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 项目名称
project(robot_arm_3d)

# 启用 C 和汇编
enable_language(C ASM)

# 创建可执行文件
add_executable(${CMAKE_PROJECT_NAME})

# 添加子目录（CubeMX 生成）
add_subdirectory(cmake/stm32cubemx)

# 添加源文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    My_task/start_task.c
    My_task/task_init.c
    My_lib/commuction.c
    # ... 更多文件
)

# 添加包含目录
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    My_task
    My_lib
)

# 链接库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
```

### CMakePresets.json 配置

```json
{
    "version": 3,
    "configurePresets": [
        {
            "name": "default",
            "hidden": true,
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build/${presetName}",
            "toolchainFile": "${sourceDir}/cmake/gcc-arm-none-eabi.cmake"
        },
        {
            "name": "Debug",
            "inherits": "default",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug"
            }
        },
        {
            "name": "Release",
            "inherits": "default",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Release"
            }
        }
    ]
}
```

### gcc-arm-none-eabi.cmake 工具链配置

```cmake
# 系统信息
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 编译器
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

# MCU 特定标志（Cortex-M4）
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# 编译和链接标志
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

# 调试/优化选项
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")

# 链接脚本
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")
```

---

## 编译流程

### 首次编译

```bash
# 进入项目目录
cd /path/to/stm32_project

# 配置项目（使用 CMake Presets）
cmake --preset Debug

# 构建
cmake --build --preset Debug
```

### 快速编译

```bash
# 只重新构建
cmake --build --preset Debug

# 清理后重新构建
rm -rf build
cmake --preset Debug && cmake --build --preset Debug
```

### 编译结果

编译成功后，在 `build/Debug/` 目录下会生成：

```
robot_arm_3d.elf     # ELF 文件（含调试符号，用于调试）
robot_arm_3d.bin     # 二进制文件（用于烧录）
robot_arm_3d.map     # 符号表和内存使用情况
```

查看编译产物大小：
```bash
ls -lh build/Debug/robot_arm_3d.*
arm-none-eabi-size build/Debug/robot_arm_3d.elf
```

---

## 烧录和调试

### 硬件连接

连接 **J-Link** 调试器到 MCU 的 SWD 接口：
- SWCLK → CLK
- SWDIO → DIO
- GND → GND
- 3.3V → VCC（可选）

### 启动 OpenOCD 调试服务器

```bash
# 终端 1
openocd -f interface/jlink.cfg \
        -c "transport select swd" \
        -f target/stm32f4x.cfg
```

**预期输出**：
```
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : J-Link V9 compiled May  7 2021
Info : VTarget = 3.362 V
Info : clock speed 2000 kHz
Info : SWD DPIDR 0x2ba01477
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : starting gdb server on 3333
```

### 方法 1：使用快捷脚本（推荐）

#### 烧录固件
```bash
# 终端 2
./flash.sh
```

#### 调试程序
```bash
# 终端 2
./debug.sh
```

### 方法 2：手动 GDB 调试

```bash
# 终端 2
gdb-multiarch build/Debug/robot_arm_3d.elf

# 在 GDB 中
(gdb) set architecture arm
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) break main
(gdb) continue
```

### 方法 3：命令行一键烧录

```bash
gdb-multiarch build/Debug/robot_arm_3d.elf \
  -ex "set architecture arm" \
  -ex "target remote localhost:3333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "quit"
```

---

## 快捷脚本

### flash.sh - 烧录脚本

```bash
#!/bin/bash

if [ ! -f "build/Debug/robot_arm_3d.elf" ]; then
    echo "❌ 固件不存在，请先编译"
    exit 1
fi

if ! nc -z localhost 3333 2>/dev/null; then
    echo "❌ OpenOCD 未运行"
    exit 1
fi

echo "✓ J-Link 已连接"
echo "📥 开始烧录固件..."

gdb-multiarch build/Debug/robot_arm_3d.elf \
  -ex "set architecture arm" \
  -ex "target remote localhost:3333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "quit"

echo "✅ 烧录完成！"
```

### debug.sh - 调试脚本

```bash
#!/bin/bash

if ! nc -z localhost 3333 2>/dev/null; then
    echo "❌ OpenOCD 未运行"
    exit 1
fi

echo "✓ J-Link 已连接"
echo "📥 启动 GDB 调试器..."

gdb-multiarch build/Debug/robot_arm_3d.elf \
  -ex "set architecture arm" \
  -ex "target remote localhost:3333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "break main" \
  -ex "continue"
```

**赋予执行权限**：
```bash
chmod +x flash.sh debug.sh
```

---

## GDB 调试命令

| 命令 | 功能 |
|------|------|
| `n` | 单步执行（不进入函数） |
| `s` | 单步执行（进入函数） |
| `c` | 继续运行 |
| `b main` | 在 main 处设置断点 |
| `b file.c:10` | 在指定行设置断点 |
| `info breakpoints` | 显示所有断点 |
| `delete 1` | 删除断点 1 |
| `p variable` | 打印变量值 |
| `info locals` | 查看本地变量 |
| `info registers` | 查看寄存器 |
| `backtrace` | 显示调用栈 |
| `monitor reset` | 复位 MCU |
| `monitor halt` | 停止 MCU |
| `monitor resume` | 继续运行 |
| `quit` | 退出 GDB |

---

## VS Code 集成（可选）

### .vscode/launch.json

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "J-Link Debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/Debug/robot_arm_3d.elf",
            "cwd": "${workspaceFolder}",
            "stopAtEntry": false,
            "miDebuggerPath": "gdb-multiarch",
            "miDebuggerServerAddress": "localhost:3333",
            "setupCommands": [
                {"text": "set architecture arm"},
                {"text": "target remote localhost:3333"},
                {"text": "monitor reset halt"},
                {"text": "load"}
            ]
        }
    ]
}
```

### .vscode/tasks.json

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build Debug",
            "type": "shell",
            "command": "cmake",
            "args": ["--build", "--preset", "Debug"],
            "group": {"kind": "build", "isDefault": true}
        },
        {
            "label": "Flash",
            "type": "shell",
            "command": "./flash.sh",
            "group": {"kind": "build"}
        }
    ]
}
```

---

## 完整工作流程

```bash
# 1. 编译
cmake --build --preset Debug

# 2. 启动 OpenOCD（终端 1）
openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg

# 3. 烧录（终端 2）
./flash.sh

# 或者调试（终端 2）
./debug.sh
```

---

## 常见问题

### Q1: "OpenOCD 未运行"
**A**: 在另一个终端启动 OpenOCD：
```bash
openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg
```

### Q2: "J-Link 识别不到"
**A**: 检查 USB 连接和驱动：
```bash
lsusb | grep -i segger
```

### Q3: "加载固件失败"
**A**: 确认：
1. 链接脚本正确（`STM32F407XX_FLASH.ld`）
2. MCU 型号匹配（`stm32f4x.cfg`）
3. 编译成功生成 `.elf` 文件

### Q4: "目标断开连接"
**A**: 复位 MCU 和调试器：
```bash
openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg
```

### Q5: 如何修改 MCU 型号？
**A**: 修改以下文件：
1. `cmake/gcc-arm-none-eabi.cmake` - 改 `-mcpu=cortex-mX`
2. `CMakeLists.txt` - 改链接脚本路径
3. OpenOCD 命令 - 改 `stm32fXx.cfg`

---

## 新项目快速复制清单

创建新 STM32 项目时，复制以下文件和目录：

- ✅ `CMakeLists.txt`
- ✅ `CMakePresets.json`
- ✅ `cmake/gcc-arm-none-eabi.cmake`
- ✅ `cmake/stm32cubemx/CMakeLists.txt`
- ✅ `flash.sh`
- ✅ `debug.sh`
- ✅ `.vscode/launch.json`
- ✅ `.vscode/tasks.json`
- ✅ 链接脚本 (`STM32XXXX_FLASH.ld`)

然后根据新项目调整：
- MCU 型号、时钟、内存大小
- 源文件路径
- 包含目录
- 链接脚本

---

**最后更新**: 2026-05-17  
**STM32 版本**: F407 (Cortex-M4)  
**调试器**: J-Link V9
