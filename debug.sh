#!/bin/bash

# 检查OpenOCD是否运行
if ! nc -z localhost 3333 2>/dev/null; then
    echo "❌ OpenOCD 未在运行。请先启动："
    echo "   openocd -f interface/jlink.cfg -c 'transport select swd' -f target/stm32f4x.cfg"
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
