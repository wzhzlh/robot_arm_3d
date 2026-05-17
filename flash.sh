#!/bin/bash

# 检查编译产物
if [ ! -f "build/Debug/robot_arm_3d.elf" ]; then
    echo "❌ 固件不存在，请先编译："
    echo "   cmake --build --preset Debug"
    exit 1
fi

# 检查OpenOCD是否运行
if ! nc -z localhost 3333 2>/dev/null; then
    echo "❌ OpenOCD 未在运行。请先启动："
    echo "   openocd -f interface/jlink.cfg -c 'transport select swd' -f target/stm32f4x.cfg"
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
