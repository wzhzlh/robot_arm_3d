#!/bin/bash
# ===================================================
# Ubuntu STM32 开发环境一键安装脚本
# 适用：Ubuntu 20.04 / 22.04 / 24.04
# 目标：robot_arm_3d 项目
# ===================================================
set -e

echo "=========================================="
echo "  STM32 开发环境安装脚本"
echo "=========================================="

# ----- 1. 基础依赖 -----
echo "[1/7] Installing system dependencies..."
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    wget \
    curl \
    git \
    python3 \
    python3-pip \
    libusb-1.0-0-dev \
    libhidapi-dev \
    pkg-config

# ----- 2. ARM GCC 工具链 -----
echo "[2/7] Installing ARM GCC toolchain (arm-none-eabi-gcc)..."
TOOLCHAIN_URL="https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz"
TOOLCHAIN_DIR="/opt/arm-gnu-toolchain"

if [ ! -d "$TOOLCHAIN_DIR" ]; then
    sudo mkdir -p "$TOOLCHAIN_DIR"
    wget -O /tmp/arm-gnu-toolchain.tar.xz "$TOOLCHAIN_URL"
    sudo tar -xJf /tmp/arm-gnu-toolchain.tar.xz -C "$TOOLCHAIN_DIR" --strip-components=1
    rm /tmp/arm-gnu-toolchain.tar.xz
    echo "ARM GCC toolchain installed to $TOOLCHAIN_DIR"
else
    echo "ARM GCC toolchain already exists at $TOOLCHAIN_DIR"
fi

# 添加到 PATH（写入 ~/.bashrc）
if ! grep -q "$TOOLCHAIN_DIR/bin" ~/.bashrc; then
    echo "export PATH=\$PATH:$TOOLCHAIN_DIR/bin" >> ~/.bashrc
    echo "Added $TOOLCHAIN_DIR/bin to PATH in ~/.bashrc"
fi
export PATH="$PATH:$TOOLCHAIN_DIR/bin"

# 验证
arm-none-eabi-gcc --version

# ----- 3. OpenOCD -----
echo "[3/7] Installing OpenOCD..."
if ! command -v openocd &> /dev/null; then
    sudo apt install -y openocd
else
    echo "OpenOCD already installed: $(openocd --version)"
fi

# ----- 4. JLink 驱动 -----
echo "[4/7] Installing JLink driver..."
JLINK_URL="https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb"
JLINK_DEB="/tmp/jlink.deb"

if ! command -v JLinkExe &> /dev/null; then
    wget --post-data="accept_license_agreement=accepted&submit=Download+software" \
         -O "$JLINK_DEB" "$JLINK_URL"
    sudo dpkg -i "$JLINK_DEB" || sudo apt install -f -y
    rm "$JLINK_DEB"
else
    echo "JLink already installed: $(JLinkExe --version 2>&1 | head -1)"
fi

# ----- 5. udev 规则（JLink + OpenOCD 免 sudo）-----
echo "[5/7] Configuring udev rules for JLink..."
if [ ! -f /etc/udev/rules.d/99-jlink.rules ]; then
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0101", MODE="0666"' \
        | sudo tee /etc/udev/rules.d/99-jlink.rules
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0104", MODE="0666"' \
        | sudo tee -a /etc/udev/rules.d/99-jlink.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "udev rules added for JLink"
else
    echo "udev rules already exist"
fi

# ----- 6. 下载 STM32F407 SVD 文件（用于 cortex-debug 外设寄存器视图）-----
echo "[6/7] Downloading STM32F407 SVD file for debugging..."
SVD_DIR="$(dirname "$0")/../debug"
SVD_FILE="$SVD_DIR/STM32F407.svd"

if [ ! -f "$SVD_FILE" ]; then
    # 从 cmsis-svd 仓库下载
    curl -L -o "$SVD_FILE" \
        "https://raw.githubusercontent.com/cmsis-svd/cmsis-svd/master/data/STMicro/STM32F407.svd"
    echo "SVD file downloaded to $SVD_FILE"
else
    echo "SVD file already exists"
fi

# ----- 7. VS Code 扩展推荐 -----
echo "[7/7] Done! Recommended VS Code extensions:"
echo "  - ms-vscode.cpptools           (C/C++ IntelliSense)"
echo "  - ms-vscode.cmake-tools        (CMake integration)"
echo "  - marus25.cortex-debug         (Embedded debugging)"
echo "  - twxs.cmake                   (CMake syntax highlighting)"
echo "  - llvm-vs-code-extensions.vscode-clangd (Clangd LSP)"

echo ""
echo "=========================================="
echo "  安装完成！请执行以下操作："
echo "  1. source ~/.bashrc         # 刷新 PATH"
echo "  2. 重新插拔 JLink 调试器    # 使 udev 规则生效"
echo "  3. 打开 VSCode，安装推荐扩展"
echo "  4. cmake --preset Debug     # 配置项目"
echo "  5. cmake --build --preset Debug  # 编译"
echo "  6. VSCode F5 启动调试       # 快捷键"
echo "=========================================="
