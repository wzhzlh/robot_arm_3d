# SSH 连接与机器人项目启动指南

## 📋 前置条件

### 网络要求
- **主机**和**狗PC**必须连接到**同一个局域网**

---

## 🔐 一、SSH 连接

### 连接命令
```bash
ssh dog@192.168.18.64
```

> ⚠️ **注意**：每个终端都需要单独执行此命令进行连接

---

## 🚀 二、启动工程

### 步骤 1: 进入项目目录

#### 方法 A: 使用 fd-find（推荐）
```bash
# 安装 fd-find（可选，首次使用需要）
sudo apt install fd-find

# 查找 AT_robot-lab 目录
fdfind AT_robot-lab ~

# 或查找 dog 目录
fdfind dog ~
```

#### 方法 B: 手动导航
```bash
cd ~/AT_robot-lab    # 或实际的项目路径
```

---

### 步骤 2: 刷新与启动工程

#### 在 **dog** 目录下执行：
```bash
# 1. 刷新环境配置
. install/setup.bash

# 2. 启动 bringup
ros2 launch bring_up bringup.launch.py
```

#### 在 **AT_robot-lab** 目录下执行：
```bash
# 1. 刷新环境配置
. install/setup.bash

# 2. 运行 RL SAR 节点
ros2 run rl_sar rl_real_atdog2

# 3. 启动障碍物游戏远程测试
ros2 launch obstacle_game remote_test.launch.py
```

---

## 📝 注意事项

1. **终端顺序**：建议先启动 dog 端的 bringup，再启动 AT_robot-lab 端的服务
2. **环境变量**：每次打开新终端都需要执行 `. install/setup.bash`
3. **网络连接**：确保两台设备在同一局域网，且 IP 地址 `192.168.18.64` 可达
4. **权限问题**：如遇到权限错误，可能需要使用 `sudo` 或检查文件权限

---

## 🔧 故障排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| SSH 连接失败 | 网络不通或IP错误 | 检查网络连接，确认IP地址 |
| 找不到命令 | 环境变量未加载 | 执行 `. install/setup.bash` |
| ROS2 启动失败 | 依赖缺失 | 检查 ros2 和相关包是否安装 |
| fd-find 未找到 | 未安装工具 | 执行 `sudo apt install fd-find` |
