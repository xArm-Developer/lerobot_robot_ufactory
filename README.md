# UFACTORY LeRobot

UFACTORY 机械臂与 LeRobot 框架集成项目，支持多种遥操作方式的数据采集、策略训练和部署推理。

## 功能特性

- 🤖 UFACTORY 机械臂控制（xArm 系列）
- 🎮 多种遥操作方式：GELLO / Pika / UMI / SpaceMouse
- 📷 多摄像头数据采集（RealSense / UMI 相机）
- 📊 数据集录制与管理（兼容 LeRobot 格式）
- 🧠 模仿学习训练（ACT / Diffusion Policy 等）
- 🚀 策略评估与实时推理
- 🔧 Mock 机器人模拟（只用遥操作设备采集数据）

## 环境要求

- Ubuntu 22.04 / 24.04
- Python >= 3.10
- CUDA >= 12.0（GPU 训练推荐）
- UFACTORY 机械臂（xArm 系列，可选）

## 安装

### 基础项目安装

```bash
git clone https://github.com/xArm-Developer/ufactory_lerobot.git
cd ufactory_lerobot

# 创建 conda 环境
conda create -n uf_lerobot python=3.10 -y
conda activate uf_lerobot

# 安装项目
pip install -e .
```

包含：`lerobot==0.4.3`、`xarm-python-sdk`、`numpy`、`pyyaml`（lerobot 已自动携带 torch、opencv、wandb 等训练相关依赖）。

### 外设模块安装

外设依赖以可选模块形式提供，通过 `[模块名]` 安装。

#### GELLO 遥操作

适用于 GELLO 示教臂（Dynamixel 舵机方案），控制空间为关节空间。

```bash
# 1. 安装 GELLO 模块
pip install -e ".[gello]"

# 2. 添加串口权限（重新登录后生效）
sudo usermod -aG dialout $USER
```

#### Pika 遥操作

适用于 Pika Sense 手持示教器 + Vive Tracker，控制空间为笛卡尔空间。

```bash
# 1. 安装外设依赖（不需要它们的间接依赖）
pip install pysurvive agx-pypika --no-deps

# 2. 安装 udev 规则（重新插拔设备后生效）
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Vive Tracker 首次使用前需校准：`uf-vive-calibrate`

#### UMI 遥操作

适用于 UMI（Universal Manipulation Interface）方案，含 Vive Tracker 追踪，支持双机械臂。

```bash
# 1. 安装 XVSDK（系统级依赖，仅支持 Ubuntu Focal）
sudo dpkg -i src/xvsdk/XVSDK_focal_amd64.deb
sudo apt install -y --fix-broken

# 2. 安装外设依赖
pip install pysurvive --no-deps

# 3. 安装 udev 规则（重新插拔设备后生效）
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Vive Tracker 首次使用前需校准：`uf-vive-calibrate`

**多 UMI 设备配置**（使用两台及以上时）：

```bash
# 增加 USB 缓冲区大小
sudo sed -i '/GRUB_CMDLINE_LINUX_DEFAULT/s/quiet splash/quiet splash usbcore.usbfs_memory_mb=128/' /etc/default/grub
sync
sudo update-grub
sudo reboot
```

#### SpaceMouse 遥操作

适用于 3Dconnexion SpaceMouse / SpaceNavigator。

```bash
# 1. 安装 SpaceMouse 模块
pip install -e ".[spacemouse]"

# 2. 安装 udev 规则（重新插拔设备后生效）
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```


## 使用

### 1. 遥操作测试

测试遥操作设备与机械臂的联动，不录制数据。

```bash
uf-robot-teleop -c path/to/config.yaml
uf-robot-teleop -c path/to/config.yaml -f 60  # 指定频率
```

### 2. 数据采集

通过遥操作录制数据集。

```bash
uf-lerobot-record -c path/to/record_config.yaml
uf-lerobot-record -c path/to/config.yaml --resume  # 续录
```

### 3. 策略训练

采集数据后，使用 LeRobot 训练管道进行模仿学习训练。

```bash
lerobot-train --policy act --dataset your_dataset_name
```

### 4. 策略评估

```bash
uf-lerobot-eval -c path/to/eval_config.yaml
```

### Mock 模式（无实体机械臂）

```yaml
type: "uf::mock_robot"           # 单臂模拟
type: "uf::multiple_mock_robot"  # 双臂模拟
```

## 遥操作方式对比

| 特性 | GELLO | Pika | UMI | SpaceMouse |
|------|-------|------|-----|------------|
| 控制空间 | 关节空间 | 任务空间 | 任务空间 | 任务空间 |
| 跟踪方式 | Dynamixel 舵机 | Vive Tracker | UMI SLAM / Vive | 3D 鼠标 |
| 双臂支持 | ❌ | ❌ | ✅ | ❌ |
| 系统依赖 | dialout 组 | — | XVSDK deb | — |

## 项目结构

```
ufactory_lerobot/
├── src/
│   ├── ufactory_lerobot/
│   │   ├── robots/                 # 机器人控制
│   │   │   ├── uf_robot/           #   xArm 实体机器人
│   │   │   ├── uf_mock_robot/      #   仿真 Mock 机器人
│   │   ├── teleoperators/          # 遥操作器
│   │   │   ├── gello_teleop/       #   GELLO (Dynamixel 示教臂)
│   │   │   ├── pika_teleop/        #   Pika Sense (手持示教器 + Vive)
│   │   │   ├── umi_teleop/         #   UMI (含双机械臂)
│   │   │   ├── space_mouse/        #   SpaceMouse (3D 鼠标)
│   │   ├── cameras/                # 摄像头模块
│   │   │   └── umi_camera/         #   UMI 相机
│   │   ├── devices/                # 外部设备驱动
│   │   │   ├── pika/               #   Pika 串口驱动
│   │   │   └── umi/                #   XVLib / Vive Tracker
│   │   ├── scripts/                # 执行脚本
│   │   │   ├── uf_robot_teleop.py     # 遥操作测试
│   │   │   ├── uf_lerobot_record.py   # 数据采集
│   │   │   ├── uf_lerobot_eval.py     # 策略评估
│   │   │   └── vive_calibrate.py      # Vive Tracker 校准
│   │   └── utils/                  # 工具函数
│   ├── rules/                      # udev 设备规则
│   └── xvsdk/                      # XVSDK 系统依赖
├── config/                         # YAML 配置文件
│   ├── gello/
│   ├── pika/
│   ├── umi/
│   └── spacemouse/
├── pyproject.toml
└── README.md
```

## 许可证

本项目基于 Apache License 2.0 发布，详见 [LICENSE](LICENSE) 文件。
