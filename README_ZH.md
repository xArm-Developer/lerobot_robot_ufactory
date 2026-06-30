# UFACTORY LeRobot

> [English Version](README.md)

UFACTORY(深圳市众为创造科技有限公司) 机械臂与 LeRobot 框架集成项目，支持多种遥操作方式的数据采集、策略训练和部署推理。

## 训练推理效果

点击下载开发时[采集的数据集](https://drive.google.com/drive/folders/1Ms25rd2YYGdh3tHPEsTTMU-m1fE7uNYY)，**仅供参考，不可复用**。因为用户机械臂和摄像头位置和开发测试时不一致。

<table>
<tr>
  <td width="50%">
    <a href="https://www.bilibili.com/video/BV12xFjzzEaX" target="_blank">
      <img src="https://i2.hdslb.com/bfs/archive/7b325df5fb4c16e922b66d27b56d8fb6534f8b46.jpg" width="100%">
    </a>
  </td>
  <td width="50%">
    <a href="https://www.bilibili.com/video/BV16ccizHE2P" target="_blank">
      <img src="https://i2.hdslb.com/bfs/archive/8c5d5e9370577fa89d06a175aceea282d9b2eb9a.jpg" width="100%">
    </a>
  </td>
</tr>
<tr>
  <td width="50%">
    <a href="https://www.bilibili.com/video/BV1xGEy6mE3i" target="_blank">
      <img src="https://i2.hdslb.com/bfs/archive/c8bbac04a736b7043a20b753e11afea7627bdae2.jpg" width="100%">
    </a>
  </td>
</tr>
</table>

## 功能特性

- 🤖 UFACTORY 机械臂控制（[xArm 系列](https://www.ufactory.cc/)）
- 🎮 多种遥操作方式：GELLO / [Pika](https://global.agilex.ai/products/pika) / [UMI](https://lumosumi.lumosbot.tech/pro/) / [SpaceMouse](https://3dconnexion.com/sg/product/spacemouse-wireless/)
- 📷 多摄像头数据采集（[RealSense](https://www.realsenseai.com/products/depth-camera-d435i/) / UMI 相机）
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
git clone https://github.com/xArm-Developer/lerobot_robot_ufactory.git
cd lerobot_robot_ufactory

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
* 一旦开始数据采集，机械臂与摄像头（D435 / D435i）的**相对位置必须保持不变**。
* 推理时的摄像头位置必须与采集时相同。若机械臂或摄像头发生变化，此前采集的数据将无效。

```bash
# 1. 安装 GELLO 模块
pip install -e ".[gello]"

# 2. 添加串口权限（重新登录后生效）
sudo usermod -aG dialout $USER
```

#### Pika 遥操作

适用于 Pika Sense 手持示教器 + Vive Tracker，控制空间为笛卡尔空间。
* 两个基站和机械臂相对位置没有要求，只需要保证采集时pika sense在基站范围内，但**基站移动后需要重新校准**。
* 采集和推理时基站位置可不相同。

```bash
# 1. 安装外设依赖（不需要它们的间接依赖）
pip install pysurvive agx-pypika --no-deps

# 2. 安装 udev 规则（重新插拔设备后生效）
sudo cp rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Vive Tracker 首次使用前需校准：`uf-vive-calibrate`

#### UMI 遥操作

适用于 UMI（Universal Manipulation Interface）方案，含 Vive Tracker 追踪，支持双机械臂。

```bash
# 1. 安装 XVSDK（系统级依赖，仅支持 Ubuntu Focal）
curl -sL https://raw.githubusercontent.com/xArm-Developer/ufactory_resources/main/fastumi/sdk/XVSDK_focal_amd64.deb -o /tmp/xvsdk.deb && sudo dpkg -i /tmp/xvsdk.deb
sudo apt install -y --fix-broken

# 2. 安装外设依赖
pip install pysurvive --no-deps

# 3. 安装 udev 规则（重新插拔设备后生效）
sudo cp rules/*.rules /etc/udev/rules.d/
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
sudo cp rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 使用

### 1. 遥操作测试

测试遥操作设备与机械臂的联动，不录制数据。

```bash
# 通用格式
uf-robot-teleop --config_path path/to/config.yaml
uf-robot-teleop --config_path path/to/config.yaml --fps 60  # 指定频率

# 示例: xArm6 + UMI 遥操作
uf-robot-teleop --config_path config/umi/xarm6_umi_record_config.yaml
```

### 2. 数据采集

通过遥操作录制数据集。

```bash
# 通用格式
uf-lerobot-record --config_path path/to/record_config.yaml
uf-lerobot-record --config_path path/to/config.yaml --resume true     # 续录

# 示例: xArm6 + UMI 数据采集
uf-lerobot-record --config_path config/umi/xarm6_umi_record_config.yaml
```

### 3. Lerobot训练

采集数据后，使用 LeRobot 训练管道进行模仿学习训练。

```bash
# 通用格式
lerobot-train --policy act --dataset your_dataset_name
```

参数示例：

```bash
# 注意: repo_id就是采集时配置文件里面的repo_id
# 这里训练策略policy.type选用act，训练steps为80w次
# 训练过程每2w次保存一次结果，结果输出到和lerobot同级目录下的lerobot_datas/train里面
lerobot-train \
  --dataset.root=../../../../lerobot_datas/record/ufactory/xarm6_umi_datas \
  --dataset.repo_id=ufactory/xarm6_umi_datas \
  --policy.type=act \
  --policy.device=cuda \
  --policy.repo_id=ufactory/xarm6_umi_datas \
  --output_dir=../../../../lerobot_datas/train/xarm6_umi_datas \
  --job_name=xarm6_umi_datas \
  --steps=800000 \
  --batch_size=8 \
  --save_freq=20000 
```

### 4. 推理

指定模型进行推理

```bash
# 通用格式
uf-lerobot-eval --config_path path/to/config.yaml --policy.path your_train_path

# 示例：使用训练好的 ACT 策略进行推理
uf-lerobot-eval --config_path config/umi/xarm6_umi_record_config.yaml --policy.path ../../../../lerobot_datas/train/xarm6_umi_datas/checkpoints/last/pretrained_model/
```

## 工具集

### 1. 摄像头查看器

查看和拼接多路摄像头画面。

```bash
uf-camera-view -l                           # 列出所有摄像头
uf-camera-view -l -T xvisio                 # 仅列出 XVisio 摄像头
uf-camera-view -T xvisio                    # 查看 XVisio 摄像头（默认 1280x1280 YU12）
uf-camera-view -T xvisio -W 640 -H 1920 -F NV12  # 指定格式
uf-camera-view -T other                     # 查看其他类型摄像头
```

### 2. Lerobot数据集工具
Lerobot提供一些数据集工具，方便对采集的数据集进行增删查操作。

### 查看某个索引的episode:
例如查看索引号为17的episode:
```bash
lerobot-dataset-viz \
  --root=../../../../lerobot_datas/record/ufactory/xarm7_record_datas \
  --repo-id ufactory/xarm7_record_datas \
  --display-compressed-images true \
  --episode-index 17
```

### 删除某些索引的episodes:
例如删除索引号为18和19的episode:
```bash
lerobot-edit-dataset \
  --root=../../../../lerobot_datas/record/ufactory/xarm7_record_datas \
  --repo_id ufactory/xarm7_record_datas \
  --new_repo_id ../xarm7_record_datas_new \
  --operation.type delete_episodes \
  --operation.episode_indices "[18, 19]"
```

### 合并数据集
```bash
lerobot-edit-dataset \
  --root=../../../../lerobot_datas/record \
  --repo_id ufactory/xarm7_record_datas_merge_1_2 \
  --operation.type merge \
  --operation.repo_ids "['ufactory/xarm7_record_datas_1', 'ufactory/xarm7_record_datas_2']"
```


## 遥操作方式对比

| 特性 | GELLO | Pika | UMI | SpaceMouse |
|------|-------|------|-----|------------|
| 控制空间 | 关节空间 | 笛卡尔空间 | 笛卡尔空间 | 笛卡尔空间 |
| 跟踪方式 | Dynamixel 舵机 | Vive Tracker | UMI SLAM / Vive | 3D 鼠标 |
| 双臂支持 | ❌ | ❌ | ✅ | ❌ |
| 系统依赖 | dialout 组 | — | XVSDK deb | — |

## 项目结构

```
lerobot_robot_ufactory/
├── src/
│   ├── lerobot_robot_ufactory/      # LeRobot 插件包
│   │   ├── robots/                 # 机器人控制
│   │   │   ├── uf_robot/           #   xArm 实体机器人
│   │   │   ├── uf_mock_robot/      #   仿真 Mock 机器人
│   │   ├── teleoperators/          # 遥操作器
│   │   │   ├── base_teleop/        #   共享基类
│   │   │   ├── gello_teleop/       #   GELLO (Dynamixel 示教臂)
│   │   │   ├── pika_teleop/        #   Pika Sense (手持示教器 + Vive)
│   │   │   ├── umi_teleop/         #   UMI (含双机械臂)
│   │   │   └── space_mouse/        #   SpaceMouse (3D 鼠标)
│   │   ├── cameras/                # 摄像头模块
│   │   │   └── umi_camera/         #   UMI 相机
│   │   ├── devices/                # 外部设备驱动
│   │   │   ├── pika/               #   Pika 串口驱动
│   │   │   └── umi/                #   XVLib / Vive Tracker
│   │   ├── scripts/                # 执行脚本
│   │   │   ├── uf_robot_teleop.py     # 遥操作测试
│   │   │   ├── uf_lerobot_record.py   # 数据采集
│   │   │   ├── uf_lerobot_eval.py     # 策略评估
│   │   │   ├── uf_camera_view.py      # 摄像头查看工具
│   │   │   └── vive_calibrate.py      # Vive Tracker 校准
│   │   ├── context.py              # Teleop 上下文注册
│   │   └── utils/                  # 工具函数
├── config/                         # YAML 配置文件
│   ├── gello/
│   ├── pika/
│   ├── umi/
│   └── spacemouse/
├── rules/                         # udev 设备规则
├── pyproject.toml
└── README.md
```

## 重要提示
用户需要全面研究整个代码库，并了解相关的配置参数，因为代码中所写的配置并非适用于所有使用场景和设置，所以用户需要研究代码或相关理论，以获取相关知识，并自行进行修改和调整。特别是对于扩散策略(diffusion policy)，LeRobot 中的默认参数可能仅用于模拟，并未针对实际机器人场景进行优化。


## 许可证

本项目基于 Apache License 2.0 发布，详见 [LICENSE](LICENSE) 文件。

