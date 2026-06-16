# UFACTORY LeRobot

> [中文版本](README_ZH.md)

UFACTORY robot arm integration with the LeRobot framework for robot learning, data collection, and policy deployment.

Reference project: [ufactory_teleop](https://github.com/xArm-Developer/ufactory_teleop)

## Features

- xArm robot control (xArm series)
- Multiple teleop modes: GELLO / Pika / UMI / SpaceMouse
- Multi-camera data collection (RealSense / UMI camera)
- Dataset recording & management (LeRobot-compatible)
- Imitation learning training (ACT / Diffusion Policy / etc.)
- Policy evaluation & real-time inference
- Mock mode (no physical robot needed)

## Requirements

- Ubuntu 22.04 / 24.04
- Python >= 3.10
- CUDA >= 12.0 (recommended for GPU training)
- UFACTORY xArm (optional)

## Installation

### Base Install

```bash
git clone https://github.com/xArm-Developer/ufactory_lerobot.git
cd ufactory_lerobot

# Create conda environment
conda create -n uf_lerobot python=3.10 -y
conda activate uf_lerobot

# Install project
pip install -e .
```

Includes: `lerobot==0.4.3`, `xarm-python-sdk`, `numpy`, `pyyaml`. LeRobot already pulls in torch, opencv, wandb, etc.

### Peripheral Modules

Peripheral dependencies are available as optional extras via `[module]` install.

#### GELLO Teleop

Dynamixel-based leader arm, joint-space control.

```bash
# 1. Install GELLO module
pip install -e ".[gello]"

# 2. Add serial port permissions (re-login required)
sudo usermod -aG dialout $USER
```

#### Pika Teleop

Pika Sense handheld + Vive Tracker, task-space control.

```bash
# 1. Install peripheral deps (skip transitive deps)
pip install pysurvive agx-pypika --no-deps

# 2. Install udev rules (re-plug devices afterwards)
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Calibrate Vive Tracker before first use: `uf-vive-calibrate`

#### UMI Teleop

Universal Manipulation Interface + Vive Tracker, supports dual-arm.

```bash
# 1. Install XVSDK (system-level, Ubuntu Focal only)
sudo dpkg -i src/xvsdk/XVSDK_focal_amd64.deb
sudo apt install -y --fix-broken

# 2. Install peripheral deps
pip install pysurvive --no-deps

# 3. Install udev rules (re-plug devices afterwards)
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Calibrate Vive Tracker before first use: `uf-vive-calibrate`

**Multi-UMI device configuration** (two or more devices):

```bash
# Increase USB buffer size
sudo sed -i '/GRUB_CMDLINE_LINUX_DEFAULT/s/quiet splash/quiet splash usbcore.usbfs_memory_mb=128/' /etc/default/grub
sync
sudo update-grub
sudo reboot
```

#### SpaceMouse Teleop

3Dconnexion SpaceMouse / SpaceNavigator.

```bash
# 1. Install SpaceMouse module
pip install -e ".[spacemouse]"

# 2. Install udev rules (re-plug device afterwards)
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```


## Usage

### 1. Teleop Testing

Test teleop-to-robot control loop without recording.

```bash
uf-robot-teleop -c path/to/config.yaml
uf-robot-teleop -c path/to/config.yaml -f 60  # specify frequency
```

### 2. Data Collection

Record datasets via teleop.

```bash
uf-lerobot-record -c path/to/record_config.yaml
uf-lerobot-record -c path/to/config.yaml --resume  # resume recording
```

### 3. Policy Training

Train imitation learning policies on collected data.

```bash
lerobot-train --policy act --dataset your_dataset_name
```

### 4. Policy Evaluation

Evaluate trained policies.

```bash
uf-lerobot-eval -c path/to/eval_config.yaml
```

### 5. Camera Viewer

View and stitch multiple camera feeds.

```bash
uf-camera-view -l                           # list all cameras
uf-camera-view -l -T xvisio                 # list XVisio cameras only
uf-camera-view -T xvisio                    # view XVisio cameras (default 1280x1280 YU12)
uf-camera-view -T xvisio -W 640 -H 1920 -F NV12  # specify format
uf-camera-view -T other                     # view other camera types
```

### Mock Mode (no physical robot)

```yaml
type: "uf::mock_robot"           # single arm simulation
type: "uf::multiple_mock_robot"  # dual arm simulation
```

## Teleop Comparison

| Feature | GELLO | Pika | UMI | SpaceMouse |
|---------|-------|------|-----|------------|
| Control space | Joint space | Task space | Task space | Task space |
| Tracking | Dynamixel servos | Vive Tracker | UMI SLAM / Vive | 3D mouse |
| Dual-arm | ❌ | ❌ | ✅ | ❌ |
| System dep | dialout group | — | XVSDK deb | — |

## Project Structure

```
ufactory_lerobot/
├── src/
│   ├── ufactory_lerobot/
│   │   ├── robots/                 # Robot control
│   │   │   ├── uf_robot/           #   xArm physical robot
│   │   │   ├── uf_mock_robot/      #   Mock robot simulator
│   │   │   └── utils.py            #   make_robot_from_config patch
│   │   ├── teleoperators/          # Teleop drivers
│   │   │   ├── gello_teleop/       #   GELLO (Dynamixel leader)
│   │   │   ├── pika_teleop/        #   Pika Sense (handheld + Vive)
│   │   │   ├── umi_teleop/         #   UMI (dual-arm support)
│   │   │   ├── space_mouse/        #   SpaceMouse (3D mouse)
│   │   ├── cameras/                # Camera modules
│   │   │   └── umi_camera/         #   UMI camera
│   │   ├── devices/                # External device drivers
│   │   │   ├── pika/               #   Pika serial driver
│   │   │   └── umi/                #   XVLib / Vive Tracker
│   │   ├── scripts/                # Entry-point scripts
│   │   │   ├── uf_robot_teleop.py     # Teleop testing
│   │   │   ├── uf_lerobot_record.py   # Data recording
│   │   │   ├── uf_lerobot_eval.py     # Policy evaluation
│   │   │   ├── uf_camera_view.py      # camera viewer tool
│   │   │   └── vive_calibrate.py      # Vive Tracker calibration
│   │   └── utils/                  # Utilities
│   ├── rules/                      # udev device rules
│   └── xvsdk/                      # XVSDK system dependency
├── config/                         # YAML config files
│   ├── gello/
│   ├── pika/
│   ├── umi/
│   └── spacemouse/
├── pyproject.toml
└── LICENSE
```

## License

This project is released under the Apache License 2.0. See [LICENSE](LICENSE).
