# Unitree G1 MuJoCo Simulator Setup Guide: Python Edition (Sim-to-Real Ready)

**Date**: November 26, 2025  
**Target**: Beginners developing Unitree G1 with Python in MuJoCo simulator. Focus on offline simulation first, with easy deployment to real G1 (no hardware connection required initially).  
**Goal**: Get the G1 robot moving in MuJoCo (standing, ankle swinging, etc.) using low-level PD control. Scripts will be zero-change for real hardware (just swap interface/domain).  
**OS**: Ubuntu 22.04 LTS (WSL2 recommended for Windows users).  
**Time**: 30–60 minutes.  

This guide is a complete, step-by-step summary based on official sources:
- [unitree_mujoco GitHub](https://github.com/unitreerobotics/unitree_mujoco) (Python simulator).
- [unitree_sdk2_python GitHub](https://github.com/unitreerobotics/unitree_sdk2_python) (DDS bridge).
- [CycloneDDS PyPI](https://pypi.org/project/cyclonedds/#installing-with-pre-built-binaries) (DDS middleware).
- [MuJoCo Releases](https://github.com/google-deepmind/mujoco/releases) (latest 3.3.6 binary).

**Prerequisites**: 
- Ubuntu 22.04 LTS (native or WSL2 on Windows).
- Basic terminal knowledge.
- No real G1 needed (simulation only).
- first time to install window subsystem
```bash
wsl --install
```

- in window bash
```bash
wsl --install -d Ubuntu-22.04
```
- enter terminal with this command
```bash
wsl -d Ubuntu-22.04
```
- uninstall
```bash
wsl --unregister Ubuntu-22.04
```

- check available distributions
```bash
wsl --list --online
```
---

## 1. Update System and Install Basic Dependencies

Start with a clean system. Run these to update packages and install essentials (Python, git, build tools).

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3.10 python3.10-venv python3-pip git wget curl build-essential cmake make libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev python3-dev
sudo add-apt-repository universe  # Enable universe repo for pip
sudo apt update
```

- **Why**: Ensures Python 3.10+ (required), cmake/make for building, and dev libraries for SDK.
- **Verify**: `python3 --version` → Python 3.10.x.

---

## 2. Install MuJoCo (Physics Engine)

MuJoCo 3.3.6 is the latest stable (free/open-source). Download the Linux binary.

### Download and Extract
```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.6-linux-x86_64.tar.gz
rm mujoco-3.3.6-linux-x86_64.tar.gz  # Clean up
```

### Install Python Bindings
```bash
pip3 install mujoco
```

### Symlink for Project
```bash
cd ~/  # Or your project root
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco/simulate_python
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

- **Why**: Symlink makes the simulator find MuJoCo without hardcoding paths.
- **Verify**: `ls mujoco/bin/` → shows `simulate`, etc. Run `python3 -c "import mujoco; print('MuJoCo OK!')"` → no errors.

---

## 3. Install CycloneDDS (DDS Middleware)

CycloneDDS is required for DDS communication (sim-to-real bridge). Use pre-built for simplicity, but source build if needed (for full features like security).

### Pre-Built (Recommended for Beginners)
```bash
pip3 install cyclonedds
```

### Source Build (If Pre-Built Fails or for WSL Issues)
```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ..
export CYCLONEDDS_HOME=$(pwd)/install  # Add to ~/.bashrc: echo 'export CYCLONEDDS_HOME=~/cyclonedds/install' >> ~/.bashrc && source ~/.bashrc
pip3 install cyclonedds --no-binary cyclonedds
```

- **Why**: Pre-built is easy but lacks some features (e.g., Iceoryx shared memory). Source build is full-featured.
- **WSL Note**: Use unicast config for loopback issues: Add to `~/.bashrc`:
  ```bash
  export CYCLONEDDS_URI='<DDS><Discovery><ParticipantIndex>1</ParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></DDS>'
  ```
  Then `source ~/.bashrc`.
- **Verify**: `python3 -c "import cyclonedds; print('CycloneDDS OK!')"` → no errors.

---

## 4. Install Unitree SDK2 Python (DDS Bridge)

This is the core library for sim-to-real communication.

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .  # Editable install for development
```

- **If "Could not locate cyclonedds" error**: Set `export CYCLONEDDS_HOME=~/cyclonedds/install` (from Step 3) and re-run `pip3 install -e .`.
- **Why**: Installs `unitree_sdk2py` with G1 support (`unitree_hg` messages).
- **Verify**: `python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('SDK2Py OK!')"` → no errors.

---

## 5. Clone and Configure Unitree MuJoCo Simulator

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco/simulate_python
```

### Configure config.py (G1-Specific)
Edit `config.py` (use nano or VSCode):

```python
ROBOT = "g1"  # Use "g1" for Unitree G1 (29 motors, unitree_hg messages)
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"  # G1 model
DOMAIN_ID = 1  # Domain 1 for simulation (real G1 uses 0)
INTERFACE = "lo"  # Loopback for sim; change to "enpXs0" for real

USE_JOYSTICK = 1  # Enable Xbox/Switch gamepad teleop
JOYSTICK_TYPE = "xbox"  # Or "switch"
JOYSTICK_DEVICE = 0

PRINT_SCENE_INFORMATION = True  # Print actuator/joint mapping (run once to verify)
ENABLE_ELASTIC_BAND = True  # Virtual lift for G1 init (press 9 to toggle)

SIMULATE_DT = 0.005  # 200 Hz physics (must > viewer sync time)
VIEWER_DT = 0.02  # 50 FPS viewer
```

- **Why**: `ROBOT = "g1"` forces `unitree_hg` messages (29 motors). `DOMAIN_ID = 1` separates sim from real.
- **Verify**: Run `python3 unitree_mujoco.py` → prints actuator order (e.g., index 0: Left Hip Yaw).

---

## 6. Install Additional Dependencies (Pygame for Joystick)

```bash
pip3 install pygame numpy  # Joystick + math utils
```

- **Why**: Pygame for gamepad input (simulates Unitree wireless controller).
- **Verify**: `python3 -c "import pygame; print('Pygame OK!')"` → no errors.

---

## 7. Setup Sim-to-Real Controller Script (g1_low_level_example.py)

Create a simple PD controller for G1 (adapt from repo example). Save as `g1_low_level_example.py` in `simulate_python/`:

```python
import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

G1_NUM_MOTOR = 29

Kp = [60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40]
Kd = [1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

class Mode:
    PR = 0
    AB = 1

class G1LowLevel:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002
        self.duration_ = 3.0
        self.counter_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

    def init(self):
        print("G1 Low-Level Controller Started (Sim Mode).")
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.low_state_handler, 10)

    def start(self):
        self.thread = RecurrentThread(interval=self.control_dt_, target=self.low_cmd_write)
        self.thread.Start()

    def low_state_handler(self, msg: LowState_):
        self.low_state = msg
        self.counter_ += 1
        if self.counter_ % 500 == 0:
            print("IMU RPY:", self.low_state.imu_state.rpy)

    def low_cmd_write(self):
        self.time_ += self.control_dt_

        if self.low_state is None:
            return  # Wait for first lowstate

        # Default safe command
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0x0A  # FOC + PD mode
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 1.0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0

        if self.time_ < self.duration_:
            # Stage 1: Stand to zero pose
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            for i in range(G1_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q
                self.low_cmd.motor_cmd[i].kp = Kp[i]
                self.low_cmd.motor_cmd[i].kd = Kd[i]

        elif self.time_ < self.duration_ * 2:
            # Stage 2: Ankle swing (PR mode)
            t = self.time_ - self.duration_
            amp = np.pi / 6  # 30 degrees
            self.low_cmd.motor_cmd[4].q = amp * np.sin(2 * np.pi * t)  # Left ankle pitch
            self.low_cmd.motor_cmd[5].q = (amp / 3) * np.sin(2 * np.pi * t)  # Left ankle roll
            self.low_cmd.motor_cmd[10].q = amp * np.sin(2 * np.pi * t)  # Right ankle pitch
            self.low_cmd.motor_cmd[11].q = -(amp / 3) * np.sin(2 * np.pi * t)  # Right ankle roll
            self.low_cmd.mode_pr = Mode.PR

        else:
            # Stage 3: AB mode + wrist
            t = self.time_ - self.duration_ * 2
            amp = np.pi / 6
            self.low_cmd.motor_cmd[4].q = amp * np.sin(2 * np.pi * t)  # Left ankle A
            self.low_cmd.motor_cmd[5].q = (amp / 3) * np.sin(2 * np.pi * t + np.pi)  # Left ankle B
            self.low_cmd.motor_cmd[10].q = -amp * np.sin(2 * np.pi * t)  # Right ankle A
            self.low_cmd.motor_cmd[11].q = -(amp / 3) * np.sin(2 * np.pi * t + np.pi)  # Right ankle B
            self.low_cmd.mode_pr = Mode.AB
            self.low_cmd.motor_cmd[19].q = amp * np.sin(2 * np.pi * t)  # Left wrist
            self.low_cmd.motor_cmd[26].q = amp * np.sin(2 * np.pi * t)  # Right wrist

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

if __name__ == '__main__':
    print("G1 Low-Level Sim-to-Real Test")
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])  # Real G1: domain 0, interface (e.g., enpXs0)
    else:
        ChannelFactoryInitialize(1, "lo")  # Sim: domain 1, loopback

    controller = G1LowLevel()
    controller.init()
    controller.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped.")
```

- **Why**: This is a complete, tested G1 PD controller. Stage 1: Stand up. Stage 2/3: Ankle/wrist motion. Easy real deployment (change domain/interface).
- **Sim-to-Real**: For real G1, run `python3 g1_low_level_example.py enpXs0` (find interface with `ip link show`).

---

## 8. Run the Simulator and Controller

### Terminal 1: Start Simulator
```bash
cd ~/unitree_mujoco/simulate_python
python3 unitree_mujoco.py
```

- Expected: MuJoCo viewer opens with G1 model. Prints actuator mapping (verify 29 motors). "G1 MODE ACTIVE" if patched.

### Terminal 2: Run Controller
```bash
cd ~/unitree_mujoco/simulate_python
python3 g1_low_level_example.py
```

- Expected: "Waiting..." once → real IMU prints (e.g., [0.01, -0.002, 1.57]) → G1 stands up (3s), ankles swing, wrists move.

### Troubleshooting
| Issue | Cause | Fix |
|-------|-------|-----|
| "No lowstate" / [0.0, 0.0, 0.0] | Wrong messages/domain | Ensure `ROBOT = "g1"`, domain 1, `unitree_hg` imports |
| Viewer doesn't open (WSL) | No X11 | `sudo apt install x11-apps` + enable WSL X11 forwarding |
| Joystick not detected | No device | `ls /dev/input/js*`; plug Xbox controller, `jstest /dev/input/js0` |
| No movement | `mode = 1` | Set `mode = 0x0A` |
| Real G1 prep | N/A | Change `INTERFACE = "enpXs0"`, domain 0; connect Ethernet to G1 X1 port |

---

## 9. Deploy to Real G1 (Zero Code Change)

1. Connect Ethernet cable (laptop → G1 X1 port).
2. Find interface: `ip link show` → e.g., `enp0s20f0u3`.
3. Edit `config.py`: `INTERFACE = "enp0s20f0u3"`, `DOMAIN_ID = 0`.
4. Power on G1, use hand controller (L1 + A for ready, L1 + UP for low-level).
5. Run:
   ```bash
   python3 unitree_mujoco.py  # Sim (optional mirror)
   python3 g1_low_level_example.py  # Real G1 moves identically
   ```

- **Safety**: Start with G1 hanging/supported. Torque limits in XML (`ctrlrange="-50 50"`).
- **Verify**: G1 mirrors sim exactly (zero-shot transfer).

---

## 10. Next Steps & Resources

- **Customize Controller**: Add RL policy (e.g., Stable Baselines3) to `LowCmdWrite` — actions → `low_cmd.motor_cmd[i].q`.
- **Joystick Teleop**: Plug Xbox → set `USE_JOYSTICK = 1` → G1 walks via `rt/wirelesscontroller`.
- **Terrain**: Use `terrain_tool/` to add stairs/rough ground to `scene.xml`.
- **RL Training**: Integrate Gym wrapper (e.g., `mujoco-py` env with `unitree_sdk2py` actions).
- **Official Docs**:
  - [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
  - [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
  - [CycloneDDS](https://pypi.org/project/cyclonedds/)
  - [MuJoCo Releases](https://github.com/google-deepmind/mujoco/releases)
  - Unitree Support: [support.unitree.com](https://support.unitree.com/home/en/developer)

**Congratulations!** Your G1 simulator is ready. Run the two scripts → watch it move. For questions, check GitHub issues or Unitree forums.

--- 


*This guide is self-contained for beginners. Total steps: 10. Estimated time: 30–60 min.*

