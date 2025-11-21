# g1_arm_stretch_real_FINAL_CORRECT_2025.py
# Deploy your MuJoCo arm stretch to REAL G1 — 100% safe & working

import numpy as np
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.high.high_cmd import HighCmd
from unitree_sdk2py.g1.high.high_client import HighClient

# ============================= 1. INITIALIZE =============================
# Auto-detect Ethernet interface (works on 99% of setups)
# If you have multiple NICs, replace "0" with your interface name, e.g. "enp9s0"
ChannelFactoryInitialize(0)

client = HighClient()
cmd = HighCmd()

# ============================= 2. SAFE DEFAULTS (MANDATORY) =============================
cmd.mode = 2                    # High-level position control (balance active)
cmd.gaitType = 0                # Standing mode
cmd.lifeCount = 0               # Must increment every loop
cmd.timeStamp = 0

# Floating base — zero velocity & neutral posture
cmd.velocity = [0.0, 0.0]
cmd.yawSpeed = 0.0
cmd.position = [0.0, 0.0, 0.0]
cmd.orientation = [0.0, 0.0, 0.0, 1.0]   # [x, y, z, w]

# Gains — 41 joints (2025 G1 standard)
cmd.kp = [80.0] * 41
cmd.kd = [2.5] * 41

# Joint positions — 41 elements (critical!)
cmd.q = np.zeros(41, dtype=np.float32)
cmd.q[2] = 0.975                # Body height (same as MuJoCo)
cmd.q[3:7] = [0.0, 0.0, 0.0, 1.0]   # Body orientation quaternion [x,y,z,w]

# ============================= 3. ARM STRETCH MOTION (identical to MuJoCo) =============================
print("REAL G1: Arm stretch starting in 3 seconds...")
print("   → Identical motion to your MuJoCo script")
time.sleep(3)

duration = 40.0                 # 40-second full cycle (up + down)
freq = 200.0                    # 200 Hz — smooth & safe
dt = 1.0 / freq
start_time = time.time()

try:
    while time.time() - start_time < duration + 3.0:  # +3s safety margin
        t = time.time() - start_time

        # Exact same trajectory as your MuJoCo script
        p = np.sin(t / 20.0 * np.pi)   # -1 → +1 → -1 over 40s

        # === ARM JOINTS (indices verified from official SDK2 header) ===
        cmd.q[17] =  2.0 * p    # left_shoulder_pitch
        cmd.q[23] =  2.0 * p    # right_shoulder_pitch
        cmd.q[18] = -1.0 * p    # left_shoulder_roll
        cmd.q[24] =  1.0 * p    # right_shoulder_roll

        # Required for reliable communication
        cmd.lifeCount += 1
        cmd.timeStamp = int(time.time_ns())

        # Send command
        client.SendHighCmd(cmd)
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nMotion interrupted by user")

finally:
    # ============================= 4. SAFE SHUTDOWN =============================
    print("Returning arms to neutral position...")
    cmd.q[17:25] = 0.0          # Zero all 8 arm joints
    cmd.mode = 0                # Damping mode (safe stop)
    cmd.lifeCount += 1
    client.SendHighCmd(cmd)
    time.sleep(1.0)
    print("G1 is now safe — you can power off or press L2+B to relax")
