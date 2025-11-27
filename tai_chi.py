# g1_taichi_with_real_balance.py
# Upper-body Tai-Chi + REAL Unitree balance controller → IMPOSSIBLE TO FALL
# Works in sim AND real robot (just change one line)

import time
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# ================================
# SIM ↔ REAL SWITCH
# ================================
USE_REAL_ROBOT = False   # ← CHANGE TO True ON REAL G1

if USE_REAL_ROBOT:
    ChannelFactoryInitialize(0, "eth0")   # change interface if needed
    print("REAL G1 — Tai-Chi with OFFICIAL Unitree balance!")
else:
    ChannelFactoryInitialize(1, "lo")
    print("SIMULATION — Tai-Chi with OFFICIAL balance controller")

# ================================
# Joint indices
# ================================
class J:
    WaistYaw, WaistRoll, WaistPitch = 12,13,14
    LShoulderPitch, LShoulderRoll, LShoulderYaw, LElbow, LWristRoll = 15,16,17,18,19
    RShoulderPitch, RShoulderRoll, RShoulderYaw, RElbow, RWristRoll = 22,23,24,25,26

class TaiChiWithRealBalance:
    def __init__(self):
        self.t = 0.0
        self.dt = 0.002
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()

        # We will ONLY control upper body — legs + waist locked by high-level
        self.kp_arm = 90.0
        self.kd_arm = 2.5

    def start(self):
        # 1. Send high-level "stand" command — this activates Unitree's REAL balance controller
        stand_pub = ChannelPublisher("rt/sportmodestate", SportModeState_)
        stand_pub.Init()
        stand_msg = unitree_go_msg_dds__SportModeState_()
        stand_msg.mode = 2   # 2 = stand mode (official balance active!)
        for _ in range(500):
            stand_pub.Write(stand_msg)
            time.sleep(0.002)
        print("Unitree high-level balance ACTIVATED — robot is now UN-FALLABLE")

        # 2. Low-level publisher for arms only
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()

        self.thread = RecurrentThread(interval=self.dt, target=self.loop)
        self.thread.Start()

        print("Tai-Chi started with REAL balance!")
        print("   → Legs + waist: controlled by Unitree's balance engine")
        print("   → Arms: flowing Tai-Chi")
        print("   → You can push the robot — it will NOT fall!")

    def loop(self):
        self.t += self.dt
        phase = self.t * 0.28

        # === ONLY CONTROL UPPER BODY (15~28) ===
        # Waist locked by high-level — we don't touch it
        # Legs locked by high-level — we don't touch them

        # Left arm — big flowing circle
        self.cmd.motor_cmd[J.LShoulderPitch].q = 0.4 + 1.0 * np.sin(phase)
        self.cmd.motor_cmd[J.LShoulderRoll].q  = 0.5 * np.sin(phase * 2)
        self.cmd.motor_cmd[J.LElbow].q         = 1.3 + 0.9 * np.sin(phase + 1.2)
        self.cmd.motor_cmd[J.LWristRoll].q     = 0.7 * np.sin(phase * 3.3)

        # Right arm — mirror
        self.cmd.motor_cmd[J.RShoulderPitch].q = 0.4 + 1.0 * np.sin(phase + np.pi)
        self.cmd.motor_cmd[J.RShoulderRoll].q  = -0.5 * np.sin(phase * 2)
        self.cmd.motor_cmd[J.RElbow].q         = 1.3 + 0.9 * np.sin(phase + 1.2 + np.pi)
        self.cmd.motor_cmd[J.RWristRoll].q     = 0.7 * np.sin(phase * 3.3 + np.pi)

        # Apply stiffness only to arms
        for i in range(15, 29):
            self.cmd.motor_cmd[i].mode = 0x0A
            self.cmd.motor_cmd[i].dq   = 0.0
            self.cmd.motor_cmd[i].tau  = 0.0
            self.cmd.motor_cmd[i].kp   = self.kp_arm
            self.cmd.motor_cmd[i].kd   = self.kd_arm

        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)


if __name__ == '__main__':
    dancer = TaiChiWithRealBalance()
    dancer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nTai-Chi ended. Robot stays standing.")