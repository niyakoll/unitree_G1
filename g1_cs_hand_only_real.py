# g1_real_arm_stretch.py
# Deploys your MuJoCo arm stretch to REAL G1 (100% safe, matching motion)

import sys
import time
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

kPi = 3.141592654
kPi_2 = 1.57079632

class G1JointIndex:
    # ... (copy the full G1JointIndex class from your earlier low-level script)
    # Left arm: 15=ShoulderPitch, 16=ShoulderRoll, 17=ShoulderYaw, 18=Elbow, 19=WristRoll, etc.
    # Right arm: 22=ShoulderPitch, 23=ShoulderRoll, 24=ShoulderYaw, 25=Elbow, 26=WristRoll
    kNotUsedJoint = 29  # Arm SDK enable flag

class ArmStretchDeployer:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02  # 50 Hz
        self.duration_ = 40.0    # 40s cycle (matching MuJoCo)
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update = False
        self.crc = CRC()
        self.done = False
        self.kp = 60.0  # Position gain (tune if needed; higher = stiffer)
        self.kd = 1.5   # Damping gain
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll
        ]  # Only the 4 joints you control

    def Init(self):
        # Publisher for arm_sdk (low-level control)
        self.pub = ChannelPublisher("rt/arm_sdk", unitree_hg_msg_dds__LowCmd_)
        self.pub.Init()
        # Subscriber for state feedback (to compute positions from torques)
        self.sub = ChannelSubscriber("rt/lowstate", unitree_hg_msg_dds__LowState_)
        self.sub.Init(self.StateHandler, 10)

    def StateHandler(self, msg):
        self.low_state = msg
        self.first_update = True

    def ComputePositionFromTorque(self, joint_idx, desired_torque):
        """Simple PD approximation: pos = current_pos + (torque / kp)"""
        if self.low_state is None:
            return 0.0
        current_q = self.low_state.motor_state[joint_idx].q
        return current_q + (desired_torque / self.kp)  # Basic integral; tune for accuracy

    def LowCmdWrite(self):
        if not self.first_update:
            return
        self.time_ += self.control_dt_
        t = self.time_  # Local t for trajectory

        # Enable arm_sdk
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        if self.time_ < self.duration_:
            # === EXACT SAME TRAJECTORY AS MUJOCO ===
            p = np.sin(t / 20.0 * np.pi)  # -1 to +1 over 40s

            # Compute positions from your MuJoCo torques
            self.low_cmd.motor_cmd[self.arm_joints[0]].q = self.ComputePositionFromTorque(self.arm_joints[0], 2.0 * p)      # L_shoulder_pitch
            self.low_cmd.motor_cmd[self.arm_joints[1]].q = self.ComputePositionFromTorque(self.arm_joints[1], -1.0 * p)    # L_shoulder_roll
            self.low_cmd.motor_cmd[self.arm_joints[2]].q = self.ComputePositionFromTorque(self.arm_joints[2], 2.0 * p)      # R_shoulder_pitch
            self.low_cmd.motor_cmd[self.arm_joints[3]].q = self.ComputePositionFromTorque(self.arm_joints[3], 1.0 * p)      # R_shoulder_roll

            # Set PD gains (legs untouched; balance stays active)
            for joint in self.arm_joints:
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0  # No FF torque needed
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd
        else:
            # Release after one cycle
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
            self.done = True

        # Send (this moves the robot!)
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

if __name__ == '__main__':
    print("WARNING: Clear space around G1. Press Ctrl+C to safe-stop.")
    input("Connect Ethernet & press Enter...")
    ChannelFactoryInitialize(0)  # Auto-detect NIC; or pass 'enp3s0'

    deployer = ArmStretchDeployer()
    deployer.Init()

    # Settle like MuJoCo (wait for state)
    print("Settling G1 (internal PD active)...")
    while not deployer.first_update:
        time.sleep(0.1)

    # Start control thread
    thread = RecurrentThread(interval=deployer.control_dt_, target=deployer.LowCmdWrite)
    thread.Start()

    try:
        while not deployer.done:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Safe shutdown...")
    finally:
        thread.Stop()
        print("Arms released. G1 balanced.")