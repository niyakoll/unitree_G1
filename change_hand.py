import time
import sys
sys.path.insert(0,"$HOME/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np

kPi = 3.141592654
kPi_2 = 1.57079632

class G1JointIndex:
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof

    kNotUsedJoint = 29 # NOTE: Weight

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02  
        self.duration_ = 3.0   
        self.counter_ = 0
        self.weight = 0.
        self.weight_rate = 0.2
        self.kp = 60.
        self.kd = 1.5
        self.dq = 0.
        self.tau_ff = 0.
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False

        self.target_pos = [
            0.0,      kPi_2,  0.0,    kPi_2,  0.0,
            0.0,     -kPi_2,  0.0,    kPi_2,  0.0,
            0.0,      0.0,    0.0
        ]

        self.arm_joints = [
          G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
          G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
          G1JointIndex.LeftWristRoll,
          G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
          G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
          G1JointIndex.RightWristRoll,
          G1JointIndex.WaistYaw,
          G1JointIndex.WaistRoll,
          G1JointIndex.WaistPitch
        ]

    def Init(self):
        # create publisher #
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.first_update_low_state == False:
            time.sleep(1)

        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.first_update_low_state == False:
            self.first_update_low_state = True
        
    def LowCmdWrite(self):
        self.time_ += self.control_dt_
        t = self.time_  # Local t for trajectory

        # ------------------------------------------------------------------
        # PHASE 0: Start from current pose (0–3 s) → just enable arm_sdk
        # ------------------------------------------------------------------
        if t < 3.0:
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0   # enable arm_sdk
            # keep current joint angles (smooth takeover)

        # ------------------------------------------------------------------
        # PHASE 1: Big upward stretch (arms up high, like reaching the sky) – 3→13 s
        # ------------------------------------------------------------------
        elif t < 13.0:
            ratio = (t - 3.0) / 10.0
            q = [
                -1.8,  0.0,  0.0,  1.8,  0.0,   # Left: shoulder pitch up high, no roll
                1.8,  0.0,  0.0, -1.8,  0.0,   # Right: mirrored
                0.0,  0.0,  0.0
            ]
            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].q = (1 - ratio) * self.low_state.motor_state[joint].q + ratio * q[i]

        # ------------------------------------------------------------------
        # PHASE 2: Wide open stretch (arms out to the sides, palms forward) – 13→23 s
        # ------------------------------------------------------------------
        elif t < 23.0:
            ratio = (t - 13.0) / 10.0
            q = [
                -0.2,  1.4,  0.0,  0.3,  0.0,   # Left: shoulder roll out wide, elbow slightly bent
                0.2, -1.4,  0.0, -0.3,  0.0,   # Right: mirrored
                0.0,  0.0,  0.0
            ]
            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].q = (1 - ratio) * self.low_state.motor_state[joint].q + ratio * q[i]

        # ------------------------------------------------------------------
        # PHASE 3: Forward reach + slight lean (like hugging a big beach ball) – 23→30 s
        # ------------------------------------------------------------------
        elif t < 30.0:
            ratio = (t - 23.0) / 7.0
            q = [
                -1.2,  0.4,  0.0,  1.4,  0.0,   # Left: classic “hug” pose but softer
                1.2, -0.4,  0.0, -1.4,  0.0,   # Right: mirrored
                0.0,  0.0,  0.0
            ]
            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].q = (1 - ratio) * self.low_state.motor_state[joint].q + ratio * q[i]

        # ------------------------------------------------------------------
        # PHASE 4: Slowly return to neutral – 30→35 s → then release
        # ------------------------------------------------------------------
        elif t < 35.0:
            ratio = (t - 30.0) / 5.0
            for joint in self.arm_joints:
                self.low_cmd.motor_cmd[joint].q = (1 - ratio) * self.low_state.motor_state[joint].q + ratio * 0.0

        else:
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0   # release arm_sdk
            self.done = True
  
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:        
        time.sleep(1)
        if custom.done: 
           print("Done!")
           sys.exit(-1)     