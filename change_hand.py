# Add this inside your LowCmdWrite() function (replace the old stage logic)

self.time_ += self.control_dt_
t = self.time_

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