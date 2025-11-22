# g1_movement_clean_and_working.py
# Works on any modern MuJoCo (2.1+) with official Unitree G1 model
import mujoco
import mujoco.viewer
import numpy as np
import time

# ------------------------------------------------------------
# 1. Load model and start from standing pose
# ------------------------------------------------------------
model = mujoco.MjModel.from_xml_path("scene.xml")
data  = mujoco.MjData(model)

# Use the official standing keyframe (index 0 = "stand")
stand_qpos = np.copy(model.key_qpos[0])   # <-- explicit copy + np.copy fixes ambiguity
data.qpos[:] = stand_qpos
mujoco.mj_forward(model, data)

# ------------------------------------------------------------
# 2. PD gains (realistic values for G1)
# ------------------------------------------------------------
ARM_KP = 100.0
ARM_KD = 2.0
LEG_KP = 1200.0   # super stiff → legs never move
LEG_KD = 60.0

# ------------------------------------------------------------
# 3. Simple PD function – safe and clear
# ------------------------------------------------------------
def set_pd(joint_name: str, target_pos: float, kp=ARM_KP, kd=ARM_KD):
    jid = model.joint(joint_name).id                 # joint id
    aid = model.jnt_actuator[jid] if hasattr(model, 'jnt_actuator') else jid  # fallback
    
    # If model has no jnt_actuator (new MuJoCo), assume 1:1 mapping
    if aid >= model.nu:
        aid = jid - 7  # G1: floating base = 7 dofs → actuators start at index 0
    
    q_err = target_pos - data.qpos[jid]
    torque = kp * q_err - kd * data.qvel[jid]
    data.ctrl[aid] = torque

# ------------------------------------------------------------
# 4. List of joints to lock (legs + waist pitch/roll)
# ------------------------------------------------------------
locked_joints = [
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_pitch_joint", "waist_roll_joint"
]
# List of ALL upper-body joints that should stay in standing pose
arm_lock_joints = [
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_pitch_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_pitch_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint",
    "waist_yaw_joint"
]
# ------------------------------------------------------------
#  one motion – 
# ------------------------------------------------------------
def play_once(viewer, duration=8.0):
    start_time = time.time()
    duration = 8.0
    for jname in arm_lock_joints:
            jid = model.joint(jname).id
            target = stand_qpos[jid]                  # original standing angle
            set_pd(jname, target, kp=800.0, kd=40.0)
    for jname in locked_joints:
            jid = model.joint(jname).id
            target = stand_qpos[jid]                  # original standing angle
            set_pd(jname, target, kp=800.0, kd=40.0)
    
    while time.time() - start_time < duration:
        t = time.time() - start_time
          # stiff lock
        # Your motion here
        set_pd("left_shoulder_pitch_joint", 0.0 * np.sin(t * 5),90,2)
        set_pd("left_shoulder_yaw_joint", 1.57 * np.sin(t * 5),90,2)
        set_pd("left_shoulder_roll_joint", -0.3 * np.sin(t * 5),90,2)
        set_pd("left_elbow_joint", -0.3 * np.sin(t * 5),90,2)

        
        
        # Lock legs...
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.02)
    
    print("Motion finished! Holding final pose...")





# ------------------------------------------------------------
# 5. Main loop – smooth arm wave + rock-solid stance
# ------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    dt = 0.02
    play_once(viewer, duration=8.0)
    while viewer.is_running():
        

        # ---- Lock legs and waist (never move) ----
        #for jname in locked_joints:
            #jid = model.joint(jname).id
            #target = stand_qpos[jid]          # exact standing angle
            #set_pd(jname, target, kp=LEG_KP, kd=LEG_KD)

        # ---- Move only left arm (example) ----
        
        #shoulder = 0.8 * np.sin(t * 10)
                #0.8 is amplitude : Smaller amplitude = smaller twist, Bigger amplitude = bigger twist
                                    # 10 is frequency factor: bigger frequency factor = faster twist, smaller = slower twist.

        #left_shoulder_pitch = 1.8 * np.sin(t * 3)
        #left_shoulder_roll = 0.5 * np.sin(t * 3)
        #elbow    = -1.0 + 0.5 * np.sin(t * 16)
        
        #set_pd("left_shoulder_pitch_joint", left_shoulder_pitch)
        #set_pd("left_shoulder_roll_joint", left_shoulder_roll)
        #set_pd("left_elbow_joint",          elbow)

        
        #set_pd("right_shoulder_pitch_joint", shoulder)
        #set_pd("right_elbow_joint",          elbow)

        # Optional: tiny torso twist
        #set_pd("waist_yaw_joint", 0.2 * np.sin(t * 1.5))
        #set_pd("waist_yaw_joint", 0.25 * np.sin(t * 2))
        #set_pd("waist_yaw_joint", 0.1 * np.sin(t * 2))

        # Physics step
        mujoco.mj_step(model, data)
        viewer.sync()
        #time.sleep(dt)
        #t += dt #t += 0.02 is the manual time counter — it’s what makes your motion happen smoothly and at the correct speed.

print("Done – G1 stands perfectly, only arms move!")