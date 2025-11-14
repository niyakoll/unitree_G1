# g1_viewer.py  (WORKS WITH OLD MUJOCO 2.3.x)
import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# --------------------------------------------------
# AUTO-TRY BACKENDS (OLD API: launch_passive(model, data))
# --------------------------------------------------
backends = ["glfw", "egl", "osmes:osmesa"]
model = None
data = None
viewer = None

for backend in backends:
    backend_name = backend.split(":")[-1] if ":" in backend else backend
    print(f"\nTrying MUJOCO_GL={backend_name}...")
    os.environ["MUJOCO_GL"] = backend_name

    try:
        # Load model + data
        model = mujoco.MjModel.from_xml_path("scene.xml")
        data = mujoco.MjData(model)
        data.qpos[:] = 0.0
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

        # Disable timers
        if hasattr(model.opt, "timer"):
            for i in range(15): model.opt.timer[i] = 0

        # OLD API: launch_passive(model, data)
        viewer = mujoco.viewer.launch_passive(model, data)
        if viewer is not None:
            print(f"SUCCESS: {backend_name} works!")
            break
    except Exception as e:
        print(f"  → {backend_name} failed: {e}")
        viewer = None

if viewer is None:
    raise RuntimeError("ALL BACKENDS FAILED. Try: pip install mujoco --upgrade")

# --------------------------------------------------
# MAIN LOOP
# --------------------------------------------------
print("G1 ready. Ankles will swing in 2s...")
start = time.time()
def set_joint_by_name(name, value):
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name in act_name:
            data.ctrl[i] = value
            print(f"→ {act_name} = {value:.3f}")
            return
    print(f"WARNING: Joint '{name}' not found!")


try:
    while viewer.is_running():
        t = time.time() - start

        if t > 2.0:
            set_joint_by_name("left_shoulder_pitch",  np.pi * 0.3 * np.sin(2 * np.pi * (t - 2)))
            set_joint_by_name("left_shoulder_roll",   np.pi * 0.1 * np.sin(2 * np.pi * (t - 2)))
            set_joint_by_name("right_shoulder_pitch",  np.pi * 0.3 * np.sin(2 * np.pi * (t - 5)))
            set_joint_by_name("right_shoulder_roll",  -np.pi * 0.1 * np.sin(2 * np.pi * (t - 5)))
        if t > 5.0:
            set_joint_by_name("right_wrist_pitch",  np.pi * 0.3 * np.sin(2 * np.pi * (t - 5)))
            set_joint_by_name("right_wrist_roll",  -np.pi * 0.1 * np.sin(2 * np.pi * (t - 5)))

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.0005)

finally:
    viewer.close()
    print("Viewer closed. SUCCESS!")
