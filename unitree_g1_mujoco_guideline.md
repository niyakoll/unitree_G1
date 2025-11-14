### Using the Unitree G1 EDU Robot Model in MuJoCo on Windows

---
#### Prerequisites:
1.GPU(RTX-4060 laptop version have been tested sucessfully)
2.Update GPU Drivers
-NVIDIA: https://www.nvidia.com/Download/
-Intel: https://www.intel.com/content/www/us/en/download-center/home.html
-AMD: https://www.amd.com/en/support
3.Install Visual C++ Redistributable
-Download: https://aka.ms/vs/17/release/vc_redist.x64.exe
-Run it

#### Native Windows – MuJoCo Menagerie (Recommended for Quick Setup)
This uses Google DeepMind's curated MJCF models for the G1 (37 DoF base; extendable for EDU hands). It's fully Windows-compatible via `pip install mujoco`. Ideal for visualization, physics testing, or Gymnasium envs.

##### Step 1: Install MuJoCo and Dependencies
- Install MuJoCo Python bindings (includes binaries; no manual download needed):
  ```cmd
  pip install mujoco
  pip install gymnasium  # For env wrappers (optional)
  ```
- If you need the viewer (OpenGL rendering), ensure you have a GPU or use `MUJOCO_GL=egl` env var for headless.
- Clone Menagerie:
  ```cmd
  git clone https://github.com/google-deepmind/mujoco_menagerie.git
  cd mujoco_menagerie
  ```

##### Step 2: Load and Visualize the G1 Model
- **Interactive Viewer** (opens a window; use mouse/WASD to interact):
  ```cmd
  python -m mujoco.viewer --mjcf unitree_g1\scene.xml
  ```
  - This loads the G1 in a flat terrain with lighting/skybox. Simulation runs at ~500 Hz.
  - Controls: Drag to rotate camera; scroll to zoom; `Space` to pause.
  - This script will swing the robot arm after 2s 

- **Programmatic Python Script** (save as `g1_viewer.py` and run with `python g1_viewer.py`):
- place the 'g1_viewer.py' inside the "unitree_g1" directory
  ```
set MUJOCO_GL=glfw && python g1_viewer.py


