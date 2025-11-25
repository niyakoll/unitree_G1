import time
import mujoco
#for the simulator window(what we see)
import mujoco.viewer
#for runnning the physics simulation and showing the 3D window at the same time
from threading import Thread
import threading
#tells the Unitree communication system (DDS) which network to use and which robot we're talking to
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
#an optional "rubber band" tool for lifting the robot with the mouse.
from unitree_sdk2py_bridge import ElasticBand
#load setting from config.py
import config
#When one thread is touching the robot data, the other must wait → prevents crashes.
#prevent race condition when using threading
locker = threading.Lock()
#blueprint of the robot(such as joints,motors,sensors...etc)
mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
#he current state of the robot (where are the joints right now? what speed? what forces?)
#changes 1000 times per second during simulation
mj_data = mujoco.MjData(mj_model)
#optional setting (just like link a rubber band with the robot and pull it up for standing)
#usally turn off
#If you press keys 7/8/9 in the viewer, you can pull the robot up like with a rope (useful for getting H1/G1 to stand up).
if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    #start a seprate GUI thread with OpenGL window
    #creae a shared mjData
    #return immediately(dont block)
    #keep rendering at 60fps
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)
#Sets how fast the physics runs.
#If SIMULATE_DT = 0.002, physics updates every 2 milliseconds → 500 Hz (very realistic).
#Real Unitree robots run at 500 Hz too
mj_model.opt.timestep = config.SIMULATE_DT
#Counts how many motors (actuators) the robot has.
#G1 has 29 motors → nu = 29
num_motor_ = mj_model.nu
#Each motor gives 3 values: position, velocity, torque → 29 × 3 = 87 sensor values
dim_motor_sensor_ = 3 * num_motor_
#Small pause so the 3D window has time to open properly.
time.sleep(0.2)

#This function runs the physics simulation in the background.
def SimulationThread():
    global mj_data, mj_model

    # ============================ DOMAIN_ID ======================
    #Rule: Always use Domain ID = 1 for G1 (and H1)
    #DDS Domain ID (Data Distribution Service Domain Participant ID)
    #like a Wi-Fi network name (SSID) for the robot’s real-time control network
    #All devices that want to talk to each other must be on the same DDS domain (0–232 range).
    #If two devices are on different domain IDs → they cannot see each other, even if connected by cable.
    # ===================================================================================================
    # ============================ INTERFACE ================================================================
    #the name of your computer’s network interface (Ethernet port) that is physically connected to the robot.
    #Your computer must tell CycloneDDS:
    #“Send and listen for robot packets only on this network card.”
    #ip link show
    #INTERFACE = "lo"          # ← "lo" = loopback = your own PC
    #INTERFACE = "eth0"/"enp7s0" your real Ethernet port connected to read G1
    # ========================================================================
    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)

    # ============================ Creates the bridge ========================
    # 1. Listen for commands from your Python controller
    # 2. Send back robot state (rt/lowstate)
    # 3. Convert everything between MuJoCo and real Unitree messages
    # ========================================================================
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    #If you plug in an Xbox/PS/Switch controller, you can move the robot with it in simulation.
    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    #Prints all joint names and sensor names → very helpful when you start.
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()
    # ================================= simulator running =========================
    #Keep running as long as the 3D window is open.
    while viewer.is_running():
        #Records the exact time when this physics step begins (for accurate timing).
        #Records the exact current time in seconds (nanosecond precision)
        #for later Measuring how long the physics step + bridge code actually took
        step_start = time.perf_counter()
        #"Red light" – only one thread can touch mj_data now.
        locker.acquire()
        #If rubber band is on, apply upward pulling force.
        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        #The heart of physics!
        #This line makes the robot fall, stand, walk — everything physical happens here.
        mujoco.mj_step(mj_model, mj_data)
        #"Green light" – other thread can now read the new robot state.
        locker.release()
        #Makes sure physics runs at exactly 500 Hz (or whatever you set).
        #Very important for matching real robot timing!

        #mj_model.opt.timestep: The desired time per step
        #time_until_next_step: How much time is left before next step?
        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)#How long did the work actually take?
        #Sleep exactly that remaining time
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        # ==== Result: Each physics step takes exactly 5.000 ms → perfect 200 Hz ====
        #This code makes your simulation run in perfect real time — exactly like the real robot expects.


#This second thread only updates the 3D window at 50 FPS (smooth but not too heavy).
def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    #Starts both threads at the same time:
    #One does physics (fast)
    viewer_thread = Thread(target=PhysicsViewerThread)
    #One shows graphics (smooth)
    sim_thread = Thread(target=SimulationThread)
    #The program ends only when you close the MuJoCo window
    viewer_thread.start()
    sim_thread.start()
    