import mujoco
import mujoco.viewer
import numpy as np
import os
import time

# -------------------------------------------------
# Load model
# -------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
XML_PATH = os.path.join(BASE_DIR, "scene.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

# -------------------------------------------------
# Joint addresses
# -------------------------------------------------
j1 = model.joint("robot_joint1").qposadr
j2 = model.joint("robot_joint2").qposadr
j3 = model.joint("gripper_joint").qposadr

# -------------------------------------------------
# Target angles (degrees)
# -------------------------------------------------
user_j1_deg = 75
user_j2_deg = 50
user_j3_deg = 0

q_target = np.array([
    np.deg2rad(user_j1_deg),
    np.deg2rad(user_j2_deg),
    np.deg2rad(user_j3_deg)
])

# -------------------------------------------------
# Reset simulation and start at ZERO
# -------------------------------------------------
mujoco.mj_resetData(model, data)

# Ensure all joints start at zero
data.qpos[:] = 0.0

mujoco.mj_forward(model, data)

# -------------------------------------------------
# Smooth motion parameters
# -------------------------------------------------
motion_time = 3.0   # seconds
dt = 0.01
steps = int(motion_time / dt)

q_start = np.zeros(3)

# -------------------------------------------------
# Viewer + smooth motion
# -------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    for i in range(steps):
        alpha = i / steps

        q = q_start + alpha * (q_target - q_start)

        data.qpos[j1] = q[0]
        data.qpos[j2] = q[1]
        data.qpos[j3] = q[2]

        mujoco.mj_forward(model, data)

        viewer.sync()
        time.sleep(dt)

    # Keep viewer open at final pose
    while viewer.is_running():
        viewer.sync()
        time.sleep(0.01)
