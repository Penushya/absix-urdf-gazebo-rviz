import numpy as np
import mujoco
import mujoco.viewer
import os
import time

# -------------------------------------------------
# Load MuJoCo model
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
# LINK LENGTHS (cm)
# -------------------------------------------------
L1 = 10.0    # base → link1 (cm)
L2 = 7.0    # link1 → link2 (cm)
L3 = 5.0    # link2 → gripper (cm)

# -------------------------------------------------
# Desired end-effector position (cm)
# -------------------------------------------------
x_target = 10.0
y_target = 10.0

# Desired end-effector orientation (rad)
phi = 0.0   # gripper angle

# -------------------------------------------------
# INVERSE KINEMATICS (ANALYTICAL)
# -------------------------------------------------

# Step 1: Wrist position (cm)
x_w = x_target - L3 * np.cos(phi)
y_w = y_target - L3 * np.sin(phi)

# Step 2: Distance to wrist (cm)
r = np.sqrt(x_w**2 + y_w**2)

# Workspace check
if r > (L1 + L2):
    raise ValueError("Target is outside reachable workspace")

# Step 3: Joint 2 (elbow)
cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))  # elbow-down

# Step 4: Joint 1 (shoulder)
k1 = L1 + L2 * np.cos(theta2)
k2 = L2 * np.sin(theta2)

theta1 = np.arctan2(y_w, x_w) - np.arctan2(k2, k1)

# Step 5: Joint 3 (wrist)
theta3 = phi - theta1 - theta2

# -------------------------------------------------
# Apply joint angles to MuJoCo
# -------------------------------------------------
data.qpos[j1] = theta1
data.qpos[j2] = theta2
data.qpos[j3] = theta3

mujoco.mj_forward(model, data)

# -------------------------------------------------
# Print results
# -------------------------------------------------
print("\n===== INVERSE KINEMATICS (cm) =====")
print(f"Target position:")
print(f"x = {x_target:.2f} cm")
print(f"y = {y_target:.2f} cm")

print("\nComputed joint angles:")
print(f"θ1 = {np.rad2deg(theta1):.2f} deg")
print(f"θ2 = {np.rad2deg(theta2):.2f} deg")
print(f"θ3 = {np.rad2deg(theta3):.2f} deg")

# -------------------------------------------------
# Viewer
# -------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        viewer.sync()
        time.sleep(0.01)
