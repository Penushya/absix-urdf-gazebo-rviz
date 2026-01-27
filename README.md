# absix-urdf-gazebo-rviz
# 🤖 Forward & Inverse Kinematics - Launch Guide

## 🚀 Quick Launch Commands

### Option 1: Forward Kinematics Demo

**Step 1: Launch FK System**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 launch forward_kinematics fk_demo.launch.py
```

**Step 2: Control Robot (Open New Terminal)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 run forward_kinematics joint_commander
```

**Available Commands:**
- `home` - Move to home position
- `up` - Move arm up
- `down` - Move arm down
- `left` - Move arm left
- `right` - Move arm right
- `grip` - Close gripper
- `release` - Open gripper
- `demo` - Run automatic demo
- `custom` - Enter custom angles
- `q` - Quit

---

### Option 2: Inverse Kinematics Demo

**Step 1: Launch IK System (Interactive terminal opens automatically)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 launch forward_kinematics ik_demo.launch.py
```

**Step 2: Use Interactive IK Terminal**

The interactive terminal opens automatically. Type these commands:
- `demo` - Run full demo sequence
- `p1` - Move to point 1 (0.15, 0.10, 0.0)
- `p2` - Move to point 2 (0.10, 0.15, 0.0)
- `p3` - Move to point 3 (-0.10, 0.10, 0.0)
- `p4` - Move to point 4 (0.20, 0.0, 0.0)
- `0.15 0.10` - Move to custom position (x y)
- `q` - Quit

---

### Option 3: Manual Step-by-Step Launch

**Terminal 1: Launch Gazebo**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 launch manipulator_description gazebo.launch.py
```

**Terminal 2: Launch Forward Kinematics Node**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 run forward_kinematics fk_node
```

**Terminal 3: Launch Inverse Kinematics Node**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 run forward_kinematics ik_node
```

**Terminal 4: Launch Workspace Checker**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 run forward_kinematics workspace_checker
```

**Terminal 5: Interactive Control**
```bash
source /opt/ros/jazzy/setup.bash
source ~/absix_ws/install/setup.bash
ros2 run forward_kinematics interactive_ik
```

---

## 📝 Notes

### Before Running
Make sure you have built the workspace:
```bash
cd ~/absix_ws
colcon build --symlink-install
source install/setup.bash
```

### Auto-Source Setup (Optional)
Add to `~/.bashrc` to avoid typing source commands every time:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/absix_ws/install/setup.bash" >> ~/.bashrc
```

After adding to `~/.bashrc`, restart terminal or run:
```bash
source ~/.bashrc
```

Then you can launch directly:
```bash
ros2 launch forward_kinematics fk_demo.launch.py
```

---

## 🎯 What Launches

### FK Demo Includes:
- ✅ Gazebo simulation
- ✅ RViz visualization
- ✅ Forward kinematics node
- ✅ Workspace checker
- ✅ Robot controllers

### IK Demo Includes:
- ✅ Gazebo simulation
- ✅ RViz visualization
- ✅ Forward kinematics node
- ✅ Inverse kinematics node
- ✅ Workspace checker
- ✅ Interactive IK interface (auto-opens)
- ✅ Robot controllers

---

## 🔍 Verify Launch

After launching, check if everything is running:

```bash
# Check nodes (in new terminal)
ros2 node list

# Check controllers
ros2 control list_controllers

# Should show:
# joint_state_broadcaster [active]
# joint_trajectory_controller [active]
```

---

## 🐛 Troubleshooting

### Controllers Not Active
```bash
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state joint_trajectory_controller active
```

### Rebuild After Changes
```bash
cd ~/absix_ws
colcon build --packages-select forward_kinematics --symlink-install
source install/setup.bash
```

### Kill All Processes
```bash
pkill -9 -f ros2
pkill -9 -f gazebo
pkill -9 ruby
```

---

**Quick Start:** Run Option 2 (IK Demo) for the full experience! 🚀
