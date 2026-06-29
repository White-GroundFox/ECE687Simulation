# ROS 2 Package Setup and Usage

## 1. Create a New Package

Navigate to the ROS workspace source directory:

```bash
cd /ros_ws/src
```

Create a new ROS 2 Python package:

```bash
ros2 pkg create --build-type ament_python <your_package_name> \
--dependencies rclpy geometry_msgs sensor_msgs robomaster_msgs
```

---

## 2. Create the Python Nodes

Navigate into your package:

```bash
cd <your_package_name>/<your_package_name>
```

Create the following files:

```bash
touch stick_grabber_node.py
touch move_robot_node.py
```

Copy the provided code into the respective files.

---

## 3. Make the Scripts Executable

Run:

```bash
chmod +x stick_grabber_node.py
chmod +x move_robot_node.py
```

---

## 4. Modify `setup.py`

Navigate to your package root:

```bash
cd /root/ros2_ws/src/<your_package_name>
```

Update the `entry_points` section in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'stick_grabber_node = your_package_name.stick_grabber_node:main',
        'move_robot_node = your_package_name.move_robot_node:main',
    ],
},
```

> **Note:** Replace `your_package_name` with the actual package name.

---

## 5. Build the Package

Navigate to the workspace:

```bash
cd /root/ros2_ws
```

Build the package:

```bash
colcon build --packages-select <your_package_name> --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

---

## 6. Run a Node

Run one of the nodes:

```bash
ros2 run <your_package_name> stick_grabber_node
```

or

```bash
ros2 run <your_package_name> move_robot_node
```

> **Tip:** You can temporarily remove one of the scripts from the `entry_points` list for isolated testing.

---

# Robot Control Commands

## Manipulator Controls

### Move Forward

```bash
ros2 topic pub /robot3/cmd_arm geometry_msgs/msg/Vector3 \
"{x: 0.5, y: 0.0, z: 0.0}" --once
```

### Move Backward

```bash
ros2 topic pub /robot3/cmd_arm geometry_msgs/msg/Vector3 \
"{x: -0.5, y: 0.0, z: 0.0}" --once
```

### Move Up

```bash
ros2 topic pub /robot3/cmd_arm geometry_msgs/msg/Vector3 \
"{x: 0.0, y: 0.0, z: 0.5}" --once
```

### Move Down

```bash
ros2 topic pub /robot3/cmd_arm geometry_msgs/msg/Vector3 \
"{x: 0.0, y: 0.0, z: -0.5}" --once
```

---

## Gripper Controls

### Open Gripper

```bash
ros2 action send_goal /robot2/gripper robomaster_msgs/action/GripperControl \
"{target_state: 1, power: 0.5}"
```

### Close Gripper

```bash
ros2 action send_goal /robot2/gripper robomaster_msgs/action/GripperControl \
"{target_state: 2, power: 0.5}"
```

---

## Robot Movement

### Rotate

```bash
ros2 topic pub /robot3/cmd_vel geometry_msgs/msg/Twist \
"{angular: {z: 2}}" --once
```

### Move Forward / Backward

```bash
ros2 topic pub /robot3/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 2, y: 2}}" --once
```

---

# Quick Workflow

```bash
cd /ros_ws/src

ros2 pkg create --build-type ament_python <your_package_name> \
--dependencies rclpy geometry_msgs sensor_msgs robomaster_msgs

cd <your_package_name>/<your_package_name>

touch stick_grabber_node.py
touch move_robot_node.py

chmod +x stick_grabber_node.py
chmod +x move_robot_node.py

cd /root/ros2_ws

colcon build --packages-select <your_package_name> --symlink-install

source install/setup.bash

ros2 run <your_package_name> stick_grabber_node
```
