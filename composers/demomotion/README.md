# DemoMotion Docker Compose

Docker Compose setup for DemoMotion project with two ROS2 Jazzy services:
- **arm_api2_ur**
- **mediapipe**

## Build Arguments

**arm_api2_ur** service is built with all build arguments set to "yes":
- `INSTALL_GAZEBO="yes"` - Installs Gazebo Ionic simulator
- `LEROBOT_PY="yes"` - Installs LeRobot Python libraries
- `LEROBOT_ROS="yes"` - Clones SO_ARM100 workspace
- `USE_UR_ROBOT="yes"` - Clones Universal Robots ROS2 driver and description packages

## How to Use

### 1. Build both images

```bash
cd /home/toni/git/docker_files/composers/demomotion
DOCKER_BUILDKIT=1 ./docker-compose-up.sh build
```

**Note:** DOCKER_BUILDKIT=1 is required because mediapipe uses SSH mount for private repositories.

### 2. Start both containers in detached mode

```bash
./docker-compose-up.sh up -d
```

### 3. Attach to the container you want

**arm_api2_ur:**
```bash
docker exec -it ros2_armapi2_ur_cont bash
```

**mediapipe:**
```bash
docker exec -it ros2_mp_cont bash
```

### 4. Start only one service

```bash
./docker-compose-up.sh up -d arm_api2_ur
# or
./docker-compose-up.sh up -d mediapipe
```

### 5. Interactive mode for one service

```bash
./docker-compose-up.sh run --rm arm_api2_ur
# or
./docker-compose-up.sh run --rm mediapipe
```

### 6. Stop all services

```bash
./docker-compose-up.sh down
```

## Running Commands

### SO_ARM100 Robotic Arm

**Launch with mock components:**
```bash
ros2 launch so_arm100_moveit_config demo.launch.py hardware_type:=mock_components
```

**Launch with real hardware:**
```bash
ros2 launch so_arm100_moveit_config demo.launch.py hardware_type:=real usb_port:=/dev/ttyACM0
```

**Launch arm_api2 node for SO_ARM100:**
```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=so_arm100
```

### MediaPipe ROS Wrapper

**Launch mp_ros_wrapper node:**
```bash
ros2 launch mp_wrapper_ros mp_ros_wrapper.launch.py
```

**Launch usb_cam node (edit params_1.yaml according to your camera settings):**
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /root/hpe_ws/src/usb_cam/config/params_1.yaml
```

### UR Robot

**Launch arm_api2 node for UR robot:**
```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=ur use_sim_time:=true
```

**Switch arm_api2 to servo control:**
```bash
ros2 service call /arm/change_state arm_api2_msgs/srv/ChangeState "{state: SERVO_CTL}"
```

**Launch UR robot driver:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=127.0.0.1 use_mock_hardware:=true launch_rviz:=false
```

**Launch UR robot in RViz:**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

**Launch UR robot in Gazebo:**
```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

### Controller Switching

**For RViz planning mode:**

1. Switch controller:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: ['joint_trajectory_controller'], strictness: 1, activate_asap: true}"
```

2. Switch arm_api2 to trajectory mode:
```bash
ros2 service call /arm/change_state arm_api2_msgs/srv/ChangeState "{state: 'CART_TRAJ_CTL'}"
```

**For Servo/Finger control mode:**

1. Switch controller:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['joint_trajectory_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1, activate_asap: true}"
```

2. Enable non-zero velocities:
```bash
ros2 param set /joint_trajectory_controller allow_nonzero_velocity_at_trajectory_end true
```

3. Switch arm_api2 to servo mode:
```bash
ros2 service call /arm/change_state arm_api2_msgs/srv/ChangeState "{state: 'SERVO_CTL'}"
```

### Control Mode Toggling

**Topic:** `/arm/toggle_control_mode` (Bool)

**Toggle command:**
```bash
ros2 topic pub --once /arm/toggle_control_mode std_msgs/msg/Bool '{data: true}'
```

**Note:** You can also use thumb up gesture to switch between orientation mode and position mode.

**Visual Joystick Usage:**

1. Start visual_joystick:
```bash
python3 visual_joystick.py
```

2. Test POSITION mode (default):
   - Move your finger → robot translates

3. Switch to ORIENTATION mode:
```bash
ros2 topic pub --once /arm/toggle_control_mode std_msgs/msg/Bool '{data: true}'
```

4. Test ORIENTATION mode:
   - Move your finger → robot rotates end effector

5. Return to POSITION mode:
```bash
ros2 topic pub --once /arm/toggle_control_mode std_msgs/msg/Bool '{data: true}'
```

