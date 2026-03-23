Package that enables end effector velocity commands for Piper arm for ROS2 Foxy

### Important! ### 
There is a known issue where the joint_state_broadcaster publishes joint states in a random order: https://github.com/ros-controls/ros2_controllers/issues/159. Unofrtunately, the [fix](https://github.com/ros-controls/ros2_controllers/pull/1572) does not work for Foxy. Instead, make sure to launch a custom joint states publisher.

# Basic Usage
## Suggested: Use velocity control with Jacobians and pseudoinverse
1. `ros2 launch ee_velocity_controller jacobian_ee_control.launch.py`
Note that there are a multiude of paramters you can pass in, inlcuding support for position control (such as what's used with MoveIt). Run `ros2 launch ee_velocity_controller jacobian_ee_control.launch.py --show-arguments` to see them.
2. In a separate terminal, `ros2 run ee_velocity_controller keyboard_relmove --ros-args -p scale_by:=0.1` (where the scale by parameter is used to increase or decrease speed)
(Note that rotation of the gripper, i.e., dtheta, is not supported yet.)

## Alternate: Use MoveIt Servo
1. `ros2 launch ee_velocity_controller servo_ee_control.launch.py`. Wait until MoveIt servo is done loading.
2. In a separate terminal, `ros2 run ee_velocity_controller keyboard_relmove --ros-args -p scale_by:=0.1` (where the scale by parameter is used to increase or decrease speed)

Process for downloading moveit for foxy
1. create new directory
2. git clone https://github.com/moveit/moveit2.git -b $ROS_DISTRO
3. code moveit2/moveit2.repos --> change to the following:
repositories:
    moveit_resources:
        type: git
        url: https://github.com/moveit/moveit_resources.git
        version: ros2
4. for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
5. rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
6. sudo apt install ros-$ROS_DISTRO-control-toolbox \
ros-$ROS_DISTRO-moveit-resources-panda-moveit-config \
ros-$ROS_DISTRO-moveit-resources-panda-description \
ros-$ROS_DISTRO-moveit-resources-fanuc-moveit-config \
ros-$ROS_DISTRO-moveit-resources-fanuc-description \
ros-$ROS_DISTRO-ros-testing \
ros-$ROS_DISTRO-eigen-stl-containers \
ros-$ROS_DISTRO-geometric-shapes \
ros-$ROS-DISTRO-launch-param-builder
7. colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
8. source install/setup.bash
9. add install path to ~/.bashrc