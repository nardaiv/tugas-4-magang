# Tugas 4
Solution of [Task.md](Task.md) to run drone through maze in the simulation world. This implementation uses uorb topic provided by PX4 firmware, such as ```px4_msgs/msg/trajectory_setpoint``` for guide drone to target position and ```px4_msgs/msg/vehicle_odometry``` for tracking current position. 

## Build and Run Simulation

Run [uXRCE middleware](https://docs.px4.io/main/en/middleware/uxrce_dds.html) :
```bash
micro-xrce-dds-agent udp4 -p 8888
```

Connect QGround Control :
- Windows : https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl.html#qgroundcontrol


Open PX4-AutoPilot directory, normally located in ```~/PX4-Autopilot```. Also, make sure to have place the appropriate world gazebo simulation file (with file extensition ```sdf``` ) in the PX4-AutoPilot source code, specifically in ``` Tools/simulation/gz/worlds ``` directory.
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_krti2024
```
Open this repo directory, and then build packages. Finally run the node :
```bash
colcon build
source install/setup.bash
ros2 run px4_ros_com offboard_control
```