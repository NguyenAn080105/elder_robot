## 📦 Requirements
- Ubuntu 18.04 LTS  
- ROS 2 Dashing Diademata  
- Gazebo (cài theo ROS 2 Dashing)  
- RViz2  
- Colcon build tool
---
## Installation
1. Create a ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
2. Clone into the src folder:
   ```bash
   git clone https://github.com/NguyenAn080105/elderly-care-robot-ros-dashing.git
3. Build the workspace:
  ```bash
  cd ~/ros2_ws
  colcon build --packages-select elder_robot
  ```
4. Source the workspace:
  ```bash
  source install/setup.bash
  ```
---
##▶️ Run Simulation
1. Launch the robot in Gazebo:
  ```bash
  ros2 launch elder_robot elder_robot.launch.py
  ```
3. Open RViz2 to visualize the robot:
  ```bash
  rviz2
  ```
