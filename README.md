# cobot_ws

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

ROS2 workspace for the myCobot 280 Pi.

### Installation
1. Clone this repository onto the cobot:
```
cd ~
```
```
git clone https://github.com/Team4-UoM-RSDP/cobot_ws
```
2. Navigate to the cloned workspace:
```
cd ~/cobot_ws
```
3. Install ROS2 dependencies:
```
rosdep update
```
```
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```
3. Build and source the project:
```
colcon build --symlink-install
```
```
source ~/cobot_ws/install/setup.bash
```
### Usage

###### Team 4 – AERO62520 Robotic System Design Project 
