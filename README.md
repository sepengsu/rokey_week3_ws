### 3주차 프로젝트 
패키지 출처 
두산 로보틱스: https://github.com/doosan-robotics/doosan-robot2
    
#### 1. 설치 가이드 
필요 패키지 설치 
```
### Prerequisite installation elements before package installation
sudo apt-get update
sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget
sudo apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group

### install gazebo sim
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y libignition-gazebo6-dev
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs ros-humble-ros-gz-sim ros-humble-ros-gz
```

필수적인 패키지 clone 
```
cd src  
https://github.com/Juwan-s/doosan-robot2
git clone -b humble https://github.com/ros-controls/gz_ros2_control

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/rokey_week3_ws/colcon build
```
