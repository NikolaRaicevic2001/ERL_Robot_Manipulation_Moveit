# Python Environment 
cd ../
python -m venv venv_Nikola_Moveit
source venv_Nikola_Moveit/bin/activate
pip install -r requirements.txt
cd ERL_Robot_Manipulation_Moveit/

# Moveit2
cd src/
source /opt/ros/humble/setup.bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
sudo apt install python3-vcstool
git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
vcs import < moveit2_tutorials/moveit2_tutorials.repos
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
sudo apt-get install ros-${ROS_DISTRO}-rviz-visual-tools
cd ../
colcon build --executor sequential
source /install/setup.bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # You may want to add this to ~/.bashrc to source it automatically

# Xarm_ros2
cd src/
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
cd xarm_ros2
git pull
git submodule sync
git submodule update --init --remote
cd ../
$ rosdep update
$ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ../
source /opt/ros/humble/setup.bash 
colcon build

# Realsense
cd src/
sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
git clone https://github.com/IntelRealSense/realsense-ros.git

# Octomap
git clone https://github.com/OctoMap/octomap.git
cd octomap
mkdir build
cd build
cmake ..
make

cd ../../
git clone https://github.com/iKrishneel/octomap_server2.git
cd octomap_server2/
vcs import . < deps.repos
