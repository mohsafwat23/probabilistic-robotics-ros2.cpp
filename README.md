# probabilistic-robots-

#### Installs ####:
#### Joint State publisher and Xacro
`sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher-gui`

#### Gazebo and CV bridge install
`sudo apt install gazebo11 libgazebo11 libgazebo11-dev`

`sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-cv-bridge`


#### Sourcing ####
`source /opt/ros/foxy/setup.bash`

`export GAZEBO_MODEL_PATH=${PWD}/install/gazebo_envs/share/gazebo_envs/models`

`source /usr/share/gazebo/setup.sh`

#### Commands ####
#### Build
`colcon build --symlink-install`

#### Source the workspace 
`source install/setup.bash`

#### Run the particle filter
`ros2 launch gazebo_envs robot.launch.py`

#### Control Robot with keyboard
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot/cmd_vel`

#### Kill gazebo server
`killall -9 gzserver`
