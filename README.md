# iqr_teleop
ros joystick and keyboard control

# usage

## 1.Installation
- Create a ros workspace and clone
```shell
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/QuartzYan/iqr_teleop.git
```
- installation dependencies
```shell
cd ~/ros_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
- compile & environment setup
```shell
catkin_make
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 2.Launch
- keyboard control
```shell
roslaunch iqr_teleop keyboard_teleop.launch
```

- joystick control
```shell
roslaunch iqr_teleop joy_teleop.launch
```
