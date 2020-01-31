# Robotics-Final-Project and Assignment

## Solutions:
- Solution to Assignment - Model Design: robotics-master
- Solution to Project: robotics

## Group members:
1. Daniel Zelalem     Atr/2026/08
2. Elias Andualem     Atr/9391/08
3. Nabil seid         Atr/5725/08
4. Zelalem Getahun    Atr/9374/08
5. Eyob yirgu         Atr/9987/08
6. Abel Tilahun       Atr/0600/08

## Steps to run robotics-master
1. The project uses ROS Melodic running on Ubuntu 18.04.
2. Verify the version of gazebo installed with ROS.
3. Create a catkin workspace if haven't already.
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_init_workspace
$ ls -l
```
4. Clone or download project repository into the src directory of the catkin workspac
```
cd ~/catkin_ws/src
$ git clone https://github.com/eandualem/Robotics-Final-Project
```
5. Install missing dependencies if any.
```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
```
6. Change the permissions of script files to turn them executable.
```
$ cd ~/catkin_ws/src/Robotics-Final-Project/robotics-master/src/robot_gazebo/scripts/
$ sudo chmod u+x gripper_test.py
$ sudo chmod u+x gripper_control.py
$ sudo chmod u+x constants.py
```
7. Build the project.
```
$ cd ~/catkin_ws
$ catkin_make
```
8. Open .bashrc file found in the home directory and add the following commands at the end
```
source ~/catkin_ws/devel/setup.bash
```
9. Save the .bashrc file and open a new terminal for changes to take effect.
10. Launch project.
```
roslaunch robotics-master 
```
  



