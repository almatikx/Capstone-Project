# Capstone Project: IN pipe mobile robot for inspection and Maintenance

# Tasks:

### 1. Flexible Manipulator Design with Teleoperation
### 2. Simulation: Before analysis of the Capabilities, workspace calculation, performance 
### 3. Controlling the Robot from RviZ and Real Time simulating the motion of the real robot in the ROS environment
### 4. Future Adaptation to the META Quest Pro virtual World
### 5. Manipulator with Added camera and motion based on the Machine Learning. 

#### By downloading capstone_project, you can simulate the Robot in Gazebo. MoveIt control also is available with Rviz Interface.

```bash
# Simulate the Manipulator inside the Gazebo also by activating controllers for 3 Joints
cd ~/catkin_ws
source devel/setup.bash
roslaunch inpipe_simulation moveit_gazebo.launch

```
Then in another terminal:
```bash
roslaunch moveit_inpipe moveit_planning_execution.launch

```
It will run the RVIZ and you can navigate the robot's links by giving any random poses. 

