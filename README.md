# LoCO Gazebo

This package allows for simulation of LoCO using Gazebo.

## System Setup(s) Verified

- ROS: Melodic, Noetic
- Gazebo: 9.12.0-1~bionic
- Python: 3

## Package Downloads

To install these packages, use the command `sudo apt install ros-ros_version-insert_package_name`. After each new package download, run the command `rospack profile`.

- teleop-twist-keyboard
- mavros
- urdfdom-py
- joint-state-publisher-gui
- gazebo-ros
- robot-state-publisher

To install these packages, use the command `sudo apt install insert_package_name`. After each new package download, run the command `rospack profile`.

- liburdfdom-tools

## Environment Setup

Since development of the simulation resides in various packages and there is not a workspace available for download, a workspace must first be created. Instructions for this can be found at `http://wiki.ros.org/catkin/Tutorials/create_a_workspace`. It is recommended to name the workspace 'loco_ws' rather than 'catkin_ws' for keeping the workspace identifiable.

The four packages required to operate the simulation are listed below. The repositories should be cloned and located under the 'loco_ws/src/' path.

- loco_description: `https://github.com/LoCO-AUV/loco_description`
- loco_gazebo: `https://github.com/LoCO-AUV/loco_gazebo` (though you are already here if you are reading this)
- loco_pilot: `https://github.com/LoCO-AUV/loco_pilot`
- loco_teleop: `https://github.com/LoCO-AUV/loco_teleop`

Once these packages have been installed, navigate back to the 'loco_ws' folder and enter the command `catkin_make` to finish configuring the workspace.

### If Using ROS Melodic

With the launch of ROS Noetic, the appropriate compatibility changes have been made to support the use of Python 3. However, ROS Melodic defaults to using Python 2 and will not successfully run all files without a slight setup modification. This can be found at https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/, but the same instructions are given below. This must be performed in a workspace with no "build" or "devel" folder.

Install some prerequisites to use Python3 with ROS

`sudo apt update`

`sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy`

Prepare catkin workspace

`mkdir -p ~/catkin_ws/src; cd ~/catkin_ws` (this step is not required if using an existing workspace)

`catkin_make`

`source devel/setup.bash`

`wstool init`

`wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5`

`wstool up`

`rosdep install --from-paths src --ignore-src -y -r`

Finally compile for Python 3

`catkin_make --cmake-args
            -DCMAKE_BUILD_TYPE=Release
            -DPYTHON_EXECUTABLE=/usr/bin/python3
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so`

## Running the Interactive Simulation

To begin running the interactive simulation, first, create a terminal window (or tab) and enter the command `roscore` to initialize ROS. Open a new window and navigate to the 'loco_ws' folder and enter the command `catkin_make` to ensure the workspace is up to date (if `catkin_make` has not already been performed), then `source devel/setup.bash` to source the setup. Any time a new terminal window or tab is created to run something from this workspace, the `source devel/setup.bash` command should be performed first.

### Launch Simulation in Gazebo
In a new tab,
`roslaunch loco_gazebo loco_general.launch`
### Launch Simulation Control Node
In a new tab,
`rosrun loco_gazebo sim_control_node.py`
### Teleoperation
Currently, only keyboard teleoperation of the the simulation has been tested. In a new tab,
`rosrun loco_teleop teleop_keyboard.py`

Teleoperation directions are given in the terminal window and will reappear after enough messages have been output. Control is meant to resemble the left-hand joystick of a controller, where the 's' key is the middle position, and any inputs around that key provide directional commands. For example, 'w' is forward, 'e' provides a gradual turn forward and to the right, and 'd' is to turn directly forward to the right.

Press the play button in the lower left of the simulation window in Gazebo to begin the simulation. Click back onto the teleoperation terminal space and then LoCO can be operated.

### Viewing Cameras
In a new tab,
`rqt_image_view`

Choose either /left_camera/image_raw or /right_camera/image_raw from the dropdown menu. 

Camera specifications follow Blue Robotics Low-Light Cameras and can be found in `/loco-auv/loco_description/urdf`  

## Loading Different Gazebo Worlds
The default world for the LoCO simulation is a pool-like environment. To load a different world, when launching the simulation in gazebo, add the 'world_name' launch file argument to the launch command:
`roslaunch loco_gazebo loco_general.launch world_name:='insert_world_name'`

For example, to launch an empty world with LoCO:
`roslaunch loco_gazebo loco_general.launch world_name:='empty_world.world'`

The world files tested and included with the simulation can be found in the 'loco_gazebo' package under the 'worlds' folder.
