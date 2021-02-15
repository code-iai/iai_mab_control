# iai_mab_control

Control components for the Model Acquisition Bot (MAB).

MAB is a bot to automatically create a 3D model of an object which was previously put on the turntable in a specific photogrammetry environment.

## Authors

- [Augsten, Andy](mailto:a.augsten@uni-bremen.de)
- [Augsten, Dustin](mailto:augsten@uni-bremen.de)

## Requirements & Dependencies

The bot is developed in Python 2 on top of [ROS Melodic](http://wiki.ros.org/melodic) using a [Universal Robots UR5](https://www.universal-robots.com/products/ur5-robot/).

To install and configure your ROS environment follow this tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

In our setup we use Ubuntu 18.04 LTS.

After creating your workspace it's convenient to also source its _setup.bash_ in _~/.bashrc_.

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

In the photogrammetry environment there's a turntable which requires the robot driver packages **iai_scanning_table** and **iai_scanning_table_msgs** of the IAI group, University Bremen, Germany. These can be found here: https://github.com/code-iai/iai_robot_drivers

They should be placed inside _~/catkin_ws/src/iai_robot_drivers/_. It is recommended to only download these two packages as the others are not needed and might require additional dependencies upon building the workspace.

To work with the Universal Robot UR5 the **Universal\_Robots\_ROS\_Driver** is required. Installation instructions can be found here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver. We recommend changing the joint limits for the UR5 MoveIt configuration in the ur5\_moveit\_config package when working on real hardware as by default the velocity and acceleration is very high. The YAML file should be located at _~/catkin_ws/src/fmauch_universal_robot/ur5_moveit_config/config/joint_limits.yaml_.

The robot is controlled using the **moveit_commander** package which can be installed using _apt_.

```
$ sudo apt install ros-melodic-moveit-commander
```

For keyboard control the **keyboard_pub** package is needed. This can be found here: https://github.com/Jayadev22/ubiquitousROS

It is to be placed inside _~/catkin_ws/src/ubiquitousROS/_.

For the bot to take photos using the Nikon Z6 II camera which is mounted on the UR5 **python-gphoto2** is required. For installation instructions follow: https://pypi.org/project/gphoto2/

We recommend the installation using _pip_. Note that the dependency **libgphoto2-devel** is already included in the Desktop-Full Install of ROS Melodic and thus might not have to be installed manually.

The Graphical User Interface uses WebSockets for communication between frontend and backend. For this the backend uses the **simple-websocket-server** which can be installed using _pip_.

```
$ pip install simple-websocket-server
```

And finally **Meshroom by AliceVision** needs to be downloaded as it is used by the application to create 3D models from the images. The most recent version can be downloaded from https://github.com/alicevision/meshroom/releases. We used version 2019.2.0. On Ubuntu the .tar.gz archive can be unpacked from the terminal using _tar_.

```
$ tar -xvzf Meshroom-2019.2.0-linux.tar.gz
```

## Installation

Once all requirements are taken care of simply clone the repository inside _~/catkin_ws/src/_ and build the workspace.

```
$ cd ~/catkin_ws/src
$ git clone -b devel https://github.com/code-iai/iai_mab_control.git
$ cd ..
$ sudo apt update
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y
$ catkin_make
```

After building the workspace make sure to source _setup.bash_ again.

```
$ source ~/catkin_ws/source.bash
```

Alternatively, start a new shell if you source it in _~/.bashrc_.

## Configuration

While most settings can be configured in the Graphical User Interface, some more general ones are fixed and have to be changed in _~/catkin_ws/src/iai_mab_control/app/settings/general.json_.

## Usage

The bot can be started using _rosrun_.

```
$ rosrun iai_mab_control acquisition.py
```

Remapping arguments might be required depending on the controllers namespace. By default the MoveIt configuration expects follow_joint_trajectory to be the controller topic but when working on the real robot we figured that the robot listened on scaled_pos_traj_controller/follow_joint_trajectory instead.

```
$ rosrun iai_mab_control acquisition.py follow_joint_trajectory:=scaled_pos_traj_controller/follow_joint_trajectory
```

The following optional parameters can be set by adding them to the _rosrun_ command.

| Parameter              | Description                                                                                                                                          | Example                                                           |
|------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| camera_mesh            | Path of the mesh to use for the camera.                                                                                                              | _camera_mesh:='~/catkin_ws/src/iai_mab_control/assets/camera.stl' |
| camera_orientation     | The camera mesh rotation around the x, y and z axes in degrees.                                                                                      | _camera_orientation:='[90, 0, 90]'                                |
| camera_pos             | Position of the attached object relative to the attachment link in meters.                                                                           | _camera_pos:='[0.0, 0.0, 0.0]'                                    |
| camera_size            | Length, width and height of the attached camera in meters. If a camera mesh is used it describes the scale factor for width, length and height.      | _camera_size:='[0.1, 0.1, 0.1]'                                   |
| distance_camera_object | The distance between the camera and the object at all positions in meters.                                                                           | _distance_camera_object:=0.2                                      |
| max_velocity           | The maximum velocity the robot arm is moving at.                                                                                                     | _max_velocity:=0.1                                                |
| num_positions          | Number of positions around the object that the robot should move to.                                                                                 | _num_positions:=15                                                |
| num_spins              | Number of spins the turntable should do for each position of the robot.                                                                              | _num_spins:=8                                                     |
| object_size            | Length, width and height of the object on the turntable in meters.                                                                                   | _object_size:='[0.2, 0.2, 0.2]'                                   |
| output_directory       | The directory to store captured photos into.                                                                                                         | _output_directory:='out'                                          |
| photobox_pos           | Center position of the photobox ground plane relative to the robot arm position in meters.                                                           | _photobox_pos:='[0.0, -0.6, 0.0]'                                 |
| photobox_size          | Width, length and height of the inner photobox in meters.                                                                                            | _photobox_size:='[0.7, 0.7, 1.0]'                                 |
| reach                  | The reach of the robot arm in meters.                                                                                                                | _reach:=0.85                                                      |
| simulation             | Defines whether the script is run in simulation or not. If _True_ the robot arm will move faster and not wait for the turntable to become available. | _simulation:=True                                                 |
| test                   | The program will run in test mode if set _True_. In test mode the turntable will not spin and no photos are taken.                                   | _test:=True                                                       |
| turntable_pos          | Center position of the turntable ground plane relative to the robot arm position in meters.                                                          | _turntable_pos:='[0.0, -0.6, 0.02]'                               |
| turntable_radius       | The radius of the turntable in meters.                                                                                                               | _turntable_radius:=0.2                                            |
| wall_thickness         | The thickness of the photobox walls in meters.                                                                                                       | _wall_thickness:=0.04                                             |

The easiest way to work with the bot during development is to use the provided launch files.

### Bringup

_bringup.launch_ launches all the required nodes for the usage with real hardware.

```
$ roslaunch iai_mab_control bringup.launch robot_ip:=IP_OF_THE_ROBOT action_ns:=MOVEIT_CONTROLLER_TOPIC
```
By default the parameter _robot\_ip_ is set to _192.168.102.99_ and the parameter _action\_ns_ is set to _scaled\_pos\_traj\_controller/follow\_joint\_trajectory_.

### Simulation

_simulation.launch_ launches all the required nodes for the usage in Gazebo.

```
$ roslaunch iai_mab_control simulation.launch
```

### Keyboard control

_keyboard_controller.launch_ launches a keyboard controller.

```
$ roslaunch iai_mab_control keyboard_controller.launch
```

If the bot is used in simulation the additional parameters _simulation_ and _command_topic_ can be passed.

```
$ roslaunch iai_mab_control keyboard_controller.launch simulation:=True command_topic:=arm_controller/command
```

## Graphical User Interface (GUI)

To use the GUI the server of the application needs to be started first.

```
$ roscd iai_mab_control
$ app/server.py
```

After starting the server any web browser can be used to access the GUI by browsing to http://MACHINE_IP_ADDRESS:8000 where _MACHINE_IP_ADDRESS_ is the ip address of the machine the server is running on. To access the GUI on the same machine as the server is running on, you can visit http://localhost:8000.

While the default ports for HTTP and WebSocket are set to 8000 and 8080, the optional arguments _--http_port_ and _--socket_port_ can be passed to change them.

```
$ roscd iai_mab_control
$ app/server.py --http_port 9000 --socket_port 9090
```

## Calibration

The photobox and turntables size and position can be determined using the calibration script. The user will be asked to attach the [calibration tool](https://github.com/code-iai/iai_mab_control/blob/devel/assets/calibration_tool.stl) to the robot arm and move the tip of the calibration tool to given [points](https://github.com/code-iai/iai_mab_control/blob/devel/assets/calibration.png). The sizes and positions are being calculated and printed by the calibration script.

The calibration script can be started using _rosrun_.

```
$ rosrun iai_mab_control calibration.py
```
