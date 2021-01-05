# iai_mab_control

Control components for the Model Acquisition Bot (MAB).

MAB is a bot to automatically create a 3D model of an object which was previously put on the turntable in a specific photogrammetry environment.

## Authors

- [Andy Augsten](mailto:a.augsten@uni-bremen.de)
- [Dustin Augsten](mailto:augsten@uni-bremen.de)

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

To work with the Universal Robot UR5 the **ROS-Industrial Universal Robot meta-package** is required. Installation instructions can be found here: https://github.com/ros-industrial/universal_robot

The robot is controlled using the **moveit_commander** package which can be installed using apt.

```
$ sudo apt install ros-melodic-moveit-commander
```

For keyboard control the **keyboard_pub** package is needed. This can be found here: https://github.com/Jayadev22/ubiquitousROS

It is to be placed inside _~/catkin_ws/src/ubiquitousROS/_.

For the bot to take photos using the Nikon Z6 II camera which is mounted on the UR5 **python-gphoto2** is required. For installation instructions follow: https://pypi.org/project/gphoto2/

We recommend the installation using _pip_. Note that the dependency **libgphoto2-devel** is already included in the Desktop-Full Install of ROS Melodic and thus might not have to be installed manually.

## Installation

Once all requirements are taken care of simply clone the repository inside _~/catkin_ws/src/_ and build the workspace.

```
$ cd ~/catkin_ws/src
$ git clone -b devel https://github.com/code-iai/iai_mab_control.git
$ cd ..
$ catkin_make
```

After building the workspace make sure to source _setup.bash_ again.

```
$ source ~/catkin_ws/source.bash
```

Alternatively, start a new shell if you source it in _~/.bashrc_.

## Usage

The bot can be started using _rosrun_.

```
$ rosrun iai_mab_control acquisition.py
```

The following optional parameters can be set by adding them to the _rosrun_ command.

| Parameter               | Description                                                                                                                                        | Example                           |
|-------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------|
| _camera_size            | Width, length and height of the attached camera.                                                                                                   | _camera_size:='[0.1, 0.1, 0.1]'   |
| _distance_camera_object | The distance between the camera and the object at all positions.                                                                                   | _distance_camera_object:=0.2      |
| _num_positions          | Number of positions around the object that the robot should move to.                                                                               | _num_positions:=15                |
| _num_spins              | Number of spins the turntable should do for each position of the robot.                                                                            | _num_spins:=8                     |
| _object_size            | Width, length and height of the object on the turntable.                                                                                           | _object_size:='[0.2, 0.2, 0.2]'   |
| _photobox_pos           | Center position of the photobox ground plane.                                                                                                      | _photobox_pos:='[0.0, -0.7, 0.0]' |
| _photobox_size          | Width, length and height of the photobox.                                                                                                          | _photobox_size:='[1.0, 1.0, 1.0]' |
| _simulation             | Defines whether the script is run in simulation or not. If True the robot arm will move faster and not wait for the turntable to become available. | _simulation:=True                 |
| _output_directory       | The directory to store captured photos into.                                                                                                       | _output_directory:='out'          |

The easiest way to work with the bot during development is to use the provided launch files.

### Simulation

_simulation.launch_ launches a simulation in Gazebo.

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
