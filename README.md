# iai_mab_control
Control components for the Model Acquisition Bot (MAB)

####################################################################################################################################################
######################################################UR5 Robot Controller Documents ###############################################################
################################ PLEASE BE CLOSE TO THE EMERGENCY SWITCH OF THE ROBOT TOUCH PANEL###################################################
####################################################################################################################################################

1/ start the robot and from the pannel choose RUN PROGRAM > on top FILE and choose the 3rd file(ros_control.urp). select the file and from the bottom just open it.
Then do nothing. It will pop up another screen where at the bottom start button(just symbol) will be appeared. No need to press it now. Without running roslaunch file\
it will not work.
2/ Keep in the mind that in robot pannel IP address would be host pc's IP address
3/ open a terminal and source workspace. 

------------------------------------------------------ UR5 -----------------------------------------------------------------
** How to Connect the UR5 to the PC
	roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.102.99

** GUI to controll the UR5 
	rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

------------------------------------------------------ TURN TABLE ----------------------------------------------------------
** run the following commands from another two terminal
	rosrun iai_scanning_table st_action_server.py 

------------------------------------------------------ Camera Trigger ------------------------------------------------------
** Python Server trigger the RGB camera and save the picture to actuall workspace. 
** The Client that trigger the Action will be implemented in to the Controller. 
	rosrun spinnaker_code spinnaker_server.py

** Client for direkt trigger.
	rosrun spinnaker_code client.py 

------------------------------------------------------ Xtion ---------------------------------------------------------------
** Start RGB-D Camera.
	roslaunch openni2_launch openni2.launch
** Start Robosherlock pipeline. 
	rosrun robosherlock run _ae:=save_images _vis:=true

------------------------------------------------------ Controller ----------------------------------------------------------
** Controlls the ur5 and the turntable at the same time. The script bring the arm to a recording pose.
** The turntable get triggert and at the end the arm goes to the home position.
** run the following command in the catkin_ws: 
	rosrun iai_mab_control Controller.py 

------------------------------------------------------ Spinnaker -----------------------------------------------------------
** Software for testing the Camera and change Parameters. 
	spinview

------------------------------------------------------ Oracle --------------------------------------------------------------
** Script for the Robot Arm Inverse kinematics
	rosrun iai_mab_control Oracle.py

-------------------------------------------------------Dependencies---------------------------------------------------------
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

https://github.com/code-iai/iai_robot_drivers.git

https://github.com/RoboSherlock/robosherlock

https://github.com/Vanessa-rin/rs_turn_table

----------------------------------------------------------------------------------------------------------------------------
