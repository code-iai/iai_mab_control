#!/usr/bin/env python

import rospy
import roslib;
import actionlib
import math
import iai_scanning_table_action.msg
import spinnaker_action.msg

from std_srvs.srv import *
from std_msgs.msg import Header
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from time import sleep

# Names used for the Messages
joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint",
               "wrist_3_joint"]

actual_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

home_position = [0.0, -1.57, 0.0, -1.57, -0.0, -0.0]


# Update joint Position
# @param data = sensor_msgs/JointState.msg
def update(data):
    global actual_joint_position
    actual_joint_position = data.position


# Subscribe to joint_states and Update actual_joint_position if any changes happens.
def updateJointStates():
    rospy.Subscriber("joint_states", JointState, update)


# Move the ur5 using the given rotations
# @param goal = List[floats]
def moveArm(goal):
    pub = rospy.Publisher('scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)

    rate = rospy.Rate(10)  # 10hz

    goalNotReached = True
    print("Moving Arm")
    while goalNotReached:

        waypoint = JointTrajectory()
        waypoint.joint_names = joint_names
        points = JointTrajectoryPoint()

        count = 0
        for x in range(0, 6):
            # Buffer against noise
            if goal[x] - 0.0125 <= actual_joint_position[x] <= goal[x] + 0.0125:
                count += 1

        if count == 6:
            goalNotReached = False

        points.positions = goal
        # Roboter speed is difined by this value
        # 10sec to 1sec is 5% to 50% and every thing under 1sec to 0.5sec is 50% to 100%
        points.time_from_start = rospy.Duration(3.0)
        waypoint.points = [points]

        waypoint.header = Header()
        waypoint.header.frame_id = ''
        waypoint.header.seq = count
        count += 1

        pub.publish(waypoint)
        rate.sleep()


# @param angle = radians
def moveTurntable(angle):
    print("Moving TurnTable")
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('scanning_table_action_server',
                                          iai_scanning_table_action.msg.scanning_tableAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("TurnTable Waiting for Server!")
    client.wait_for_server()
    print("Done!")
    # Creates a goal to send to the action server.
    goal = iai_scanning_table_action.msg.scanning_tableGoal()

    goal.angle = angle
    goal.release_brake_timeout = 10.0
    goal.apply_modulo = True

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Returns True in case of success.
    return client.get_result()


# @param image_name = String
def triggerCamera(image_name):
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('camera_controller', spinnaker_action.msg.take_pictureAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("RGB Waiting for Server!")
    client.wait_for_server()
    print("Done!")
    # Creates a goal to send to the action server.
    goal = spinnaker_action.msg.take_pictureGoal(modus=image_name)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


def triggerRGBD():
    print("RGBD Waiting for Server!")
    rospy.wait_for_service('/RoboSherlock_ur5robot/trigger')
    print("Done!")
    try:
        handler = rospy.ServiceProxy('/RoboSherlock_ur5robot/trigger', Trigger)
        resp = handler()
        return str(resp.success) + " " + resp.message
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def recording():
    angles = []
    # Move arm to a Start position
    moveArm([0.0, -1.57, -1.63, -1.57, -0.0, -0.0])

    # Create a list with angles for the Turntable
    for x in range(37):
        angles.append(x * 10)

    armPosition = [[2.71, -3.12, -1.63, -2.72, -1.63, -0.0], [2.40, -2.91, -1.63, -2.46, -1.63, -0.0],
                   [2.09, -2.68, -1.63, -2.20, -1.63, -0.0], [1.76, -2.44, -1.63, -1.94, -1.63, -0.0],
                   [1.38, -2.18, -1.63, -1.66, -1.63, -0.0], [0.92, -1.86, -1.63, -1.33, -1.63, -0.0],
                   [0.27, -1.51, -1.63, -1.00, -1.63, -0.0]]

    # Move arm to angle 0
    moveArm([2.71, -3.12, -1.63, -2.72, -1.63, -0.0])

    # Take 20 images for better light adjustment
    for a in range(20):
        image_name = "Leerlauf : " + str(a)
        print(triggerCamera(image_name))

    # Take x pictures for each arm Position. x = number of angles
    for a in range(len(armPosition)):
        if len(armPosition):
            moveArm(armPosition[a])
            for b in angles:
                image_name = "ArmPosition_" + str(a) + "_Angle_" + str(b)
                print(triggerCamera(image_name))
                print(triggerRGBD())
                moveTurntable(math.radians(b))

    moveArm([0.0, -1.57, -1.63, -1.57, -0.0, -0.0])
    moveArm(home_position)


if __name__ == '__main__':
    try:
        rospy.init_node('hermes', anonymous=True)
        roslib.load_manifest('iai_scanning_table')
        updateJointStates()
        recording()
        print("Done!")
        sleep(1.5)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
