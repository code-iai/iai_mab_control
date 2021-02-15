#!/usr/bin/env python

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math
import rospy

joint = None
joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def set_joints(elbow_joint=None, shoulder_lift_joint=None, shoulder_pan_joint=None, wrist_1_joint=None, wrist_2_joint=None, wrist_3_joint=None, radians=False):
    for i, joint in enumerate(joint_names):
        goal = locals()[joint]
        if goal is not None:
            joint_goals[i] = goal if radians else math.radians(goal)

    msg = JointTrajectory()
    msg.joint_names = joint_names
    points = JointTrajectoryPoint()
    points.positions = joint_goals
    points.time_from_start = rospy.Duration(1.0 if simulation else 5.0)
    msg.points = [points]
    arm_pub.publish(msg)
    rospy.sleep(1.0 if simulation else 5.0)

def on_key(key):
    global joint

    key = key.data
    if (key == 43 or key == 45) and joint is None:
        print('\nYou have to select a joint first')
    elif key == 43 or key == 45:
        if key == 43: # +
            joint_goals[joint_names.index(joint)] += math.radians(5)
        else: # -
            joint_goals[joint_names.index(joint)] -= math.radians(5)

        set_joints(joint_goals[0], joint_goals[1], joint_goals[2], joint_goals[3], joint_goals[4], joint_goals[5], radians=True)
        print('\nNew position at {} degrees'.format(math.degrees(joint_goals[joint_names.index(joint)])))
    else:
        if key == 49: # 1
            joint = 'elbow_joint'
        elif key == 50: # 2
            joint = 'shoulder_lift_joint'
        elif key == 51: # 3
            joint = 'shoulder_pan_joint'
        elif key == 52: # 4
            joint = 'wrist_1_joint'
        elif key == 53: # 5
            joint = 'wrist_2_joint'
        elif key == 54: # 6
            joint = 'wrist_3_joint'
        print('\nSwitched joint to {}'.format(joint))
        print('Current position at {} degrees.'.format(math.degrees(joint_goals[joint_names.index(joint)])))

if __name__ == '__main__':
    global simulation, joint_goals, arm_pub

    rospy.init_node('keyboard_controller')

    simulation = rospy.get_param('~simulation', False)
    joint_goals = [position for position in rospy.wait_for_message('joint_states', JointState).position]
    arm_pub = rospy.Publisher(rospy.get_param('~command_topic', 'arm_controller/command'), JointTrajectory, latch=True, queue_size=10)

    print('This tool will move the selected joint by 5 degrees for each key press +/-.\n')
    print('Press the following keys to control the robot arm:')
    print('1: Switch to elbow_joint')
    print('2: Switch to shoulder_lift_joint')
    print('3: Switch to shoulder_pan_joint')
    print('4: Switch to wrist_1_joint')
    print('5: Switch to wrist_2_joint')
    print('6: Switch to wrist_6_joint')
    print('+: Increase joint angle')
    print('-: Decrease joint angle')

    rospy.Subscriber('key', Int32, on_key)
    rospy.spin()

