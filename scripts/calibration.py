#!/usr/bin/env python

from geometry_msgs.msg import PointStamped
from moveit_commander import MoveGroupCommander, RobotCommander
from tf import TransformListener

import numpy
import os
import rospy
import sys

def init():
    rospy.init_node('calibration')

    global input
    if sys.version_info.major == 2:
        input = raw_input

    global ps
    ps = PointStamped()
    ps.point.x = 0.1
    for group_name in RobotCommander().get_group_names():
        ee_link = MoveGroupCommander(group_name).get_end_effector_link()
        if ee_link:
            ps.header.frame_id = ee_link
            break

    global tl
    tl = TransformListener()

def get_position(timeout=5.0):
    tl.waitForTransform('world', ps.header.frame_id, rospy.Time(), rospy.Duration(timeout))
    return tl.transformPoint('world', ps)

def point_to_array(p):
    return numpy.array([p.x, p.y, p.z])

def get_distance(p1, p2):
    return numpy.linalg.norm(point_to_array(p1) - point_to_array(p2))

if __name__ == '__main__':
    init()

    print('\nPlease follow the instructions to calibrate the photobox and turntables size and position.')
    input('\nBefore you continue make sure that the calibration tool is mounted on the robot. Press [Enter] to continue.')
    print('\nYou will now be asked to move the tip of the calibration tool to given points. A graph explaining where each point is located at can be found in {}.\n'.format(os.path.abspath(__file__ + '/../assets/calibration.png')))

    input('Move the tip of the calibration tool to point A and press [Enter] to continue.')
    A = get_position().point

    input('Move the tip of the calibration tool to point B and press [Enter] to continue.')
    B = get_position().point

    input('Move the tip of the calibration tool to point C and press [Enter] to continue.')
    C = get_position().point

    input('Move the tip of the calibration tool to point D and press [Enter] to continue.')
    D = get_position().point

    input('Move the tip of the calibration tool to point E and press [Enter] to continue.')
    E = get_position().point

    input('Move the tip of the calibration tool to point F and press [Enter] to continue.')
    F = get_position().point

    photobox_size = [
        get_distance(A, B) - 0.1, # make box width smaller by 0.1 to keep more distance to the walls
        get_distance(B, C) * 2,
        get_distance(B, D) * 2]
    photobox_pos = [
        (A.x + B.x) / 2,
        C.y,
        A.z + 0.05] # lift box a little bit to keep more distance to box bottom

    turntable_radius = get_distance(E, F)
    turntable_pos = [E.x, E.y, E.z + 0.05] # lift turntable to match lifting of box

    print('\nReplace the appropriate values in {} with the following:\n'.format(os.path.abspath(__file__ + '/../../app/settings/general.json')))
    print('photobox_pos: {}'.format(photobox_pos))
    print('photobox_size: {}'.format(photobox_size))
    print('turntable_pos: {}'.format(turntable_pos))
    print('turntable_radius: {}'.format(turntable_radius))

