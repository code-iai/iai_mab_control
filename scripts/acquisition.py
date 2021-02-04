#!/usr/bin/env python

from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped
from iai_scanning_table_msgs.msg import scanning_tableAction, scanning_tableGoal
from imp import load_source
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from rospkg import RosPack
from tf2_msgs.msg import TFMessage

import camera
import math
import numpy
import os
import rospy
import sys
import tf

def init():
    global move_group, robot, scene, turntable
    global camera_pos, camera_size, distance_camera_object, num_positions, num_spins, object_size, reach, simulation, test, turntable_pos

    rospy.init_node('photogrammetry')

    camera_pos = rospy.get_param('~camera_pos', [0.0, 0.0, 0.0])
    camera_size = rospy.get_param('~camera_size', [0.1, 0.1, 0.1])
    distance_camera_object = rospy.get_param('~distance_camera_object', 0.2)
    max_velocity = rospy.get_param('~max_velocity', 0.1)
    num_positions = rospy.get_param('~num_positions', 15)
    num_spins = rospy.get_param('~num_spins', 8)
    object_size = numpy.array(rospy.get_param('~object_size', [0.2, 0.2, 0.2]))
    photobox_pos = rospy.get_param('~photobox_pos', [0.0, -0.6, 0.0])
    photobox_size = rospy.get_param('~photobox_size', [0.7, 0.7, 1.0])
    reach = rospy.get_param('~reach', 0.85)
    simulation = rospy.get_param('~simulation', True)
    test = rospy.get_param('~test', True)
    turntable_pos = rospy.get_param('~turntable_pos', photobox_pos[:2] + [photobox_pos[2] + 0.02])
    turntable_radius = rospy.get_param('~turntable_radius', 0.2)
    wall_thickness = rospy.get_param('~wall_thickness', 0.04)

    move_group = MoveGroupCommander('manipulator')
    move_group.set_max_velocity_scaling_factor(1.0 if simulation else max_velocity)

    robot = RobotCommander()

    scene = PlanningSceneInterface(synchronous=True)

    try:
        turntable = load_source('st_control', RosPack().get_path('iai_scanning_table') + '/scripts/iai_scanning_table/st_control.py').ElmoUdp()
        turntable.configure()
        turntable.reset_encoder()
    except:
        turntable = None

    if simulation or test:
        move_home()
        rospy.Subscriber('tf', TFMessage, send_turntable_tf, tf.TransformBroadcaster())
    elif turntable is None or not turntable.start_controller():
        sys.exit('Could not connect to turntable.')
    elif not camera.init(rospy.get_param('~output_directory', 'out')):
        sys.exit('Could not initialize camera.')

    # add ground plane
    ps = PoseStamped()
    ps.header.frame_id = robot.get_planning_frame()
    scene.add_plane('ground', ps)

    # add photobox
    ps.pose.position.x = photobox_pos[0] + photobox_size[0] / 2 + wall_thickness / 2
    ps.pose.position.y = photobox_pos[1]
    ps.pose.position.z = photobox_pos[2] + photobox_size[2] / 2
    scene.add_box('box_wall_left', ps, (wall_thickness, photobox_size[1], photobox_size[2]))

    ps.pose.position.x = photobox_pos[0] - photobox_size[0] / 2 - wall_thickness / 2
    ps.pose.position.y = photobox_pos[1]
    ps.pose.position.z = photobox_pos[2] + photobox_size[2] / 2
    scene.add_box('box_wall_right', ps, (wall_thickness, photobox_size[1], photobox_size[2]))

    ps.pose.position.x = photobox_pos[0]
    ps.pose.position.y = photobox_pos[1] - photobox_size[1] / 2 - wall_thickness / 2
    ps.pose.position.z = photobox_pos[2] + photobox_size[2] / 2
    scene.add_box('box_wall_back', ps, (photobox_size[0], wall_thickness, photobox_size[2]))

    ps.pose.position.x = photobox_pos[0]
    ps.pose.position.y = photobox_pos[1]
    ps.pose.position.z = photobox_pos[2] - wall_thickness / 2
    scene.add_box('box_ground', ps, (photobox_size[0], photobox_size[1], wall_thickness))

    # add turntable
    turntable_height = turntable_pos[2] - photobox_pos[2]
    ps.pose.position.x = turntable_pos[0]
    ps.pose.position.y = turntable_pos[1]
    ps.pose.position.z = photobox_pos[2] + turntable_height / 2
    scene.add_cylinder('turntable', ps, turntable_height, turntable_radius)

    # add object on turntable
    ps.pose.position.x = turntable_pos[0]
    ps.pose.position.y = turntable_pos[1]
    ps.pose.position.z = turntable_pos[2] + object_size[2] / 2
    scene.add_cylinder('object', ps, object_size[2], max(object_size[:2]) / 2)

    # add camera
    eef_link = move_group.get_end_effector_link()
    ps = PoseStamped()
    ps.header.frame_id = eef_link

    scene.remove_attached_object(eef_link, 'camera')
    scene.remove_world_object('camera')

    ps.pose.position.x = camera_pos[0] + camera_size[0] / 2
    ps.pose.position.y = camera_pos[1]
    ps.pose.position.z = camera_pos[2]

    scene.attach_box(eef_link, 'camera', ps, camera_size)

def send_turntable_tf(msg, tb):
    try:
        tb.sendTransform(turntable_pos, (0, 0, 0, 1), rospy.Time(), 'turntable', 'world')
    except:
        pass

def move_home():
    result = move_group.go([0, math.radians(-90), 0, math.radians(-90), 0, 0], wait=True)
    move_group.stop()
    return result

def move_joint(joint, goal, radians=False):
    goals = move_group.get_current_joint_values()
    goals[move_group.get_joints().index(joint)] = goal if radians else math.radians(goal)
    result = move_group.go(goals, wait=True)
    move_group.stop()
    return result

def move_to(position, orientation=None, face=None):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]

    if orientation:
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
    else:
        if face == 'top':
            angle_y = math.atan2(pose.position.z - turntable_pos[2] - object_size[2], pose.position.y - turntable_pos[1] - camera_pos[2])
        elif face == 'bottom':
            angle_y = math.atan2(pose.position.z - turntable_pos[2], pose.position.y - turntable_pos[1] - camera_pos[2])
        else: # center
            angle_y = math.atan2(pose.position.z - turntable_pos[2] - object_size[2] / 2, pose.position.y - turntable_pos[1] - camera_pos[2])

        angle_z = math.atan2(turntable_pos[1] - pose.position.y, turntable_pos[0] - pose.position.x - camera_pos[1])
        quaternion = tf.transformations.quaternion_from_euler(0.0, angle_y, angle_z)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

    move_group.set_pose_target(pose)
    result = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return result

def set_turntable_deg(deg, speed=30):
    if simulation:
        return True

    turntable.set_speed_deg(speed)
    turntable.move_to_deg(deg)
    return turntable.wait_to_reach_target()

def create_arm_positions(n=15):
    positions = []
    distance = max(camera_size[:2]) + distance_camera_object
    div = max(object_size[:2]) / 2 + object_size[2] + math.pi * 2 * distance_camera_object / 4

    min_y = turntable_pos[1]
    max_y = turntable_pos[1] + max(object_size[:2]) / 2
    min_z = turntable_pos[2]
    max_z = turntable_pos[2] + object_size[2]

    for y in numpy.linspace(min_y, max_y, round(n / div * max(object_size[:2]) / 2)):
        position = [turntable_pos[0] - camera_pos[1], y, max_z + distance]
        if numpy.linalg.norm(position) <= reach:
            positions.append(position)

    num = round(n / div * math.pi * 2 * distance_camera_object / 4)
    for i in range(int(num)):
        position = [turntable_pos[0] - camera_pos[1], max_y + math.sin(math.pi / 2 / num * i) * distance, max_z + math.cos(math.pi / 2 / num * i) * distance]
        if numpy.linalg.norm(position) <= reach:
            positions.append(position)

    for z in numpy.linspace(max_z, min_z, round(n / div * object_size[2])):
        position = [turntable_pos[0] - camera_pos[1], max_y + distance, z]
        if numpy.linalg.norm(position) <= reach:
            positions.append(position)

    return positions

if __name__ == '__main__':
    init()

    positions = create_arm_positions(num_positions)
    print('Number positions: {}'.format(len(positions)))
    for position in positions:
        print('Position: {}'.format(position))
        for face in ['top', 'center', 'bottom']:
            print('Face: {}'.format(face))
            if move_to(position, face=face) and not test:
                for i in range(num_spins):
                    deg = 360 * i / num_spins
                    print('Turntable angle: {} degrees'.format(deg))
                    set_turntable_deg(deg)

                    if not simulation:
                        print('Capturing photo')
                        file = camera.capture()

                        if file is not None:
                            print('preview: ' + file);
        print('progress: {}'.format(int(float(positions.index(position) + 1) / len(positions) * 100)))
        rospy.sleep(1)
    camera.exit()
    move_home()
