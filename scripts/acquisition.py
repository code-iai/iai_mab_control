#!/usr/bin/env python

from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped
from iai_scanning_table_msgs.msg import scanning_tableAction, scanning_tableGoal
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from tf2_msgs.msg import TFMessage

import camera
import math
import numpy
import rospy
import sys
import tf

def init():
    global move_group, robot, scene, turntable_client
    global camera_size, distance_camera_object, num_positions, num_spins, object_size, simulation, test, turntable_pos

    rospy.init_node('photogrammetry')

    camera_size = rospy.get_param('~camera_size', [0.1, 0.1, 0.1])
    distance_camera_object = rospy.get_param('~distance_camera_object', 0.2)
    num_positions = rospy.get_param('~num_positions', 15)
    num_spins = rospy.get_param('~num_spins', 8)
    object_size = numpy.array(rospy.get_param('~object_size', [0.2, 0.2, 0.2]))
    object_size[:2] = max(object_size[:2])
    photobox_pos = rospy.get_param('~photobox_pos', [0.0, -0.7, 0.0])
    photobox_size = rospy.get_param('~photobox_size', [1.0, 1.0, 1.0])
    simulation = rospy.get_param('~simulation', False)
    test = rospy.get_param('~test', False)
    turntable_pos = photobox_pos
    turntable_pos[2] += 0.1 # TODO: measure turntable z-value in lab

    move_group = MoveGroupCommander('manipulator')
    move_group.set_max_velocity_scaling_factor(1.0 if simulation else 0.1)

    robot = RobotCommander()

    scene = PlanningSceneInterface(synchronous=True)

    turntable_client = SimpleActionClient('scanning_table_action_server', scanning_tableAction)

    if simulation or test:
        move_home()
        rospy.Subscriber('tf', TFMessage, send_turntable_tf, tf.TransformBroadcaster())
    elif not turntable_client.wait_for_server(rospy.Duration(10)):
        sys.exit('Could not connect to turntable.')
    elif not camera.init(rospy.get_param('~output_directory', 'out'), not simulation):
        sys.exit('Could not initialize camera.')

    # add ground plane
    ps = PoseStamped()
    ps.header.frame_id = robot.get_planning_frame()
    scene.add_plane('ground', ps)

    # add photobox
    ps.pose.position.x = photobox_size[0] / 2
    ps.pose.position.y = photobox_pos[1]
    ps.pose.position.z = photobox_size[2] / 2
    scene.add_box('box_wall_left', ps, (0.01, photobox_size[1], photobox_size[2]))

    ps.pose.position.x = -photobox_size[0] / 2
    ps.pose.position.y = photobox_pos[1]
    ps.pose.position.z = photobox_size[2] / 2
    scene.add_box('box_wall_right', ps, (0.01, photobox_size[1], photobox_size[2]))

    ps.pose.position.x = photobox_pos[0]
    ps.pose.position.y = photobox_pos[1] - photobox_size[1] / 2
    ps.pose.position.z = photobox_size[2] / 2
    scene.add_box('box_wall_back', ps, (photobox_size[0], 0.01, photobox_size[2]))

    ps.pose.position.x = photobox_pos[0]
    ps.pose.position.y = photobox_pos[1]
    ps.pose.position.z = 0.0
    scene.add_box('box_ground', ps, (photobox_size[0], photobox_size[1], 0.01))

    # add object on turntable
    ps.pose.position.x = turntable_pos[0]
    ps.pose.position.y = turntable_pos[1]
    ps.pose.position.z = turntable_pos[2] + object_size[2] / 2
    scene.add_box('object', ps, object_size)

    # add camera
    eef_link = move_group.get_end_effector_link()

    scene.remove_attached_object(eef_link, 'camera')
    scene.remove_world_object('camera')

    ps = PoseStamped()
    ps.header.frame_id = eef_link
    ps.pose.position.x = camera_size[0] / 2
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
            angle_y = math.atan2(pose.position.z - turntable_pos[2] - object_size[2], pose.position.y - turntable_pos[1])
        elif face == 'bottom':
            angle_y = math.atan2(pose.position.z - turntable_pos[2], pose.position.y - turntable_pos[1])
        else: # center
            angle_y = math.atan2(pose.position.z - turntable_pos[2] - object_size[2] / 2, pose.position.y - turntable_pos[1])

        angle_z = math.atan2(turntable_pos[1] - pose.position.y, turntable_pos[0] - pose.position.x)
        quaternion = tf.transformations.quaternion_from_euler(0, angle_y, angle_z)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

    move_group.set_pose_target(pose)
    result = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return result

def set_turntable_angle(angle, radians=False):
    msg = scanning_tableGoal()
    msg.angle = angle if radians else math.radians(angle)
    msg.apply_modulo = True
    turntable_client.send_goal(msg)
    return True if simulation else turntable_client.wait_for_result(rospy.Duration(0))

def create_arm_positions(n=15):
    min_y = turntable_pos[1]
    max_y = turntable_pos[1] + object_size[1] / 2 + camera_size[1] + distance_camera_object
    min_z = turntable_pos[2]
    max_z = turntable_pos[2] + object_size[2] + camera_size[1] + distance_camera_object

    div = object_size[1] / 2 + object_size[2]
    positions = []
    for y in numpy.linspace(min_y, max_y, round(n / div * object_size[1] / 2)):
        positions.append([0.0, y, max_z])
    for z in numpy.linspace(max_z, min_z, round(n / div * object_size[2]) + 1)[1:]:
        positions.append([0.0, max_y, z])

    return positions

if __name__ == '__main__':
    print('progress: 0')
    sys.stdout.flush()

    init()

    positions = create_arm_positions(num_positions)
    for position in positions:
        for face in ['top', 'center', 'bottom']:
            if move_to(position, face=face) and not test:
                for i in range(num_spins):
                    set_turntable_angle(360 * i / num_spins)

                    if not simulation:
                        if camera.capture() is None:
                            pass # TODO: handle capture failure
        print('progress: {}'.format(int(float(positions.index(position) + 1) / len(positions) * 100)))
        sys.stdout.flush()
        rospy.sleep(1)
    camera.exit()
    move_home()
