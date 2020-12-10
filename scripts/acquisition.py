#!/usr/bin/env python

from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped
from iai_scanning_table_action.msg import scanning_tableAction, scanning_tableGoal
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from tf2_msgs.msg import TFMessage

import math
import rospy
import sys
import tf

def init():
    global move_group, robot, scene, turntable_client
    global simulation, turntable_pos

    rospy.init_node('photogrammetry')
    simulation = rospy.get_param('~simulation', False)
    turntable_pos = [0.0, -0.7, 0.1] # TODO: get position from GUI configuration (rospy.get_param())

    move_group = MoveGroupCommander('manipulator')
    move_group.set_max_velocity_scaling_factor(1.0 if simulation else 0.1)

    robot = RobotCommander()

    scene = PlanningSceneInterface(synchronous=True)

    turntable_client = SimpleActionClient('scanning_table_action_server', scanning_tableAction)
    if not simulation and not turntable_client.wait_for_server(rospy.Duration(10)):
        sys.exit('Could not connect to turntable.')

    if simulation:
        move_home()
        rospy.Subscriber('tf', TFMessage, send_turntable_tf, tf.TransformBroadcaster())

    # add ground plane
    ps = PoseStamped()
    ps.header.frame_id = robot.get_planning_frame()
    scene.add_plane('ground', ps)

    # add photobox
    ps.pose.position.x = 0.5
    ps.pose.position.y = -0.7
    ps.pose.position.z = 0.5
    scene.add_box('box_wall_left', ps, (0.1, 1.0, 1.0))

    ps.pose.position.x = -0.5
    ps.pose.position.y = -0.7
    ps.pose.position.z = 0.5
    scene.add_box('box_wall_right', ps, (0.1, 1.0, 1.0))

    ps.pose.position.x = 0.0
    ps.pose.position.y = -1.25
    ps.pose.position.z = 0.5
    scene.add_box('box_wall_back', ps, (1.0, 0.1, 1.0))

    ps.pose.position.x = 0.0
    ps.pose.position.y = -0.7
    ps.pose.position.z = 0.05
    scene.add_box('box_ground', ps, (1.0, 1.0, 0.1))

    # add object on turntable
    # TODO: add box for object on turntable

    # add camera
    eef_link = move_group.get_end_effector_link()

    scene.remove_attached_object(eef_link, 'camera')
    scene.remove_world_object('camera')

    ps = PoseStamped()
    ps.header.frame_id = eef_link
    ps.pose.position.x = 0.05
    scene.attach_box(eef_link, 'camera', ps, (0.1, 0.1, 0.1))


def send_turntable_tf(msg, tb):
    try:
        tb.sendTransform(turntable_pos, (0, 0, 0, 1), rospy.Time(), 'turntable', 'world')
    except:
        pass

def move_home():
    move_group.go([0, math.radians(-90), 0, math.radians(-90), 0, 0], wait=True)
    move_group.stop()

def move_joint(joint, goal, radians=False):
    goals = move_group.get_current_joint_values()
    goals[move_group.get_joints().index(joint)] = goal if radians else math.radians(goal)
    move_group.go(goals, wait=True)
    move_group.stop()

def move_to(x, y, z, face_turntable=True):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    if face_turntable:
        angleY = math.atan2(pose.position.z - turntable_pos[2], pose.position.y - turntable_pos[1])
        angleZ = math.atan2(turntable_pos[1] - pose.position.y, turntable_pos[0] - pose.position.x)
        quaternion = tf.transformations.quaternion_from_euler(0, angleY, angleZ)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
    else:
        pose.orientation.w = 1.0

    move_group.set_pose_target(pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def set_turntable_angle(angle, radians=False):
    msg = scanning_tableGoal()
    msg.angle = angle if radians else math.radians(angle)
    msg.apply_modulo = True
    turntable_client.send_goal(msg)
    return True if simulation else turntable_client.wait_for_result(rospy.Duration(0))

if __name__ == '__main__':
    init()

    move_to(0.2, -0.35, 0.3)

