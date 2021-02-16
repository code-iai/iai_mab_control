#!/usr/bin/env python

from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from iai_scanning_table_msgs.msg import scanning_tableAction, scanning_tableGoal
from imp import load_source
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from rospkg import RosPack
from visualization_msgs.msg import Marker

import camera
import math
import numpy
import os
import rospy
import sys
import tf

def init():
    global marker_pub
    global move_group, turntable
    global camera_mesh, camera_pos, camera_size, distance_camera_object, num_positions, num_spins, object_size, reach, simulation, test, turntable_pos

    rospy.init_node('acquisition')

    camera_mesh = rospy.get_param('~camera_mesh', None)
    camera_orientation = rospy.get_param('~camera_orientation', None)
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
    turntable_pos = rospy.get_param('~turntable_pos', photobox_pos[:2] + [photobox_pos[2] + 0.05])
    turntable_radius = rospy.get_param('~turntable_radius', 0.2)
    wall_thickness = rospy.get_param('~wall_thickness', 0.04)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1, latch=True)

    move_group = MoveGroupCommander('manipulator')
    move_group.set_max_acceleration_scaling_factor(1.0 if simulation else max_velocity)
    move_group.set_max_velocity_scaling_factor(1.0 if simulation else max_velocity)

    #min_x, min_y = -photobox_size[0] / 2, photobox_pos[1] - photobox_size[1] / 2
    #max_x, max_y = -min_x, 0.5
    #move_group.set_workspace([min_x, min_y, max_x, max_y])

    planner_ids = move_group.get_interface_description().planner_ids
    if 'manipulator[PRMstarkConfigDefault]' in planner_ids:
        move_group.set_planner_id('PRMstarkConfigDefault')
    elif 'manipulator[RRTstarkConfigDefault]' in planner_ids:
        move_group.set_planner_id('RRTstarkConfigDefault')

    robot = RobotCommander()

    scene = PlanningSceneInterface(synchronous=True)

    try:
        st_control = load_source('st_control', RosPack().get_path('iai_scanning_table') + '/scripts/iai_scanning_table/st_control.py')
        turntable = st_control.ElmoUdp()
        if turntable.check_device():
            turntable.configure()
            turntable.reset_encoder()
            turntable.start_controller()
    except Exception as e:
        print(e)
        turntable = None

    if simulation or test:
        move_group.set_planner_id('')
        move_home()
    elif turntable is None:
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

    # add cable mounts
    scene.remove_attached_object('upper_arm_link', 'upper_arm_cable_mount')
    scene.remove_attached_object('forearm_link', 'forearm_cable_mount')
    scene.remove_world_object('upper_arm_cable_mount')
    scene.remove_world_object('forearm_cable_mount')

    size = [0.08, 0.08, 0.08]

    ps.header.frame_id = 'upper_arm_link'
    ps.pose.position.x = -0.13
    ps.pose.position.y = -0.095
    ps.pose.position.z = 0.135
    scene.attach_box(ps.header.frame_id, 'upper_arm_cable_mount', ps, size)

    ps.header.frame_id = 'forearm_link'
    ps.pose.position.x = -0.275
    ps.pose.position.y = -0.08
    ps.pose.position.z = 0.02
    scene.attach_box(ps.header.frame_id, 'forearm_cable_mount', ps, size)

    # add camera
    eef_link = move_group.get_end_effector_link()
    ps = PoseStamped()
    ps.header.frame_id = eef_link

    scene.remove_attached_object(eef_link, 'camera_0')
    scene.remove_attached_object(eef_link, 'camera_1')
    scene.remove_world_object('camera_0')
    scene.remove_world_object('camera_1')

    ps.pose.position.y = camera_pos[1]
    ps.pose.position.z = camera_pos[2]

    if camera_mesh:
        ps.pose.position.x = camera_pos[0]

        quaternion = tf.transformations.quaternion_from_euler(math.radians(camera_orientation[0]), math.radians(camera_orientation[1]), math.radians(camera_orientation[2]))
        ps.pose.orientation.x = quaternion[0]
        ps.pose.orientation.y = quaternion[1]
        ps.pose.orientation.z = quaternion[2]
        ps.pose.orientation.w = quaternion[3]

        scene.attach_mesh(eef_link, 'camera_0', ps, os.path.expanduser(camera_mesh), camera_size)
        scene.attach_mesh(eef_link, 'camera_1', ps, os.path.expanduser(camera_mesh), numpy.array(camera_size) * 1.5)

        vertices = scene.get_attached_objects(['camera_1'])['camera_1'].object.meshes[0].vertices
        camera_size[0] = max(vertice.x for vertice in vertices) - min(vertice.x for vertice in vertices)
        camera_size[1] = max(vertice.y for vertice in vertices) - min(vertice.y for vertice in vertices)
        camera_size[2] = max(vertice.z for vertice in vertices) - min(vertice.z for vertice in vertices)
    else:
        ps.pose.position.x = camera_pos[0] + camera_size[0] / 2
        scene.attach_box(eef_link, 'camera_0', ps, camera_size)

def show_marker(pose, lifetime_secs=0.0):
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.pose = pose
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = 1.0
    marker.color.a = 1.0
    marker.lifetime.secs = lifetime_secs
    marker_pub.publish(marker)

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

def get_orientation_from_pos_to_object(position):
    angle_y = math.atan2(position.z - turntable_pos[2], position.y - turntable_pos[1] - camera_pos[2])
    angle_z = math.atan2(turntable_pos[1] - position.y, turntable_pos[0] - position.x - camera_pos[1])
    quaternion = tf.transformations.quaternion_from_euler(0.0, angle_y, angle_z)
    orientation = Quaternion()
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]
    return orientation

def move_to(position, x_tolerance=0.2):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation = get_orientation_from_pos_to_object(pose.position)
    show_marker(pose)

    poses = [pose]

    if x_tolerance > 0:
        for x in numpy.arange(-x_tolerance, x_tolerance + 0.01, 0.01):
            p = Pose()
            p.position.x = pose.position.x + x
            p.position.y = pose.position.y
            p.position.z = pose.position.z
            p.orientation = get_orientation_from_pos_to_object(p.position)
            poses.append(p)
        poses.sort(key=lambda pose: abs(pose.position.x))

    move_group.set_pose_targets(poses)
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
    distance = max(camera_size) + distance_camera_object
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
        if move_to(position) and not test:
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
