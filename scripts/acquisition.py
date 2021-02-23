#!/usr/bin/env python

from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from iai_scanning_table_msgs.msg import scanning_tableAction, scanning_tableGoal
from imp import load_source
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from rospkg import RosPack
from visualization_msgs.msg import Marker, MarkerArray

import camera
import math
import numpy
import os
import rospy
import sys
import tf

def init():
    global marker_pub, marker_array_pub
    global move_group, turntable
    global camera_mesh, camera_pos, camera_size, distance_camera_object, num_positions, num_spins, object_size, photobox_pos, photobox_size, reach, simulation, test, turntable_pos

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
    marker_array_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1, latch=True)

    move_group = MoveGroupCommander('manipulator')
    move_group.set_max_acceleration_scaling_factor(1.0 if simulation else max_velocity)
    move_group.set_max_velocity_scaling_factor(1.0 if simulation else max_velocity)
    #move_group.set_planning_time(30.0)

    if not simulation:
        planner_ids = move_group.get_interface_description().planner_ids
        if 'manipulator[PRMstarkConfigDefault]' in planner_ids:
            pass#move_group.set_planner_id('PRMstarkConfigDefault')
        elif 'manipulator[RRTstarkConfigDefault]' in planner_ids:
            pass#move_group.set_planner_id('RRTstarkConfigDefault')

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

    if simulation:
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
        if not simulation:
            scene.attach_mesh(eef_link, 'camera_1', ps, os.path.expanduser(camera_mesh), numpy.array(camera_size) * 1.5)

        vertices = scene.get_attached_objects(['camera_0'])['camera_0'].object.meshes[0].vertices
        camera_size[0] = max(vertice.x for vertice in vertices) - min(vertice.x for vertice in vertices)
        camera_size[1] = max(vertice.y for vertice in vertices) - min(vertice.y for vertice in vertices)
        camera_size[2] = max(vertice.z for vertice in vertices) - min(vertice.z for vertice in vertices)
    else:
        ps.pose.position.x = camera_pos[0] + camera_size[0] / 2
        scene.attach_box(eef_link, 'camera_0', ps, camera_size)

def show_markers(poses, lifetime_secs=0.0):
    marker_array = MarkerArray()

    for i, pose in enumerate(poses):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.id = i + 1
        marker.pose = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.secs = lifetime_secs
        marker_array.markers.append(marker)

    marker_array_pub.publish(marker_array)

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

def get_orientation_from_pos_to_pos(pos_from, pos_to, y_offset=0.0):
    angle_y = math.atan2(pos_from.z - pos_to[2], pos_from.y - pos_to[1] - y_offset - camera_pos[2])
    angle_z = math.atan2(pos_to[1] - y_offset - pos_from.y, pos_to[0] - pos_from.x - camera_pos[1])
    quaternion = tf.transformations.quaternion_from_euler(0.0, angle_y, angle_z)
    orientation = Quaternion()
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]
    return orientation

def move_to(positions, x_tolerance=0.2):
    poses = []

    pose = Pose()
    pose.position.x = positions[0][0]
    pose.position.y = positions[0][1]
    pose.position.z = positions[0][2]
    pose.orientation = get_orientation_from_pos_to_pos(pose.position, turntable_pos, max(camera_size) / 1.5)
    poses.append(pose)

    if x_tolerance > 0:
        for position in positions:
            _poses = []
            for x in arange(-x_tolerance, x_tolerance, 0.05):
                pose = Pose()
                pose.position.x = photobox_pos[0] + x
                pose.position.y = position[1]
                pose.position.z = position[2]
                pose.orientation = get_orientation_from_pos_to_pos(pose.position, turntable_pos, max(camera_size) / 1.5)
                _poses.append(pose)
            _poses.sort(key=lambda pose: abs(pose.position.x))
            poses += _poses

    show_markers(poses)
    show_marker(poses[0])

    move_group.set_pose_targets(poses)
    result = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return result

def set_turntable_deg(deg, wait=True):
    if simulation:
        return True

    turntable.set_speed_deg(30)
    turntable.move_to_deg(deg)

    return turntable.wait_to_reach_target() if wait else True
    

def arange(start, stop, step):
    if start == stop:
        return [start]
    else:
        return numpy.arange(start, stop + step, step)

def create_arm_positions(n=15, distance_tolerance=0.0):
    positions = []
    distance = max(camera_size) + distance_camera_object
    div = max(object_size[:2]) / 2 + object_size[2] + math.pi * 2 * distance_camera_object / 4

    min_y = turntable_pos[1]
    max_y = turntable_pos[1] + max(object_size[:2]) / 2
    min_z = turntable_pos[2]
    max_z = turntable_pos[2] + object_size[2]

    for y in numpy.linspace(min_y, max_y, round(n / div * max(object_size[:2]) / 2)):
        _positions = []
        for z in arange(max_z + distance, max_z + distance + distance_tolerance, 0.1):
            position = [turntable_pos[0] - camera_pos[1], y, z]
            if numpy.linalg.norm(position) <= reach:
                _positions.append(position)
        if len(_positions):
            positions.append(_positions)

    num = round(n / div * math.pi * 2 * distance_camera_object / 4)
    for i in range(int(num)):
        _positions = []
        for d in arange(distance, distance + distance_tolerance, 0.1):
            position = [turntable_pos[0] - camera_pos[1], max_y + math.sin(math.pi / 2 / num * i) * d, max_z + math.cos(math.pi / 2 / num * i) * d]
            if numpy.linalg.norm(position) <= reach:
                _positions.append(position)
        if len(_positions):
            positions.append(_positions)

    for z in numpy.linspace(max_z, min_z, round(n / div * object_size[2])):
        _positions = []
        for y in arange(max_y + distance, max_y + distance + distance_tolerance, 0.1):
            position = [turntable_pos[0] - camera_pos[1], y, z]
            if numpy.linalg.norm(position) <= reach:
                _positions.append(position)
        if len(_positions):
            positions.append(_positions)

    return positions

if __name__ == '__main__':
    init()

    positions = create_arm_positions(num_positions, 0.0)
    print('Number positions: {}'.format(len(positions)))

    for _positions in positions:
        print('Position: {}'.format(_positions[0]))
        if move_to(_positions, photobox_size[0] / 2 - 0.10) and not test:
            for i in range(num_spins):
                deg = 360 * i / num_spins
                print('Turntable angle: {} degrees'.format(deg))
                set_turntable_deg(deg)
                if not simulation:
                    print('Capturing photo')
                    file = camera.capture()

                    if file is not None:
                        print('preview: ' + file);
            set_turntable_deg(0.0, False)
        print('progress: {}'.format(int(float(positions.index(_positions) + 1) / len(positions) * 100)))
        rospy.sleep(1)
    camera.exit()
    move_home()
