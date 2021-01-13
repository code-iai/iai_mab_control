from time import time

import sys
import os
import math

bin_dir = sys.argv[1]
model_dir = sys.argv[2]
images_dir = sys.argv[3]
verbose_level = 'error'

def run_01_camera_init():
    print('01/13 CAMERA INITIALIZATION')
    task_dir = '01_camera_init'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    sensor_db = bin_dir + '/../share/aliceVision/cameraSensors.db'

    output = '{}/{}/cameraInit.sfm'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_cameraInit.exe'
    cmd += ' --imageFolder {} --sensorDatabase {} --output {}'.format(images_dir, sensor_db, output)
    cmd += ' --defaultFieldOfView 45'
    cmd += ' --allowSingleView 1'
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_02_feature_extraction(num_images, num_images_per_group=40):
    print('02/13 FEATURE EXTRACTION')
    task_dir = '02_feature_extraction'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/01_camera_init/cameraInit.sfm'
    output = '{}/{}'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_featureExtraction'
    cmd += ' --input {} --output {}'.format(_input, output)
    cmd += ' --forceCpuExtraction 1'


    #when there are more than 40 images, it is good to send them in groups
    if(num_images > num_images_per_group):
        num_groups = int(math.ceil(num_images / num_images_per_group))
        for i in range(num_groups):
            _cmd = cmd + ' --rangeStart {} --rangeSize {}'.format(i * num_images_per_group, num_images_per_group)
            print('group {}/{}'.format(i + 1, num_groups))
            print(_cmd)
            os.system(_cmd)
    else:
        print(cmd)
        os.system(cmd)

def run_03_image_matching():
    print('03/13 IMAGE MATCHING')
    task_dir = '03_image_matching'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/01_camera_init/cameraInit.sfm'
    features = model_dir + '/02_feature_extraction'
    output = '{}/{}/imageMatches.txt'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_imageMatching.exe'
    cmd += ' --input {} --featuresFolders {} --output {}'.format(_input, features, output)
    cmd += ' --tree {}/../share/aliceVision/vlfeat_K80L3.SIFT.tree'.format(bin_dir)
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_04_feature_matching(num_images, num_images_per_group=20):
    print('04/13 FEATURE MATCHING')
    task_dir = '04_feature_matching'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/01_camera_init/cameraInit.sfm'
    features = model_dir + '/02_feature_extraction'
    image_pairs = model_dir + '/03_image_matching/imageMatches.txt'
    output = '{}/{}'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_imageMatching.exe'
    cmd += ' --input {} --featuresFolders {} --imagePairsList {} --output {}'.format(_input, features, image_pairs, output)
    cmd += ' --knownPosesGeometricErrorMax 5'
    cmd += ' --veboseLevel ' + verbose_level
    cmd += ' --describerTypes sift --photometricMatchingMethod ANN_L2 --geometricEstimator acransac --geometricFilterType fundamental_matrix --distanceRatio 0.8'
    cmd += ' --maxIteration 2048 --geometricError 0.0 --maxMatches 0'
    cmd += ' --savePutativeMatches False --guidedMatching False --matchFromKnownCameraPoses False --exportDebugFiles True'

    #when there are more than 20 images, it is good to send them in groups
    if(num_images > num_images_per_group):
        num_groups = math.ceil(num_images / num_images_per_group)
        for i in range(num_groups):
            _cmd = cmd + ' --rangeStart {} --rangeSize {}'.format(i * num_images_per_group, num_images_per_group)
            print('group {}/{}'.format(i, num_groups))
            print(_cmd)
            os.system(_cmd)
    else:
        print(cmd)
        os.system(cmd)

def run_05_structure_from_motion():
    print('05/13 STRUCTURE FROM MOTION')
    task_dir = '05_structure_from_motion'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/01_camera_init/cameraInit.sfm'
    features = model_dir + '/02_feature_extraction'
    matches = model_dir + '/04_feature_matching'
    output = '{}/{}/sfm.abc'.format(model_dir, task_dir)
    views_poses = '{}/{}/cameras.sfm'.format(model_dir, task_dir)
    extra_info = '{}/{}'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_incrementalSfm.exe'
    cmd += ' --input {} --featuresFolders {} --matchesFolders {} --output {} --outputViewsAndPoses {} --extra_info {}'.format(_input, features, matches, output, views_poses, extra_info)
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_06_prepare_dense_scene():
    print('06/13 PREPARE DENSE SCENE')
    task_dir = '06_prepare_dense_scene'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/05_structure_from_motion/sfm.abc'
    output = '{}/{}'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_prepareDenseScene.exe'
    cmd += ' --input {} --output {}'.format(_input, output)
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_07_depth_map(num_images, group_size=6, downscale=2):
    print('07/13 DEPTH MAP')
    task_dir = '07_depth_map'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/05_structure_from_motion/sfm.abc'
    images = model_dir + '/06_prepare_dense_scene'
    output = '{}/{}'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_depthMapEstimation.exe'
    cmd += ' --input {} --imagesFolder {} --output {}'.format(
        _input, images, output)
    cmd += ' --downscale ' + str(downscale)
    cmd += ' --veboseLevel ' + verbose_level

    num_batches = int(math.ceil( num_images / group_size ))
    for i in range(num_batches):
        group_start = group_size * i
        current_group_size = min(group_size, num_images - group_start)
        if group_size > 1:
            print('DepthMap Group {} of {} : {} to {}'.format(i, num_batches, group_start, current_group_size))
            _cmd = cmd + (' --rangeStart {} --rangeSize {}'.format(str(group_start),str(group_size)))
            print(_cmd)
            os.system(_cmd)

def run_08_depth_map_filter():
    print('08/13 DEPTH MAP FILTER')
    task_dir = '08_depth_map_filter'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/05_structure_from_motion/sfm.abc'
    depth_maps = model_dir + '/07_depth_map'
    output = '{}/{}'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_depthMapFiltering.exe'
    cmd += ' --input {} --depthMapsFolder {} --output {}'.format(_input, depth_maps, output)
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_09_meshing(max_input_points=50000000, max_points=1000000):
    print('09/13 MESHING')
    task_dir = '09_meshing'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    _input = model_dir + '/05_structure_from_motion/sfm.abc'
    depth_maps = model_dir + '/08_depth_map_filter'
    output = '{}/{}/densePointCloud.abc'.format(model_dir, task_dir)
    output_mesh = '{}/{}/mesh.obj'.format(model_dir, task_dir)

    cmd = bin_dir + '\\aliceVision_meshing.exe'
    cmd += ' --input {} --depthMapsFolder {} --output {} --outputMesh {}'.format(_input, depth_maps, output, output_mesh)
    cmd += ' --max_input_points ' + str(max_input_points)
    cmd += ' --max_points ' + str(max_points)
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_10_mesh_filtering(keep_largest_mesh_only='True'):
    print('10/13 MESH FILTERING')
    task_dir = '10_mesh_filtering'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    input_mesh = model_dir + '/09_meshing/mesh.obj'
    output_mesh = '{}/{}/mesh.obj'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_meshFiltering.exe'
    cmd += ' --inputMesh {} --outputMesh {}'.format(input_mesh, output_mesh)
    cmd += ' --keep_largest_mesh_only' + keep_largest_mesh_only

    print(cmd)
    os.system(cmd)

def run_11_mesh_decimate(simplification_factor=0.8, max_vertices=15000):
    print('11/13 MESH DECIMATE')
    task_dir = '11_mesh_decimate'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    input_mesh = model_dir + '/10_mesh_filtering/mesh.obj'
    output_mesh = '{}/{}/mesh.obj'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_meshDecimate.exe'
    cmd += ' --input {} --output {}'.format(input_mesh, output_mesh)
    cmd += ' --simplification_factor ' + str(simplification_factor)
    cmd += ' --max_vertices ' + str(max_vertices)
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_12_mesh_resampling(simplification_factor=0.8, max_vertices=15000):
    print('12/13 MESH RESAMPLING')
    task_dir = '12_mesh_resampling'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    input_mesh = model_dir + '/11_mesh_decimate/mesh.obj'
    output_mesh = '{}/{}/mesh.obj'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_meshResampling.exe'
    cmd += ' --input {} --output {}'.format(input_mesh, output_mesh)
    cmd += ' --simplification_factor ' + str(simplification_factor)
    cmd += ' --max_vertices ' + str(max_vertices)
    cmd += ' --veboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)

def run_13_texturing(texture_side=4096, downscale=4, unwrap_method='Basic'):
    print('13/13 TEXTURING')
    task_dir = '13_texturing'
    os.mkdir('{}/{}'.format(model_dir, task_dir))

    images = model_dir + '/06_prepare_dense_scene'
    _input = model_dir + '/09_meshing/densePointCloud.abc'
    input_mesh = model_dir + '/12_mesh_resampling/mesh.obj'
    output = '{}/{}'.format(model_dir, task_dir)

    cmd = bin_dir + '/aliceVision_texturing.exe'
    cmd += ' --imagesFolder {} --input {} --inputMesh {} --output {}'.format(images, _input, input_mesh, output)
    cmd += ' --textureSide ' + str(texture_side)
    cmd += ' --downscale ' + str(downscale)
    cmd += ' --unwrapMethod ' + unwrap_method
    cmd += ' --verboseLevel ' + verbose_level

    print(cmd)
    os.system(cmd)



def main():
    startTime = time()
    os.mkdir(model_dir)
    num_images = len([name for name in os.listdir(imgDir) if os.path.isfile(os.path.join(imgDir, name))])

    print('progress: {}'.format(0 / 13. * 100))
    run_01_camera_init(bin_dir, model_dir, imgDir)
    print('progress: {}'.format(1 / 13. * 100))
    run_02_feature_extraction(bin_dir, model_dir, num_images)
    print('progress: {}'.format(2 / 13. * 100))
    run_03_image_matching(bin_dir, model_dir)
    print('progress: {}'.format(3 / 13. * 100))
    run_04_feature_matching(bin_dir, model_dir, num_images)
    print('progress: {}'.format(4 / 13. * 100))
    run_05_structure_from_motion(bin_dir, model_dir)
    print('progress: {}'.format(5 / 13. * 100))
    run_06_prepare_dense_scene(bin_dir, model_dir)
    print('progress: {}'.format(6 / 13. * 100))
    run_07_depth_map(bin_dir, model_dir, num_images)
    print('progress: {}'.format(7 / 13. * 100))
    run_08_depth_map_filter(bin_dir, model_dir)
    print('progress: {}'.format(8 / 13. * 100))
    run_09_meshing(bin_dir, model_dir)
    print('progress: {}'.format(9 / 13. * 100))
    run_10_mesh_filtering(bin_dir, model_dir)
    print('progress: {}'.format(10 / 13. * 100))
    run_11_mesh_decimate(bin_dir, model_dir)
    print('progress: {}'.format(11 / 13. * 100))
    run_12_mesh_resampling(bin_dir, model_dir)
    print('progress: {}'.format(12 / 13. * 100))
    run_13_texturing(bin_dir, model_dir)
    print('progress: {}'.format(13 / 13. * 100))

    print('DONE')
    endTime = time()
    hours, rem = divmod(endTime - startTime, 3600)
    minutes, seconds = divmod(rem, 60)
    print('time elapsed: {:0>2}:{:0>2}:{:05.2f}'.format(int(hours),int(minutes),seconds))
    #raw_input('press any key to close')

main()
