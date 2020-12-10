import gphoto2 as gp
import os
import time

_camera = None

def _print(msg, prefix = None):
    if prefix is not None:
        msg = '{0}: {1}'.format(prefix, msg)

    print('[Camera] {0}'.format(msg))

def init(output_directory, retry = True):
    global _output_directory, _camera

    if os.path.exists(output_directory):
        if not os.path.isdir(output_directory):
            _print('Given output directory is no directory', 'Error')
            return

        if len(os.listdir(output_directory)) != 0:
            _print('Output directory is not empty, files might get overridden', 'Warning')
    else:
        try:
            os.mkdir(output_directory)
        except:
            _print('Failed to create output directory', 'Error')
            return

    _output_directory = output_directory
    camera = gp.Camera()

    while True:
        _print('Initializing...')

        try:
            camera.init()
            _camera = camera
            break
        except gp.GPhoto2Error as err:
            _print(err, 'Error')

            if wait:
                time.sleep(2)
            else:
                break

def capture():
    file_name = None

    if _camera is None:
        _print('Not initialized', 'Error')
    else:
        try:
            camera_capture = _camera.capture(gp.GP_CAPTURE_IMAGE)
            camera_file = _camera.file_get(camera_capture.folder, camera_capture.name, gp.GP_FILE_TYPE_NORMAL)
            camera_file.save(os.path.join(_output_directory, camera_capture.name))
            file_name = camera_capture.name
        except gp.GPhoto2Error as err:
            _print(err, 'Error')

    return file_name

def exit():
    if _camera is not None:
        try:
            _camera.exit()
        except gp.GPhoto2Error as err:
            _print(err, 'Error')
