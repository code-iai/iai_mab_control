import os
import subprocess


def _print(msg, prefix = None):
    if prefix is not None:
        msg = '{0}: {1}'.format(prefix, msg)

    print('[Camera] {0}'.format(msg))

def init(output_directory, ptpip=None):
    global _output_directory, _process

    if os.path.exists(output_directory):
        if not os.path.isdir(output_directory):
            _print('Given output directory is no directory', 'Error')
            return False

        if len(os.listdir(output_directory)) != 0:
            _print('Output directory is not empty, files might get overridden', 'Warning')
    else:
        try:
            os.makedirs(output_directory)
        except:
            _print('Failed to create output directory', 'Error')
            return False

    _output_directory = output_directory

    if os.name == 'posix':
        subprocess.Popen(['pkill', '-f', 'gphoto2']).wait()

    args = ['gphoto2']
    if ptpip:
        args += ['--port', 'ptpip:{}'.format(ptpip)]
    args += ['--summary', '--shell']

    _process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE, env=dict(os.environ, **dict(PYTHONUNBUFFERED='1')), cwd=_output_directory)
    line = _process.stdout.readline()

    if line.startswith('connect cmd: No route to host'):
        _print(line, 'Error')
        return False

    return True

def capture():
    _process.stdin.write('capture-image-and-download\n')
    while _process.poll() is None:
        line = _process.stdout.readline()

        if line.startswith('connect cmd: No route to host'):
            _print(line, 'Error')
            return None

        if line.startswith('Saving file as'):
            return os.path.join(_output_directory, line.split(' ')[-1][:-1])

