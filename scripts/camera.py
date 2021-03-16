from multiprocessing import Manager, Process

import os
import subprocess
import time

def _print(msg, prefix = None):
    if prefix is not None:
        msg = '{0}: {1}'.format(prefix, msg)

    print('[Camera] {0}'.format(msg))

def readline(timeout=60):
    line = Manager().dict({'value': None})

    if _process and _process.poll() is None:
        def _readline(line):
            line['value'] = _process.stdout.readline()

        p = Process(target=_readline, args=[line])
        p.start()
        p.join(timeout)

        if p.is_alive():
            p.terminate()
            p.join()

    return line['value']

def init(output_directory, ptpip=None):
    global _files, _output_directory, _process

    if os.path.exists(output_directory):
        if not os.path.isdir(output_directory):
            _print('Given output directory is no directory', 'Error')
            return False

        _files = [file for file in os.listdir(output_directory)]

        if len(_files) != 0:
            _print('Output directory is not empty, files might get overridden', 'Warning')
    else:
        try:
            os.makedirs(output_directory)
            _files = []
        except:
            _print('Failed to create output directory', 'Error')
            return False

    _output_directory = output_directory

    if os.name == 'posix':
        subprocess.Popen(['pkill', '-f', 'gphoto2']).wait()

    args = ['gphoto2', '--port']
    if ptpip:
        args.append('ptpip:{}'.format(ptpip))
    else:
        args.append('usb:')
    args += ['--summary', '--shell']

    _process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE, env=dict(os.environ, **dict(PYTHONUNBUFFERED='1')), cwd=_output_directory)
    line = readline()

    if line is None or line.startswith('connect cmd:') or _process.poll() is not None:
        _print('Could not connect to camera', 'Error')
        return False

    _process.stdin.write('set-config capturetarget=Memory card\n')

    return True

def capture():
    _process.stdin.write('capture-image-and-download\n')
    line = readline()

    while line is not None:
        for file in os.listdir(_output_directory):
            if 'tmp' not in file and file not in _files:
                _files.append(file)
                return os.path.join(_output_directory, file)

        line = readline()

    _print('Capture failed', 'Error')
    return None

