#!/usr/bin/env python

from BaseHTTPServer import HTTPServer
from re import match
from SimpleHTTPServer import SimpleHTTPRequestHandler
from simple_websocket_server import WebSocketServer, WebSocket
from threading import Thread

import base64
import getopt
import json
import os
import subprocess
import sys
import time

if os.name == 'posix':
    from pty import openpty

class HTTPHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        path = SimpleHTTPRequestHandler.translate_path(self, path)
        relpath = os.path.relpath(path, os.getcwd())
        fullpath = os.path.join(self.server.base_path, relpath)
        return fullpath

    def do_POST(self):
        if self.path.startswith('/port'):
            body = str(socket_port)
            self.send_response(200)
            self.send_header('Content-Length', len(body))
            self.send_header('Content-Type', 'text/plain;charset=utf-8');
            self.end_headers()
            self.wfile.write(body.encode('utf-8'))
        else:
            self.send_error(404);

class MyHTTPServer(HTTPServer):
    def __init__(self, port, path = 'htdocs'):
        self.base_path = os.path.join(os.path.dirname(__file__), path)
        HTTPServer.__init__(self, ('', port), HTTPHandler)

class WebSocketHandler(WebSocket):
    authenticated = False

    def _send(self, op, msg):
        self.send_message(json.dumps({ 'op': op, 'msg': msg }).decode('utf-8'))

    def connected(self):
        clients.append(self)

    def handle_close(self):
        clients.remove(self)

    def handle(self):
        global process_model, process_photogrammetry

        try:
            data = json.loads(self.data)
        except:
            return

        if 'op' not in data or 'msg' not in data:
            return

        op = data['op']
        msg = data['msg']

        if op == 'AUTH':
            if self.authenticated:
                return

            with open(settings_dir + 'general.json') as f:
                if msg == json.loads(f.read())['password']:
                    self.authenticated = True
                    settings = {}

                    with open(settings_dir + 'model.json') as f:
                        settings['model'] = json.loads(f.read())

                    with open(settings_dir + 'photogrammetry.json') as f:
                        settings['photogrammetry'] = json.loads(f.read())

                    self._send('AUTH_RESPONSE', settings)

                    if process_model is not None:
                        self._send('START', 'model')
                        self._send('PROGRESS', { 'type': 'model', 'value': process_model_progress })
                        self._send('CONSOLE', { 'type': 'model', 'text': process_model_log })

                        if process_model_preview is not None:
                            self._send('PREVIEW', { 'data': process_model_preview })

                    if process_photogrammetry is not None:
                        self._send('START', 'photogrammetry')
                        self._send('PROGRESS', { 'type': 'photogrammetry', 'value': photogrammetry_model_progress })
                        self._send('CONSOLE', { 'type': 'photogrammetry', 'text': photogrammetry_model_log })
                else:
                    self._send('AUTH_RESPONSE', None)
        else:
            if not self.authenticated:
                return

            if op == 'DELETE':
                with open(settings_dir + msg['type'] + '.json') as f:
                    settings = json.loads(f.read())

                settings.pop(msg['name'], None)

                with open(settings_dir + msg['type'] + '.json', 'w') as f:
                    f.write(json.dumps(settings))
            elif op == 'SAVE':
                with open(settings_dir + msg['type'] + '.json') as f:
                    settings = json.loads(f.read())

                settings[msg['name']] = msg['params']

                with open(settings_dir + msg['type'] + '.json', 'w') as f:
                    f.write(json.dumps(settings))
            elif op == 'START':
                if msg['type'] == 'model':
                    if process_model is not None:
                        return

                    params = ['_output_directory:={}/out/images'.format(os.path.expanduser(msg['workingDir']))]

                    with open(settings_dir + 'general.json') as f:
                        settings = json.loads(f.read())

                    for param in ['camera_pos', 'camera_size', 'max_velocity', 'photobox_pos', 'photobox_size']:
                        params.append('_{}:={}'.format(param, settings[param]))

                    for param in msg:
                        if param.startswith('_'):
                            params.append('{}:={}'.format(param, msg[param]))

                    if os.name == 'posix':
                        master, slave = openpty()
                        stdout, stderr = slave, slave
                        env = None
                    else:
                        master, slave = None, None
                        stdout, stderr = subprocess.PIPE, subprocess.STDOUT
                        env = dict(os.environ, **dict(PYTHONUNBUFFERED='1'))

                    process_model = subprocess.Popen(['rosrun', 'iai_mab_control', 'acquisition.py'] + params, stdout=slave, stderr=slave, env=env)
                    monitor(process_model, os.fdopen(master), os.fdopen(slave, 'w', 0))
                elif msg['type'] == 'photogrammetry':
                    if process_photogrammetry is not None:
                        return

                    if os.name == 'posix':
                        master, slave = openpty()
                        stdout, stderr = slave, slave
                        env = None
                    else:
                        master, slave = None, None
                        stdout, stderr = subprocess.PIPE, subprocess.STDOUT
                        env = dict(os.environ, **dict(PYTHONUNBUFFERED='1'))

                    with open(settings_dir + 'general.json') as f:
                        settings = json.loads(f.read())

                        with open(settings_dir + 'meshroom/pipeline.mg') as f_pipeline:
                            pipeline = json.loads(f_pipeline.read())
                        with open(settings_dir + 'meshroom/pipeline.mg', 'w') as f_pipeline:
                            pipeline['graph']['CameraInit_1']['inputs']['sensorDatabase'] = settings['meshroom_dir'] + '/aliceVision/share/aliceVision/cameraSensors.db'
                            pipeline['graph']['ImageMatching_1']['inputs']['tree'] = settings['meshroom_dir'] + '/aliceVision/share/aliceVision/vlfeat_K80L3.SIFT.tree'
                            f_pipeline.write(json.dumps(pipeline))

                        with open(settings_dir + 'meshroom/overrides.json', 'w+') as f_overrides:
                            overrides = {
                                'MeshDecimate_1': {
                                    'maxVertices': msg['maxNumberVertices']
                                }
                            }
                            f_overrides.write(json.dumps(overrides))

                        process_photogrammetry = subprocess.Popen([
                            os.path.expanduser('{}/meshroom_photogrammetry'.format(settings['meshroom_dir'])),
                            '--input', os.path.expanduser('{}/out/images'.format(msg['workingDir'])),
                            '--output', os.path.expanduser('{}/out/model'.format(msg['workingDir'])),
                            '--pipeline', settings_dir + 'meshroom/pipeline.mg',
                            '--overrides', settings_dir + 'meshroom/overrides.json'
                        ], stdout=stdout, stderr=stderr, env=env)

                    monitor(process_photogrammetry, master, slave)

                broadcast('START', msg['type'])
            elif op == 'STOP':
                if msg == 'model' and process_model is not None:
                    process_model.kill()
                elif msg == 'photogrammetry' and process_photogrammetry is not None:
                    process_photogrammetry.kill()

settings_dir = os.path.dirname(os.path.abspath(__file__)) + '/settings/'

clients = []

process_model = None
process_model_progress = 0
process_model_log = ''
process_model_preview = None

process_photogrammetry = None
process_photogrammetry_progress = 0
process_photogrammetry_log = ''
process_photogrammetry_step = None
process_photogrammetry_step_max = None

def broadcast(op, msg, sender=None):
    for client in clients:
        client._send(op, msg)

def monitor(process, master, slave):
    def killer():
        process.wait()
        slave.write('\n')

    def updater():
        global process_model, process_photogrammetry

        if process == process_model:
            global process_model_progress, process_model_log, process_model_preview

            while process.poll() is None:
                line = master.readline()

                if len(line) == 0:
                    continue

                if line.startswith('progress:'):
                    process_model_progress = line.rstrip().split()[1]
                    broadcast('PROGRESS', { 'type': 'model', 'value': process_model_progress })
                elif line.startswith('preview:'):
                    with open(line.rstrip().split()[1], 'rb') as f:
                        process_model_preview = base64.b64encode(f.read()).decode('utf-8')

                    broadcast('PREVIEW', { 'data': process_model_preview })
                else:
                    process_model_log += line
                    broadcast('CONSOLE', { 'type': 'model', 'text': line })

            process_model = None
            process_model_progress = 0
            process_model_log = ''
            process_model_preview = None
            broadcast('STOP', 'model')
        elif process == process_photogrammetry:
            global process_photogrammetry_progress, process_photogrammetry_log, process_photogrammetry_step, process_photogrammetry_step_max

            while process.poll() is None:
                line = master.readline()

                if len(line) == 0:
                    continue

                if match('\[\d+/\d+\] *', line):
                    process_photogrammetry_step = int(line[1:line.index('/')])
                    process_photogrammetry_step_max = float(line[line.index('/') + 1:line.index(']')])
                    process_photogrammetry_progress = str(int((process_photogrammetry_step - 1) / process_photogrammetry_step_max * 100))
                    broadcast('PROGRESS', { 'type': 'photogrammetry', 'value': process_photogrammetry_progress })
                elif line.startswith(' - elapsed time: '):
                    if process_photogrammetry_step is not None and process_photogrammetry_step_max is not None:
                        process_photogrammetry_progress = str(int(process_photogrammetry_step / process_photogrammetry_step_max * 100))
                        broadcast('PROGRESS', { 'type': 'photogrammetry', 'value': process_photogrammetry_progress })

                process_photogrammetry_log += line
                broadcast('CONSOLE', { 'type': 'photogrammetry', 'text': line })

            # TODO convert to file formats: fbx, stl, dae
            process_photogrammetry = None
            process_photogrammetry_progress = 0
            process_photogrammetry_log = ''
            process_photogrammetry_step = None
            process_photogrammetry_step_max = None
            broadcast('STOP', 'photogrammetry')

    if os.name == 'posix':
        master = os.fdopen(master)
        slave = os.fdopen(slave, 'w', 0)
        Thread(target=killer).start()
    else:
        master = process.stdout

    Thread(target=updater).start()

http_port = 8000
socket_port = 8080
opts, args = getopt.getopt(sys.argv[1:], '', ['http_port=', 'socket_port='])

for opt, arg in opts:
    if opt == '--http_port':
        http_port = int(arg)
    elif opt == '--socket_port':
        socket_port = int(arg)

http_thread = Thread(target=lambda : MyHTTPServer(http_port).serve_forever())
http_thread.daemon = True
http_thread.start()
print('HTTP listening on {}'.format(http_port))

socket_thread = Thread(target=lambda : WebSocketServer('', socket_port, WebSocketHandler).serve_forever())
socket_thread.daemon = True
socket_thread.start()
print('WebSocket listening on {}'.format(socket_port))

while True:
    time.sleep(1)
