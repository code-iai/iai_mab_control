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
        if self.path == '/init':
            with open(os.path.join(settings_dir, 'general.json')) as f:
                body = str(socket_port) + ',' + ('t' if len(json.loads(f.read())['password']) else 'f')

            self.send_response(200)
            self.send_header('Content-Length', len(body))
            self.send_header('Content-Type', 'text/plain;charset=utf-8');
            self.end_headers()
            self.wfile.write(body.encode('utf-8'))
        elif self.path == '/save':
            length = int(self.headers.getheader('Content-Length', 0))
            body = self.rfile.read(length)

            try:
                request = json.loads(body)
                password = request['password']
            except:
                self.send_error(400)
                return

            with open(os.path.join(settings_dir, 'general.json')) as f:
                if password != json.loads(f.read())['password']:
                    self.send_error(400)
                    return

            working_dir = os.path.join(app_dir, '..', '..', 'out', request['workingDir'], 'images')

            if os.path.exists(working_dir):
                if not os.path.isdir(working_dir): # not a directory
                    self.send_error(400)
                    return
            else:
                try:
                    os.makedirs(working_dir)
                except:
                    self.send_error(400)
                    return

            with open(os.path.join(working_dir, request['fileName']), 'wb') as f:
                f.write(base64.b64decode(request['data']))

            self.send_response(200)
            self.end_headers()
        else:
            self.send_error(404);

class MyHTTPServer(HTTPServer):
    def __init__(self, server_address, path = 'htdocs'):
        self.base_path = os.path.join(os.path.dirname(__file__), path)
        HTTPServer.__init__(self, server_address, HTTPHandler)

class WebSocketHandler(WebSocket):
    authenticated = False

    def _send(self, op, msg):
        self.send_message(json.dumps({ 'op': op, 'msg': msg }).decode('utf-8'))

    def connected(self):
        clients.append(self)

    def handle_close(self):
        clients.remove(self)

    def handle(self):
        global process

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

            with open(os.path.join(settings_dir, 'general.json')) as f:
                if msg == json.loads(f.read())['password']:
                    self.authenticated = True

                    with open(os.path.join(settings_dir, 'saved.json')) as f:
                        saved = json.loads(f.read())

                    self._send('AUTH_RESPONSE', saved)

                    if process is not None:
                        self._send('START')
                        self._send('PROGRESS', process_progress)
                        self._send('CONSOLE', process_log)
                else:
                    self._send('AUTH_RESPONSE', None)
        else:
            if not self.authenticated:
                return

            if op == 'DELETE':
                with open(os.path.join(settings_dir, 'saved.json')) as f:
                    saved = json.loads(f.read())

                saved.pop(msg, None)

                with open(os.path.join(settings_dir, 'saved.json'), 'w') as f:
                    f.write(json.dumps(saved))
            elif op == 'SAVE':
                with open(os.path.join(settings_dir, 'saved.json')) as f:
                    saved = json.loads(f.read())

                saved[msg['name']] = msg['params']

                with open(os.path.join(settings_dir, 'saved.json'), 'w') as f:
                    f.write(json.dumps(saved))
            elif op == 'START':
                if process is not None:
                    return

                with open(os.path.join(settings_dir, 'general.json')) as f:
                    settings = json.loads(f.read())

                msg['workingDir'] = os.path.join(app_dir, '..', '..', 'out', msg['workingDir'])
                settings['meshroom_dir'] = os.path.expanduser(settings['meshroom_dir'])

                tmp_dir = os.path.join(settings_dir, 'meshroom', 'tmp')
                if not os.path.exists(tmp_dir):
                    os.mkdir(tmp_dir)

                with open(os.path.join(settings_dir, 'meshroom', 'pipeline.mg')) as f:
                    pipeline = json.loads(f.read())

                with open(os.path.join(tmp_dir, 'pipeline.mg'), 'w+') as f:
                    pipeline['graph']['CameraInit_1']['inputs']['sensorDatabase'] = os.path.join(settings['meshroom_dir'], 'aliceVision', 'share', 'aliceVision', 'cameraSensors.db')
                    pipeline['graph']['ImageMatching_1']['inputs']['tree'] = os.path.join(settings['meshroom_dir'], 'aliceVision', 'share', 'aliceVision', 'vlfeat_K80L3.SIFT.tree')
                    f.write(json.dumps(pipeline))

                with open(os.path.join(tmp_dir, 'overrides.json'), 'w+') as f:
                    overrides = {
                        'DepthMap_1': {
                            'downscale': int(msg['depthMapDownscaling'])
                        },
                        'MeshFiltering_1': {
                            'keepLargestMeshOnly': msg['keepLargestMeshOnly']
                        },
                        'MeshDecimate_1': {
                            'maxVertices': int(msg['maxNumberVertices'])
                        }
                    }

                    f.write(json.dumps(overrides))

                if os.name == 'posix':
                    master, slave = openpty()
                    stdout, stderr = slave, slave
                    env = None
                else:
                    master, slave = None, None
                    stdout, stderr = subprocess.PIPE, subprocess.STDOUT
                    env = dict(os.environ, **dict(PYTHONUNBUFFERED='1'))

                process = subprocess.Popen([
                    os.path.join(settings['meshroom_dir'], 'meshroom_photogrammetry'),
                    '--input', os.path.join(msg['workingDir'], 'images'),
                    '--output', os.path.join(msg['workingDir'], 'model'),
                    '--pipeline', os.path.join(tmp_dir, 'pipeline.mg'),
                    '--overrides', os.path.join(tmp_dir, 'overrides.json')
                ], stdout=stdout, stderr=stderr, env=env)

                monitor(master, slave)
                broadcast('START')
            elif op == 'STOP':
                if process is not None:
                    process.kill()

app_dir = os.path.dirname(os.path.abspath(__file__))
settings_dir = os.path.join(app_dir, 'settings')

clients = []

process = None
process_progress = 0
process_log = ''
process_step = None
process_step_max = None

def broadcast(op, msg=None):
    for client in clients:
        client._send(op, msg)

def monitor(master, slave):
    def killer():
        process.wait()
        slave.write('\n')

    def updater():
        global process, process_progress, process_log, process_step, process_step_max

        while process.poll() is None:
            line = master.readline()

            if len(line) == 0:
                continue

            if match('\[\d+/\d+\] *', line):
                process_step = int(line[1:line.index('/')])
                process_step_max = float(line[line.index('/') + 1:line.index(']')])
                process_progress = str(int((process_step - 1) / process_step_max * 100))
                broadcast('PROGRESS', process_progress)
            elif line.startswith(' - elapsed time: '):
                if process_step is not None and process_step_max is not None:
                    process_progress = str(int(process_step / process_step_max * 100))
                    broadcast('PROGRESS', process_progress)

            process_log += line
            broadcast('CONSOLE', line)

        process = None
        process_progress = 0
        process_log = ''
        process_step = None
        process_step_max = None
        broadcast('STOP')

    if os.name == 'posix':
        master = os.fdopen(master)
        slave = os.fdopen(slave, 'w', 0)
        Thread(target=killer).start()
    else:
        master = process.stdout

    Thread(target=updater).start()

http_port = 9000
socket_port = 9090
opts, args = getopt.getopt(sys.argv[1:], '', ['http_port=', 'socket_port='])

for opt, arg in opts:
    if opt == '--http_port':
        http_port = int(arg)
    elif opt == '--socket_port':
        socket_port = int(arg)

http_thread = Thread(target=lambda : MyHTTPServer(('', http_port)).serve_forever())
http_thread.daemon = True
http_thread.start()
print('HTTP listening on {}'.format(http_port))

socket_thread = Thread(target=lambda : WebSocketServer('0.0.0.0', socket_port, WebSocketHandler).serve_forever())
socket_thread.daemon = True
socket_thread.start()
print('WebSocket listening on {}'.format(socket_port))

while True:
    time.sleep(1)
