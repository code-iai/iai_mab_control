#!/usr/bin/env python

from BaseHTTPServer import HTTPServer
from re import match
from SimpleHTTPServer import SimpleHTTPRequestHandler
from simple_websocket_server import WebSocketServer, WebSocket
from threading import Thread

import json
import os
import subprocess
import time

class HTTPHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        path = SimpleHTTPRequestHandler.translate_path(self, path)
        relpath = os.path.relpath(path, os.getcwd())
        fullpath = os.path.join(self.server.base_path, relpath)
        return fullpath

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

                    for param in ['camera_mesh', 'camera_orientation', 'camera_pos', 'camera_size', 'max_velocity', 'photobox_pos', 'photobox_size']:
                        params.append('_{}:={}'.format(param, settings[param]))

                    for param in msg:
                        if param.startswith('_'):
                            params.append('{}:={}'.format(param, msg[param]))

                    process_model = subprocess.Popen(['rosrun', 'iai_mab_control', 'acquisition.py'] + params, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    Thread(target=update_process, args=[process_model]).start()
                elif msg['type'] == 'photogrammetry':
                    if process_photogrammetry is not None:
                        return

                    with open(settings_dir + 'general.json') as f:
                        process_photogrammetry = subprocess.Popen([os.path.expanduser('{}/meshroom_photogrammetry'.format(json.loads(f.read())['meshroom_dir'])), '--input', os.path.expanduser('{}/out/images'.format(msg['workingDir'])), '--output', os.path.expanduser('{}/out/model'.format(msg['workingDir']))], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

                    Thread(target=update_process, args=[process_photogrammetry]).start()

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

process_photogrammetry = None
process_photogrammetry_progress = 0
process_photogrammetry_log = ''
process_photogrammetry_step = None
process_photogrammetry_step_max = None

def broadcast(op, msg, sender=None):
    for client in clients:
        client._send(op, msg)

def update_process(process):
    global process_model, process_model_log, process_photogrammetry, process_photogrammetry_log

    if process == process_model:
        while process.poll() is None:
            stdout = process.stdout.readline()

            if len(stdout) == 0:
                continue

            if stdout.startswith('progress:'):
                process_model_progress = stdout.rstrip().split()[1]
                broadcast('PROGRESS', { 'type': 'model', 'value': process_model_progress })
            else:
                process_model_log += stdout
                broadcast('CONSOLE', { 'type': 'model', 'text': stdout })

        process_model = None
        process_model_progress = 0
        process_model_log = ''
        broadcast('STOP', 'model')
    elif process == process_photogrammetry:
        while process.poll() is None:
            stdout = process.stdout.readline()

            if len(stdout) == 0:
                continue

            if match('\[\d+/\d+\] *', stdout):
                process_photogrammetry_step = int(stdout[1:stdout.index('/')])
                process_photogrammetry_step_max = float(stdout[stdout.index('/') + 1:stdout.index(']')])
                process_photogrammetry_progress = str(int(process_photogrammetry_step - 1 / process_photogrammetry_step_max * 100))
                broadcast('PROGRESS', { 'type': 'photogrammetry', 'value': process_photogrammetry_progress })
            elif stdout.startswith(' - elapsed time: '):
                if process_photogrammetry_step is not None and process_photogrammetry_step_max is not None:
                    process_photogrammetry_progress = str(int(process_photogrammetry_step / process_photogrammetry_step_max * 100))
                    broadcast('PROGRESS', { 'type': 'photogrammetry', 'value': process_photogrammetry_progress })
            else:
                process_photogrammetry_log += stdout
                broadcast('CONSOLE', { 'type': 'photogrammetry', 'text': stdout })

        process_photogrammetry = None
        process_photogrammetry_progress = 0
        process_photogrammetry_log = ''
        broadcast('STOP', 'photogrammetry')

http_thread = Thread(target=lambda : MyHTTPServer(8000).serve_forever()) # TODO add optional parameter to set port
http_thread.daemon = True
http_thread.start()

socket_thread = Thread(target=lambda : WebSocketServer('', 8080, WebSocketHandler).serve_forever()) # TODO add optional parameter to set port
socket_thread.daemon = True
socket_thread.start()

while True:
    time.sleep(1)
