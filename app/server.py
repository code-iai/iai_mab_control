#!/usr/bin/env python

from BaseHTTPServer import HTTPServer
from re import match
from SimpleHTTPServer import SimpleHTTPRequestHandler
from simple_websocket_server import WebSocketServer, WebSocket
from threading import Thread

import json
import os
import subprocess
import threading
import time

class HTTPHandler(SimpleHTTPRequestHandler):
    process_model = None
    process_photogrammetry = None
    process_photogrammetry_step = None
    process_photogrammetry_step_max = None

    def translate_path(self, path):
        path = SimpleHTTPRequestHandler.translate_path(self, path)
        relpath = os.path.relpath(path, os.getcwd())
        fullpath = os.path.join(self.server.base_path, relpath)
        return fullpath

    def _send_error(self, code):
        self.send_error({
            'BAD_REQUEST': 400
        ,   'NOT_FOUND': 404
        }.get(code, code))

    def _send_ok(self, body='', type='text/html'):
        self.send_response(200)
        self.send_header('Content-Length', len(body))

        self.send_header('Content-Type', {
            'JSON': 'application/json'
        }.get(type, type) + ';charset=utf-8')

        self.end_headers()
        self.wfile.write(body.encode('utf8'))

    def _update_process(self, process):
        while process.poll() is None:
            stdout = process.stdout.readline()
            if stdout.startswith('progress:') and process == self.process_model:
                progress = stdout.rstrip().split()[1]
                print(progress + '%') # TODO: send progress to model acquisition progress websocket
            elif match('\[\d+/\d+\] *', stdout) and process == self.process_photogrammetry:
                self.process_photogrammetry_step = int(stdout[1:stdout.index('/')])
                self.process_photogrammetry_step_max = float(stdout[stdout.index('/') + 1:stdout.index(']')])
                progress = str(int(self.process_photogrammetry_step - 1 / self.process_photogrammetry_step_max * 100))
                print(progress + '%') # TODO: send progress to photogrammetry progress websocket
            elif self.process_photogrammetry_step is not None and self.process_photogrammetry_step_max is not None and stdout.startswith(' - elapsed time: ') and process == self.process_photogrammetry:
                progress = str(int(self.process_photogrammetry_step / self.process_photogrammetry_step_max * 100))
                print(progress + '%') # TODO: send progress to photogrammetry progress websocket
            else:
                if process == self.process_model:
                    print(stdout) # TODO: send stdout to model acquisition log websocket
                elif process == self.process_photogrammetry:
                    print(stdout) # TODO: send stdout to photogrammetry log websocket
        # TODO: inform client about process termination

    def do_POST(self):
        if self.path.startswith('/api/'):
            try:
                length = int(self.headers['Content-Length'])

                if length:
                    request = json.loads(self.rfile.read(length))
                else:
                    request = {}
            except:
                self._send_error('BAD_REQUEST')
                return

            path = self.path[5:]

            if path == 'start':
                if 'working_dir' in request and 'type' in request and request['type'] == 'model' and (self.process_model is None or self.process_model.poll() is not None):
                    params = ['_output_directory:={}/out/images'.format(request['working_dir'])]
                    for key in request:
                        if key.startswith('_'):
                            params.append('{}:={}'.format(key, request[key]))

                    self.process_model = subprocess.Popen(['rosrun', 'iai_mab_control', 'acquisition.py'] + params, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    threading.Thread(target=self._update_process, args=[self.process_model]).start()
                    self._send_ok()
                elif 'working_dir' in request and 'meshroom_dir' in request and 'type' in request and request['type'] == 'photogrammetry' and (process_photogrammetry is None or process_photogrammetry.poll() is not None):
                    self.process_photogrammetry = subprocess.Popen([request['meshroom_dir'], '--input {}/out/images'.format(request['working_dir']), '--output {}/out/model'.format(request['working_dir'])], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    threading.Thread(target=self._update_process, args=[self.process_photogrammetry]).start()
                    self._send_ok()
                else:
                    self._send_error('BAD_REQUEST')
            elif path == 'cancel':
                if 'type' in request and request['type'] == 'model' and process_model is not None and process_model.poll() is None:
                    process_model.kill()
                    self._send_ok()
                elif 'type' in request and request['type'] == 'photogrammetry' and process_photogrammetry is not None and process_photogrammetry.poll() is None:
                    process_photogrammetry.kill()
                    self._send_ok()
                else:
                    self._send_error('BAD_REQUEST')
            else:
                self._send_error('NOT_FOUND')
        else:
            self._send_error('NOT_FOUND')

class MyHTTPServer(HTTPServer):
    def __init__(self, port = 8000, path = 'htdocs'):
        self.base_path = os.path.join(os.path.dirname(__file__), path)
        HTTPServer.__init__(self, ('', port), HTTPHandler)

class WebSocketHandler(WebSocket):
    def _send(self, op, msg=None):
        self.send_message(json.dumps({ 'op': op, 'msg': msg }).decode('utf-8'))

    def connected(self):
        self.authenticated = False

    def handle(self):
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

                    self._send('AUTH', settings)
                else:
                    self._send('AUTH')
        else:
            if not self.authenticated:
                return

            if op == 'DELETE':
                if 'type' in msg and (msg['type'] == 'model' or msg['type'] == 'photogrammetry') and 'name' in msg:
                    with open(settings_dir + msg['type'] + '.json') as f:
                        settings = json.loads(f.read())

                    settings.pop(msg['name'], None)

                    with open(settings_dir + msg['type'] + '.json', 'w') as f:
                        f.write(json.dumps(settings))

                    self._send('DELETE', { 'type': msg['type'], 'name': msg['name'] })
            elif op == 'SAVE':
                if 'type' in msg and (msg['type'] == 'model' or msg['type'] == 'photogrammetry') and 'name' in msg and isinstance(msg['name'], basestring) and len(msg['name']) > 0 and 'params' in msg:
                    with open(settings_dir + msg['type'] + '.json') as f:
                        settings = json.loads(f.read())

                    settings[msg['name']] = msg['params']

                    with open(settings_dir + msg['type'] + '.json', 'w') as f:
                        f.write(json.dumps(settings))

                    self._send('SAVE', { 'type': msg['type'], 'name': msg['name'], 'params': msg['params'] })

app_dir = os.path.dirname(os.path.abspath(__file__))
settings_dir = app_dir + '/settings/'

http_thread = Thread(target=lambda : MyHTTPServer().serve_forever())
http_thread.daemon = True
http_thread.start()

socket_thread = Thread(target=lambda : WebSocketServer('', 8080, WebSocketHandler).serve_forever())
socket_thread.daemon = True
socket_thread.start()

while True:
    time.sleep(1)
