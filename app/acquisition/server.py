#!/usr/bin/env python

from BaseHTTPServer import HTTPServer
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

                        if process_preview is not None:
                            self._send('PREVIEW', process_preview)
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

                params = ['_working_dir:={}'.format(msg['workingDir'])]

                with open(os.path.join(settings_dir, 'general.json')) as f:
                    settings = json.loads(f.read())

                for param in settings:
                    if param is not 'password':
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

                process = subprocess.Popen(['rosrun', 'iai_mab_control', 'acquisition.py'] + params, stdout=stdout, stderr=stderr, env=env)
                monitor(master, slave)
                broadcast('START')
            elif op == 'STOP':
                if process is not None:
                    process.kill()

settings_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'settings')

clients = []

process = None
process_progress = 0
process_log = ''
process_preview = None

def broadcast(op, msg=None):
    for client in clients:
        client._send(op, msg)

def monitor(master, slave):
    def killer():
        process.wait()
        slave.write('\n')

    def updater():
        global process, process_progress, process_log, process_preview

        while process.poll() is None:
            line = master.readline()

            if len(line) == 0:
                continue

            if line.startswith('progress:'):
                process_progress = line.rstrip().split()[1]
                broadcast('PROGRESS', process_progress)
            elif line.startswith('preview:'):
                with open(line.rstrip().split()[1], 'rb') as f:
                    process_preview = base64.b64encode(f.read()).decode('utf-8')

                broadcast('PREVIEW', process_preview)
            elif not line.startswith('Pos = '): # do not print turntable position
                process_log += line
                broadcast('CONSOLE', line)

        process = None
        process_progress = 0
        process_log = ''
        process_preview = None
        broadcast('STOP')

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
