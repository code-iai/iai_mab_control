#!/usr/bin/env python

from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler

import json
import os
import subprocess
import threading

class HTTPHandler(SimpleHTTPRequestHandler):
    process_model = None
    process_photogrammetry = None

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
            if stdout.startswith('progress:'):
                progress = stdout.rstrip().split()[1]
                if process == self.process_model:
                    print(progress + '%') # TODO: send progress to model acquisition progress websocket
                elif process == self.process_photogrammetry:
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
                elif 'working_dir' in request and 'meshroom_bin_dir' in request and 'type' in request and request['type'] == 'photogrammetry' and (process_photogrammetry is None or process_photogrammetry.poll() is not None):
                    self.process_photogrammetry = subprocess.Popen(['python', app_dir + '../scripts/meshroom_cli.py', request['meshroom_bin_dir'], request['working_dir'] + '/out/model', request['working_dir'] + '/out/images'])
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
            elif path == 'loadSettings':
                response = {}

                with open(app_dir + '/settings/environment.json') as f:
                    response['environment'] = { 'settings': json.loads(f.read()), 'path': os.path.abspath(f.name) }

                with open(app_dir + '/settings/model.json') as f:
                    response['model'] = json.loads(f.read())

                with open(app_dir + '/settings/photogrammetry.json') as f:
                    response['photogrammetry'] = json.loads(f.read())

                self._send_ok(json.dumps(response), 'JSON')
            elif path == 'deleteSettings':
                if 'type' in request and (request['type'] == 'model' or request['type'] == 'photogrammetry') and 'name' in request:
                    with open(app_dir + '/settings/' + request['type'] + '.json') as f:
                        settings = json.loads(f.read())

                    settings.pop(request['name'], None)

                    with open(app_dir + '/settings/' + request['type'] + '.json', 'w') as f:
                        f.write(json.dumps(settings))

                    self._send_ok()
                else:
                    self._send_error('BAD_REQUEST')
            elif path == 'saveSettings':
                if 'type' in request and 'name' in request and 'settings' in request:
                    if request['type'] == 'model' and 'camera_distance' in request['settings'] and 'num_positions' in request['settings'] and 'num_spins' in request['settings']:
                        with open(app_dir + '/settings/model.json') as f:
                            settings = json.loads(f.read())

                        settings[request['name']] = {
                            'camera_distance': request['settings']['camera_distance']
                        ,   'num_positions': request['settings']['num_positions']
                        ,   'num_spins': request['settings']['num_spins']
                        }

                        with open(app_dir + '/settings/model.json', 'w') as f:
                            f.write(json.dumps(settings))

                        self._send_ok()
                    elif request['type'] == 'photogrammetry' and 'max_num_triangles' in request['settings'] and 'max_num_vertices' in request['settings']:
                        with open(app_dir + '/settings/photogrammetry.json') as f:
                            settings = json.loads(f.read())

                        settings[request['name']] = {
                            'max_num_triangles': request['settings']['max_num_triangles']
                        ,   'max_num_vertices': request['settings']['max_num_vertices']
                        }

                        with open(app_dir + '/settings/photogrammetry.json', 'w') as f:
                            f.write(json.dumps(settings))

                        self._send_ok()
                    else:
                        self._send_error('BAD_REQUEST')
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

app_dir = os.path.dirname(os.path.abspath(__file__))
httpd = MyHTTPServer()

try:
    httpd.serve_forever()
except KeyboardInterrupt:
    pass

httpd.server_close()

