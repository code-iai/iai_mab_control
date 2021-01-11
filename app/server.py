#!/usr/bin/env python

from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler

import json
import os

class HTTPHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        path = SimpleHTTPRequestHandler.translate_path(self, path)
        relpath = os.path.relpath(path, os.getcwd())
        fullpath = os.path.join(self.server.base_path, relpath)
        return fullpath

    def _send_error(self, code, message=None, explain=None):
        self.send_error({
            'BAD_REQUEST': 400
        ,   'NOT_FOUND': 404
        }.get(code, code), message, explain)

    def _send_ok(self, body='', type='text/html'):
        self.send_response(200)
        self.send_header('Content-Length', len(body))

        self.send_header('Content-Type', {
            'JSON': 'application/json'
        }.get(type, type) + ';charset=utf-8')

        self.end_headers()
        self.wfile.write(body.encode('utf8'))

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

            if path == 'loadSettings':
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

