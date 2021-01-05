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
                with open('settings.json') as f:
                    self._send_ok(f.read(), 'JSON')
            elif path == 'saveSettings':
                # TODO save settings (stored in request)
                self._send_ok()
            else:
                self._send_error('NOT_FOUND')
        else:
            self._send_error('NOT_FOUND')

class MyHTTPServer(HTTPServer):
    def __init__(self, port = 8000, path = 'htdocs'):
        self.base_path = os.path.join(os.path.dirname(__file__), path)
        HTTPServer.__init__(self, ('', port), HTTPHandler)

httpd = MyHTTPServer()

try:
    httpd.serve_forever()
except KeyboardInterrupt:
    pass

httpd.server_close()
