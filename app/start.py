#!/usr/bin/env python

import json
import os
import webview

class Api:
    def __init__(self):
        self.settings_dir = os.path.dirname(os.path.abspath(__file__))

        with open('{}/settings.json'.format(self.settings_dir)) as settings_file:
            self.settings = json.load(settings_file)

    def example(self, params):
        return 'Hello World {}'.format(params)

    def update_settings(self, params):
        with open('{}/settings.json'.format(self.settings_dir), 'w') as settings_file:
            self.settings[params[0]] = params[1]
            json.dump(self.settings, settings_file)

if __name__ == '__main__':
    webview.create_window('Model Acquisition Bot', 'assets/index.html', js_api=Api())

