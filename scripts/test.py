import os, camera
camera.init(os.getcwd(), '192.168.1.1')
camera.capture()
