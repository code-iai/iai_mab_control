# iai_mab_control

Control components for the Model Acquisition Bot (MAB).

MAB is a bot to automatically create a 3D model of an object which was previously put on the turntable in a specific photogrammetry environment.

<p align="center"><img src="https://github.com/code-iai/iai_mab_control/blob/main/assets/overview.png" width="100%"></p>

The whole setup is split into several parts as seen above. These parts are the robot, a camera, a computer running ROS + Python + gPhoto2, a computer running CUDA + Python + Meshroom and a web browser to access the graphical user interface. It is also possible to use one computer only that runs ROS + Python + gPhoto2 + CUDA + Meshroom, which in our case was not possible as the robot we were using required to have a real time linux kernel for which CUDA is not working.

The acquisition machine is controlling the camera and the robot using ROS and gPhoto2. The images taken by the camera are being transfered to the acquisition machine which then sends the images to the photogrammetry machine. The photogrammetry machine uses these images as input for Meshroom to create a 3D model. Both, the acquisition and photogrammetry machines, are running a webserver that can be accessed from any web browser to start the acquisition and photogrammetry processes.

## Authors

- [Augsten, Andy](mailto:a.augsten@uni-bremen.de)
- [Augsten, Dustin](mailto:augsten@uni-bremen.de)

## Acquisition

For setup and usage instructions on the model acquisition part head over to https://github.com/code-iai/iai_mab_control/blob/main/README_acquisition.md

## Photogrammetry

For setup and usage instructions on the photogrammetry part head over to https://github.com/code-iai/iai_mab_control/blob/main/README_photogrammetry.md
