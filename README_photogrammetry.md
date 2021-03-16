# Photogrammetry

## Requirements & Dependencies

The photogrammetry application is developed in Python 2 on top of [Meshroom](https://alicevision.org/#meshroom).

First **Meshroom by AliceVision** needs to be downloaded as it is used by the application to create 3D models from the images. The most recent version can be downloaded from https://github.com/alicevision/meshroom/releases. We used version 2019.2.0 on a Windows computer.

The Graphical User Interface uses WebSockets for communication between frontend and backend. For this the backend uses the **simple-websocket-server** which can be installed using _pip_.

```
$ pip install simple-websocket-server
```

## Installation

Once all requirements are taken care of simply clone the repository.

```
$ git clone -b devel https://github.com/code-iai/iai_mab_control.git
```

## Configuration

While some settings can be configured in the Graphical User Interface, some more general ones are fixed and have to be changed in _app/photogrammetry/settings/general.json_.

| Parameter | Description | Example |
|-|-|-|
| meshroom_dir | Path of the Meshroom application. | C:\\\\Meshroom |
| password | A password to access the application. Optional. | 654321 |

## Graphical User Interface (GUI)

To use the GUI the server of the application needs to be started first.

```
$ python app\\photogrammetry\\server.py
```

After starting the server any web browser can be used to access the GUI by browsing to http://MACHINE_IP_ADDRESS:9000 where _MACHINE_IP_ADDRESS_ is the ip address of the machine the server is running on. To access the GUI on the same machine as the server is running on, you can visit http://localhost:9000.

While the default ports for HTTP and WebSocket are set to 9000 and 9090, the optional arguments _--http_port_ and _--socket_port_ can be passed to change them.

```
$ python app\\photogrammetry\\server.py --http_port 9000 --socket_port 9090
```
