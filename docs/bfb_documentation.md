# Project Folder Structure

```text 
bigfootbot/
│
├── docs/
│   └── bfb_documentation.md
│
├── docker/
│   ├── camera_d435i/
│   │   ├── Dockerfile.bfb_camera_d435i
│   │   ├── docker-compose_camera_d435i.yml
│   │
│   ├── web/
│   │   └── webrtc/
│   │       ├── app/
│   │           ├── public/
│   │           │   ├── img/
│   │           │   ├── scripts/
│   │           │   │   └── client.js
│   │           │   ├── index.html
│   │           │   └── style.css
│   │           ├── src/
│   │           │   ├── server.ts
│   │           │   └── index.ts
│   │           └── Dockerfile.bfb_webrtc
│   │
│   └── ros_entrypoint.sh
│
└── README.md
```

## docs/
This folder contains the documentation for the project.

*bfb_documentation.md* - this file.

## docker/
In this folder, you can find the Dockerfiles for the different components of the robot.


### camera_d435i/
This folder contains the Dockerfiles for the camera component (Intel RealSense D435i).

- [Dockerfile.bfb_camera_d435i](https://github.com/jevgenik/bigfootbot/blob/master/docker/camera_d435i/Dockerfile.bfb_camera_d435i)
- [docker-compose_camera_d435i.yml](https://github.com/jevgenik/bigfootbot/blob/master/docker/camera_d435i/docker-compose_camera_d435i.yml)

To start the camera component, run the following command:
```bash
docker compose -f docker-compose_camera_d435i.yml up
```

Container *bfb_camera_d435i_container* will be created and started.

The container will contain:
- Linux Ubuntu 22.04
- ROS 2 Humble
- Intel RealSense SDK 2.0
- Intel RealSense ROS Wrapper. [GitHub realsense-ros](https://github.com/IntelRealSense/realsense-ros)

To enter to the running container and to start a bash shell in it:
```bash
docker exec -it bfb_camera_d435i_container bash
```

To manually start the camera node:
```bash
ros2 launch realsense2_camera rs_launch.py
```

## ros_entrypoint.sh 
this file is added to every Dockerfile as ENTRYPOINT. Script is responsible for setting up the ROS environment.

### web/
#### webrtc/
##### app/
This folder contains the WebRTC node.js application.
https://github.com/jevgenik/bigfootbot/tree/master/docker/web/webrtc/app

- [public/img]
- [public/scripts]
- [public/scripts/client.js](https://github.com/jevgenik/bigfootbot/blob/master/docker/web/webrtc/app/public/scripts/client.js) - handles the setup of 
  the WebRTC connection facilitates real-time video streaming from the robot to the web client (browser)
- [public/index.html](https://github.com/jevgenik/bigfootbot/blob/master/docker/web/webrtc/app/public/index.html) - web page with the camera stream 
  (video element) 
- [public/style.css]
- [src/server.ts](https://github.com/jevgenik/bigfootbot/blob/master/docker/web/webrtc/app/src/server.ts) - WebRTC signaling server
- [src/index.ts](https://github.com/jevgenik/bigfootbot/blob/master/docker/web/webrtc/app/src/index.ts) - starting point of the WebRTC application (server)
- [Dockerfile.bfb_webrtc](https://github.com/jevgenik/bigfootbot/blob/master/docker/web/WebRTC/Dockerfile.webrtc) - Dockerfile for the WebRTC application



# Installation and Usage

1. Clone the repository: https://github.com/jevgenik/bigfootbot

2. Go to docker/ folder and run needed components (like teleop, camera, etc.)