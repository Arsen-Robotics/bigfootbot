# Project Folder Structure

```text 
bigfootbot/
│
├── docker/
│   ├── camera_d435i/
│   │   ├── Dockerfile.bfb_camera_d435i
│   │   ├── docker-compose_camera_d435i.yml
│   │
│   ├── web/
│   │   └── webrtc/
│   │       ├── Dockerfile.bfb_webrtc
│   │
│   ├── ros_entrypoint.sh
│
└── README.md
```

## docker/
In this folder, you can find the Dockerfiles for the different components of the robot.


### camera_d435i/
This folder contains the Dockerfiles for the camera component (Intel RealSense D435i).

- [Dockerfile.bfb_camera_d435i](https://github.com/jevgenik/bigfootbot/blob/master/docker/camera_d435i/Dockerfile.bfb_camera_d435i)
- [docker-compose_camera_d435i.yml](https://github.com/jevgenik/bigfootbot/blob/master/docker/camera_d435i/docker-compose_camera_d435i.yml)

To start the camera component, run the following command:
```bash
docker-compose -f docker-compose_camera_d435i.yml up
```

Container *bfb_camera_d435i_container* will be created and started.

The container will contain:
- Linux Ubuntu 22.04
- ROS 2 Humble
- Intel RealSense SDK 2.0
- Intel RealSense ROS Wrapper. [GitHub realsense-ros](https://github.com/IntelRealSense/realsense-ros)


## ros_entrypoint.sh 
this file is added to every Dockerfile as ENTRYPOINT. Script is responsible for setting up the ROS environment.

### web/
#### webrtc/
This folder contains the Dockerfiles for the WebRTC component.

- [Dockerfile.bfb_webrtc](https://github.com/jevgenik/bigfootbot/blob/master/docker/web/WebRTC/Dockerfile.webrtc)