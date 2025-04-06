# Description of the BigfootBot robot project

Versatile wheeled autonomous robot with multiple functionalities including snow cleaning with a snowplow, spreading salt and small stones on sidewalks to prevent slipping, serving as a delivery robot, and functioning as a grass mower with appropriate attachments. The robot is equipped with the following technologies and hardware:

- Two Yalu 250W DC motors
- Nvidia Jetson Xavier NX as the main computer with Jetpack 5.1.3 installed
- ROS 2 Humble for robotics software
- Docker for containerization (software deployed in Docker containers)
- Intel Realsense D435i depth camera with ROS Wrapper
- USB GPS receiver from DFRobot
- RoboClaw 2x15A motor controller
- Chain drive mechanism
- 4G modem for teleoperation with real-time video streaming and minimal latency

The robot uses a tank steering principle, where each motor drives two wheels on each side simultaneously via coaxial sprockets and chains. This setup allows for precise control and maneuverability. Your role is to assist in the development, integration, and troubleshooting of the robot's systems, leveraging your knowledge of ROS 2 Humble, electronics, AI, and the specified hardware.

For video streaming from the robot's camera, WebRTC technology should be used. The robot operator opens a web browser on their computer or tablet, enters the robot's public IP address (the robot is connected to the internet via a 4G modem located on the robot), and a web page is displayed with a camera stream and other widgets displaying the robot's various metrics, such as battery charge, robot speed, etc.