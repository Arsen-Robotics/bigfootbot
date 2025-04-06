## Ignition / Gazebo guide
- [Installation and Tutorials](https://gazebosim.org/docs)  
- [Sim API and Tutorials (Ignition)](https://gazebosim.org/api/sim/6/)
- [Gazebo architecture](https://gazebosim.org/docs/garden/architecture)
- [Server Configuration](https://gazebosim.org/api/gazebo/4.3/server_config.html)
- [ROS Gazebo Demos](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_sim_demos/README.md)

#### Open Gazebo UI
- `ign gazebo` (Old naming, Fortress)  
- `gz sim` (NEW naming, Garden)

**Open an empty world**  
`ign gazebo /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf` (Old)

**Default worlds (in Ignition Fortress) are located here:**  
`/usr/share/ignition/ignition-gazebo6/worlds`

> **NB! Before calling services, a world (sdf file with defined services) must be launched**

#### Get the list of available services
- `ign service -l` (Old naming, Fortress)  
- `gz service -l` (NEW naming, Garden)

**Check types of service request and response messages (-i: info, -s: service name)**  
- `ign service -is /world/empty/create` (Old)  
- `gz service -is /world/empty/create` (NEW)

**The following command spawns the URDF file model.urdf into the Gazebo Sim world as a model named urdf_model:**  
- `ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'` (Old)

- `gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'` (NEW)

#### Gazebo topics
- `ign topic -l` - list all of the topics that are currently available in the Ignition transport system  
- `ign tipic -t /topic_name -i` - display information about a specific topic  
- `ign topic -t /topic_name -e` - echo topic /topic_name (-t - topic name. -e -echo)

## ROS + Gazebo Sim (launch files, ROS-enabled executables) (package `ros_gz_sim`)
[GitHub ros_gz_sum](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_sim)  
For example:  
Launch the sim with an empty world: `ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf`  
Spawn the robot in the launched world (named 'empty'): `ros2 run ros_gz_sim create -world empty -topic /robot_description`

### PLugins
> A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation (loaded at runtime). Plugins make us control many aspects of the simulation like world, models, etc

Plugin files (shared libraries) are located here:  
/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins (Ignition)