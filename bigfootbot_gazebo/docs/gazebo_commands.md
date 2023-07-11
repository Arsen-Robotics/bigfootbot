#### Open Gazebo UI
`ign gazebo` (Old naming, Fortress)  
`gz sim` (NEW naming, Garden)

**Open an empty world**  
`ign gazebo /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf` (Old)

**Default worlds (in Ignition Fortress) are located here:**  
`/usr/share/ignition/ignition-gazebo6/worlds`

> **NB! Before calling services, a world (sdf file with defined services) must be launched**

#### Get the list of available services
`ign service -l` (Old naming, Fortress)  
`gz service -l` (NEW naming, Garden)

**Check types of service request and response messages (-i: info, -s: service name)**  
`ign service -is /world/empty/create` (Old)  
`gz service -is /world/empty/create` (NEW)

**The following command spawns the URDF file model.urdf into the Gazebo Sim world as a model named urdf_model:**  
`ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'` (Old)

`gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'` (NEW)