# Object Spawner
Author: Roberto Zegers R.  

Ros node that reads in SDF and URDF models specified in a YAML file and spawns them according to the defined pose in Gazebo.
The development of this package required following expertise: **python**, **ROS parameter server**, **yaml**, **Gazebo services**.

### Features
+ Easily configurable: No need to change the source code or include command line arguments the spawn a different list of models and/or poses. Just modify the YAML file and relaunch.
+ Portable: the location of the .yaml file can be set from the launch file. This effectivelly allows to separate the location of the package's source code from the package's configuration files.

### Installation

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/object_spawner.  
Make sure to **source** the correct setup file according to your workspace hierarchy, then use **catkin_make** to compile.  

Assuming your catkin workspace folder is ~/catkin_ws, you should use the following commands:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/rfzeg/object_spawner.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

### Usage

1. Edit the models.yaml file found in the /config folder defining the model names and pose to suit your needs
2. Start a Gazebo simulation of your choice (e.g. ` $ roslaunch gazebo_ros empty_world.launch`)
3. Start up the object_spawner node: ` $ roslaunch object_spawner object_spawner.launch `

### YAML file structure

The package requires a `.yaml` file like this exemplary `models.yaml`:

```yml
models:
  - name: wood_cube_10cm
    type: sdf
    package: object_spawner
    pose: [0.5,1.0,0.2,0.0,0.0,45.0] # x,y,z,R,P,Y (in degrees)
  - name: coke_can
    type: sdf
    package: object_spawner
    pose: [0.0,0.0,0.25,90.0,0.0,0.0] # x,y,z,R,P,Y (in degrees)
  - name: coke_can
    type: sdf
    package: object_spawner
    pose: [0.5,0.0,0.25,45.0,0.0,135.0] # x,y,z,R,P,Y (in degrees)
```

As showed above the `.yaml` file has to have the following structure:

- `models` (required): root object that identifies the YAML file as containing simulation models
- `name` (required): defines the filename of an object. Do not include the file extention.
- `type` (required): defines the file extension of that object (can be either `sdf` or `urdf`)
- `package` (required): defines the name of the ROS package that contains the object
- `pose` (required): the coordinates in mt. (x,y,z) and orientation in deg. (roll, pitch, yaw) where the object will be spawned

It is possible to add comments to the YAML file by using the # sign. All `name` elements should begin with a dash (-) and must be prefixed with the same amount of spaces, in the example above two spaces are used, the number of spaces can vary from file to file, but tabs are not allowed.  
  
The YAML file can be adjusted by adding or deleting models, changing the order and adding/removing comments. It is important to do not break the formatting rules described above while editing the file or the models will not spawn.  

**Important:**  
URDF and SDF models must adhere to a specific folder/filename structure, otherwise they will not be spawned:
+ All URDF files must be located in the `urdf` folder of the package defined.
+ All SDF models must follow this folder and filename convention: **models/model_name/model.sdf**.

### Parameters
**Yaml file location**  
The .yaml` file can be located in any package and any folder as long as both are specified in the node's launch file.  
**Randomness**  
To randomize the order in which the objects are spawned into simulation set the "random_order" argument to true, false is the default value.  
**Spawn timing**  
To configure the time required to pass before spawning the next object modify the value of the time_interval argument (in seconds).  

An exemplary launch file has been provided with the default values set for all parameters:

```xml
  <!-- Arguments for object_spawner node with defaults provided -->
  <!-- The name of the package where the .yaml cfg file is located -->
  <arg name="yaml_package_name" default="object_spawner"/>
  <!-- The relative path of the .yaml cfg file (inside the package just defined above) -->
  <arg name="yaml_relative_path" default="/config/models.yaml"/>
  <!-- Boolean value that indicates whether to spawn a models in random order or iterate the dictionary -->
  <arg name="random_order" default="false"/> <!-- set "true" or "false" (not case-sensitive) -->
  <!-- Spawn next object after the specified time period (in seconds) -->
  <arg name="time_interval" default="1.0"/>

  <node name="object_spawner_node" pkg="object_spawner" type="object_spawner.py" output="screen">
    <!-- Load node configuration parameters to parameter server -->
    <param name="yaml_package_name" type="string" value="$(arg yaml_package_name)"/>
    <param name="yaml_relative_path" type="string" value="$(arg yaml_relative_path)"/>
    <param name="random_order" type="bool" value="$(arg random_order)"/>
    <param name="time_interval" type="double" value="$(arg time_interval)"/>
  </node>
```
### Limitations
+ This node is not able to spawn models that already exist in simulation it will cause this error message:  
  [ERROR] SpawnModel: Failure - entity already exists

### Further extensions
+ Expose functionality via ROS services to enable programmatic interaction with the node
+ Add support for xacro files

### Resources

+ http://nlamprian.me/blog/software/ros/2019/03/31/ros-parameters/
