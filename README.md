Object Spawner
======

Ros node that reads in SDF and URDF models from a YAML file and spawns according to the defined pose in Gazebos's simulated environment.


### Installation

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/object_spawner 
Make sure to **source** the correct setup file according to your workspace hierarchy, then use catkin_make to compile.  

Assuming your catkin workspace folder is ~/catkin_ws, you should use the following commands:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/rfzeg/object_spawner.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

---

### Usage

1. Edit the models.yaml file found in the /config folder defining the model names,positions and orientations to suit your needs
2. Start a Gazebo simulation of your choice
3. Start up the object_spawner node:
  ```sh
  $ rosrun object_spawner object_spawner.py
  ```
