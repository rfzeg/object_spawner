#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('object_spawner', log_level=rospy.INFO)
rospy.wait_for_service('gazebo/spawn_urdf_model')

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0.5

# Spawn SDF model
model_xml = ''

with open('../models/coke_can/model.sdf', 'r') as xml_file:
    model_xml = xml_file.read().replace('\n', '')

# create a handle for calling the service
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
# then use this handle just like a normal function and call it
spawn_model_prox('auto_spawned_model_name',
                 model_xml, '', initial_pose, 'world')
