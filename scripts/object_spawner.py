#! /usr/bin/env python

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('object_spawner', log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0.5

# Spawn SDF model
model_path = rospkg.RosPack().get_path('object_spawner')+'/models/'
model_name = 'coke_can'
model_xml = ''

with open(model_path + model_name + '/model.sdf', 'r') as xml_file:
    model_xml = xml_file.read().replace('\n', '')

try:
    rospy.wait_for_service('gazebo/spawn_urdf_model',5.0)
    # create a handle for calling the service
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    # then use this handle just like a normal function and call it
    spawn_model_prox('auto_spawned_model_name',model_xml, '', initial_pose, 'world')

except (rospy.ServiceException, rospy.ROSException), e:
    rospy.logerr("Service call failed: %s" % (e,))