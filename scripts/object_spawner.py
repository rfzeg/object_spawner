#! /usr/bin/env python

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def spawn_model(model_name):
    """ Spawns a model in a particular position
        Args: name of the model (as in folder and model.config file)
        Returns: None
    """
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 0.5

    # Spawn SDF model
    try:
        package_name = 'object_spawner'
        model_path = rospkg.RosPack().get_path(package_name)+'/models/'
        model_xml = ''

    except rospkg.ResourceNotFound as e:
        rospy.logerr("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, e))

    try:
        with open(model_path + model_name + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
    except IOError as err:
        rospy.logerr("Cannot find model [%s], check model name and that model exists, I/O error message:  %s"%(model_name,err))
    except UnboundLocalError as error:
        rospy.logdebug("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, error))

    try:
        rospy.wait_for_service('gazebo/spawn_urdf_model',5.0)
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        # then use this handle just like a normal function and call it
        res = spawn_model_prox('auto_spawned_model_name',model_xml, '', initial_pose, 'world')
        # evaluate response
        if res.success == True:
            rospy.loginfo(res.status_message + " " + model_name)
        else:
            rospy.logerr("Error: model %s not spawn, error message = "% model_name + res.status_message)

    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
    except UnboundLocalError as error:
        rospy.logdebug("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, error))

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('object_spawner', log_level=rospy.INFO)

    spawn_model('coke_can')