#! /usr/bin/env python

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from yaml import load
from tf.transformations import quaternion_from_euler
from math import pi

class Model(object):
    def __init__(self, **entries): 
        self.__dict__.update(entries)

    def __repr__(self):
       return '{}({!r}, {!r}, {!r})'.format(
           self.__class__.__name__,
           self.name,self.type,self.package)

    def __unicode__(self):
        return u'name: %s, type: %s, package: %s' % (self.name,self.type,self.package)

    def __str__(self):
        return unicode(self).encode('utf-8')

def parse_yaml(package_name,yaml_filename):
    """ Parse a yaml into a dict of objects containing all data to spawn models
        Args:
        name of the package (string, refers to package where yaml file is located)
        name of the yaml file (string, complete name incl. file extension)
        Returns: dictionary with model objects
    """
    complete_path = rospkg.RosPack().get_path(package_name)+'/config/'+yaml_filename
    f = open(complete_path, 'r')
    # populate dictionary that equals to the yaml file tree data
    yaml_dict = load(f)

    # create a list of the names of all models parsed
    modelNames = [yaml_dict['models'][k]['name'] for k in range(len(yaml_dict['models']))]
    
    rospy.loginfo("List of model names: %s" % modelNames)
    rospy.logdebug("Total number of models: ", len(yaml_dict['models']))

    # create a dict of Model objects where each key is the name of the model
    model_dict = {name: Model() for name in modelNames}
    # create list containing all nested dictionaries that hold data about each model   
    list_of_dict = [x for x in yaml_dict['models']]
    # parse YAML dictionary entries to Model class object attributes
    for idx, name in enumerate(modelNames):
        args = list_of_dict[idx]
        model_dict[name] = Model(**args)

    return model_dict

def spawn_model(model_object):
    """ Spawns a model in a particular position and orientation
        Args:
        - model object containing name, type, package and pose of the model
        Returns: None
    """
    ## unpack object attributes
    package_name = model_object.package
    spawn_pose = Pose()
    spawn_pose.position.x = model_object.pose[0]
    spawn_pose.position.y = model_object.pose[1]
    spawn_pose.position.z = model_object.pose[2]
    # conversion from Euler angles (RPY) in degrees to radians
    degrees2rad = pi / 180.0
    roll = model_object.pose[3] * degrees2rad
    pitch = model_object.pose[4] * degrees2rad
    yaw = model_object.pose[5] * degrees2rad
    # create list that contains conversion from Euler to Quaternions
    quat = quaternion_from_euler (roll,pitch,yaw)
    spawn_pose.orientation.x = quat[0]
    spawn_pose.orientation.y = quat[1]
    spawn_pose.orientation.z = quat[2]
    spawn_pose.orientation.w = quat[3]
    
    # Spawn SDF model
    if model_object.type == 'sdf':
        try:
            model_path = rospkg.RosPack().get_path(package_name)+'/models/'
            model_xml = ''
        except rospkg.ResourceNotFound as e:
            rospy.logerr("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, e))

        try:
            with open(model_path + model_object.name + '/model.sdf', 'r') as xml_file:
                model_xml = xml_file.read().replace('\n', '')
        except IOError as err:
            rospy.logerr("Cannot find model [%s], check model name and that model exists, I/O error message:  %s"%(model_object.name,err))
        except UnboundLocalError as error:
            rospy.logdebug("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, error))

        try:
            rospy.logdebug("Waiting for service gazebo/spawn_sdf_model")
            # block max. 5 seconds until the service is available
            rospy.wait_for_service('gazebo/spawn_sdf_model',5.0)
            # create a handle for calling the service
            spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

    elif model_type == "urdf":
        try:
            model_path = rospkg.RosPack().get_path(package_name)+'/urdf/'
            file_xml = open(model_path + model_object.name + '.' + model_object.type, 'r')
            model_xml = file_xml.read()
        except IOError as err:
            rospy.logerr("Cannot find model [%s], check model name and that model exists, I/O error message:  %s"%(model_object.name,err))
        except UnboundLocalError as error:
            rospy.logdebug("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, error))

        try:
            rospy.logdebug("Waiting for service gazebo/spawn_urdf_model")
            # block max. 5 seconds until the service is available
            rospy.wait_for_service('gazebo/spawn_urdf_model')
            # create a handle for calling the service
            spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

    else:
        rospy.logerr("Error: Model type not know, model_type = " + model_object.type)

    try:
        # use handle / local proxy just like a normal function and call it
        res = spawn_model_prox(model_object.name,model_xml, '',spawn_pose, 'world')
        # evaluate response
        if res.success == True:
            # SpawnModel: Successfully spawned entity 'model_name'
            rospy.loginfo(res.status_message + " " + model_object.name)
        else:
            rospy.logerr("Error: model %s not spawn, error message = "% model_object.name + res.status_message)
    except UnboundLocalError as error:
        rospy.logdebug("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, error))

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('object_spawner_node', log_level=rospy.INFO)

    ###### usage example
    
    # parse yaml file to dictionary of Model objects
    cfg_package_name = 'object_spawner'
    cfg_yaml_filename = 'models.yaml'
    m = parse_yaml(cfg_package_name,cfg_yaml_filename) # create dict called 'm'
 
    spawn_model(m['wood_cube_10cm'])

    # sleep for duration (seconds, nsecs)
    d = rospy.Duration(2, 0)
    rospy.sleep(d)

    #coke = Model(model_name,model_type,model_pkg_name,coke_pose)
    spawn_model(m['coke_can'])
