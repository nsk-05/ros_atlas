#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point,Pose,Quaternion
import rospkg


def main():
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    rpkg=rospkg.RosPack()
    file=rpkg.get_path("bot_package_manager")
    file+="/urdf/box.urdf"
    spawner(model_name="package1",model_xml=open(file, 'r').read(), robot_namespace="/", initial_pose=Pose(position= Point(0,0,2),orientation=Quaternion(0,0,0,0)),reference_frame="world")
        
if __name__=="__main__":
    main()