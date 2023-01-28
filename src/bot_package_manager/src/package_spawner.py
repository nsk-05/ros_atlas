#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point,Pose,Quaternion
import rospkg
from bot_package_manager.srv import BoxSpawn,BoxSpawnResponse

class PackageSpawn():
    def __init__(self) -> None:
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        self.pack_spawn_service=rospy.Service("package_spawn_service",BoxSpawn,self.spawner_cb)
        self.spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        rpkg=rospkg.RosPack()
        file=rpkg.get_path("bot_package_manager")
        file+="/urdf/box.urdf"
        self.box_file=open(file, 'r').read()
        self.package_count=0

    def spawner_cb(self,req):
        pose=[8,7.35,3]
        box_xml=self.box_file.replace("Red",req.colour)
        model_name=f"package_{self.package_count}"
        print(model_name)
        res=self.spawner(model_name=model_name,model_xml=box_xml, robot_namespace="/", initial_pose=Pose(position= Point(pose[0],pose[1],pose[2]),orientation=Quaternion(0,0,0,0)),reference_frame="world")
        if(res.success):
            rospy.loginfo(self.package_count)
            self.package_count+=1
            return BoxSpawnResponse(status=True)
        else:
            return BoxSpawnResponse(status=False)

def main():
    rospy.init_node("package_spawner")
    pack_spawner=PackageSpawn()
    rospy.spin()
    
        
if __name__=="__main__":
    main()