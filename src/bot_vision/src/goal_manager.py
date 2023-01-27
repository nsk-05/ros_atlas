#!/usr/bin/env python3
import rospy
import actionlib
import threading
import time
from tf.transformations import quaternion_from_euler, euler_from_quaternion as euler
from math import hypot, sqrt, atan2, radians
from bot_vision.srv import Goal_Service, Goal_ServiceResponse  #, GoalStatus, GoalStatusResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import rospkg

class goal_manager():
    def __init__(self) -> None:
        self.init_variables()
        self.in_ser = rospy.Service('goal_manager/goal',Goal_Service,self.goal_cb)
        # self.goal_status_service = rospy.Service('/goal_manager/goal_status', GoalStatus, self.goal_status_cb)
        # self.robot_pose = rospy.Subscriber('robot_pose', Pose,self.robot_pose_cb)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def init_variables(self):
        self.current_robot_pose = None
        self.bot_size=1.2 #it is in meters we nww
        rspkg=rospkg.RosPack()
        pkg_path=rspkg.get_path("bot_vision")
        file=pkg_path+"/targets/targets.txt"
        self.target_dict=dict()
        rospy.loginfo(file)
        with open(file) as f:
            targets_list=f.read().splitlines()
        for i in targets_list:
            j=i.split(" ")
            self.target_dict[j[0]]=[int(j[1]),int(j[2])]
        rospy.loginfo(self.target_dict)

    def goal_cb(self,req):
        
        id=(req.bot_id%4)
        if(id==0):
            x=self.target_dict[req.goal][0]+1+self.bot_size
            y=self.target_dict[req.goal][1]
            orientation=quaternion_from_euler(0,0,3.1417)
        rospy.loginfo(f"{str(x)} {str(y)}") #str(orientation[2]),str(orientation[3]) 

        # self.x = x
        # self.y = y
        # self.z = orientation[2]
        # self.w = orientation[3]
        action_goal = MoveBaseGoal()
        action_goal.target_pose.header.frame_id = "map"
        action_goal.target_pose.header.stamp = rospy.Time.now()
        action_goal.target_pose.pose.position.x = x
        action_goal.target_pose.pose.position.y = y
        action_goal.target_pose.pose.orientation.z = orientation[2]
        action_goal.target_pose.pose.orientation.w = orientation[3]
        rospy.loginfo("sending move base goal")
        self.client.send_goal(action_goal)
        rospy.loginfo("goal sent")
        self.on_goal = True

        while not self.client.get_result():
            time.sleep(0.1)
            self.goal_state = self.client.get_state()
            if (self.goal_state == 1):
                print("new_goal_recieved.........!!robot in progress...")
            if (self.goal_state == 2):
                print("i got an interrupt!!!......goal_aborted...")
            if (self.goal_state == 3):
                print("destination reached!!!......goal_succed...")
                return Goal_ServiceResponse(status=True)
        # rospy.loginfo("got result",self.goal_state)
        return Goal_ServiceResponse(status=False)
    # def send_goal(self,x, y, z, w):
    #     self.got_result = False

    #     print("sending move base goal")
    #     self.client.send_goal(action_goal)
    #     print("goal sent")
    #     self.on_goal = True
    #     while not self.client.get_result():
    #         time.sleep(0.1)
    #         self.goal_state = self.client.get_state()
    #         if (self.goal_state == 1):
    #             print("new_goal_recieved.........!!robot in progress...")
    #         if (self.goal_state == 2):
    #             print("i got an interrupt!!!......goal_aborted...")
    #         if (self.goal_state == 3):
    #             print("destination reached!!!......goal_succed...")
    #     print("got result",self.goal_state)
    #     self.on_goal = False
    #     print(">>>>>>>>>>>>>>>> got_result force_reached"+str(self.force_reached))
    #     if self.update_result:
    #         self.got_result = True


    # def goal_status_cb(self,req):
    #     if self.goal_state != None:
    #         return GoalStatusResponse(status=self.goal_state)
    #     else:
    #         return GoalStatusResponse(status=-1)


def main(args=None):
    rospy.init_node('goal_manager')
    node = goal_manager()
    rospy.spin()

if __name__== '__main__':
    main()