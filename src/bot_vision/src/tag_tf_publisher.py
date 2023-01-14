#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray

class tag_to_odom():
    def __init__(self) -> None:
        rospy.init_node('map_saver')
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_detect_callback)
        self.odom_broadcaster = tf.TransformBroadcaster()

   
    def tag_detect_callback(self,msg):
        #create message and header
        tagodom = Odometry()
        tagodom.header.stamp = rospy.Time.now()
        tagodom.header.frame_id = "odom"
        for i in msg.detections:
            child_frame=f"Bot_{i.id[0]}"
            current_time=rospy.Time.now()
            pose=i.pose.pose.pose
            x=pose.position.x
            y=-1 * pose.position.y
            z=(-1 * pose.position.z)+5

            x_=pose.orientation.x
            y_=pose.orientation.y
            z_=pose.orientation.z
            w_=pose.orientation.w
            # print(x,y,x_,y_,z_,w_)
            # euler=tf.transformations.euler_from_quaternion((x_,y_,z_,w_))
            # yaw=euler[2]-2.106
            # quat=tf.transformations.quaternion_from_euler(0,0,yaw)
            # print(euler)
            #broadcast transform
            self.odom_broadcaster.sendTransform((x, y, z),(x_,y_,z_,w_),current_time,child_frame, "camera_link_optical")

if __name__ == '__main__':
    tag2odom=tag_to_odom()
    rospy.spin()   
