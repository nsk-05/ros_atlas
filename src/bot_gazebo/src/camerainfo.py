#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo

def repeater(msg):
    camInfoPub.publish(msg)
def main():
    global camInfoPub
    rospy.init_node("cam_info_pub")
    camInfoPub=rospy.Publisher("/camera_info",CameraInfo,queue_size=1)
    rospy.Subscriber("/camera/camera_info",CameraInfo,repeater,queue_size=1)
if __name__=='__main__':
    main()
    rospy.spin()