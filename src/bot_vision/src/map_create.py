#!/usr/bin/env python3
import rospy
# ROS Image message
from std_srvs.srv import Trigger,TriggerResponse
from bot_vision.srv import mapSave,mapSaveRequest,mapSaveResponse
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import os


class Map_save():
    def __init__(self) -> None:
        self.save=0
        # Instantiate CvBridge
        self.bridge = CvBridge()

        # Define your image topic
        self.image_topic = "/camera/image_raw"
        

    def image_callback(self,msg):
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError as e:
            print(e)
        
    def map_service(self,req):
        os.chdir("maps")
        img=cv2.cvtColor(self.cv2_img,cv2.COLOR_BGR2GRAY)
        ret,gray=cv2.threshold(img,170,255,cv2.THRESH_BINARY)
        map_file=f"{req.map_name}.pgm"
        cv2.imwrite(map_file, gray)
        yaml_file_name=f"{req.map_name}.yaml"
        yaml = open(yaml_file_name, "w")
        # Write some information into the file
        yaml.write("image: " + os.getcwd() + "/" + req.map_name + ".pgm\n")
        yaml.write("resolution: 0.040000\n")
        yaml.write("origin: [" + str(-10) + "," +  str(-10) + ", 0.000000]\n")
        yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196")
        yaml.close()
        res=mapSaveResponse()
        res.success=True
        return res


if __name__ == '__main__':
    # main()
    MapSaver=Map_save()
    rospy.init_node('map_saver')
    rospy.Subscriber(MapSaver.image_topic, Image, MapSaver.image_callback)
    rospy.Service("/map_save",mapSave,MapSaver.map_service)
    rospy.spin()