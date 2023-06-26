#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class colour_goals():
    def __init__(self) -> None:
        self.bridge = CvBridge()
        # Define your image topic
        self.image_topic = "/camera/image_raw"


    def goal_callback(self,msg):
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _,thresh = cv2.threshold(blurred, 175, 255, cv2.THRESH_BINARY)
            cnts,_ = cv2.findContours(thresh.copy(), cv2.RETR_TREE,
                        cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                
                M = cv2.moments(c)
                cX = int((M["m10"] / M["m00"]))
                cY = int((M["m01"] / M["m00"]))
                c = c.astype("float")
                c = c.astype("int")
                area=cv2.contourArea(c)
                if(area>10000):
                    continue
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                if len(approx) == 4:
                    # compute the bounding box of the contour and use the
                    # bounding box to compute the aspect ratio
                    (x, y, w, h) = cv2.boundingRect(approx)
                    cX+=25
        except CvBridgeError as e:
            print(e)
if __name__ == '__main__':
    colour_goals_detector=colour_goals()
    rospy.init_node('colour_goal_detector')
    rospy.Subscriber(colour_goals_detector.image_topic, Image, colour_goals_detector.goal_callback)
    rospy.spin()


# class ShapeDetector:
# 	def __init__(self):
# 		pass

# 	def detect(self, c):
# 		# initialize the shape name and approximate the contour
# 		shape = "unidentified"


# 		# if the shape has 4 vertices, it is either a square or
# 		# a rectangle

# 			ar = w / float(h)

# 			# a square will have an aspect ratio that is approximately
# 			# equal to one, otherwise, the shape is a rectangle
# 			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

# 		return shape

# if __name__=="__main__":
#     image=cv2.imread("img/summa.jpeg")

#     # find contours in the thresholded image and initialize the
#     # shape detector
#     sd = ShapeDetector()
#     # # loop over the contours
#     for c in cnts:
#         # compute the center of the contour, then detect the name of the
#         # shape using only the contour

#         shape = sd.detect(c)

#     #     # multiply the contour (x, y)-coordinates by the resize ratio,
#     #     # then draw the contours and the name of the shape on the image
#         c = c.astype("float")
#         c = c.astype("int")
#         area=cv2.contourArea(c)
#         if(area>10000):
#             continue
#         text=f"{shape} {area}"
#         # cX+=25
#         # cY+=25
#         cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
#         cv2.putText(image, text, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
#             0.5, (255, 255, 255), 2)