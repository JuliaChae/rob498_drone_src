#!/usr/bin/env python

import cv2
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np 

K = np.asarray([[411.838509092687, 0, 289.815738517589], 
                [0, 546.791755994532, 278.771000222491],
                [0, 0, 1]])

D = np.asarray([-0.337287855055825, 0.117096291002566,  0, 0])


def capture_and_save_images():
    rospy.init_node("imx219_pub")
    image_pub= rospy.Publisher("imx219_image", Image, queue_size=10)
    bridge=CvBridge()
    rate=rospy.Rate(10)

    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if not cap.isOpened():
        print("Error")
        return
    while not rospy.is_shutdown():
        ret,frame = cap.read()
        if not ret:
            print("Error")
            break
        frame = cv2.resize(frame, (frame.shape[1]//3, frame.shape[0]//3))
        frame = cv2.undistort(frame, K, D)
        image_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = rospy.Time.now()
        image_pub.publish(image_msg)


        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_and_save_images()