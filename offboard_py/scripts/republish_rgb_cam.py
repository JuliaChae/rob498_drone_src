#!/usr/bin/env python3

import cv2
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def capture_and_save_images():
    rospy.init_node("imx219_pub")
    image_pub= rospy.Publisher("imx219_image", Image, queue_size=10)
    bridge=CvBridge()
    rate=rospy.Rate(10)

    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if not cap.isOpened():
        print("Error")
        return
    while not rospy.is_shutdown():
        ret,frame = cap.read()
        if not ret:
            print("Error")
            break
        image_msg = bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        image_msg.header.stamp = rospy.Time.now()
        image_pub.publish(image_msg)

        #output_filename=f"/home/jetson/image_data/{output_prefix}_{i}.jpg"
        #cv2.imwrite(output_filename, frame)
        #print(f"Saved image {i+1}")
        #time.sleep(delay)

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_and_save_images()