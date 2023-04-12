#! /usr/bin/env python
import numpy as np
import cv2 
import argparse 
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np 
import rospy
import tf
from tf.transformations import quaternion_matrix
import pdb 

# construct the argument parser and parse the arguments
# ap = argparse.ArgumentParser()
# # ap.add_argument("-s", "--server-ip",
# #                 help="ip address of the server to which the client will connect")
# ap.add_argument("-v", "--video", help="path to the video file")
# ap.add_argument("-a", "--min-area", type=int, default=500, help="minimum area size")
# args = vars(ap.parse_args())

lower_yellow = (20, 50, 50)
upper_yellow = (40, 255, 255)

lower_black = (0, 0, 0)
upper_black = (50, 50, 50)



# K = np.asarray([[411.838509092687, 0, 289.815738517589], 
#                 [0, 546.791755994532, 278.771000222491],
#                 [0, 0, 1]])
K = np.asarray([[334.94030171, 0, 280.0627713], 
                [0, 595.99313333, 245.316628], 
                [0, 0, 1]])

class RGBOccupancyGrid:

    def __init__(self):
        self.bridge=CvBridge()
        self.detect_pub = rospy.Publisher("detected_pillars", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("imx219_image", Image, self.pillar_detection)
        self.odom_sub = rospy.Subscriber("/mavros/odometry/out", Odometry, self.odom_callback)
        self.obstacle_pub = rospy.Publisher("/obstacles", Odometry, queue_size=0)

        self.listener = tf.TransformListener()

        self.detected_pillars = []

        self.pose = np.eye(4)
        self.num = 0 
        

    def stream_video(self): 
        # Create a VideoCapture object and read from input file
        # If the input is the camera, pass 0 instead of the video file name
        cap = cv2.VideoCapture(args['video'])
        
        # Check if camera opened successfully
        if (cap.isOpened()== False): 
            print("Error opening video stream or file")
        
        # Read until video is completed
        ind = 0
        while(cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret == True:
            
                # Display the resulting frame
                cv2.imshow('Frame',frame)
                print(ind)
                ind += 1
                cv2.imwrite('pillar_data/frame'+str(ind)+'.jpg',frame)
            
                # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
            # Break the loop
            else: 
                break
    
        # When everything done, release the video capture object
        cap.release()
        
        # Closes all the frames
        cv2.destroyAllWindows()

    def odom_callback(self, odom_msg):
        R = quaternion_matrix(np.array([odom_msg.pose.pose.orientation.x,
                                        odom_msg.pose.pose.orientation.y,
                                        odom_msg.pose.pose.orientation.z,
                                        odom_msg.pose.pose.orientation.w]))
        #print(R)
        t = odom_msg.pose.pose.position
        self.pose = np.eye(4)
        self.pose[:3,:3] = R[:3, :3]
        self.pose[:3, 3] = np.array([t.x, t.y, t.z]).T
        

    def pillar_detection(self, img): 
        # Create a VideoCapture object and read from input file
        # If the input is the camera, pass 0 instead of the video file name
        # Read a frame from the video stream
        obs_type = None
        frame = self.bridge.imgmsg_to_cv2(img)
        ret = cv2.imwrite("/home/rob498/rob498_drone_ws/src/images/frame" + str(self.num) + ".jpg", frame)
        self.num += 1

        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the yellow and black colors
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        # Combine the masks to detect the yellow and black stripes
        combined_mask = cv2.bitwise_or(yellow_mask, black_mask)

        # Apply a morphological opening to the combined mask to remove noise
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        # Find the contours in the combined mask
        contours, hierarchy = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        # Loop through the contours
        for cnt in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(cnt)

            # If the contour area is greater than a threshold, it's likely a pillar
            if area > 1000:
                # Draw a bounding box around the contour
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate the aspect ratio of the bounding box
                aspect_ratio = float(w) / h

                # If the aspect ratio is close to 1, it's a fully yellow pillar
                if aspect_ratio >= 0.8 and aspect_ratio <= 1.2:
                    dist = self.get_obstacle(x,y,w,h,"full")
                    # cv2.putText(frame, "Fully Yellow Pillar", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, str(round(dist, 3)), (x+100, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                # If the aspect ratio is not close to 1, it's a yellow and black striped pillar
                else:
                    dist = self.get_obstacle(x,y,w,h,"partial")
                    cv2.putText(frame, str(round(dist, 3)), (x+100, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display the frame
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = rospy.Time.now()
        self.detect_pub.publish(image_msg)

    def get_obstacle(self, x, y, w, h, type):

        fx = K[0,0]
        fy = K[1,0]

        correction = 0.77
        w_gt = 0.3183
        d = (fx*w_gt)/w
        print("The pillar is this far away: ", d)

        # p_cam = np.matmul(K, np.asarray([x,y,1]).T)

        # p_odom = np.matmul(self.T_odom_cam, p_cam)
        return d

    def calibrate_camera(self):
        # Define the size of the checkerboard
        checkerboard_size = (7, 10)

        # Define the length of each square on the checkerboard in meters
        square_size = 0.02

        # Load the image of the checkerboard
        img = cv2.imread('checkerboard.jpg')

        lwr = np.array([0, 0, 143])
        upr = np.array([179, 61, 252])

        # Convert the image to grayscale
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        msk = cv2.inRange(hsv, lwr, upr)

        krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
        dlt = cv2.dilate(msk, krn, iterations=5)
        res = 255 - cv2.bitwise_and(dlt, msk)

        # Displaying chess-board features
        res = np.uint8(res)
        cv2.imshow("board", hsv)
        pdb.set_trace()

        # Find the corners of the checkerboard
        ret, corners = cv2.findChessboardCorners(res, checkerboard_size, None)

        # If the corners are found, refine the corners to subpixel accuracy and draw them on the image
        if ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)

            # Define the object points of the checkerboard
            objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
            objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
            objp = objp * square_size

            # Get the intrinsic parameters of the camera
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([objp], [corners2], gray.shape[::-1], None, None)

            print("Camera matrix:")
            print(mtx)

            print("Distortion coefficients:")
            print(dist)
        else:
            print("Failed to find corners of the checkerboard")

        # Display the image
        cv2.imshow('img', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("occ_grid")
    occ_grid = RGBOccupancyGrid()
    # occ_grid.calibrate_camera()
    while not rospy.is_shutdown():
        pass