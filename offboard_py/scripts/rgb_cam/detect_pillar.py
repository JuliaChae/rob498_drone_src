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
from geometry_msgs.msg import Pose, PoseArray

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

# Measured with ruler, assuming there's no rotaional offset
T_CAMERA_TO_PIXHAWK = np.eye(4)
T_CAMERA_TO_PIXHAWK[0, 3] = 0.115
T_CAMERA_TO_PIXHAWK[1, 3] = -0.06
T_CAMERA_TO_PIXHAWK[2, 3] = -0.05

NEW_OBS_DISTANCE_THRESHOLD = 0.2
MIN_FRAMES_FOR_NEW_PILLAR = 25
RADIUS = 0.159
MAX_POSSIBLE_DIST_TO_OBS = 5
MIN_POSSIBLE_DIST_TO_OBS = 1.2
MIN_DIST_BETWEEN_PILLARS = 2

class RGBOccupancyGrid:
    def __init__(self):
        self.bridge=CvBridge()
        self.detect_pub = rospy.Publisher("detected_pillars", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("imx219_image", Image, self.pillar_detection_callback)
        self.odom_sub = rospy.Subscriber("/mavros/odometry/out", Odometry, self.odom_callback)
        # Why was the queue size 0?
        self.obstacle_pub = rospy.Publisher("/obstacles_map", PoseArray, queue_size=10)
        self.transform_listener = tf.TransformListener()

        # This dictionary maps from obstacle ID to a dictionary of obstacle data, which includes
        # the x, y position and the number of frames that the obstacle has been tracked for.
        # It is being updated within the pillar_detection_callback function.
        self.tracked_obstacles = {}
        self.final_pillars = []
        # This is the pose of the pixhawk in the world frame from odom_sub
        self.pixhawk_to_world = np.eye(4)
        self.num = 0 
        self.rows = 540
        self.cols = 540
        
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
        self.pixhawk_to_world = np.eye(4)
        self.pixhawk_to_world[:3,:3] = R[:3, :3]
        self.pixhawk_to_world[:3, 3] = np.array([t.x, t.y, t.z]).T
        
    def pillar_detection_callback(self, img): 
        # Create a VideoCapture object and read from input file
        # If the input is the camera, pass 0 instead of the video file name
        # Read a frame from the video stream
        obs_type = None
        detected_dists = []
        frame = self.bridge.imgmsg_to_cv2(img)
        # ret = cv2.imwrite("/home/rob498/rob498_drone_ws/src/images/frame" + str(self.num) + ".jpg", frame)
        self.num += 1

        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the yellow and black colors
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Apply a morphological opening to the combined mask to remove noise
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        # Find the contours in the combined mask
        contours, hierarchy = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = -1 
        valid_pillar = None
        for cnt in contours: 
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            fx = K[0,0]
            w_gt = 0.3183
            d = (fx*w_gt)/w
            if area > 1000 and d > 0.5 and d < 3 and max_area < area and h > w:
                valid_pillar = [x, y, w, h]
                max_area = area 
        
        if valid_pillar is not None:
            x, y, w, h = valid_pillar
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            dist, x_m = self.get_obstacle(x,y,w,h,"full")
            detected_dists.append((dist, x_m))

        # cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        # Loop through the contours
        # for cnt in contours:
        #     detected_dists = []
        #     # Calculate the area of the contour
        #     area = cv2.contourArea(cnt)

        #     # If the contour area is greater than a threshold, it's likely a pillar
        #     if area > 1000:
        #         # Draw a bounding box around the contour
        #         x, y, w, h = cv2.boundingRect(cnt)
        #         cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #         # Calculate the aspect ratio of the bounding box
        #         aspect_ratio = float(w) / h

        #         # If the aspect ratio is close to 1, it's a fully yellow pillar
        #         if aspect_ratio >= 0.8 and aspect_ratio <= 1.2:
        #             dist, x_m = self.get_obstacle(x,y,w,h,"full")
        #             # print(dist)
        #             # print(x_m)
        #             # cv2.putText(frame, "Fully Yellow Pillar", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        #             cv2.putText(frame, str(round(dist, 3)), (x+100, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        #         # If the aspect ratio is not close to 1, it's a yellow and black striped pillar
        #         else:
        #             dist, x_m = self.get_obstacle(x,y,w,h,"partial")
        #             # print(dist)
        #             # print(x_m)
        #             cv2.putText(frame, str(round(dist, 3)), (x+100, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        #         detected_dists.append((dist, x_m))
            
        # Update the tracked obstacles
        self.update_tracked_obstacles(detected_dists)
                
        # Display the frame
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = rospy.Time.now()
        self.detect_pub.publish(image_msg)

    def get_obstacle(self, x, y, w, h, type):

        fx = K[0,0]
        fy = K[1,0]
        cx = K[0,2]

        correction = 0.77
        w_gt = 0.3183
        d = (fx*w_gt)/w

        x_m = (x - cx)/fx + RADIUS
        # print("The pillar is this far away: ", d)
        return d, x_m
    
    def update_tracked_obstacles(self, detected_dists):
        # Loop over each new obstacle detection
        for dist_pair in detected_dists:
            self.process_single_detection(dist_pair)
        
        # Check if mapping is complete. It's complete when there are 4 pillars
        # each with frame count over MIN_FRAMES_FOR_NEW_PILLAR
        # Grab all the detections in tracked_obstacles that have frame count over MIN_FRAMES_FOR_NEW_PILLAR
        long_term_detections = [d for d in self.tracked_obstacles.values() if d['frames'] > MIN_FRAMES_FOR_NEW_PILLAR]

        # If we haven't found the map yet, check if we can finalize the map
        if len(self.final_pillars) == 0:
            map_done, final_pillars = self.verify_final_map(long_term_detections)
            if map_done:
                self.final_pillars = final_pillars
        else:
            assert(len(self.final_pillars) == 4)

    def publish_final_map(self):
        if len(self.final_pillars) == 0:
            return
        else:
            assert(len(self.final_pillars) == 4)

        pose_array = PoseArray()
        # Set the frame ID for the PoseArray (optional)
        pose_array.header.frame_id = "pillars_map"
        # Loop over each sub-dictionary and create a new Pose object
        for pillar in self.final_pillars:
            pose = Pose()
            pose.position.x = pillar['x']
            pose.position.y = pillar['y']
            pose.position.z = 0.0

            # Add the Pose object to the PoseArray
            pose_array.poses.append(pose)

        # Publish the PoseArray
        self.obstacle_pub.publish(pose_array)

    def verify_final_map(self, candidates):
        if len(candidates) < 4:
            return False, None
        
        # Find the 4 best candidates. First sort by largest frame count first. Then, find 
        # the first 4 candidates that are each at least MIN_DIST_BETWEEN_PILLARS apart
        candidates.sort(key=lambda x: x['frames'], reverse=True)
        final_pillars = []
        for c in candidates:
            if len(final_pillars) == 4:
                break
            if len(final_pillars) == 0:
                final_pillars.append(c)
            else:
                # Check if the candidate is at least 
                # MIN_DIST_BETWEEN_PILLARS away from all the other pillars
                if all([np.linalg.norm(np.array([p['x'], p['y']]) - np.array([c['x'], c['y']])) > MIN_DIST_BETWEEN_PILLARS for p in final_pillars]):
                    final_pillars.append(c)

        if len(final_pillars) == 4:
            return True, final_pillars
        else:
            return False, final_pillars
                
    def process_single_detection(self, dist_pair):
        # Increase the distance by the radius of the pillar to get the center of the pillar
        # dist += 0.31
        dist = dist_pair[0]
        y_m = -dist_pair[1]

        # Threshold the distance to the pillar
        if dist > MAX_POSSIBLE_DIST_TO_OBS or dist < MIN_POSSIBLE_DIST_TO_OBS:
            return

        T_camera_to_world = np.matmul(self.pixhawk_to_world, T_CAMERA_TO_PIXHAWK)
        D_1 = np.sin(y_m/dist)
        D_2 = T_camera_to_world[0,0]
        alpha = np.arctan2(D_1, np.sqrt(1 - D_1**2))
        beta = np.arctan2(np.sqrt(1 - D_2**2), D_2)
        # print("alpha: ", alpha)
        # print("beta: ", beta)

        # Find the x,y coordinates of the pillar in the world frame
        pixhawk_x = T_CAMERA_TO_PIXHAWK[0,3] + dist * np.cos(alpha)
        pixhawk_y = -T_CAMERA_TO_PIXHAWK[1,3] + dist * np.sin(alpha)

        print("ph x: ", pixhawk_x)
        print("ph y: ", pixhawk_y)

        pixhawk_coord = np.asarray([pixhawk_x, pixhawk_y, self.pixhawk_to_world[2,3], 1]).T
        world_coord = np.matmul(self.pixhawk_to_world, pixhawk_coord)
        detected_pillar_x = world_coord[0]
        detected_pillar_y = world_coord[1]
        print((beta/np.pi)*180)
        print("pillar x: ", detected_pillar_x)
        print("pillar y: ", detected_pillar_y)
        
        # Find the closest tracked obstacle to the current detection
        closest_distance = float('inf')
        for track_id, obstacle in self.tracked_obstacles.items():
            obs_to_obs_dist = np.linalg.norm(np.array([detected_pillar_x, detected_pillar_y]) - np.array([obstacle['x'], obstacle['y']]))
            if obs_to_obs_dist < closest_distance:
                closest_distance = obs_to_obs_dist
                closest_track_id = track_id

        # Check if the closest obstacle to the current detection
        # is within a certain distance threshold to the current detection
        if closest_distance < NEW_OBS_DISTANCE_THRESHOLD:
            # Update the coordinates of the closest obstacle
            self.tracked_obstacles[closest_track_id]['x'] = detected_pillar_x
            self.tracked_obstacles[closest_track_id]['y'] = detected_pillar_y
            self.tracked_obstacles[closest_track_id]['frames'] += 1
        else:
            # This is a new detection not seen before, 
            # so create a new track ID for the current detection
            new_track_id = len(self.tracked_obstacles) + 1
            self.tracked_obstacles[new_track_id] = {
                'x': detected_pillar_x,
                'y': detected_pillar_y,
                'frames': 1
            }
        # print(self.tracked_obstacles)

    def visualize_map(self):
        # Create a new image
        img = np.zeros((800, 800, 3), np.uint8)
        # Loop through the tracked obstacles
        for track_id, obstacle in self.tracked_obstacles.items():
            if obstacle['frames'] < MIN_FRAMES_FOR_NEW_PILLAR:
                continue
            # Get the x and y coordinates of the obstacle
            x = obstacle['x']
            y = obstacle['y']
            # Draw the obstacle on the image
            cv2.circle(img, (x, y), 10, (0, 255, 0), -1)  
        # Display the image
        cv2.imshow('Detected Obstacles', img)
        cv2.waitKey(1)

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
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        occ_grid.publish_final_map()
        rate.sleep()
