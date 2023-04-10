#!/usr/bin/env python
"""
Dedicated to Alex, who is the best.
roslaunch rtabmap_ros rtabmap.launch args:="-d" stereo:=true left_image_topic:=/stereo/left/image_rect right_image_topic:=/stereo/right/image_rect left_camera_info_topic:=/stereo/left/camera_info right_camera_info_topic:=/stereo/right/camera_info
export ROS_IP=192.168.0.171
"""
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage

BRIDGE = CvBridge()

class OccupancyMap():
    ''' Takes in disparity map from camera to create low-fidelity occupancy map'''
    def __init__(self):
        # Disparity map paramters
        self.width = 848
        self.height = 100

        # Numpy array indicating occupied/unoccupied coordinates
        self.occupied_loc = np.array([])
        self.occupied_loc_real = np.array([])

        # disparity maps and heading information
        self.disp_map_msg = None
        self.disp_map = None
        self.disp_map_collapsed = None
        self.disp_map_max = None
        self.f = None
        self.T = None

        # Choose fixed row (z-value) from the disparity map to identify obstacles in x-y
        # note: +x is forward, +y is to the left
        self.z = 0

        self.depth_threshold = 0.5

    def calculate_depth(self):
        ''' Uses disparity values to calculate depth 
        From the DisparityImage documentation,
        Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.'''
        #self.depth_map = (self.f)*(self.T) /self.disp_map_collapsed
        #print(self.depth_map)
        cv2.reprojectImageTo3D()
        return

    def occupied_drone_frame(self, d_map):
        x_occ_drone = self.depth_map
        return

    def pixel_to_real(self):
        ''' Gets real-world location of pixels'''

    def get_occupied(self):
        ''' Gets real-world locations of obstacles'''
        return
    
    def disp_map_callback(self, msg):
        '''Disparity map callback'''
        print('here')
        # average across
        self.disp_map = BRIDGE.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
        self.disp_map_max = msg.max_disparity
        self.f = msg.f
        self.T = msg.T
        print(self.disp_map.shape)

        # Collapse into 1D since we don't need the z info (by averaging disparity values)
        self.disp_map_collapsed = np.mean(self.disp_map, axis=0)
        print(self.disp_map_collapsed.shape)
        self.calculate_depth()
        return
    
    def main_node(self):
        rospy.init_node("occupancy_map")
        self.disp_map_msg = rospy.Subscriber("/stereo/disparity", DisparityImage, self.disp_map_callback)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    map = OccupancyMap()
    map.main_node()