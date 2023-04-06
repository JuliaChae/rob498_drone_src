#!/usr/bin/python

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.

The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:
    --> note: distortion model may not be supported by stereo_image_proc

https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""

BRIDGE = CvBridge()

class Camera():
    def __init__(self):
        self.rc_raw_sub     = rospy.Subscriber('camera/fisheye1/image_raw',     Image, callback = self.set_right_camera_raw)
        self.rc_info_sub    = rospy.Subscriber('camera/fisheye1/camera_info',   CameraInfo, callback = self.set_right_camera_info)
        self.lc_raw_sub     = rospy.Subscriber('camera/fisheye2/image_raw',     Image, callback = self.set_left_camera_raw)
        self.lc_info_sub    = rospy.Subscriber('camera/fisheye2/camera_info',   CameraInfo, callback = self.set_left_camera_info)
        self.rc_undistorted_img_pub = rospy.Publisher('undistort/right/image_raw', Image, queue_size=10)
        self.lc_undistorted_img_pub = rospy.Publisher('undistort/left/image_raw', Image, queue_size=10)
        self.rc_info_pub = rospy.Publisher('stereo/right/camera_info', CameraInfo, queue_size=10)
        self.lc_info_pub = rospy.Publisher('stereo/left/camera_info', CameraInfo, queue_size=10)
        # self.rc_info_pub = rospy.Publisher('left/image_raw', Image, queue_size=10)
        # self.lc_info_pub = rospy.Publisher('right/image_raw', Image, queue_size=10)
        self.rc_raw = None
        self.rc_info = None
        self.lc_raw = None
        self.lc_info = None

    def set_right_camera_raw(self, image_data):
        self.rc_raw = BRIDGE.imgmsg_to_cv2(image_data, desired_encoding='passthrough')

    def set_left_camera_raw(self, image_data):
        self.lc_raw = BRIDGE.imgmsg_to_cv2(image_data, desired_encoding='passthrough')

    def set_right_camera_info(self, camera_info):
        self.rc_info = camera_info
        self.rc_info_pub.publish(camera_info)
    
    def set_left_camera_info(self, camera_info):
        self.lc_info = camera_info
        self.lc_info_pub.publish(camera_info)

    def get_camera_intrinsics(self, camera):
        if camera == "right":
            camera_info = self.rc_info
        elif camera == "left":
            camera_info = self.lc_info
        else:
            Exception("invalid camera specified!")

        print(camera_info.K, camera_info.R, camera_info.P, camera_info.D)
        K = np.array(camera_info.K).reshape((9,1)).reshape((3,3))
        R = np.array(camera_info.R).reshape((9,1)).reshape((3,3))
        P = np.array(camera_info.P).reshape((12,1)).reshape((3,4))

        return (K, camera_info.D, R, P)
    
    def undistort_images(self):
        min_disp = 0
        # must be divisible by 16
        num_disp = 112 - min_disp
        max_disp = min_disp + num_disp

        (K_left, D_left, R_left, P_left)  = self.get_camera_intrinsics("left")
        (K_right, D_right, R_right, P_right)  = self.get_camera_intrinsics("right")

        stereo_height_px = 300          # 300x300 pixel stereo output
        stereo_width_px = stereo_height_px + max_disp
        stereo_size = (stereo_width_px, stereo_height_px)

        m1type = cv2.CV_32FC1
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
        undistort_rectify = {
            "left"  : (lm1, lm2),
            "right" : (rm1, rm2)
        }

        center_undistorted = {
            "left" : cv2.remap(
                src = self.lc_raw,
                map1 = undistort_rectify["left"][0],
                map2 = undistort_rectify["left"][1],
                interpolation = cv2.INTER_LINEAR
            ),
            "right" : cv2.remap(
                src = self.rc_raw,
                map1 = undistort_rectify["right"][0],
                map2 = undistort_rectify["right"][1],
                interpolation = cv2.INTER_LINEAR
            )
        }
        self.lc_undistorted_img_pub.publish(BRIDGE.cv2_to_imgmsg(center_undistorted["left"], 'mono8'))
        self.rc_undistorted_img_pub.publish(BRIDGE.cv2_to_imgmsg(center_undistorted["right"], 'mono8'))
    
def publishing():
    camera = Camera()

    while camera.lc_raw is None or camera.lc_info is None or camera.rc_raw is None or camera.rc_info is None:
        continue

    while True:
        camera.undistort_images()

def testing():

    camera = Camera()

    while camera.lc_raw is None and camera.lc_info is None:
        continue
    
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 10
    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(
        minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    (K_left, D_left, R_left, P_left)  = camera.get_camera_intrinsics("left")
    (K_right, D_right, R_right, P_right)  = camera.get_camera_intrinsics("right")

    stereo_height_px = 300          # 300x300 pixel stereo output
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)

    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {
        "left"  : (lm1, lm2),
        "right" : (rm1, rm2)
    }

    mode = "stack"
    while True:
        center_undistorted = {
            "left" : cv2.remap(
                src = camera.lc_raw,
                map1 = undistort_rectify["left"][0],
                map2 = undistort_rectify["left"][1],
                interpolation = cv2.INTER_LINEAR
            ),
            "right" : cv2.remap(
                src = camera.rc_raw,
                map1 = undistort_rectify["right"][0],
                map2 = undistort_rectify["right"][1],
                interpolation = cv2.INTER_LINEAR
            )
        }

        # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
        disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

        # re-crop just the valid part of the disparity
        disparity = disparity[:,max_disp:]

        # convert disparity to 0-255 and color it
        disp_vis = 255*(disparity - min_disp)/ num_disp
        disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
        color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)

        if mode == "stack":
            cv2.imshow(WINDOW_TITLE, np.hstack((color_image, disp_color)))
        if mode == "overlay":
            ind = disparity >= min_disp
            color_image[ind, 0] = disp_color[ind, 0]
            color_image[ind, 1] = disp_color[ind, 1]
            color_image[ind, 2] = disp_color[ind, 2]
            cv2.imshow(WINDOW_TITLE, color_image)

        key = cv2.waitKey(1)
        if key == ord('s'): mode = "stack"
        if key == ord('o'): mode = "overlay"
        if key == ord('q'): #or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
            break

if _name_ == "_main_":
    rospy.init_node("camera_testing")
    publishing()