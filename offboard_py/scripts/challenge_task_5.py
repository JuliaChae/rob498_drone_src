#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2 
import pdb 
import os
import matplotlib.pyplot as plt
# import imutils

class Yautja: 

    def __init__(self):
        # Initialize class variables
        self.name = 'rob498_drone_05'
        self.numbers = Int16MultiArray()
        self.OCR_dict = None
        self.corr_threshold = float(0.70)
        self.desired_width = 1000
        self.desired_template_width = 100
        self.kernel = 5
        self.sigma = 1
        self.expected = 4 # change to 4
        # Initialize ros subscribers and publishers 
        self.bridge = CvBridge()
        self.yautja_pub = rospy.Publisher(self.name + "/yautja_numbers", Int16MultiArray, queue_size=10)
        self.image_sub = rospy.Subscriber("imx219_image", Image, self.image_callback)
        self.visualize_pub = rospy.Publisher(self.name + "/yautja_viz", Image, queue_size=10)
        self.frame = None
        self.viz = Image()
        self.done = False

        self.gen_OCR_dict()

    def image_callback(self, img):
        img = self.bridge.imgmsg_to_cv2(img)

        scale_percent = (self.desired_width/float(img.shape[1]))*100 # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        img = cv2.resize(img, dim, interpolation = cv2.INTER_LINEAR)
        self.frame = img

        if self.OCR_dict is not None and self.done != True:
            self.detect_yautja()

    def getCol(self, arr):
        y = arr[0]
        return y

    def getVal(self,arr):
        val = arr[-1]
        return val
    
    def detect_yautja(self):
        print("Getting Yautja detections")
        detections = []
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        (thresh, self.frame) = cv2.threshold(self.frame, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        clone = np.dstack([self.frame, self.frame, self.frame])
        frame = self.frame
        
        while len(detections) < self.expected:
            print("len of detections: ", len(detections))
            for num in self.OCR_dict:
                print("testing num: ", num)
                template = self.OCR_dict[num]
                w, h = template.shape[::-1]
                found = None
                #loop over the scales of the image
                for scale in np.linspace(0.05, 2.0, 10)[::-1]:
                    # resize the image according to the scale, and keep track
                    # of the ratio of the resizing
                    blurred = cv2.GaussianBlur(frame, (self.kernel,self.kernel), self.sigma)

                    scale_percent = (int(blurred.shape[1] * scale)/float(blurred.shape[1]))*100 # percent of original size
                    width = int(blurred.shape[1] * scale_percent / 100)
                    height = int(blurred.shape[0] * scale_percent / 100)
                    dim = (width, height)
                    resized = cv2.resize(blurred, dim, interpolation = cv2.INTER_LINEAR)
                    # resized = imutils.resize(blurred, width = int(frame.shape[1] * scale))
                    r = frame.shape[1] / float(resized.shape[1])
                    # if the resized image is smaller than the template, then break
                    # from the loop

                    if resized.shape[0] < h or resized.shape[1] < w:
                        break

                    result = cv2.matchTemplate(resized, template, cv2.TM_CCOEFF_NORMED) 
                    inds = np.argwhere(result >= self.corr_threshold)
                    ys = inds[:,0]
                    xs = inds[:,1]
                    (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

                    if (found is None or maxVal > found[0]) and maxVal > self.corr_threshold:
                        print("found!")
                        found = (maxVal, maxLoc, xs, ys, result[ys, xs], r)
                
                if found is not None:
                    (maxVal, maxLoc, xs, ys, result, r) = found
                    if len(detections) != 0:
                        detections_np = np.asarray(detections)
                        if np.sum(np.abs(maxLoc[0]*r - detections_np[:,0]) < template.shape[0]/2) == 0: 
                            detections.append([int(maxLoc[0]*r), int(maxLoc[1]*r), num, r, maxVal])
                        else:
                            inds = np.argwhere(np.abs(maxLoc[0]*r - detections_np[:,0]) < template.shape[0]/2)
                            for ind in inds:
                                ind = ind[0]
                                if maxVal > detections[ind][-1]:
                                    detections.pop(ind)
                                    detections.append([int(maxLoc[0]*r), int(maxLoc[1]*r), num, r, maxVal])

                    else:
                        detections.append([int(maxLoc[0]*r), int(maxLoc[1]*r), num, r, maxVal])

                    for (x, y, res) in zip(xs, ys, result):
                        detections_np = np.asarray(detections)
                        if np.sum(np.abs(x*r - detections_np[:,0]) < template.shape[0]/2) == 0: 
                            detections.append([int(x*r), int(y*r), num, r, res])
                        else:    
                            inds = np.argwhere(np.abs(x*r - detections_np[:,0]) < template.shape[0]/2)
                            for ind in inds:
                                ind = ind[0]
                                if maxVal > detections[ind][-1]:
                                    detections.pop(ind)
                                    detections.append([int(x*r), int(y*r), num, r, res])
            self.corr_threshold *= 0.95
            print("candidate detections: ", detections)
        
        # Send out final numbers                    
        detections = sorted(detections, key=self.getVal, reverse=True)[:self.expected]
        detections_ordered = sorted(detections, key=self.getCol)
        numbers = [arr[2] for arr in detections_ordered]

        for detect in detections_ordered:
            (x, y, num, r, _) = detect
            maxLoc = (x,y)
            (startX, startY) = maxLoc
            (endX, endY) = (maxLoc[0] + int(w*r), maxLoc[1] + int(h*r))
            # draw a bounding box around the detected result and display the image
            cv2.rectangle(clone, (startX, startY), (endX, endY), (0, 0, 255), 2)
            cv2.putText(clone, str(num), (endX + 10, endY + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        print(detections_ordered)
        self.viz = self.bridge.cv2_to_imgmsg(clone, encoding='bgr8')
        self.numbers = Int16MultiArray()
        self.numbers.data = numbers
        self.done = True
        # self.yautja_pub.publish(self.numbers)
        # self.visualize_pub.publish(viz)
            
        # cv2.imwrite("result.png", clone)
        # cv2.imshow("Image", clone)
        # cv2.waitKey(0)

    def gen_OCR_dict(self):
        OCR_dict = {}
        dir_path = "/home/rob498/rob498_drone_ws/src/offboard_py/scripts/yautja"
        for file in os.listdir(dir_path):
            file_path = os.path.join(dir_path, file)
            parts = file.split("_")

            num = parts[-1].split("-")[-1].strip(".png")

            img = cv2.imread(file_path)	
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = img[750:2650, 300:2250]
            scale_percent = ((self.desired_template_width)/float(img.shape[1]))*100 # percent of original size
            width = int(img.shape[1] * scale_percent / 100)
            height = int(img.shape[0] * scale_percent / 100)
            dim = (width, height)
            # resize image
            img = cv2.resize(img, dim, interpolation = cv2.INTER_LINEAR)
            (thresh, img) = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            img = cv2.GaussianBlur(img, (self.kernel,self.kernel), self.sigma)
            OCR_dict[int(num)] = img

            # cv2.imshow("ocr", img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        
        self.OCR_dict = OCR_dict
        print("OCR done")
        
if __name__ == "__main__":
    rospy.init_node("yautja")
    detect_yautja = Yautja()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        detect_yautja.yautja_pub.publish(detect_yautja.numbers)
        detect_yautja.visualize_pub.publish(detect_yautja.viz)
        rate.sleep()



    