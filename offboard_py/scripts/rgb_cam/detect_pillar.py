import numpy as np
import cv2 
import imutils 
import argparse 

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
# ap.add_argument("-s", "--server-ip",
#                 help="ip address of the server to which the client will connect")
ap.add_argument("-v", "--video", help="path to the video file")
ap.add_argument("-a", "--min-area", type=int, default=500, help="minimum area size")
args = vars(ap.parse_args())

lower_yellow = (20, 100, 100)
upper_yellow = (30, 255, 255)

lower_black = (0, 0, 0)
upper_black = (30, 30, 30)

def stream_video(): 
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

def pillar_detection(): 
    # Create a VideoCapture object and read from input file
    # If the input is the camera, pass 0 instead of the video file name
    cap = cv2.VideoCapture(args['video'])
    
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    
    # Read until video is completed
    ind = 0
    while True:
        # Read a frame from the video stream
        ret, frame = cap.read()

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
                    cv2.putText(frame, "Fully Yellow Pillar", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # If the aspect ratio is not close to 1, it's a yellow and black striped pillar
                else:
                    cv2.putText(frame, "Yellow and Black Striped Pillar", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow("Frame", frame)

        # Exit the loop if the "q" key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video stream and close all windows
    cap.release()
    cv2.destroyAllWindows()

def calibrate_camera():
    # Define the size of the checkerboard
    checkerboard_size = (7, 9)

    # Define the length of each square on the checkerboard in meters
    square_size = 0.1

    # Load the image of the checkerboard
    img = cv2.imread('checkerboard.jpg')

    lwr = np.array([0, 0, 143])
    upr = np.array([179, 61, 252])

    # Convert the image to grayscale
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    msk = cv2.inRange(hsv, lwr, upr)

    krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
    dlt = cv2.dilate(msk, krn, iterations=5)
    res = 255 - cv2.bitwise_and(dlt, msk)

    # Displaying chess-board features
    res = np.uint8(res)
    cv2.imshow("board", res)

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
    calibrate_camera()