###############################################################################
# 
# opticalflow5.py
#
#   Python OpenCV test program to study optical flow.
#   Using sequential frame subtraction method
#   Source: https://www.pyimagesearch.com/2015/05/25/basic-motion-detection-and-tracking-with-python-and-opencv/
#   Adapted to use live webcam feed.
#
#   This implementation uses a simple frame subtraction and threshold method
#   to find differences between consecutive frames. The differences are
#   interpreted as motion. The delta image is thresholded and then searched for contours.
#   Because there may be many contours, the program finds the bounding box for
#   *all* contours, and then marks that 'super' bounding box on the video.
#
#   February 10, 2018
#
###############################################################################

import time
import sys
import numpy as np
import cv2

import opencvconst as cv

def main():
    """Captures input frame from webcam, calculates and displays frame difference"""
    
    #
    # Initialization
    #
    ref_time = time.time()
    output_string = ''
    dilate_kernel = np.ones((7, 7), np.uint8)
    min_y = 0
    max_y = 0
    min_x = 0
    max_x = 0
    cv2.namedWindow('frame', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('delta', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    
    #
    # Open the capture device and print some
    # useful properties
    #
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        #cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        #cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
        
        frameWidth = cap.get(cv.CV_CAP_PROP_FRAME_WIDTH)
        frameHeight = cap.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
        
        print 'frame: width {}, height {}'.format(frameWidth, frameHeight)
    
    #
    # Take first frame and find corners in it
    #
    cap_ok, frame = cap.read()
    if not cap_ok:
        sys.exit()

    prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    prev_frame = cv2.GaussianBlur(prev_frame, (21, 21), 0)

    while(True):

        cap_ok, frame = cap.read()
        if not cap_ok:
            break
        
        curr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        curr_frame = cv2.GaussianBlur(curr_frame, (21, 21), 0)

        #
        # Subtract one frame from the other to detect changes
        #
        frame_delta = cv2.absdiff(prev_frame, curr_frame)

        #
        # Threshold and dilation
        #
        thresh = cv2.threshold(frame_delta, 10, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, dilate_kernel, iterations = 1)
        
        #
        # Find contours, but instead of drawing all of them
        # only figure out: the topmost or lowest X pixel coordinate,
        # and the left and right most Y pixel coordinate between all
        # detected contours.
        #
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_contours = False
        coord_initialized = False
        
        for contour in contours:

            if not coord_initialized:               # initialize once
                min_y = 999
                max_y = -1
                min_x = 999
                max_x = -1
                coord_initialized = True
            
            found_contours = True
            (x, y, w, h) = cv2.boundingRect(contour)
            if y < min_y:
                min_y = y
            if (y+h) > max_y:
                max_y = (y+h)
            if x < min_x:
                min_x = x
            if (x+w) > max_x:
                max_x = x+w

        #
        # Draw the bounding rectangle over all contours
        #
        if found_contours:
            cv2.rectangle(frame, (min_x, min_y), (max_x, max_y), (0, 255, 0), 1)
        else:
            cv2.rectangle(frame, (min_x, min_y), (max_x, max_y), (0, 0, 255), 1)
        
        #
        # Display result and check for escape key
        #
        cv2.imshow('frame',frame)

        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break

        #
        # Now update the previous frame and previous points
        #
        prev_frame = curr_frame.copy()

    cv2.destroyAllWindows()
    cap.release()

#
# Startup
#
if __name__ == '__main__':
    main()
