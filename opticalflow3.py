###############################################################################
# 
# opticalflow3.py
#
#   Python OpenCV test program to study optical flow.
#   Source: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html#goal
#       and https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_shi_tomasi/py_shi_tomasi.html
#   Test program to study Shi-Tomasi Corner Detector
#
#   February 4, 2018
#
###############################################################################

import time
import numpy as np
import cv2

import opencvconst as cv

def main():

    #
    # General initializations
    #
    cv2.namedWindow('frame', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    
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
    # Create arams for ShiTomasi corner detection
    #
    feature_params = dict( maxCorners = 100,
                           qualityLevel = 0.3,
                           minDistance = 7,
                           blockSize = 7 )

    while(True):
        #
        # Captuer another frame
        #
        cap_ok, frame = cap.read()
        if not cap_ok:
            break

        #
        # Find feature to track in the latest read image
        # goodFeaturesToTrack() requires a gray scale image so convert before detecting
        # Source for Shi-Tomasi Corner Detector: https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_shi_tomasi/py_shi_tomasi.html
        #
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        good_features = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
        corners = np.int0(good_features)
        for i in corners:
            x,y = i.ravel()
            cv2.circle(frame, (x,y), 5, (0,0,255), -1)
        
        #
        # Display results
        #
        cv2.imshow('frame', frame)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    #
    # Close the capture and window then exit
    #
    cv2.destroyAllWindows()
    cap.release()
        

#
# Startup
#
if __name__ == '__main__':
    main()
