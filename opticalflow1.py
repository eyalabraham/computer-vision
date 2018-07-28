###############################################################################
# 
# opticalflow1.py
#
#   Python OpenCV test program to study optical flow.
#   Using Lucas-Kanade Optical Flow method for sparse points of interest.
#   Source: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html#goal
#   Adapted to use live webcam feed.
#
#   February 4, 2018
#
###############################################################################

import time
import numpy as np
import cv2

import opencvconst as cv

def main():

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
    # params for ShiTomasi corner detection
    #
    feature_params = dict( maxCorners = 100,
                           qualityLevel = 0.3,
                           minDistance = 7,
                           blockSize = 7 )

    #
    # Parameters for lucas kanade optical flow
    #
    lk_params = dict( winSize  = (15,15),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    #
    # Create some random colors
    #
    color = np.random.randint(0,255,(100,3))

    #
    # Take first frame and find corners in it
    #
    ret, old_frame = cap.read()
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

    #
    # Create a mask image for drawing purposes
    #
    mask = np.zeros_like(old_frame)
    
    while(True):
        ret,frame = cap.read()
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #
        # calculate optical flow
        #
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        #
        # Select good points
        #
        good_new = p1[st==1]
        good_old = p0[st==1]

        #
        # draw the tracks
        #
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
        img = cv2.add(frame,mask)

        cv2.imshow('frame',img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

        #
        # Now update the previous frame and previous points
        #
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)

    cv2.destroyAllWindows()
    cap.release()

#
# Startup
#
if __name__ == '__main__':
    main()
