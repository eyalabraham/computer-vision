###############################################################################
# 
# opticalflow2.py
#
#   Python OpenCV test program to study optical flow.
#   Using Dense Optical Flow method.
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

    cv2.namedWindow('frame2', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    
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

    ret, frame1 = cap.read()
    prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    hsv = np.zeros_like(frame1)
    hsv[...,1] = 255

    while(True):
        ret, frame2 = cap.read()
        next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        hsv[...,0] = ang*180/np.pi/2
        hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

        cv2.imshow('frame2',rgb)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
#        elif k == ord('s'):
#            cv2.imwrite('opticalfb.png',frame2)
#            cv2.imwrite('opticalhsv.png',rgb)

        prvs = next

    cap.release()
    cv2.destroyAllWindows()

#
# Startup
#
if __name__ == '__main__':
    main()
