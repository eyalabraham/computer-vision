###############################################################################
# 
# opticalflow4.py
#
#   Python OpenCV test program to study optical flow.
#   Using Lucas-Kanade Optical Flow method for sparse points of interest.
#   Source: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html#goal
#   Adapted to use live webcam feed.
#
#   This implementation predefines a set of points on the source image for the
#   LK algorithm to use, and will use the same points on every iteration.
#   This is instead of using the goodFeaturesToTrack() method to define the points.
#   The program will then attempt to draw vectors from the test points to the output
#   points described by calcOpticalFlowPyrLK().
#   This seems to work even though common recommendation is to use goodFeaturesToTrack()
#   in order to provide best corner points to track.
#
#   February 4, 2018
#
###############################################################################

import time
import sys
import numpy as np
import cv2

import opencvconst as cv

def main():
    """Captures input frame from webcam, calculates and displays optical flow for select pixels"""
    
    #
    # Initialization
    #
    ref_time = time.time()
    output_string = ''        
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
    # Parameters for Lucas-Kanade optical flow
    #
    lk_params = dict( winSize  = (15,15),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    #
    # Predefine points to track
    #
    track_points = np.array([[[220.0, 120.0]],
                             [[220.0, 200.0]],
                             [[220.0, 280.0]],
                             [[220.0, 360.0]],
                             [[420.0, 120.0]],
                             [[420.0, 200.0]],
                             [[420.0, 280.0]],
                             [[420.0, 360.0]]], 'float32')
    
    #
    # Take first frame and find corners in it
    #
    cap_ok, frame = cap.read()
    if not cap_ok:
        sys.exit()

    prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    print 'rel_time,p0dx,p0dy,p1dx,p1dy,p2dx,p2dy,p3dx,p3dy,p4dx,p4dy,p5dx,p5dy,p6dx,p6dy,p7dx,p7dy'

    while(True):

        cap_ok, frame = cap.read()
        if not cap_ok:
            break
        
        curr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #
        # Calculate optical flow
        #
        next_points, st, err = cv2.calcOpticalFlowPyrLK(prev_frame, curr_frame, track_points, None, **lk_params)

        #
        # Iterate through points and display on video frame
        # as well as output a CSV formated value list
        #
        for point_index in range(0, track_points.shape[0]):
        
            #
            # Display results on video frame
            #
            track_point = np.int0(track_points[point_index])
            x0,y0 = track_point.ravel()
            cv2.circle(frame, (x0,y0), 5, (0,255,0), -1)

            next_point = np.int0(next_points[point_index])
            x1,y1 = next_point.ravel()
            cv2.circle(frame, (x1,y1), 5, (0,0,255), -1)

            #
            # Build CSV string
            #
            output_string += ',{:.2f},{:.2f}'.format(x0-x1, y0-y1)
            
        #
        # Print out some data in a CSV format for graphing
        #
        now = time.time() - ref_time 
        print '{:.2f}{}'.format(now, output_string)
        output_string = ''

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
