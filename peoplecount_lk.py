###############################################################################
# 
# peoplecount.py
#
#   Python OpenCV program to count people entering and exiting a room.
#   Using Lucas-Kanade Optical Flow method for sparse points of interest.
#   Source: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html#goal
#   Adapted to use live webcam feed.
#
#   This implementation uses a webcam pointed across the room entry way,
#   and counts people entering and exiting the room.
#   The program uses two vertical lines of pixes, across which the optical flow
#   is measured using the Lucas-Kanade method.
#   A flow detected in a certain direction across one line then the next will
#   be interpreted as a enter or exit.
#   Because the camera is not mounted from above, counting might not be accurate.
#
#   February 11, 2018
#
###############################################################################

import time
import sys
import numpy as np
import cv2

import opencvconst as cv
import expfilter as ep

def main():
    """Captures input frame from webcam, calculates and displays optical flow for select pixels"""
    
    #
    # Initializations
    #
    trace_title = False
    trace = False
    ref_time = time.time()
    
    fps_text = '?? Fps'
    frame_count = 0
    start = time.time()
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.namedWindow('frame', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)

    sense_line1 = 240.0
    displacement1 = []
    sense_line2 = 400.0
    displacement2 = []
    
    abs_threshold = 15.0    
    room_occupancy = 0
    occupancy_text = 'Occupancy: 0'
    count = 0
    count_text = 'Count: 0'
    
    NO_CROSS = 0        # no crossing in progress
    CROSS_IN = 1        # object crossed first sense vector on way into room
    CROSS_OUT = 2       # object crossed second sense vector on way out of room
    CROSS_BLOCK = 4     # block counter to prevent spurious counting
    state = NO_CROSS
    inhibit_start = time.time()

         
    # Predefine points to track
    track_points1 = np.array([[[sense_line1,  60.0]],
                              [[sense_line1, 120.0]],
                              [[sense_line1, 180.0]],
                              [[sense_line1, 240.0]],
                              [[sense_line1, 300.0]],
                              [[sense_line1, 360.0]],
                              [[sense_line1, 420.0]]], 'float32')
                              
    track_points2 = np.array([[[sense_line2,  60.0]],
                              [[sense_line2, 120.0]],
                              [[sense_line2, 180.0]],
                              [[sense_line2, 240.0]],
                              [[sense_line2, 300.0]],
                              [[sense_line2, 360.0]],
                              [[sense_line2, 420.0]]], 'float32')
    
    # Parameters for Lucas-Kanade optical flow
    lk_params = dict( winSize  = (15,15),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    #
    # Construct two exponential filter objects.
    #
    f1 = ep.ExpFilter()
    f2 = ep.ExpFilter()
    
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

    while(True):

        cap_ok, frame = cap.read()
        if not cap_ok:
            break
        
        curr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #
        # Calculate optical flow of the two sets on points.
        # We separate this to make precessing easier later on.
        #
        next_points1, st1, err1 = cv2.calcOpticalFlowPyrLK(prev_frame, curr_frame, track_points1, None, **lk_params)
        next_points2, st2, err2 = cv2.calcOpticalFlowPyrLK(prev_frame, curr_frame, track_points2, None, **lk_params)

        #
        # Iterate through points and display on video frame
        # as well as output a CSV formated value list
        #
        del displacement1[:]
        for point_index in range(0, track_points1.shape[0]):

            track_point = np.int0(track_points1[point_index])
            x0,y0 = track_point.ravel()

            #
            # Make a list out of the X coordinate of the  tracked points
            #
            next_point = np.int0(next_points1[point_index])
            x1,y1 = next_point.ravel()
            displacement1.append(x0-x1)
        
            #
            # Display results on video frame
            #
            cv2.circle(frame, (x1,y1), 3, (0,0,255), -1)
            cv2.circle(frame, (x0,y0), 3, (0,255,0), -1)
        
        del displacement2[:]
        for point_index in range(0, track_points2.shape[0]):

            track_point = np.int0(track_points2[point_index])
            x0,y0 = track_point.ravel()

            #
            # Make a list out of the X coordinate of the  tracked points
            #
            next_point = np.int0(next_points2[point_index])
            x1,y1 = next_point.ravel()
            displacement2.append(x0-x1)
        
            #
            # Display results on video frame
            #
            cv2.circle(frame, (x1,y1), 3, (0,0,255), -1)
            cv2.circle(frame, (x0,y0), 3, (0,255,0), -1)

        #
        # Filter and calculate displacement and direction
        # over the two sensor lines.
        #
        vector1 = f1.filter(list_avg(displacement1))
        vector2 = f2.filter(list_avg(displacement2))
        
        #
        # State machine to track entries and exits
        # States:
        #   NO_CROSS    : no crossing in progress
        #   CROSS_IN    : object crossed first sense vector on way into room
        #   CROSS_OUT   : object crossed second sense vector on way out of room
        #   CROSS_BLOCK : block counter to prevent spurious counting
        #
        if vector1 < -abs_threshold:
            if state == NO_CROSS:
                state = CROSS_OUT
        
        elif vector1 > abs_threshold:
            if state == CROSS_IN:
                room_occupancy += 1
                count += 1
                state = CROSS_BLOCK
                inhibit_start = time.time()
        
        if vector2 < -abs_threshold:
            if state == CROSS_OUT:
                room_occupancy -= 1
                if room_occupancy < 0:
                    room_occupancy = 0
                state = CROSS_BLOCK
                inhibit_start = time.time()

        elif vector2 > abs_threshold:
            if state == NO_CROSS:
                state = CROSS_IN
        
        inhibit_time = time.time() - inhibit_start
        if state == CROSS_BLOCK and inhibit_time >= 1:
            state = NO_CROSS
        
        occupancy_text = 'Occupancy: {}'.format(room_occupancy)
        count_text = 'Count: {}'.format(count)
        
        #
        # Print out some trace data in a CSV format for graphing
        #
        now = time.time() - ref_time 
        if trace:
            if not trace_title:
                print 'rel_time,vector1,vector2'
                trace_title = True
            print '{:.2f},{:.2f},{:.2f}'.format(now, vector1, vector2)

        #
        # Calculate and display FPS.
        # Use the 10sec interval to also poll the NXT for battery level.
        #
        frame_count = frame_count + 1
        end = time.time()
        measure_interval = end - start
        if measure_interval > 10:
            fps = frame_count / measure_interval
            fps_text = '{:.2f} Fps'.format(fps)
            frame_count = 0
            start = time.time()

        #
        # Add information to the video
        #
        cv2.putText(frame, '1', (int(sense_line1), 50), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, '2', (int(sense_line2), 50), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, fps_text, (1, 20), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, occupancy_text, (1, 40), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, count_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        
        #
        # Display result and check for escape or keyboard command key
        #
        cv2.imshow('frame',frame)
        
        k = cv2.waitKey(1) & 0xff
        # exit
        if k == 27:
            break
        # toggle CSV trace
        if k == ord('t'):
            trace = not trace
        # reset occupancy counter
        if k == ord('r'):
            room_occupancy = 0
            count = 0
            state = NO_CROSS

        #
        # Now update the previous frame and previous points
        #
        prev_frame = curr_frame.copy()

    cv2.destroyAllWindows()
    cap.release()

###########################################################
#
# list_avg()
#
#   Remove outliers from a list and return the
#   element average without the outliers
#
#   param:  list of integers
#   return: average in float
#
def list_avg(l):
    """Calculate average of a list after removing outliers and return as float"""
    
    l.sort()
    return sum(l[1:-1])/float(len(l[1:-1]))

#
# Startup
#
if __name__ == '__main__':
    main()
