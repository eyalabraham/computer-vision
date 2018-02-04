###############################################################################
# 
# blobtrackRGB.py
#
#   Python OpenCV and Python Lego NXT implementing an object tracking
#   webcam mounted on a Lego pan-tilt device.
#   Using RGB color space to find object.
#
#   January 6, 2018
#
###############################################################################

import numpy as np
import cv2
import opencvconst as cv

def main():
    global x0,y0,x1,y1,drawing,frame,mode,lower,upper
    global MODE_TRACK,MODE_SHOW,MODE_MARK

    #
    # initialization
    #
    MODE_TRACK = 0          # track an object
    MODE_SHOW = 1           # only show tracking narkers on video
    MODE_MARK = 2           # mark region color to track

    lower = np.array([0,0,0],dtype="uint8")
    upper = np.array([255,255,255],dtype="uint8")
    mode = MODE_SHOW
    mode_text = 'Show'
    drawing = False         # true if mouse is pressed
    x0,y0 = -1,-1
    x1,y1 = -1,-1
            
    print ' m - mark color region to track\n t - track\n s - display tracking marker only\n ESC - quit'

    #
    # link event callback funtion
    #
    cv2.namedWindow('image', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('image',mark_rect)

    #
    # setup font for overlay text 
    #
    font = cv2.FONT_HERSHEY_SIMPLEX

    #
    # Set up a blob detector with some parameters
    #
    det_param = cv2.SimpleBlobDetector_Params()
    #det_param.thresholdStep = 1
    #det_param.minThreshold = 0
    #det_param.maxThreshold = 0
    #det_param.minRepeatability = 10
    #det_param.minDistBetweenBlobs = 10
    det_param.filterByColor = False
    det_param.filterByCircularity = False
    det_param.filterByInertia = False
    det_param.filterByConvexity = False
    det_param.filterByArea = True
    det_param.minArea = 1000
    det_param.maxArea = 10000
    if cv2.__version__.startswith("3."):
        detector = cv2.SimpleBlobDetector_create(det_param)
    else:
        detector = cv2.SimpleBlobDetector(det_param)

    #
    # open the capture device and print some
    # useful properties
    #
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        frameWidth = cap.get(cv.CV_CAP_PROP_FRAME_WIDTH)
        frameHeight = cap.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
        print 'frame: width {}, height {}'.format(frameWidth, frameHeight)

    #
    # frame capture and processing loop
    #
    while(True):
        # capture a frame
        # TODO consider color conversions to LAB or HSV etc
        #      to get better object detection in different lighting conditions
        ret, frame = cap.read()

        # operations on the frame done here
        if mode == MODE_MARK:
            cv2.line(frame,(x0,y0),(x1,y0),(0,255,0),1)
            cv2.line(frame,(x1,y0),(x1,y1),(0,255,0),1)
            cv2.line(frame,(x1,y1),(x0,y1),(0,255,0),1)
            cv2.line(frame,(x0,y1),(x0,y0),(0,255,0),1)
        else:
            # calculate tracking and show markers on video
            # create NumPy arrays from the 'upper' and lower' boundaries
            # source: https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
            # source: https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
            # TODO apply filtering such as dilate() or other model to the 'mask'
            mask = cv2.inRange(frame, lower, upper)
            #output = cv2.bitwise_and(frame, frame, mask = mask)

            # TODO find blob and calculate center of mass and deviation from 
            #      center of frame. this will be the tracking error
            # source: https://www.learnopencv.com/blob-detection-using-opencv-python-c/
            # TODO consider error filtering such as Kalman or Exponential moving Average (EMA)
            # Detect blobs
            mask = cv2.bitwise_not(mask)
            keypoints = detector.detect(mask)

            # Draw detected blobs as red circles.
            # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
            frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        if mode == MODE_TRACK:
            # TODO PID tracking and NXT pan-tilt control
            pass

        # add text and markers to the image
        cv2.putText(frame, mode_text, (1,20), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
        
        # display the resulting frame
        cv2.imshow('mask', mask)
        #cv2.imshow('output', output)
        cv2.imshow('image', frame)
        cv2.imshow('blob', frame_with_keypoints)
        
        # key input mode/command
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('m'):
            x0,y0  = -1,-1
            x1,y1  = -1,-1
            mode_text = 'Mark'
            mode = MODE_MARK
        elif key == ord('t'):
            mode_text = 'Track'
            mode = MODE_TRACK
        elif key == ord('s'):
            mode_text = 'Show'
            mode = MODE_SHOW

    # when done, release the capture
    cap.release()
    cv2.destroyAllWindows()

#
# mouse callback function
#
def mark_rect(event,x,y,flags,param):
    global x0,y0,x1,y1,drawing,frame,mode,lower,upper
    global MODE_TRACK,MODE_SHOW,MODE_MARK

    if mode != MODE_MARK:
        return
        
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        x0,y0 = x,y
        x1,y1 = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            x1,y1 = x,y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

        # convert any start (x0,y0) and end (X1,Y1) points to be
        # a top-left to bottom right pair
        # extract ROI and calculate lower and upper RGB limit tuples
        if x0 == x1 or y0 == y1 or x0 < 0 or x1 < 0 or y0 < 0 or y1 < 0:
            return
        if x0 > x1:
            x0,x1 = x1,x0
        if y0 > y1:
            y0,y1 = y1,y0
        roi = frame[y0+1:y1,x0+1:x1]
        lower = np.array([np.amin(roi[:,:,0]),np.amin(roi[:,:,1]),np.amin(roi[:,:,2])],dtype="uint8")
        upper = np.array([np.amax(roi[:,:,0]),np.amax(roi[:,:,1]),np.amax(roi[:,:,2])],dtype="uint8")

if __name__ == '__main__':
    main()
