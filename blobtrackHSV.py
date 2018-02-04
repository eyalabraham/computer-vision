###############################################################################
# 
# blobtrackHSV.py
#
#   Python OpenCV and Python Lego NXT implementing an object tracking
#   webcam mounted on a Lego pan-tilt device.
#   Using HSV color space for object color detection
#
#   January 9, 2018
#
###############################################################################

import numpy as np
import cv2
import opencvconst as cv

def main():
    global x0, y0, x1, y1, drawing, frameHSV, mode, lower, upper
    global MODE_TRACK, MODE_SHOW, MODE_MARK

    #
    # initialization
    #
    MODE_TRACK = 0          # track an object
    MODE_SHOW = 1           # only show tracking markers on video
    MODE_MARK = 2           # mark region color to track

    lower = np.array([0,0,0], dtype="uint8")
    upper = np.array([255,255,255], dtype="uint8")
    mode = MODE_SHOW
    mode_text = 'Show'
    drawing = False         # true if mouse is pressed
    x0, y0 = -1, -1
    x1, y1 = -1, -1
            
    print ' m - mark color region to track\n t - track\n s - display tracking marker only\n ESC - quit'

    #
    # link event callback function
    #
    cv2.namedWindow('image', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('image', mark_rect)

    #
    # setup font for overlay text 
    #
    font = cv2.FONT_HERSHEY_SIMPLEX

    #
    # kernel for dilation or erosion
    #
    dilate_kernel = np.ones((3, 3), np.uint8)
    erode_kernel = np.ones((5, 5), np.uint8)
    
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
    det_param.minArea = 500
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
        #cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        #cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
        
        frameWidth = cap.get(cv.CV_CAP_PROP_FRAME_WIDTH)
        frameHeight = cap.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
        
        print 'frame: width {}, height {}'.format(frameWidth, frameHeight)
        
        frameCenterX = int(frameWidth/2)
        frameCenterY = int(frameHeight/2)

    #
    # frame capture and processing loop
    #
    while(True):
        #
        # capture a frame
        # cover to appropriate color space to improve detection
        # in different lighting conditions
        #
        ret, frame = cap.read()
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #
        # operations on the frame done here
        #
        if mode == MODE_MARK:
            cv2.line(frame, (x0, y0), (x1, y0), (0, 255, 0), 1)
            cv2.line(frame, (x1, y0), (x1, y1), (0, 255, 0), 1)
            cv2.line(frame, (x1, y1), (x0, y1), (0, 255, 0), 1)
            cv2.line(frame, (x0, y1), (x0, y0), (0, 255, 0), 1)
        else:
            #
            # calculate tracking and show markers on video
            # create NumPy arrays from the 'upper' and lower' boundaries
            # source: https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
            # source: https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
            #
            mask = cv2.inRange(frameHSV, lower, upper)

            #
            # find blob and calculate center of mass and deviation from 
            # center of frame. this will be the tracking error
            # source: https://www.learnopencv.com/blob-detection-using-opencv-python-c/
            #
            mask = cv2.erode(mask, erode_kernel, iterations = 1)
            mask = cv2.dilate(mask, dilate_kernel, iterations = 1)
            mask = cv2.bitwise_not(mask)
            keypoints = detector.detect(mask)

            #
            # draw detected blobs 
            #
            if len(keypoints) == 1:
                #
                # draw a vector from frame center to block center of mass
                # calculate 'X' and 'Y' errors from center where
                #
                blobX = int(keypoints[0].pt[0])
                blobY = int(keypoints[0].pt[1])
                errX = frameCenterX - blobX
                errY = frameCenterY - blobY
                cv2.arrowedLine(frame, (frameCenterX, frameCenterY), (blobX, blobY), (0, 255, 0), 1)
                cv2.circle(frame, (blobX, blobY), int(keypoints[0].size), (0, 255, 0), 1)

            elif len(keypoints) == 0:
                #
                # red cross marker in center of frame if no
                # blob were detected
                #
                cv2.drawMarker(frame, (frameCenterX, frameCenterY), (0, 0, 255), cv2.MARKER_TILTED_CROSS)

            else:
                #
                # draw all detected blobs if more than one
                # were detected; ambiguous detection
                #
                frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        if mode == MODE_TRACK:
            #
            # TODO PID tracking and NXT pan-tilt control
            # TODO some form of filtering on the error
            #
            pass

        #
        # add text and markers to the image
        #
        cv2.putText(frame, mode_text, (1, 20), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        #
        # display the resulting frame
        #
        cv2.imshow('mask', mask)
        cv2.imshow('image', frame)
        
        #
        # key input mode/command
        #
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
        else:
            pass

    #
    # when done, release the capture
    #
    cap.release()
    cv2.destroyAllWindows()

#
# mouse event callback function
#
def mark_rect(event, x, y, flags, param):
    global x0, y0, x1, y1, drawing, frameHSV, mode, lower, upper
    global MODE_TRACK, MODE_SHOW, MODE_MARK

    if mode != MODE_MARK:
        return
        
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        x0, y0 = x, y
        x1, y1 = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            x1, y1 = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

        #
        # convert any start (x0,y0) and end (X1,Y1) points to be
        # a top-left to bottom right pair
        # extract ROI and calculate lower and upper RGB limit tuples
        #
        if x0 == x1 or y0 == y1 or x0 < 0 or x1 < 0 or y0 < 0 or y1 < 0:
            return
        if x0 > x1:
            x0, x1 = x1, x0
        if y0 > y1:
            y0, y1 = y1, y0

        roi = frameHSV[y0+1:y1, x0+1:x1]
        lower = np.array([np.amin(roi[:,:,0]), np.amin(roi[:,:,1]), np.amin(roi[:,:,2])], dtype="uint8")
        upper = np.array([np.amax(roi[:,:,0]), np.amax(roi[:,:,1]), np.amax(roi[:,:,2])], dtype="uint8")

if __name__ == '__main__':
    main()
