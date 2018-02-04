###############################################################################
#
# webcamperf.py
#
#   Assess FPS performance of webcam
#
#   January 20, 2018
#
###############################################################################

import cv2
import opencvconst as cv
import time

def main():
    """Assess FPS performance of webcam."""
    
    #
    # Open webcam device and set some capture properties
    #
    cap = cv2.VideoCapture(0)
    #cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
    #cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)

    print 'CV_CAP_PROP_FPS ', cap.get(cv.CV_CAP_PROP_FPS)
    print 'CV_CAP_PROP_FOURCC {:08x}'.format(int(cap.get(cv.CV_CAP_PROP_FOURCC)))
    print 'CV_CAP_PROP_CONVERT_RGB ', cap.get(cv.CV_CAP_PROP_CONVERT_RGB)

    # not supported by webcam
    #print "CV_CAP_PROP_GAIN ", cap.get(cv.CV_CAP_PROP_GAIN)
    
    #
    # Initialize tart time and frame count
    #
    frame_count = 0
    start = time.time()
    
    while True:
    
        ret, frame = cap.read()
        cv2.imshow('Image', frame)
        
        frame_count = frame_count + 1
        
        #
        # Calculate and display FPS
        #
        end = time.time()
        measure_interval = end - start
        if measure_interval > 10:
            fps = frame_count / measure_interval
            print 'FPS {:.2f}'.format(fps)
            frame_count = 0
            start = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    #
    # When done, release the capture
    #
    cap.release()
    cv2.destroyAllWindows()

#
# start
#
if __name__ == '__main__':
    main()
