###############################################################################
#
# cspace.py
#
#   test ideas with OpenCV color spaces
#
###############################################################################

import cv2
import opencvconst as cv
import numpy as np

x0,y0 = -1,-1
clicked = False

def main():
    global x0,y0,clicked

    pFlag = False
    
    cv2.namedWindow('Image', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Image',mclick)
    
    cap = cv2.VideoCapture(0)
    cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
    
    ret, frame = cap.read()
    print 'frame', frame.shape
    
    while True:
    
        ret, frame = cap.read()
        cv2.imshow('Image', frame)

#        cvtYCrCb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
#        Y = cvtYCrCb[:,:,0]
#        Cr = cvtYCrCb[:,:,1]
#        Cb = cvtYCrCb[:,:,2]
#        
#        if not pFlag:
#            print 'cvtYCrCb=', cvtYCrCb.shape, 'Y=', Y.shape, 'Cr=', Cr.shape, 'Cb=', Cb.shape
#            pFlag = True
#        
#        if clicked:
#            print '(x={},y={}) Y={}, Cr={}, Cb={}'.format(x0,y0,Y[y0,x0],Cr[y0,x0],Cb[y0,x0])
#            clicked = False
#        
#        cv2.imshow('Y', Y)
#        cv2.imshow('Cr', Cr)
#        cv2.imshow('Cb', Cb)

#        cvtHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#        H = cvtHSV[:,:,0]
#        S = cvtHSV[:,:,1]
#        V = cvtHSV[:,:,2]
#        
#        if not pFlag:
#            print 'cvtHSV=', cvtHSV.shape, 'H=', H.shape, 'S=', S.shape, 'V=', V.shape
#            pFlag = True
#        
#        if clicked:
#            print '(x={},y={}) H={}, S={}, V={}'.format(x0,y0,H[y0,x0],S[y0,x0],V[y0,x0])
#            clicked = False
#        
#        cv2.imshow('H', H)
#        cv2.imshow('S', S)
#        cv2.imshow('V', V)
                
        cvtLab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        L = cvtLab[:,:,0]
        a = cvtLab[:,:,1]
        b = cvtLab[:,:,2]
        
        if not pFlag:
            print 'cvtLab=', cvtLab.shape, 'L=', L.shape, 'a=', a.shape, 'b=', b.shape
            pFlag = True
        
        if clicked:
            print '(x={},y={}) L={}, a={}, b={}'.format(x0,y0,L[y0,x0],a[y0,x0],b[y0,x0])
            clicked = False
        
        cv2.imshow('L', L)
        cv2.imshow('a', a)
        cv2.imshow('b', b)
    
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    #
    # when done, release the capture
    #
    cap.release()
    cv2.destroyAllWindows()

def mclick(event,x,y,flags,param):
    global x0,y0,clicked

    if event == cv2.EVENT_LBUTTONDOWN:
        x0,y0 = x,y
        clicked = True

#
# start
#
if __name__ == '__main__':
    main()
