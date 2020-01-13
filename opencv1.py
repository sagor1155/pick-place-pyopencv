import numpy as np
import cv2
#from matplotlib import pyplot as plt

vid = cv2.VideoCapture(1)
ret = vid.set(3,640)
print str(ret)
ret2 = vid.set(4,480)
print str(ret2)

while(True):
 
    if cv2.waitKey(1) & 0xFF == ord('a'):
        break
    elif cv2.waitKey(1) & 0xFF == ord('s'):
        print "key pressed: S"
    else: 
        # Capture frame-by-frame
        ret, frame = vid.read()

        # change color space of the frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130, 255, 255])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        #thresholding for binary image
        ret,thresh1 = cv2.threshold(res, 100, 255, cv2.THRESH_BINARY)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C, 
                                    cv2.THRESH_BINARY,11,2)

        #(thresh, im_bw) = cv2.threshold(hsv[:,:,0], 200, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        #cv2.imshow('OTSU', im_bw)

        #cv2.imshow('frame',frame)
        #cv2.imshow('mask',mask)
        #cv2.imshow('res',res)
        #cv2.imshow('binary image', thresh1)
        #cv2.imshow('adaptive threshold', th2)

        # Display the resulting frame
        cv2.imshow('My Image', frame)
 
        



# When everything done, release the capture
vid.release()
cv2.destroyAllWindows()
