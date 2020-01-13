
import numpy as np
import cv2
#from matplotlib import pyplot as plt

vid = cv2.VideoCapture(0)

while(True):
 
    if cv2.waitKey(1) & 0xFF == ord('a'):
        break
    else: 
        # Capture frame-by-frame
        ret, frame = vid.read()

        # change color space of the frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv,5)
        # define range of red color in HSV
        #lower_red = np.array([0, 40, 40])
        #upper_red = np.array([0, 175, 255])
        lower_red = np.array([30,150,50])
        upper_red = np.array([255,255,180])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        #mask = cv2.GaussianBlur(res,(5,5),0)
        ret,thresh1 = cv2.threshold(res, 100, 255, cv2.THRESH_BINARY)
        #img  = cv2.cvtColor(thresh1, cv2.COLOR_HSV2RGB)
        #img2 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        #th2  = cv2.adaptiveThreshold(img2,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,11,2)

        kernel   = np.ones((5,5), np.uint8)
        erosion  = cv2.erode(thresh1, kernel, iterations = 1)
        dilation = cv2.dilate(erosion, kernel, iterations = 1)

        opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        cv2.imshow('frame', frame)
        cv2.imshow('res', res)
        #cv2.imshow('erosion', erosion)
        #cv2.imshow('dilation', dilation)
        cv2.imshow('opening', opening)
        cv2.imshow('closing', closing)




# When everything done, release the capture
vid.release()
cv2.destroyAllWindows()
