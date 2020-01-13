
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

        # define range of red color in HSV
        lower_red = np.array([0, 153, 100])
        upper_red = np.array([0, 175, 255])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        #mask = cv2.GaussianBlur(mask,(5,5),0)
        #ret,thresh1 = cv2.threshold(res, 100, 255, cv2.THRESH_BINARY)
        img  = cv2.cvtColor(res, cv2.COLOR_HSV2RGB)
        img2 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        th2  = cv2.adaptiveThreshold(img2,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,11,2)

        #Do morphological operation here

        #Do Hough circle detection here


        # findContours returns a list of the outlines of the white shapes in the mask (and a heirarchy that we shall ignore)
        image, contours, hierarchy = cv2.findContours(th2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
         
        # If we have at least one contour, look through each one and pick the biggest
        if len(contours)>0:
            largest = 0
            area = 0
            for i in range(len(contours)):
                # get the area of the ith contour
                temp_area = cv2.contourArea(contours[i])
                # if it is the biggest we have seen, keep it
                if temp_area > area:
                    area = temp_area
                    largest = i
            # Compute the coordinates of the center of the largest contour
            coordinates = cv2.moments(contours[largest])
            #print str(coordinates)
            if coordinates['m00'] != 0:
                target_x = int(coordinates['m10']/coordinates['m00'])
                print "Center X-coordinate: " + str(target_x)
                target_y = int(coordinates['m01']/coordinates['m00'])
                print "Center Y-coordinate: " + str(target_y)
                # Pick a suitable diameter for our target (grows with the contour)
                diam = int(np.sqrt(area)/4)
                print "diameter is: " + str(diam)
                # draw on a target
                cv2.circle(frame,(target_x,target_y),diam,(0,255,0),1)
                cv2.line(frame,(target_x-2*diam,target_y),(target_x+2*diam,target_y),(0,255,0),1)
                cv2.line(frame,(target_x,target_y-2*diam),(target_x,target_y+2*diam),(0,255,0),1)

        cv2.imshow('image',image)
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)




# When everything done, release the capture
vid.release()
cv2.destroyAllWindows()
