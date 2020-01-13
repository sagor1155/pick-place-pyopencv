import cv2
import numpy as np
import time

class imageprocess:
    def __init__(self, parent=None):
        print "Object initialized"
    
    def detectColorObject(self, rgbImage, lowerThreshold, upperThreshold):
    
        frame = cv2.medianBlur(rgbImage,5)
        # change color space of the frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv,5)
            
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask= mask)

        #mask = cv2.GaussianBlur(res,(5,5),0)
        #ret,thresh1 = cv2.threshold(res, 100, 255, cv2.THRESH_BINARY)
        #img  = cv2.cvtColor(mask, cv2.COLOR_HSV2RGB)
        #img2 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        #th2  = cv2.adaptiveThreshold(img2,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,11,2)

        kernel   = np.ones((5,5), np.uint8)
        erosion  = cv2.erode(res, kernel, iterations = 1)
        dilation = cv2.dilate(erosion, kernel, iterations = 1)

        opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        imageRGB  = cv2.cvtColor(closing, cv2.COLOR_HSV2RGB)
        imageGRAY = cv2.cvtColor(imageRGB, cv2.COLOR_RGB2GRAY)
        
        cv2.imshow('res', res)
        #cv2.imshow('erosion', erosion)
        #cv2.imshow('dilation', dilation)
        cv2.imshow('opening', opening)
        cv2.imshow('closing', closing)
            
        return imageGRAY


    def getObjectPosition(self, mainImage, processedImage):
        # findContours returns a list of the outlines of the white shapes in the mask (and a heirarchy that we shall ignore)
        image, contours, hierarchy = cv2.findContours(processedImage,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
             
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
            target_x = 0
            target_y = 0
            if coordinates['m00'] != 0:
                target_x = int(coordinates['m10']/coordinates['m00'])
                print "Center X-coordinate: " + str(target_x)
                target_y = int(coordinates['m01']/coordinates['m00'])
                print "Center Y-coordinate: " + str(target_y)
                # Pick a suitable diameter for our target (grows with the contour)
                diam = int(np.sqrt(area)/4)
                print "diameter is: " + str(diam)
                # draw on a target
                cv2.circle(mainImage,(target_x,target_y),diam,(0,255,0),1)
                cv2.line(mainImage,(target_x-2*diam,target_y),(target_x+2*diam,target_y),(0,255,0),1)
                cv2.line(mainImage,(target_x,target_y-2*diam),(target_x,target_y+2*diam),(0,255,0),1)
                return (target_x, target_y, diam)
        return (0, 0, 0)


