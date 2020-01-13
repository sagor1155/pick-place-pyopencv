import atexit
import numpy as np
import cv2
import time
import serial
import math
import sys


#global ser
ser = serial.Serial()

#RED Threshold (nice)
lower_red = np.array([0, 120, 50])
upper_red = np.array([50, 255, 255])

#GREEN Threshold (nice)
lower_green = np.array([30,150,50])
upper_green = np.array([100,255,180])

#BLUE Threshold
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

# define range of red/green/blue color in HSV
lower_rgb = np.array([30,150,50])
upper_rgb = np.array([255,255,180])

onePxToCm = 0.0359
apx = 320
apy = (847 - 70)

camera_port = 1
color = " "    # "r" for RED, "g" for GREEN, "b" for BLUE and "rgb" for RED/GREEN/BLUE
positionStr = " "
numCountLimit = 100

print "Program Started"

def searchPort():
    ports = ["COM%s" % (i + 1) for i in range(256)]
    result = []
    s = []
    for a_port in ports:
        try:
            s = serial.Serial(a_port)
            s.close()
            result.append(a_port)
        except serial.SerialException:
            pass

    if result != []:
        print "Ports Found" + '\n'
        print result
        del(s)
    else:
        print "Ports Not Found!!!" + '\n'
        del(s)

def connect():
    searchPort()
    constate = False
    while(constate != True):
        prt = raw_input("Enter Arduino Port Name: ")
        try:
            ser = serial.Serial()
            ser.port=prt
            ser.baudrate=9600
            ser.parity=serial.PARITY_NONE
            ser.stopbits=serial.STOPBITS_ONE
            ser.bytesize=serial.EIGHTBITS
            ser.open()
            
            ser.flushInput()
            ser.flushOutput()
            print "Successfully Connected With Arduino"
            constate = True
            return ser
        except:
            print "Failed To Connect: COM PORT ERROR !!!"
            constate = False
            del(ser)
            print "Disconnect Your Arduino Cable & Connect Again Then Retry"
            #exit()    

 
def detectColorObject(rgbImage, lowerThreshold, upperThreshold):
    
    frame = cv2.medianBlur(rgbImage,5)
    # change color space of the frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.medianBlur(hsv,5)
        
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

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
    
    #cv2.imshow('res', res)
    #cv2.imshow('erosion', erosion)
    #cv2.imshow('dilation', dilation)
    #cv2.imshow('opening', opening)
    cv2.imshow('closing', closing)
    cv2.waitKey(1)
        
    return imageGRAY


def getObjectPosition(mainImage, processedImage):
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
            cv2.waitKey(1)
            return (target_x, target_y, diam)
    return (0, 0, 0)    


##def exit_handler():
    #print 'My application is ending!'
    #time.sleep(2)
    #ser.close()
    #del(ser)
    #del(vid)

##atexit.register(exit_handler()) ##sys.exit(exit_handler())


def objectPositionWrtARM(opx, opy):
    AB = apy - opy
    BC = apx - opx
    dist = math.sqrt(pow(AB, 2) + pow(BC, 2))       ##AC
    cosTheta = (float)(AB/dist)
    thetaRad = math.acos(cosTheta)
    thetaDeg = float((thetaRad * 180)/3.1416)
    theta = (int)(math.ceil(thetaDeg))
    print "Theta is: " + str(theta)
    distCm = dist * onePxToCm
    distCm = (int)(math.floor(distCm))
    print "Distance is: " + str(distCm)
    if opx<320:
        print "Left Angle"
    else:
        print "Right Angle"
    
    return (distCm, theta)
    

if __name__ == "__main__":
    searchPort()
    constate = False
    while(constate != True):
        prt = raw_input("Enter Arduino Port Name: ")
        try:
            #ser = serial.Serial()
            ser.port=prt
            ser.baudrate=9600
            ser.parity=serial.PARITY_NONE
            ser.stopbits=serial.STOPBITS_ONE
            ser.bytesize=serial.EIGHTBITS
            ser.open()
            
            ser.flushInput()
            ser.flushOutput()
            print "Successfully Connected With Arduino"
            constate = True
        except:
            print "Failed To Connect: COM PORT ERROR !!!"
            constate = False
            del(ser)
            print "Disconnect Your Arduino Cable & Connect Again Then Retry"
 
     
    while(True):
        objDetected = False
        flag = False
        print "Waiting for command..."
        data = ser.readline()
        ser.flushInput()
        ser.flushOutput()
        print "Data is: " + data
        if data=="4\r\n":
            print "Got Exit Command"
            break
        elif data=="1\r\n" or data=="2\r\n" or data=="3\r\n":
            if data=="1\r\n":
                color = "r"
                print "Start Finding RED Object"
                time.sleep(1)
            elif data=="2\r\n":
                color = "g"
                print "Start Finding GREEN Object"
                time.sleep(1)
            elif data=="3\r\n":
                color = "b"
                print "Start Finding BLUE Object"
                time.sleep(1)
    
            count = 0
            vid = cv2.VideoCapture(camera_port)
            ret = vid.set(3,640)
            #print str(ret)
            ret2 = vid.set(4,480)
            #print str(ret2)
            while(count<numCountLimit):
                count = count + 1
                if cv2.waitKey(1) & 0xFF == ord('a'):  
                    vid.release()
                    cv2.destroyAllWindows() 
                    #ser.close()
                    break
                else: 
                    # Capture frame-by-frame
                    
                    ret, frm = vid.read()
                    #rows,cols = frm.shape 
                    M = cv2.getRotationMatrix2D((320,240),180,1)
                    frame = cv2.warpAffine(frm, M,(640,480))
                    cv2.imshow('frame', frame)
                    if objDetected == False:
                        
                        if color == "r":
                            try:
                                processedImage = detectColorObject(frame, lower_red, upper_red)
                                x, y, d = getObjectPosition(frame, processedImage)
                            except Exception, e:
                                print "Error Ocurred"
                                print str(e)
                                vid.release()
                                cv2.destroyAllWindows()
                        elif color == "g":
                            try:
                                processedImage = detectColorObject(frame, lower_green, upper_green)
                                x, y, d = getObjectPosition(frame, processedImage)
                            except:
                                print "Error Ocurred"
                                vid.release()
                                cv2.destroyAllWindows()

                        elif color == "b":
                            try:
                                processedImage = detectColorObject(frame, lower_blue, upper_blue)
                                x, y, d = getObjectPosition(frame, processedImage)
                            except:
                                print "Error Ocurred"
                                vid.release()
                                cv2.destroyAllWindows()
                                
                        elif color == "rgb":
                            try:
                                processedImage = detectColorObject(frame, lower_rgb, upper_rgb)
                                x, y, d = getObjectPosition(frame, processedImage)
                            except:
                                print "Error Ocurred"
                                vid.release()
                                cv2.destroyAllWindows()

                        #print "printing diameter:  " + str(d)
                        
                        if d>5 and d<25:
                            objDetected = True
                            flag = True
                            count = numCountLimit
                            print " ****** Object Detected at position: (" + str(x) + "," + str(y) + ")" + "******"

                            #### send coordinate to arduino
                            print "Sending Object Position To Arduino"
                            positionStr = str(int(x)) + ',' + str(int(y)) + '\n'

                            #### send distance & Angle to arduino
                            distance, angle = objectPositionWrtARM(x , y)
                            if x<320:
                                anglePlane = 1 ##left side
                            else:
                                anglePlane = 2  ##right side 
                            distAngle = str(distance) + ',' + str(angle) + ',' + str(anglePlane) + '\n' 

                            ser.writelines(distAngle)
                            print "Waiting For Feedback"
                            time.sleep(1)
                            print str(ser.readline())
                                    
                            time.sleep(1)
                            vid.release()
                            cv2.destroyAllWindows()

                        else:
                            objDetected = False
                            flag = False
            if objDetected==False and count==numCountLimit:
                print "Timeout: No Object Found !!!"
                time.sleep(2)
                vid.release()
                cv2.destroyAllWindows()
                time.sleep(1)
                    
        else:
            print "Didn't get command yet"
       

    # When everything done, release the capture
    ser.close()
    #vid.release()
    #cv2.destroyAllWindows()

