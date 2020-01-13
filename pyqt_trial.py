import sys
import numpy as np
import serial
import time
from ctypes import *
#from serialDev import commClass

global ser

def display(txt):
    print txt

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

if __name__ == "__main__":
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
            #return ser
        except:
            print "Failed To Connect: COM PORT ERROR !!!"
            constate = False
            del(ser)
            print "Disconnect Your Arduino Cable & Connect Again Then Retry"
            #exit()   
    while(True):
        input = raw_input(">> ")
        if input == 'x':
            ser.close()
            exit()
        else:
            data = ser.readline()
            print "Data is: " + data
            if data=="blue\r\n":
                display("start finding object")
            else:
                display("Didn't get command yet")
      


##if __name__ == "__main__":
##    ardcom = commClass()
##    ardcom.connect(ardcom, SIGNAL("displayThis(QString)"), display)
##    ardcom.serialConnect("COM7", 9600)
##    while(True):
##        print "Data: "
##        #ardcom.sendString("234")
##        sData = ardcom.readString()
##        print str(sData)
##        time.sleep(500)
    
