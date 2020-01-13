import serial
import time
import winsound
from PyQt4.QtCore import *

class commClass(QThread):
    def __init__(self, parent=None):
        super(commClass, self).__init__(parent)
        print "Serial Object Instantiated"
        global ser

    isArdThreadStop = False

    def serialConnect(self, portN, bps):
        try:
            #ss = serial.Serial()
            self.ser = serial.Serial(
                port=portN,
                baudrate=bps,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            print "Successfully Connected With Arduino"
        except:
            #ss.close()
            print "Failed To Connect"

    def serialDisconnect(self):
        self.ser.close()

    def sendByte(self, databyte):
        try:
            self.ser.write(databyte)
        except:
            print "Error !!!"

    def sendString(self, datastring):
        try:
            self.ser.writelines(datastring)
        except:
            print "Error !!!"

    #check whether meter is connected or not
    def IsArdConnected(self): 
        self.conState = False
        if self.ser.isOpen()==True:
            self.conState = True
        else:
            self.conState = False
        return self.conState

    def checkRespone(self, command, response):

        feedback = response
        responseState = False
        try:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.sendString(command)

            out = ''
            timeout = 0
            flag = 0
            print "waiting for response"
            while flag == 0:
                if timeout > 30:
                    flag = 1
                    print "Timeout !!!"
                    responseState = False
                else:
                    out += self.ser.read(1)    #or you can use readline function
                    if out != '':
                        if out == feedback:
                            print "Response OK"
                            flag = 1
                            responseState = True
                        else:
                            flag = 0
                    else:
                        timeout += 1
                        time.sleep(0.1)

            return responseState

        except Exception, e:
            print e


    def readString(self):
        self.rdata = self.ser.readline()
        return self.rdata

    def run(self):
        while self.isArdThreadStop==False:
            try:
                self.rdata = self.ser.readline()
                print "      " + self.rdata
                self.emit(SIGNAL("displayThis(QString)"), self.rdata)
            except Exception:
                pass

        if self.isArdThreadStop==True:
            self.emit(SIGNAL("displayThis(QString)"), "Arduino Thread Stopped")
