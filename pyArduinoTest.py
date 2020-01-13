import time
import serial

global ser
ser = serial.Serial()
positionStr = str(int(2)) + ',' + str(int(3)) + '\n'

try:    
    ser.port="COM36"
    ser.baudrate=9600
    ser.parity=serial.PARITY_NONE
    ser.stopbits=serial.STOPBITS_ONE
    ser.bytesize=serial.EIGHTBITS
    ser.open()
    ser.flushInput()
    ser.flushOutput()
    print "serial connected"
except:
    print "Not Connected"
time.sleep(1)
print "sending data"
time.sleep(1)
ser.writelines(positionStr)
print "waiting for feedback"
data = ser.readline()
print str(data)
time.sleep(1)

ser.close()

