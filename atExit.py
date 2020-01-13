import atexit
import time

def exit_handler():
    print 'My application is ending!'
    time.sleep(2)

atexit.register(exit_handler)

if __name__=="__main__":
    print "Program Started"
    while(True):
        print "while loop"
        time.sleep(2)
