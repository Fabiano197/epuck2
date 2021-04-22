import numpy as np
from serial import Serial
import struct
import sys
import signal
import time
from threading import Thread

#pos_file = open("D:\\_EPFL\\_Robotique\\epuck2\\Project\\pos_file.txt", "r")

#handler when closing the window
def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()

#reads the data in uint8 from the serial
def readUint8Serial(port):
    state = 0
    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout...')
            return []

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    letter = struct.unpack('B', port.read(1))
    letter = letter[0]
    print(int(letter))

#thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.alive = True

        print('Connecting to port {}'.format(port))

        try:
            self.port = Serial(port, timeout=4)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    #function called after the init
    def run(self):
        data = readUint8Serial(self.port)
        #flush the serial
        self.port.read(self.port.inWaiting())
        time.sleep(0.1)

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

        
#test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)

#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()