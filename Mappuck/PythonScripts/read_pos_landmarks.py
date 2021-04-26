import numpy as np
from serial import Serial
import struct
import sys
import signal
import time
from threading import Thread

size_uint16 = 2
size_float = 4

#pos_file = open("D:\\_EPFL\\_Robotique\\epuck2\\Project\\pos_file.txt", "r")

# the structure for a position
class Position:
    def __init__(self, x, y, z, theta, phi):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi

class Landmark:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

#handler when closing the window
def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()

def read_START(char, state):
    if(state == 0):
        if(char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 1):
        if(char == b'T'):
            state = 2
        elif(char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 2):
        if(char == b'A'):
            state = 3
        elif(char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 3):
        if(char == b'R'):
            state = 4
        elif (char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 4):
        if(char == b'T'):
            state = 5
        elif (char == b'S'):
            state = 1
        else:
            state = 0
    return state

#reads the data in uint8 from the serial
def readMessageSerial(port):
    state = 0
    while(state != 5):
        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout...')
            return []
        #update the state
        state = read_START(c1, state)

        
    
    # read the number of landmarks
    nb_lm = struct.unpack('<H', port.read(size_uint16))
    nb_lm = nb_lm[0]

    # read the current position of the robot
    current_pos = Position(0, 0, 0, 0.0, 0.0)
    current_pos.x, current_pos.y, current_pos.z  = struct.unpack('HHH', port.read(3*size_uint16))
    current_pos.theta, current_pos.phi = struct.unpack('ff',  port.read(2*size_float))

    # read the nb_lm landmarks
    landmark_buffer = port.read(nb_lm * size_uint16 * 3)
    landmarks_list = []

    #if we receive the good amount of data, convert them into landmarks
    if(len(landmark_buffer) == nb_lm * size_uint16 * 3):
        for i in range(nb_lm):
            x,y,z = struct.unpack_from('hhh', landmark_buffer, i*size_uint16*3)
            landmarks_list.append(Landmark(x,y,z))

        print('received Landmarks!')
        return landmarks_list
    else:
        print('Timout...')
        return []

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
        landmarks_list = readMessageSerial(self.port)

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

        
#test if the serial port has been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)

#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()