import numpy as np
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from serial import Serial
import struct
import string
import sys
import signal
import time
import math
from threading import Thread

size_int16 = 2
size_float = 4
outer_rim = 1000

class Numb:
    def __init__(self):
        self.new = 0
        self.tot = 0

# the structure for a position
class Position:
    def __init__(self, x, y, z, phi, theta):
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.theta = theta

    def print(self):
        print("Current position: x= " + str(self.x) + ", y=" + str(self.y) + ", z= " + str(self.z) + "")
        print("Current angles: phi= " + str(self.phi) + ",  theta= " + str(self.theta))


class Landmark:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        if(z == -32768):
            self.isWall = True
        else:
            self.isWall = False
    
    def print(self):
        print("Landmark at: x= " + str(self.x) + ",\ty=" + str(self.y) +
                 ",\tz= " + str(self.z) + "\t", end='')
        if(self.isWall):
            print("It's a wall.")
        else:
            print("\tNot a wall.")

N = Numb()
current_pos = Position(0, 0, 0, 0.0, 0.0)
TOF = -32768
IR  =  32767
landmarks_TOF = []
landmarks_IR  = []
landmarks_POS = []

#update the plot
def update_plot():
    if(reader_thd.need_to_update_plot()):
        #[TOF], [IR], [POS]
        xCoor = [[],[],[]]
        yCoor = [[],[],[]]
        zCoor = [[],[],[]]
        for i in range(len(landmarks_TOF)):
            xCoor[0].append(landmarks_TOF[i].x)
            yCoor[0].append(landmarks_TOF[i].y)
            zCoor[0].append(landmarks_TOF[i].z)
        for i in range(len(landmarks_IR)):
            xCoor[1].append(landmarks_IR[i].x)
            yCoor[1].append(landmarks_IR[i].y)
            zCoor[1].append(landmarks_IR[i].z)
        for i in range(len(landmarks_POS)):
            xCoor[2].append(landmarks_POS[i].x)
            yCoor[2].append(landmarks_POS[i].y)
            zCoor[2].append(landmarks_POS[i].z)

        maxZ = max(zCoor[2])
        if maxZ == 0: maxZ = 1
        minZ = min(zCoor[2])
        if minZ == 0: minZ = -1
        
        for z in zCoor[2]:
            z = 255*(z-minZ)/(maxZ-minZ)
        
        dataPlot.clear()
        dataPlot.plot(xCoor[0], yCoor[0], 'ro')
        dataPlot.plot(xCoor[1], yCoor[1], 'bo')
        dataPlot.scatter(xCoor[2], yCoor[2], marker='.', c=zCoor[2], cmap='nipy_spectral')
        # plt.colorbar()
        # dataPlot.scatter(current_pos.x, current_pos.y, marker='*', linewidths=4, c='black', s=200, zorder=3)
        # dataPlot.arrow(current_pos.x, current_pos.y, 
        #             50*math.cos(current_pos.phi), 
        #             50*math.sin(current_pos.phi), 
        #             width=10, 
        #             facecolor='black')
        #plt.draw()
        fig.canvas.draw_idle()
        reader_thd.plot_updated()
        

def update_data(port):
    readMessageSerial(port)
    reader_thd.tell_to_update_plot()

#handler when closing the window
def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()
    timer.stop()

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
            return
        #update the state
        state = read_START(c1, state)

    # read the number of landmarks
    temp = struct.unpack('H', port.read(size_int16))
    N.new = temp[0]
    N.tot = N.tot + N.new

    # read the current position of the robot
    temp = struct.unpack('hhhff', port.read(16))
    if temp[0] < outer_rim and temp[0] > -outer_rim and temp[1] < outer_rim and temp[1] > -outer_rim and temp[2] < outer_rim and temp[2] > -outer_rim:
        current_pos.x = temp[0]
        current_pos.y = temp[1]
        current_pos.z = temp[2]
    if temp[3] < 4.0  and temp[3] > -4.0 and temp[4] < 4.0  and temp[4] > -4.0:
        current_pos.phi = temp[3]
        current_pos.theta = temp[4]

    # read the N.new landmarks
    for i in range(N.new):
        temp = struct.unpack('hhh', port.read(3*size_int16))
        if temp[0] < outer_rim and temp[0] > -outer_rim and temp[1] < outer_rim and temp[1] > -outer_rim:
            new_landmark = Landmark(0,0,0)
            new_landmark.x = temp[0]
            new_landmark.y = temp[1]
            new_landmark.z = temp[2]
            if new_landmark.z == TOF:
                landmarks_TOF.append(new_landmark)
            elif new_landmark.z == IR:
                landmarks_IR.append(new_landmark)
            else:
                landmarks_POS.append(new_landmark)
    return

#thread used to control the communication part
class serial_thread(Thread):
    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))
        try:
            self.port = Serial(port, timeout=4)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    #function called after the init
    def run(self):
        while self.alive:
            update_data(self.port)
    
    #tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    #tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False
    
    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

#figure config
fig = plt.figure()
dataPlot = fig.add_subplot(111)
dataPlot.relim()
fig.canvas.set_window_title('Map')
fig.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c

#serial reader thread config
#begins the serial thread
reader_thd = serial_thread('com12')
reader_thd.start()

#timer to update the plot from within the state machine of matplotlib
#because matplotlib is not thread safe...
timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

plt.show()