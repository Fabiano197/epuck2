import numpy as np
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import string
import math

data_file = open("D:\\_EPFL\\_Robotique\\epuck2\\Mappuck\\PythonScripts\\testdata8_fitted.txt", "r")

# the structure for a position
class Position:
    def __init__(self, x, y, z, phi, theta):
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.theta = theta

    def print(self):
        print("Current position: x= " + str(self.x) + ",  y= " + str(self.y) + ",  z= " + str(self.z))
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
        print("Landmark at: x= " + str(self.x) + ",\ty=" + str(self.y) + ",\tz= " + str(self.z) + "\t", end='')
        if(self.isWall):
            print("It's a wall.")
        else:
            print("\tNot a wall.")

#reads the data in uint8 from the serial
def readMessageSerial(port):
    # read the number of landmarks
    N = int(port.readline())
    print(N)

    # read the current position of the robot
    current_pos = Position(0, 0, 0, 0.0, 0.0)
    line = port.readline()
    numbers = line.split()
    current_pos.x = int(numbers[0])
    current_pos.y = int(numbers[1])
    current_pos.z = int(numbers[2])
    current_pos.phi = float(numbers[3])
    current_pos.theta = float(numbers[4])
    current_pos.print()

    # read the N landmarks
    landmarks_list = []
    for i in range(N):
        line = port.readline()
        numbers = line.split()
        l = Landmark(0,0,0)
        l.x = int(numbers[0])
        l.y = int(numbers[1])
        l.z = int(numbers[2])
        landmarks_list.append(l)

    M = int(port.readline())
    corners_list = []
    corners_list.append([])
    corners_list.append([])
    for i in range(M):
        line = port.readline()
        numbers = line.split()
        corners_list[0].append(int(numbers[0]))
        corners_list[1].append(int(numbers[1]))
    print(corners_list)
    #[TOF], [IR], [POS]
    xCoor = [[],[],[]]
    yCoor = [[],[],[]]
    zCoor = [[],[],[]]
    for i in range(N):
        xCoor[i%3].append(landmarks_list[i].x)
        yCoor[i%3].append(landmarks_list[i].y)
        zCoor[i%3].append(landmarks_list[i].z)
    maxZ = max(zCoor[2])
    if maxZ == 0: maxZ = 1
    minZ = min(zCoor[2])
    if minZ == 0: minZ = -1

    for z in zCoor[2]:
        z = 1 #255*(z-minZ)/(maxZ-minZ)
    plt.plot(xCoor[0], yCoor[0], 'bo')
    plt.plot(xCoor[1], yCoor[1], 'bo')
    plt.plot(xCoor[2], yCoor[2], 'bo')

    plt.plot(corners_list[0], corners_list[1] , 'black')
    return

readMessageSerial(data_file)

plt.show()
pass
