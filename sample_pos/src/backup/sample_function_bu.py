#!/usr/bin/env python3
from time import time

class SampleIdeal:
    # 1:red
    # 2:blue
    # 3:green

    def __init__(self, id, x=0, y=0, z=0):
        self.id = id
        self.x = x
        self.y = y
        self.z = z

    
    def print(self):
        print("id={:2}, x={:4.3f}, y={:4.3f}".format(self.id, float(self.x), float(self.y)))


class SampleObserved(SampleIdeal):

    def __init__(self, id):
        # pos and direction angle
        super().__init__(id)
        # quarternion number 
        self.ang_z = 0     
        self.ang_w = 0
        self.exist = False
    
    def fresh(self, x, y, ang_z, ang_w):
        self.x = x
        self.y = y
        self.ang_z = ang_z
        self.ang_w = ang_w
        self.exist = True
    
    def clear(self):
        self.exist = False
        self.x = 0
        self.y = 0
        self.ang_z = 0


class Sample():
    ### sample initialize

    #  4 camera sensor
    observed = [[],[]]
    # Left 1:blue 2:green 3:red Right 4:blue 5:green 6:red

    camera_n = 1
    sample_n = 6
    __z = 15
    ### ---------method--------- ###
    def __init__(self):
        self.t_start = time()
        for i in range(self.camera_n):
            for j in range(self.sample_n):
                self.observed[i].append(SampleObserved(j))

    def print(self):
        for i in range(self.camera_n):
            for j in range(self.sample_n):
                self.observed[i][j].print()
        print("")

    def listExist(self):
        print("Exist:", end = '')
        status = False
        for i in range(self.sample_n):
            if self.camera_n == 1:
                if self.observed[0][i].exist == False:
                    continue
                else:
                    status = True
                    print(i, end = "")
            elif self.camera_n == 2:
                if self.observed[0][i].exist == False and self.observed[1][i].exist == False:
                    continue
                else:
                    status = True
                    print(i, end = "")
        if status == False:
            print("None",end='')
        print("")

    def find(self, n, x, y, ang_z, ang_w, color):
        color = int(color)
        if 1.200 < x and x < 1.550 and 0.800 < y and y < 1.150:
            self.observed[n][color].fresh(x, y, ang_z, ang_w)
        elif 1.200 < x and x < 1.550 and 1.850 < y and y < 2.200:
            self.observed[n][color+3].fresh(x, y, ang_z, ang_w)

    def clear(self):
        for i in range(self.camera_n):
            for j in range(self.sample_n):
                self.observed[i][j].clear()




