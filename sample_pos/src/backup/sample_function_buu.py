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
        self.counter_r = 0
        self.counter_g = 0
        self.counter_b = 0
        self.rtx = 0
        self.rty = 0
        self.rtangz = 0
        self.rtangw = 0
        self.gtx = 0
        self.gty = 0
        self.gtangz = 0
        self.gtangw = 0
        self.btx = 0
        self.bty = 0
        self.btangz = 0
        self.btangw = 0

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
            self.append(n,color,x,y,ang_z,ang_w)
            #self.observed[n][color].fresh(x, y, ang_z, ang_w)
        elif 1.200 < x and x < 1.550 and 1.850 < y and y < 2.200:
            self.append(n,color+3,x,y,ang_z,ang_w)
            #self.observed[n][color+3].fresh(x, y, ang_z, ang_w)

    def clear(self):
        for i in range(self.camera_n):
            for j in range(self.sample_n):
                self.observed[i][j].clear()

    def append(self,n,color,x,y,angz,angw):
        color = int(color)
        if color == 0 or color == 3:
                self.btx += x
                self.bty += y
                self.btangz += angz
                self.btangw += angw
                self.counter_b += 1
                if self.counter_b > 4:
                        x = self.btx / self.counter_b
                        y = self.bty / self.counter_b
                        angz = self.btangz / self.counter_b
                        angw = self.btangw / self.counter_b
                        self.observed[n][color].fresh(x, y, angz, angw)
                        self.btx = 0
                        self.bty = 0
                        self.btangz = 0
                        self.btangw = 0
                        self.counter_b = 0
        elif color == 1 or color == 4:
                self.gtx += x
                self.gty += y
                self.gtangz += angz
                self.gtangw += angw
                self.counter_g += 1
                if self.counter_g > 4:
                        x = self.gtx / self.counter_g
                        y = self.gty / self.counter_g
                        angz = self.gtangz / self.counter_g
                        angw = self.gtangw / self.counter_g
                        self.observed[n][color].fresh(x, y, angz, angw)
                        self.gtx = 0
                        self.gty = 0
                        self.gtangz = 0
                        self.gtangw = 0
                        self.counter_g = 0
        elif color == 2 or color == 5:
                self.rtx += x
                self.rty += y
                self.rtangz += angz
                self.rtangw += angw
                self.counter_r += 1
                if self.counter_r > 4:
                        x = self.rtx / self.counter_r
                        y = self.rty / self.counter_r
                        angz = self.rtangz / self.counter_r
                        angw = self.rtangw / self.counter_r
                        self.observed[n][color].fresh(x, y, angz, angw)
                        self.rtx = 0
                        self.rty = 0
                        self.rtangz = 0
                        self.rtangw = 0
                        self.counter_r = 0



