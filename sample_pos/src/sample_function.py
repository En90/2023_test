#!/usr/bin/env python3
from re import L
from time import time
import statistics


class SampleIdeal:
    # 1:red
    # 2:blue
    # 3:green

    def __init__(self, id, x=0, y=0, z=0):
        self.id = id
        self.x = x
        self.y = y
        self.z = z

class SampleObserved(SampleIdeal):

    def __init__(self, id):
        # pos and direction angle
        super().__init__(id)
        # quarternion number 
        self.ang_z = 0     
        self.ang_w = 0
        self.exist = False
        # average data
        self.avg_x = 0
        self.avg_y = 0
        self.avg_ang_z = 0
        self.avg_ang_w = 0
        
    
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
    observed = [[],[],[],[],[],[]]
    data_avg = [[],[],[],[],[],[]]
    n = []
    thres = 0.03
    # Left 1:blue 2:green 3:red Right 4:blue 5:green 6:red

    sample_n = 6
    __z = 15
    t0 = time()


    ### ---------method--------- ###
    def __init__(self):
        self.t_start = time()
        for i in range(self.sample_n):
            self.n.append(0)
            for j in range(20):
                self.observed[i].append(SampleObserved(j))
            for j in range(4):
                self.data_avg[i].append(0)
            

    def print(self):
        print("t =", time()-self.t0)
        for i in range(self.sample_n):
            # print("id=", id, "x=", self.data_avg[i][0], "y=", self.data_avg[i][1])      
            print("id={:2}, x={:1.3f}, y={:1.3f}".format(i, float(self.data_avg[i][0]), float(self.data_avg[i][1])))
        print("")

    def filter(self):


        for i in range(self.sample_n):
            mean_x = 0
            mean_y = 0
            mean_ang_z = 0
            mean_ang_w = 0
            dev_x = 0
            dev_y = 0

            # data[x, y, z, ang][how many data]
            data = [[],[],[],[]]
            for j in range(self.n[i]):
                data[0].append(self.observed[i][j].x)
                data[1].append(self.observed[i][j].y)
                data[2].append(self.observed[i][j].ang_z)
                data[3].append(self.observed[i][j].ang_w)

            # calculate the average and deviation of the data
            if(len(data[0])!=0):
                mean_x = statistics.mean(data[0])
                mean_y = statistics.mean(data[1])
                mean_ang_z = statistics.mean(data[2])
                mean_ang_w = statistics.mean(data[3])
                dev_x = statistics.pstdev(data[0])
                dev_y = statistics.pstdev(data[1])
            else:
                mean_x = 0
                mean_y = 0
                mean_ang_z = 0
                mean_ang_w = 0

            # group the data to two parts
            N = 0
            sum_x = 0
            sum_y = 0
            sum_ang_z = 0
            sum_ang_w = 0
            # assume there are two sample in the square
            for j in range(self.n[i]):
                # print(self.observed[i][j].x,  mean_x)
                if(self.observed[i][j].x<= mean_x and dev_x >= self.thres):
                    sum_x += self.observed[i][j].x
                    sum_y += self.observed[i][j].y
                    sum_ang_z += self.observed[i][j].ang_z
                    sum_ang_w += self.observed[i][j].ang_w
                    N +=1
                elif (dev_x < self.thres and self.observed[i][j].y <= mean_y):
                    sum_x += self.observed[i][j].x
                    sum_y += self.observed[i][j].y
                    sum_ang_z += self.observed[i][j].ang_z
                    sum_ang_w += self.observed[i][j].ang_w
                    N +=1    
            
            # decide whether to choose mean or the smaller one
            if(dev_x > self.thres  and dev_y > self.thres and N!=0):
                self.data_avg[i][0] = sum_x / N
                self.data_avg[i][1] = sum_y / N
                self.data_avg[i][2] = sum_ang_z / N
                self.data_avg[i][3] = sum_ang_w / N  
            elif(dev_x > self.thres  and dev_y < self.thres and N!=0):
                self.data_avg[i][0] = sum_x / N
                self.data_avg[i][1] = mean_y
                self.data_avg[i][2] = sum_ang_z / N
                self.data_avg[i][3] = sum_ang_w / N 
            elif(dev_x < self.thres  and dev_y > self.thres and N!=0):
                self.data_avg[i][0] = mean_x
                self.data_avg[i][1] = sum_y / N
                self.data_avg[i][2] = sum_ang_z / N
                self.data_avg[i][3] = sum_ang_w / N 
            elif(dev_x < self.thres  and dev_y < self.thres):
                self.data_avg[i][0] = mean_x
                self.data_avg[i][1] = mean_y
                self.data_avg[i][2] = mean_ang_z
                self.data_avg[i][3] = mean_ang_w 

            # print(i, self.data_avg[i])



    def find(self, x, y, ang_z, ang_w, color):
        color = int(color)
        if 1.200 < x and x < 1.550 and 0.800 < y and y < 1.150:
            self.observed[color][self.n[color]].fresh(x, y, ang_z, ang_w)
            self.n[color]+=1

        elif 1.200 < x and x < 1.550 and 1.850 < y and y < 2.200:
            self.observed[color+3][self.n[color+3]].fresh(x, y, ang_z, ang_w)
            self.n[color+3]+=1

    def clear(self):
        for i in range(self.sample_n):
            # self.observed[i][j].clear()
            self.n[i] = 0




