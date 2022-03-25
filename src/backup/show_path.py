import numpy as np
import matplotlib.pyplot as plot
import string

def turnTheFormat():
    f1 = open('./Tractory85.txt','r')
    f2 = open('./Tractory85_ros.txt','w')
    line = f1.readline()
    while line:
        xstr , ystr = line.split('y:')
        xstr = xstr[2:].strip()
        ystr = ystr.strip()
        # print(ystr)
        wout = "x:"+xstr + " " +"y:" + ystr
        print(wout)
        line = f1.readline()
        f2.write(wout)
        f2.write('\n')
def showPath():

    x = []
    y = []
    with open('./dataset/Tractory_carto_1.txt','r') as f:
        line = f.readline()
        while line:
            xstr,ystr = line.split(' ')
            # print(line)
            xstr = xstr[2:].strip()
            ystr = ystr[2:].strip()
            # print(xstr)
            # print(ystr)
            x.append(float(xstr))
            y.append(float(ystr))
            line = f.readline()
    plot.xlim(-3,1)
    plot.ylim(-3,1)
    X = np.array(x)
    Y = np.array(y)
    init = 1
    lastX = 100
    lastY = 100
    thresx = []
    thresy = []
    print(np.vstack((X,Y)).T.shape)
    # print(np.vstack((X,Y)))
    # print(X.c(Y))
    combine = np.vstack((X,Y)).T
    flatternX = []
    flatternY = []
    for thresX,thresY in combine[:]:
        if(init):
            lastX = thresX
            lastY = thresY
            init = 0
        else:
            if(thresX > lastX):
                flatternX.append(thresX)
            # if(thresY > lastY)
            #     flatternY.append(thresY)
            thresx.append((thresX - lastX)*100)
            thresy.append((thresY - lastY)*100)
            lastX = thresX
            lastY = thresY

    print(X)
    plot.plot(combine[:,0],combine[:,1])
    plot.plot(combine[:,0],combine[:,1])
    # plot.plot(thresx,thresy)
    plot.show()
    with open('./dataset/thresTrajectory.txt','w') as f:
        for i in range(0,len(thresx)):
            f.write('x:'+str(thresx[i])+' '+'y:'+str(thresy[i]))
            f.write('\n')
# showPath()

import time
def timeUse():
    t1 = time.time()
    x = []
    y = []
    with open('dataset/Tractory_carto_1.txt') as f:
        line = f.readline()
        while line:
            xstr, ystr = line.split(' ')
            # print(line)
            xstr = xstr[2:].strip()
            ystr = ystr[2:].strip()
            # print(xstr)
            # print(ystr)
            x.append(float(xstr))
            y.append(float(ystr))
            line = f.readline()
    k = 0
    t2 = time.time()
    for i in range(len(x)-1,0,-1):
        xk = np.random.rand(i-1)
        yk = np.random.rand(i-1)
        sign = 0
        for j in range(i-1,0,-1):
            # a = x[i] - x[j]
            # print(a)
            xk[sign] = x[j] - x[i]
            yk[sign] = y[j] - y[i]
            sign +=1
        plot.plot(xk,yk)
        plot.xlim(-3, 1)
        plot.ylim(-3, 1)
        plot.show()
            # k += 1
    # print(k)
    # print(f"use time = {time.time()-t2}")
turnTheFormat()
