#import module_manager
#module_manager.review()
import numpy as np
import imp
import math

def readFile(path):
    with open(path, "rt") as f:
        return f.read()

def convertArrays(path):
    angleData=readFile(path)
    bodyCoor={}
    for line in angleData.splitlines():
        if len(line.split())==1:
            key=int(line)
            bodyCoor[key] = []
        else:
            array=[]
            for val in line.split(" "):
                array.append(float(val))
            bodyCoor[key].append(array)
    return bodyCoor

def convertAngles(a, b, c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    return math.degrees(angle)

def getAngles(path):
    bodyAngles = dict()
    bodyCoor = convertArrays(path)
    for key in bodyCoor:
        lst=bodyCoor[key]
        lKnee = convertAngles(lst[5], lst[3], lst[1])
        rKnee = convertAngles(lst[4], lst[2], lst[0])
        lElbow = convertAngles(lst[7], lst[9], lst[11])
        rElbow = convertAngles(lst[6], lst[8], lst[10])
        lWrist = convertAngles(lst[9], lst[11], lst[13])
        rWrist = convertAngles(lst[8], lst[10], lst[12])
        lPelvis = convertAngles(lst[7], lst[5], lst[3])
        rPelvis = convertAngles(lst[6], lst[4], lst[2])
        lArmPit = convertAngles(lst[5], lst[7], lst[9])
        rArmPit = convertAngles(lst[4], lst[6], lst[8])
        bodyAngles[key] = [lKnee, rKnee, lElbow, rElbow, lWrist, rWrist, lPelvis, rPelvis, lArmPit, rArmPit]
    return bodyAngles

def cubicPolyRegression(bP,path):
    x = getAngles(path)
    k = []
    b = []
    for key in x.keys():
        k.append(key)
        b.append(x[key][bP])
    v1 = np.array(k)
    v2 = np.array(b)
    z = np.polyfit(v1,v2,4)
    return z

def evalAt(a, n):
        total = 0
        l = len(a)
        for i in range(l-1, -1, -1):
            total += (n**i)*a[l-i-1]
        return total

def checkDiff(a,b):
    t=0
    for i in range(10):
        c=cubicPolyRegression(i,a)
        d=cubicPolyRegression(i,b)
        print(c,d)
        for i in range(1,100):
            t+=abs(evalAt(c,i)-evalAt(d,i))
    return t


        
print(checkDiff("data.txt","data3.txt"))





