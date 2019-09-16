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
    z = np.polyfit(v1,v2,6)
    return z

def quadPolyRegression(bP, path):
    x = getAngles(path)
    k = []
    b = []
    for key in x.keys():
        k.append(key)
        b.append(x[key][bP])
    v1 = np.array(k)
    v2 = np.array(b)
    z = np.polyfit(v1,v2,2)
    
def getDataPts(bP, pathCompare, multiplyBy):
    #returns body angles
    x = getAngles(pathCompare) 
    k = []
    b = []
    for key in x.keys():
        k.append(key*multiplyBy)
        b.append(x[key][bP])
    #k is x-values, b is y-values
    return (k, b)

def compareShots(pathStd, pathCompare): 
    #Determining Angles of Standard Kobe
    regressionCoeff=[]
    bodyCoor=convertArrays(pathStd)
    stdLen=len(bodyCoor)
    bodyAngles=getAngles(pathStd)
    #cubic form
    for index in range(10): 
        regressionCoeff.append(cubicPolyRegression(index, pathStd))
    #Comparing to the next person
    bodyCoor=convertArrays(pathCompare)
    compareLen=len(bodyCoor)
    xArray=[]
    multiplyBy=stdLen/compareLen
    diffLst=[]
    diffAngleChanges=[]
    for bP in range(10): 
        (x, y)=getDataPts(bP, pathCompare, multiplyBy)
        for i in range(len(y)): 
            anglePredict=regressionCoeff[bP][0]*(x[i]**6)+regressionCoeff[bP][1]*(x[i]**5)+ \
        regressionCoeff[bP][2]*(x[i]**4)+regressionCoeff[bP][3]*(x[i]**3) + \
        regressionCoeff[bP][4]*(x[i]**2)+regressionCoeff[bP][5]*(x[i]**1)+ \
        regressionCoeff[bP][6]
            difference=anglePredict-y[i]
            diffLst.append(difference)
        averageChange=sum(diffLst)/len(diffLst)
        diffAngleChanges.append(averageChange)
    kobeSimilarity=sum(diffAngleChanges)/len(diffAngleChanges)
    return abs(kobeSimilarity))
        
            
        
        
        
#     bodyAngles=getAngles(pathCompare)
#     #use the xArray to get the lin reg, compare each lin reg point to the 
#     #corresponding list point in bodyAngles
#     dataPts=getDataPts(bodyAngles, multiplyBy)
#     for i in range(xArray):
#         
#         
#         
#     
#     
# 
# print(cubicPolyRegression(0,"DataTest.txt"))

compareShots("StandardKobe.txt", "StandardKobe.txt")






