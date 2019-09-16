#import module_manager
#module_manager.review()
import pygame
import numpy as np
import imp
import math
#import xarray
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline
from sklearn import linear_model
# import statsmodels.api as sm
import pandas as pd

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

def getAngles(bodyCoor): 
    bodyAngles = dict()
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
    print(bodyAngles)
    return bodyAngles

def linearRegression(bodyAngles): 
    regressionCoeff=[]
    for index in range(10):
        finalArray=[]
        yArray=[]
        for key in bodyAngles:
            lst=bodyAngles[key]
            finalArray.append([key, lst[index]])
            yArray.append(key)
        # X=np.array(finalArray)
        reg = linear_model.LinearRegression()
        reg.fit (finalArray, yArray) 
        regressionCoeff.append(reg.coef_)
    return regressionCoeff

def cubicPolyRegression(bP,path):
    x = getAngles(path)
    k = []
    b = []
    for key in x.keys():
        k.append(key)
        b.append(x[key][bP])
    v1 = np.array(k)
    v2 = np.array(b)
    z = np.polyfit(v1,v2,3)

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
    
def compareShots(path): 
    #Determining Angles of Standard Kobe
    bodyCoor=convertArrays("StandardKobe.txt")
    bodyAngles=getAngles(bodyCoor)
    #mx+b form
    regressionCoeff=linearRegression(bodyAngles)
    bodyCoor=convertArrays("DataTest.txt")
    bodyAngles=getAngles(bodyCoor)
    
    
def cubicPolyRegression(bP,path):
    x = getAngles(path)
    k = []
    b = []
    for key in x.keys():
        k.append(key)
        b.append(x[key][bP])
    v1 = np.array(k)
    v2 = np.array(b)
    z = np.polyfit(v1,v2,3)

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

# polyRegression(0,"Data 1. Omisa.txt")

###Read Data File###
bodyCoor=convertArrays("DataTest.txt")
bodyAngles=getAngles(bodyCoor)
polyRegression(bodyAngles)