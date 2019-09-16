from __future__ import division, print_function
from vpython import *
from pprint import * 
import time
def readFile(path):
    with open(path, "rt") as f:
        return f.read()

def convertArrays(path): 
    angleData=readFile(path)
    bodyCoor={}    
    for line in angleData.splitlines(): 
        if len(line.split())  ==1:
            key=int(line) 
            bodyCoor[key] = []
        else: 
            a=tuple(float(i) for i in line.split())
            bodyCoor[key].append(a)
    return bodyCoor

d = convertArrays("data2.txt")

scene=canvas(width=480, height=640)
for key in d:
    scene.delete()
    scene=canvas(width=480, height=640)
    scene.center= vector(0,0,0)
    scene.forward=vector(-.25,0,-1)
    #scene.up=vector(0,.5,0)
    fr=vector(d[key][0][0],d[key][0][1],d[key][0][2])
    fl = vector(d[key][1][0],d[key][1][1],d[key][1][2])
    kr=vector(d[key][2][0],d[key][2][1],d[key][2][2])
    kl=vector(d[key][3][0],d[key][3][1],d[key][3][2])
    hr=vector(d[key][4][0],d[key][4][1],d[key][4][2])
    hl=vector(d[key][5][0],d[key][5][1],d[key][5][2])
    sr=vector(d[key][6][0],d[key][6][1],d[key][6][2])
    sl=vector(d[key][7][0],d[key][7][1],d[key][7][2])
    er=vector(d[key][8][0],d[key][8][1],d[key][8][2])
    el=vector(d[key][9][0],d[key][9][1],d[key][9][2])
    rh=vector(d[key][10][0],d[key][10][1],d[key][10][2])
    lr=vector(d[key][11][0],d[key][11][1],d[key][11][2])
    htr=vector(d[key][12][0],d[key][12][1],d[key][12][2])
    htl=vector(d[key][13][0],d[key][13][1],d[key][13][2])
    curve(fr,kr)
    curve(kr,hr)
    curve(hr,hl)
    curve(fl,kl)
    curve(kl,hl)
    curve(sr,sl)
    curve(sr,er)
    curve(sl,el)
    curve(er,rh)
    curve(el,hl)
    curve(hl,htl)
    curve(rh,htr)
    curve(hr,sr)
    curve(hl,sl)
    rate(24)


"""fr=vector(-0.05302009731531143,-1.0680631399154663,2.2781922817230225)
fl=vector(-0.1194043755531311,-1.065081000328064,2.293147325515747)
kr=vector(-0.06191614642739296 ,-0.7105809450149536 ,2.398566961288452)
kl=vector(-0.13750745356082916,-0.6987576484680176,2.405874252319336)
hr=vector(-0.03182102367281914 ,-0.2890447974205017, 2.450571060180664)
hl=vector(-0.17555519938468933, -0.29412850737571716, 2.4573652744293213)
sr=vector(0.05484667047858238, 0.19584961235523224, 2.5375287532806396)
sl=vector(-0.2647995948791504 ,0.18394877016544342, 2.5372025966644287)
er=vector(0.11976364254951477, -0.0634268969297409 ,2.5332894325256348)
el=vector(-0.31080907583236694, -0.06997309625148773, 2.5232768058776855)
rh=vector(0.16043706238269806 ,-0.26581481099128723, 2.4510929584503174)
lr=vector(-0.3121000826358795, -0.29401201009750366, 2.4281346797943115)
htr=vector(0.16830463707447052 ,-0.4056663513183594, 2.3834168910980225)
htl=vector(-0.2842380106449127 ,-0.4314787685871124, 2.3954029083251953)
"""
