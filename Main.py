from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime


import ctypes
import _ctypes
import pygame
import sys
import numpy as np
import math

def writeFile(path, contents):
    with open(path, "wt") as f:
        f.write(contents)

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

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
    return abs(kobeSimilarity)
 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


class BodyGameRuntime(object):
    def __init__(self):
        self.recordedData = ""
        self.frame = 0
        self.record = False
        self.begin = True
        self.diff = 0
        self.returnDiff = 0
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()
    def storePos(self,joints,joint):
        return str(joints[joint].Position.x) + " " + str(joints[joint].Position.y) + " " + str(joints[joint].Position.z)
    def recordData(self,joints):
        fR = self.storePos(joints,PyKinectV2.JointType_FootRight)
        fL = self.storePos(joints,PyKinectV2.JointType_FootLeft)
        kR = self.storePos(joints,PyKinectV2.JointType_KneeRight)
        kL = self.storePos(joints,PyKinectV2.JointType_KneeLeft)
        hpR = self.storePos(joints,PyKinectV2.JointType_HipRight)
        hpL = self.storePos(joints,PyKinectV2.JointType_HipLeft)
        sR = self.storePos(joints,PyKinectV2.JointType_ShoulderRight)
        sL = self.storePos(joints,PyKinectV2.JointType_ShoulderLeft)
        eR = self.storePos(joints,PyKinectV2.JointType_ElbowRight)
        eL = self.storePos(joints,PyKinectV2.JointType_ElbowLeft)
        hR = self.storePos(joints,PyKinectV2.JointType_WristRight)
        hL = self.storePos(joints,PyKinectV2.JointType_WristLeft)
        htR = self.storePos(joints,PyKinectV2.JointType_HandTipRight)
        htL = self.storePos(joints,PyKinectV2.JointType_HandTipLeft)
        o = str(self.frame) + "\n" + fR + "\n" + fL + "\n" + kR + "\n" + kL + "\n" + hpR + "\n" + hpL + "\n" + sR + "\n" + sL + "\n" \
            + eR + "\n" + eL + "\n" + hR + "\n" + hL + "\n" + htR + "\n" + htL + "\n"
        self.recordedData += o
        self.frame += 1



    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            font = pygame.font.SysFont("comicsansms", 72)
            text = font.render("Kobe-O-Meter", True, (0, 128, 0))
            self._screen.blit(text,(320 - text.get_width() // 2, 240 - text.get_height() // 2))
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]: 
                self.record = True
                print(1)
            if keys[pygame.K_DOWN]: 
                print(3)
                self.record = False
                writeFile("data.txt",self.recordedData)
                self.recordedData = ""
                self.frame = 0
                self.diff = compareShots("Data 2. Sid.txt","data.txt")
                print(self.diff)

            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop


                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 
                    if self.record:
                        print(2)
                        self.recordData(joints)
                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            if self.diff != 0:
                if self.diff<=3: 
                    self.returnDiff=100
                elif 3<self.diff<=10: 
                    self.returnDiff=99-(12.7*(self.diff-3))
                else:
                    self.returnDiff=0

            text = font.render(("Kobe-O-Meter:"+ str(int(self.returnDiff)) + "%"), True, (0, 128, 0))
            self._screen.blit(text,(300 - text.get_width() // 2, 50 - text.get_height() // 2))
            pygame.display.update()
            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()


            # --- Limit to 60 frames per second
            self._clock.tick(60)
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime();
game.run();