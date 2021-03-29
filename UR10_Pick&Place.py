# -*- coding: utf-8 -*-
"""
Created on Thu Mar 25 09:32:44 2021

@author: ljjmj
"""

import time 
import urx
import logging
import math
import numpy as np


Length = [0.1273,0.6120,0.5723,0.1639,0.1157,0.0922]
Am = [0,-0.6120,-0.5723,0,0,0]
Dm = [0.1273,0,0,0.1639,0.1157,0.0922]
Matrice = np.zeros((6,4,4))
Arad = [math.pi/2,0,0,math.pi/2,-math.pi/2,0]
Coordinate = [0,0,0]
Angle = [0,0,0,0,0,0]






def Inverse(X,Y,Z,Pose):
    global Lenght
    StartCoordinateJ3 = np.array([-1.1843, 0.000, 0.1273])
    
    if Pose == 1:
        StartCoordinateJ6 = np.array([-1.3000, -0.16394, 0.0351])
    if Pose == 2:
        StartCoordinateJ6 = np.array([-1.06860, -0.16394, 0.0351])
    if Pose == 3:
        StartCoordinateJ6 = np.array([-1.27650, -0.16394, 0.24300])

    StartCoordinatesJ36 = np.subtract(StartCoordinateJ3, StartCoordinateJ6)
    AngleEndEffectorXY = math.degrees(math.atan(StartCoordinatesJ36[0] / StartCoordinatesJ36[1]))
    LengthEndEffectorXY = math.sqrt((StartCoordinatesJ36[0]**2)+(StartCoordinatesJ36[1]**2))

    PIT = math.sqrt((X**2) + (Y**2))
    AIT = math.degrees(math.asin(StartCoordinatesJ36[1] / PIT))
    
    if X <= 0:
        TRI =  math.degrees(math.atan(Y/X))
    if X > 0:
        if Y <= 0:
            TRI =  (180 + math.degrees(math.atan(Y/X)))
        if Y > 0:
            TRI =  (-180 + math.degrees(math.atan(Y/X)))
    
    Phi_Angle = TRI - AIT
        
    AngleEnd = (90-AngleEndEffectorXY) + Phi_Angle
    
    Xend = math.cos(math.radians(AngleEnd)) * LengthEndEffectorXY
    Yend = math.sin(math.radians(AngleEnd)) * LengthEndEffectorXY 
    X = X + Xend 
    Y = Y + Yend 
    Z = Z + StartCoordinatesJ36[2]
    
    
    A = Length[2]
    B = Length[1]
    C = math.sqrt( (math.sqrt((X**2)+(Y**2)))**2    +   (Z - Length[0])**2 )
    a = math.degrees(math.acos( ((A**2) + (B**2) - (C**2)) / (2*A*B) ))
    b = math.degrees(math.acos( ((B**2) + (C**2) - (A**2)) / (2*B*C) ))
    c = math.degrees(math.acos( ((C**2) + (A**2) - (B**2)) / (2*A*C) )) 
    Phi_1 = math.degrees(math.atan( (Z-Length[0]) / math.sqrt((X**2) + (Y**2)) )) + b
    Phi_2 = 180 - a
    if Pose == 1:
        Phi_3 = 90+(90-(a-(90 - Phi_1)))
        Phi_4 = 90
    if Pose == 2:
        Phi_3 = -180+90+(90-(a-(90 - Phi_1)))
        Phi_4 = -90
    if Pose == 3:
        Phi_3 = 180+(90-(a-(90 - Phi_1)))
        Phi_4 = 90
    return Phi_Angle,-Phi_1,Phi_2,-Phi_3,-Phi_4


# X 0.4
# Y 0.2
# Z 0.19


def Forward(Angle):   
    for x in range (6):
        global JEndEffector
        Matrice[x,0,0] = math.cos(math.radians(Angle[x]))
        Matrice[x,0,1] = -math.sin(math.radians(Angle[x])) * math.cos(Arad[x])
        Matrice[x,0,2] = math.sin(math.radians(Angle[x])) * math.sin(Arad[x])
        Matrice[x,0,3] = Am[x] * math.cos(math.radians(Angle[x]))
        
        Matrice[x,1,0] = math.sin(math.radians(Angle[x]))
        Matrice[x,1,1] = math.cos(math.radians(Angle[x])) * math.cos(Arad[x])
        Matrice[x,1,2] = -math.cos(math.radians(Angle[x])) * math.sin(Arad[x])
        Matrice[x,1,3] = Am[x] * math.sin(math.radians(Angle[x]))
        
        Matrice[x,2,0] = 0
        Matrice[x,2,1] = math.sin(Arad[x])
        Matrice[x,2,2] = math.cos(Arad[x])
        Matrice[x,2,3] = Dm[x]
        
        Matrice[x,3,0] = 0
        Matrice[x,3,1] = 0
        Matrice[x,3,2] = 0
        Matrice[x,3,3] = 1

        JEndEffector = np.zeros((5,4,4))
        
        for i in range(4):
           # iterate through columns of Y
           for j in range(4):
               # iterate through rows of Y
               for k in range(4):
                   JEndEffector[0][i][j] += Matrice[0][i][k] * Matrice[1][k][j]
                   
        for h in range(4):
            for i in range(4):
           # iterate through columns of Y
               for j in range(4):
                   # iterate through rows of Y
                   for k in range(4):
                       JEndEffector[h+1][i][j] += JEndEffector[h][i][k] * Matrice[h+2][k][j]
                       
                            
        print("EndEffector coordinates")
        print(JEndEffector[4,0,3])
        print(JEndEffector[4,1,3])
        print(JEndEffector[4,2,3])



def UserInput():
    global X, Angle
    Coordinate[0] = float(input("X Coordinate: "))
    Coordinate[1] = float(input("Y Coordinate: "))
    Coordinate[2] = float(input("Z Coordinate: "))
    Pose          = int  (input("Pose position: "))
    if math.sqrt(math.sqrt((Coordinate[0]**2) + (Coordinate[1]**2))**2 + (Coordinate[2]**2)) < 1.3 :
        Angle[0],Angle[1],Angle[2],Angle[3],Angle[4] = Inverse(Coordinate[0],Coordinate[1],Coordinate[2],Pose)
        print(Angle)
        Forward(Angle)
        for T in range(5):
            Angle[T] = math.radians(Angle[T])
    else:
        print("Coordinates out of bounds for UR10")
     
UserInput()
            


if __name__ == "__main__":
    rob = urx.Robot("192.168.0.3")
    try:
        v = 1.2
        a = 0.15
        rob.movej((Angle[0],Angle[1],Angle[2],Angle[3],Angle[4],0), a, v)

    finally:
        rob.close()