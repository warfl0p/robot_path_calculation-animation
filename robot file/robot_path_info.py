import numpy as np
# from flexie import *

N1=14                           #iteraties workspace
N2=400                          #iteraties path
stepsVerlaging=10               #resolutie animatie verlagen
dof=5                           #bepaald hoeveel joints er berekent worden

l2=0.1
l3=0.1
ee=[0,0,l3,1]

# var1start=np.pi*3/2
# var2start=np.pi/2               #initele condities
# var3start=0.2

# var1range=[0,np.pi*2]
# var2range=[0,np.pi*2]
# var3range=[0,1]

varList=[0]*dof

a2=0.7                          #waardes afmetingen robot links
a3=0.1
d1=3
d3=0.3
d4=0.3
ee=[0,0,0,1]                    #end effector vector

var1start=3
var2start=0                     #initele condities dof
var3start=0
var4start=0
var5start=0
var6start=0

varStartList=[var1start,var2start,var3start,var4start,var5start,var6start]

var1range=[0,np.pi*2]
var2range=[0,np.pi*2]
var3range=[0,np.pi*2]
var4range=[0,np.pi*2]
var5range=[0,      1]
var6range=[0,      1]
circle=True
ra= 0.4                          #radius of circle
punt1=[0.5,-0.5,  0]           #is center cirkel als circle=True
punt2=[1  ,  -2,0.1]           #einde traject als circle=False

def jacobian(givenVarList):
    theta1= givenVarList[0]
    theta2= givenVarList[1]
    theta3= givenVarList[2]
    theta4= givenVarList[3]
    d5    = givenVarList[4]
    # theta1= givenVarList[0]
    # theta2= givenVarList[1]
    # d3    = givenVarList[2]
    J=np.array([[-(a3*np.sin(theta4) + d3)*np.cos(theta1) - (a2*np.cos(theta2) + a3*np.cos(theta4)*np.cos(theta2 + theta3) + d4*np.sin(theta2 + theta3) + d5*np.sin(theta2 + theta3))*np.sin(theta1), -(a2*np.sin(theta2) + a3*np.sin(theta2 + theta3)*np.cos(theta4) - d4*np.cos(theta2 + theta3) - d5*np.cos(theta2 + theta3))*np.cos(theta1), (-a3*np.sin(theta2 + theta3)*np.cos(theta4) + d4*np.cos(theta2 + theta3) + d5*np.cos(theta2 + theta3))*np.cos(theta1), -a3*(np.sin(theta1)*np.cos(theta4) + np.sin(theta4)*np.cos(theta1)*np.cos(theta2 + theta3)), np.sin(theta2 + theta3)*np.cos(theta1)], 
                [-(a3*np.sin(theta4) + d3)*np.sin(theta1) + (a2*np.cos(theta2) + a3*np.cos(theta4)*np.cos(theta2 + theta3) + d4*np.sin(theta2 + theta3) + d5*np.sin(theta2 + theta3))*np.cos(theta1), -(a2*np.sin(theta2) + a3*np.sin(theta2 + theta3)*np.cos(theta4) - d4*np.cos(theta2 + theta3) - d5*np.cos(theta2 + theta3))*np.sin(theta1), (-a3*np.sin(theta2 + theta3)*np.cos(theta4) + d4*np.cos(theta2 + theta3) + d5*np.cos(theta2 + theta3))*np.sin(theta1), a3*(-np.sin(theta1)*np.sin(theta4)*np.cos(theta2 + theta3) + np.cos(theta1)*np.cos(theta4)), np.sin(theta1)*np.sin(theta2 + theta3)],
                [0, -a2*np.cos(theta2) - a3*np.cos(theta4)*np.cos(theta2 + theta3) - d4*np.sin(theta2 + theta3) - d5*np.sin(theta2 + theta3), -a3*np.cos(theta4)*np.cos(theta2 + theta3) - d4*np.sin(theta2 + theta3) - d5*np.sin(theta2 + theta3), a3*np.sin(theta4)*np.sin(theta2 + theta3), np.cos(theta2 + theta3)]])
    # J=np.array([[-(d3+l2+l3)*np.sin(theta1)*np.sin(theta2)  , (d3+l2+l3)*np.cos(theta1)*np.cos(theta2)  , np.sin(theta2)*np.cos(theta1) ],
    #             [(d3+l2+l3)*np.sin(theta2)*np.cos(theta1)   , (d3+l2+l3)*np.sin(theta1)*np.cos(theta2)  , np.sin(theta1)*np.sin(theta2) ], 
    #             [0                                          , -(d3+l2+l3)*np.sin(theta2)                , np.cos(theta2)                ]])

    return J

def getdh_table(givenVarList):
    theta1= givenVarList[0]
    theta2= givenVarList[1]
    theta3= givenVarList[2]
    theta4= givenVarList[3]
    d5    = givenVarList[4]
    # theta1= givenVarList[0]
    # theta2= givenVarList[1]
    # d3    = givenVarList[2]
    dh_table = np.array([[0 ,   0       , d1,  theta1],
                         [0 ,   -np.pi/2,  0,  theta2],
                         [a2,   0       , d3,  theta3],
                         [a3,   np.pi/2 , d4,  theta4],
                         [0 ,   0       , d5,  0     ]])
    # dh_table = np.array([[ 0 ,    0      ,   0     ,   theta1  ],
    #                      [ 0 ,   -np.pi/2,   0     ,   theta2  ],
    #                      [ 0 ,   np.pi/2,   d3+l2 ,   0        ]  ])
    return dh_table




