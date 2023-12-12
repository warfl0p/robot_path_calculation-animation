import numpy as np
import matplotlib.pyplot as plt
import scipy as sc
from scipy.spatial import KDTree
from datetime import datetime
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import PillowWriter
import mpl_toolkits.mplot3d.axes3d as p3
start=datetime.now()

N1=40                           #iteraties workspace
N2=200                          #iteraties path
stepsVerlaging=10               #resolutie animatie verlagen
dof=3                           #bepaald hoeveel joints er berekent worden

l2=0.1
l3=0.1
ee=[0,0,l3,1]

var1start=np.pi*3/2
var2start=np.pi/2               #initele condities
var3start=0.2

var1range=[0,np.pi*2]
var2range=[0,np.pi*2]
var3range=[0,1]

varList=[0]*dof


# a2=0.7                          #waardes afmetingen robot links
# a3=0.1
# d1=3
# d3=0.3
# d4=0.3
# ee=[0,0,0,1]                    #end effector vector

# var1start=3
# var2start=0                     #initele condities dof
# var3start=0
var4start=0
var5start=0

varStartList=[var1start,var2start,var3start,var4start,var5start]
# var1range=[0,np.pi*2]
# var2range=[0,np.pi*2]
# var3range=[0,np.pi*2]
var4range=[0,np.pi*2]
var5range=[0,      1]

circle=True
ra= 2                          #radius of circle
punt1=[0.5,-0.5,  1]           #is center cirkel als circle=True
punt2=[1  ,  -2,0.1]           #einde traject als circle=False

def getdh_table(var1,var2,var3,var4,var5):
    # theta1= var1
    # theta2= var2
    # theta3= var3
    # theta4= var4
    # d5    = var5
    theta1=var1
    theta2=var2
    d3    =var3
    # dh_table = np.array([[0 ,   0       , d1,  theta1],
    #                      [0 ,   -np.pi/2,  0,  theta2],
    #                      [a2,   0       , d3,  theta3],
    #                      [a3,   np.pi/2 , d4,  theta4],
    #                      [0 ,   0       , d5,  0     ]])
    dh_table = np.array([[ 0 ,    0      ,   0     ,   theta1  ],
                         [ 0 ,   -np.pi/2,   0     ,   theta2  ],
                         [ 0 ,   np.pi/2,   d3+l2 ,   0        ]  ])
    return dh_table

def dhTransform(dh_table,onlyee):
    T_list = []
    for i in range(dh_table.shape[0]):
        a     = dh_table[i,0]
        alpha = dh_table[i,1]
        d     = dh_table[i,2]
        theta = dh_table[i,3]
        T_i = np.array([
            [                np.cos(theta),                -np.sin(theta),              0,                  a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha),  np.cos(alpha),  np.cos(alpha) * d],
            [                            0,                             0,              0,                  1]])
        T_list.append(T_i)
    T_list=np.array(T_list)
    T_list2=T_list[0]
    T_listprod=T_list[0]
    if onlyee==False:
        for i in range(1,len(T_list)):          
            T_listprod=T_listprod@T_list[i]
            T_list2=np.hstack((T_list2,T_listprod))
    else:
        for i in range(1,len(T_list)):
            T_list2=T_list2@T_list[i]
    return T_list2

def xyz(Tlist,ee,onlyee):
    xlist=[]
    ylist=[]
    zlist=[]
    if onlyee==False:
        xlist=[0,0]
        ylist=[0,0]
        zlist=[0,Tlist[2,3]]
        for i in range(1,round(Tlist.shape[1]/4)+1,1):
            xlist.append(Tlist[0,i*4-1])
            ylist.append(Tlist[1,i*4-1])
            zlist.append(Tlist[2,i*4-1])
        ee=np.dot(Tlist[:,-4:],ee)
    else:
        ee=np.dot(Tlist,ee)                 #als onlee false is, zal Tlist enkel transfomatrix van laatste joint bevatten
    xlist.append(ee[0])
    ylist.append(ee[1])
    zlist.append(ee[2])    
    xyz=np.vstack((xlist,ylist,zlist))
    return xyz

def jacobian(var1,var2,var3,var4,var5):
    # theta1= var1
    # theta2= var2
    # theta3= var3
    # theta4= var4
    # d5    = var5
    theta1=var1
    theta2=var2
    d3    =var3
    # J=np.array([[-(a3*np.sin(theta4) + d3)*np.cos(theta1) - (a2*np.cos(theta2) + a3*np.cos(theta4)*np.cos(theta2 + theta3) + d4*np.sin(theta2 + theta3) + d5*np.sin(theta2 + theta3))*np.sin(theta1), -(a2*np.sin(theta2) + a3*np.sin(theta2 + theta3)*np.cos(theta4) - d4*np.cos(theta2 + theta3) - d5*np.cos(theta2 + theta3))*np.cos(theta1), (-a3*np.sin(theta2 + theta3)*np.cos(theta4) + d4*np.cos(theta2 + theta3) + d5*np.cos(theta2 + theta3))*np.cos(theta1), -a3*(np.sin(theta1)*np.cos(theta4) + np.sin(theta4)*np.cos(theta1)*np.cos(theta2 + theta3)), np.sin(theta2 + theta3)*np.cos(theta1)], 
    #             [-(a3*np.sin(theta4) + d3)*np.sin(theta1) + (a2*np.cos(theta2) + a3*np.cos(theta4)*np.cos(theta2 + theta3) + d4*np.sin(theta2 + theta3) + d5*np.sin(theta2 + theta3))*np.cos(theta1), -(a2*np.sin(theta2) + a3*np.sin(theta2 + theta3)*np.cos(theta4) - d4*np.cos(theta2 + theta3) - d5*np.cos(theta2 + theta3))*np.sin(theta1), (-a3*np.sin(theta2 + theta3)*np.cos(theta4) + d4*np.cos(theta2 + theta3) + d5*np.cos(theta2 + theta3))*np.sin(theta1), a3*(-np.sin(theta1)*np.sin(theta4)*np.cos(theta2 + theta3) + np.cos(theta1)*np.cos(theta4)), np.sin(theta1)*np.sin(theta2 + theta3)],
    #             [0, -a2*np.cos(theta2) - a3*np.cos(theta4)*np.cos(theta2 + theta3) - d4*np.sin(theta2 + theta3) - d5*np.sin(theta2 + theta3), -a3*np.cos(theta4)*np.cos(theta2 + theta3) - d4*np.sin(theta2 + theta3) - d5*np.sin(theta2 + theta3), a3*np.sin(theta4)*np.sin(theta2 + theta3), np.cos(theta2 + theta3)]])
    J=np.array([[-(d3+l2+l3)*np.sin(theta1)*np.sin(theta2)  , (d3+l2+l3)*np.cos(theta1)*np.cos(theta2)  , np.sin(theta2)*np.cos(theta1) ],
                [(d3+l2+l3)*np.sin(theta2)*np.cos(theta1)   , (d3+l2+l3)*np.sin(theta1)*np.cos(theta2)  , np.sin(theta1)*np.sin(theta2) ], 
                [0                                          , -(d3+l2+l3)*np.sin(theta2)                , np.cos(theta2)                ]])

    return J


def checkWorkspace():
    global workspace
    workspace=np.empty([0,3])
    def calculateWorkspace():
        global workspace
        dh_table = getdh_table(var1,var2,var3,var4,var5)
        Tlist=dhTransform(dh_table,True)
        total_robot=xyz(Tlist,ee,True)
        workspace=np.vstack([workspace,np.hstack([total_robot[0],total_robot[1],total_robot[2]])])
    if dof>=1:
        for i in range(N1):
            var1=var1range[0]+(var1range[1]-var1range[0])*i/N1
            if dof>=2:
                for j in range(N1):
                    var2=var2range[0]+(var2range[1]-var2range[0])*j/N1
                    if dof>=3:
                        for k in range(N1):
                            var3=var3range[0]+(var3range[1]-var3range[0])*k/N1
                            if dof>=4:
                                for l in range(N1):
                                    var4=var4range[0]+(var4range[1]-var4range[0])*l/N1
                                    if dof>=5:
                                        for m in range(N1):
                                            var5=var5range[0]+(var5range[1]-var5range[0])*m/N1
                                    else:
                                        var5=0
                                        calculateWorkspace()
                            else:
                                var4=0
                                var5=0
                                calculateWorkspace()
                    else:
                        var3=0
                        var4=0
                        var5=0
                        calculateWorkspace()
            else:
                var2=0
                var3=0
                var4=0
                var5=0
                calculateWorkspace()
    else:
        var1=0
        var2=0
        var3=0
        var4=0
        var5=0
        calculateWorkspace()
    checkWorkspace.__code__ = (lambda:None).__code__                                #zet de functie na 1 keer runnen naar None, omdat workspace global variable is en dus niet veranderd, moet maar 1 keer uitgerekend worden

dh_table = getdh_table(var1start,var2start,var3start,var4start,var5start)           #dhtable restten 
Tlist=dhTransform(dh_table,True)
p_0_ee=xyz(Tlist,ee,True)
startpunt=[p_0_ee[0][0], p_0_ee[1][0],p_0_ee[2][0]]
x1_traj = np.linspace(startpunt[0], punt1[0] , num=N2)                              #starten in start positie, gaan naar begin punt
y1_traj = np.linspace(startpunt[1], punt1[1] , num=N2)
z1_traj = np.linspace(startpunt[2], punt1[2] , num=N2)
if circle==True:
    x1_traj = np.linspace(startpunt[0], punt1[0]+ra , num=N2)                       #compenseren voor straal ra om naar 1ste punt van traject te gaan
    x2_traj = punt1[0] + ra*np.cos(-np.linspace(0, 2*np.pi, num=N2))
    y2_traj = punt1[1] + ra*np.sin(-np.linspace(0, 2*np.pi, num=N2))
else:
    x2_traj = np.linspace(punt1[0], punt2[0] , num=N2)                            #pad van punt 1 naar 2 
    y2_traj = np.linspace(punt1[1], punt2[1] , num=N2)
z2_traj     = np.linspace(punt1[2], punt2[2] , num=N2)                                #cirkel varieert in hoogte==pog
path=np.hstack([np.vstack([x1_traj, y1_traj, z1_traj]),np.vstack([x2_traj, y2_traj, z2_traj])]).T

def dist_closest():
    global workspace
    checkWorkspace()
    tree=KDTree(workspace)
    dist=tree.query(path)
    return dist

def plotDistWorkspaceTraj():
    plt.plot(np.linspace(0, N2*2, num=N2*2), dist_closest()[0])
    plt.title('distance workspace-trajectory')
    plt.savefig("output.jpg")
    plt.show()

def plotWorkspace():
    global workspace
    fig1 = plt.figure(1)                                                                #workspace vergelijken met pad van 1 naar 2
    ax1 = fig1.add_subplot(1,1,1, projection='3d',title='workspace')
    ax1.plot( workspace[:,0], workspace[:,1], workspace[:,2], 'b.' )
    ax1.plot( x1_traj, y1_traj, z1_traj, 'r-' )
    ax1.plot( x2_traj, y2_traj, z2_traj, 'g-' )
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    plt.show()

var1 = np.append(np.array(var1start),np.zeros(N2*2-1))
var2 = np.append(np.array(var2start),np.zeros(N2*2-1))
var3 = np.append(np.array(var3start),np.zeros(N2*2-1))
var4 = np.append(np.array(var4start),np.zeros(N2*2-1))
var5 = np.append(np.array(var5start),np.zeros(N2*2-1))

for i in range( 1, N2 ):
    dxyz = np.array([  [x1_traj[i] - x1_traj[i-1]] , [y1_traj[i] - y1_traj[i-1]],[z1_traj[i] - z1_traj[i-1] ]])
    jointDiff = np.linalg.pinv ( jacobian (var1[i-1],var2[i-1], var3[i-1], var4[i-1], var5[i-1]) ) @dxyz
    var1[i] = var1[i-1] + jointDiff[0]
    var2[i] = var2[i-1] + jointDiff[1]
    var3[i] = var3[i-1] + jointDiff[2]
    # var4[i] = var4[i-1] + jointDiff[3]
    # var5[i] = var5[i-1] + jointDiff[4]
for i in range( 0, N2-1):
    dxyz = np.array([  [x2_traj[i+1] - x2_traj[i]] , [y2_traj[i+1] - y2_traj[i]],[z2_traj[i+1] - z2_traj[i] ]])
    jointDiff = np.linalg.pinv ( jacobian (var1[i+N2-1],var2[i+N2-1], var3[i+N2-1], var4[i+N2-1], var5[i+N2-1]) ) @dxyz 
    var1[i+N2] = var1[i+N2-1] + jointDiff[0]
    var2[i+N2] = var2[i+N2-1] + jointDiff[1]
    var3[i+N2] = var3[i+N2-1] + jointDiff[2]
    # var4[i+N2] = var4[i+N2-1] + jointDiff[3]
    # var5[i+N2] = var5[i+N2-1] + jointDiff[4]


fig2 = plt.figure()
ax2 = fig2.add_subplot(1,1,1, projection='3d',title='path')
for i in range (0, N2*2-1):
    print(var1[i])
    dh_table = getdh_table(var1[i],var2[i],var3[i],var4[i],var5[i])
    Tlist=dhTransform(dh_table,False)
    total_robot=xyz(Tlist,ee,False)
    p_1_ee=Tlist=np.dot(dhTransform(dh_table,True),ee)
    if (i/30).is_integer():
        ax2.plot(total_robot[0],total_robot[1],total_robot[2])
        ax2.plot( p_1_ee[0], p_1_ee[1], p_1_ee[2], 'b.')
ax2.plot(x1_traj, y1_traj, z1_traj, 'r-')
ax2.plot(x2_traj, y2_traj, z2_traj, 'g-')
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')
plt.show()


fig3 = plt.figure()
ax3 = fig3.add_subplot( projection='3d',title='animation path')
sct, = ax3.plot([], [], [], "or--", markersize=2)
steps=int(N2*2/stepsVerlaging)
def generateFrame(ifrm):
    dh_table = getdh_table(var1[ifrm*stepsVerlaging],var2[ifrm*stepsVerlaging],var3[ifrm*stepsVerlaging],var4[ifrm*stepsVerlaging],var5[ifrm*stepsVerlaging])
    Tlist=dhTransform(dh_table,False)
    total_robot=xyz(Tlist,ee,False)
    sct.set_data(total_robot[0], total_robot[1])
    sct.set_3d_properties(total_robot[2])
ax3.plot(x1_traj, y1_traj, z1_traj, 'r-')
ax3.plot(x2_traj, y2_traj, z2_traj, 'g-')
ani = animation.FuncAnimation(fig3, generateFrame, steps, interval=1000/30*stepsVerlaging)
ani.save('pen.gif',writer='pillow',fps=30/stepsVerlaging)
plt.show()
print ('runtime: ',datetime.now()-start)



