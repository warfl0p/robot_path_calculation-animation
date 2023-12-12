import numpy as np
import matplotlib.pyplot as plt
import sys
from datetime import datetime
start=datetime.now()
onlyee=True

N1=40
N2=200              #iteraties
l2=0.1
l3=0.1
ee=[0.0,0,l3,1]

var1start=np.pi*3/2
var2start=np.pi/2                              #initele condities
var3start=0.2

punt1=[0.2,0.2,0.1]
punt2=[0.4,0.2,1]

def getdh_table(var1,var2,var3):
    theta1=var1
    theta2=var2
    d3    =var3
    dh_table = np.array([   [ 0 ,    0      ,   0     ,   theta1  ],
                            [ 0 ,   -np.pi/2,   0     ,   theta2  ],
                            [ 0 ,   np.pi/2,   d3+l2 ,   0        ]  ])
    return dh_table

def posee(dh_table,ee):
    T_list = []
    for i in range(dh_table.shape[0]):
        a = dh_table[i,0]
        alpha = dh_table[i,1]
        d = dh_table[i,2]
        theta = dh_table[i,3]
        T_i = np.array([
            [                np.cos(theta),                -np.sin(theta),              0,                   a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),  -np.sin(alpha) * d],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha),  np.cos(alpha),   np.cos(alpha) * d],
            [                            0,                             0,              0,                   1]])
        T_list.append(T_i)
    T_list=np.array(T_list)
    T_list2=T_list[0]
    for i in range(1,len(T_list)):
        T_list2=T_list2@T_list[i]
    ee=np.dot(T_list2,ee)
    return ee

def dhTransform(dh_table,onlyee):
    T_list = []
    for i in range(dh_table.shape[0]):
        a = dh_table[i,0]
        alpha = dh_table[i,1]
        d = dh_table[i,2]
        theta = dh_table[i,3]
        T_i = np.array([
            [np.cos(theta),                 -np.sin(theta),             0,                                  a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha),  np.cos(alpha),  np.cos(alpha) * d],
            [0,                                                         0,              0,                  1]])
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
        ee=np.dot(Tlist,ee)
    xlist.append(ee[0])
    ylist.append(ee[1])
    zlist.append(ee[2])    
    xyz=np.vstack((xlist,ylist,zlist))
    return xyz

def dist_closest(point, workspace):
    dist = sys.float_info.max
    for i in range(workspace.shape[0]):
        x1 = point[0]
        y1 = point[1]
        z1 = point[2]
        x2 = workspace[ i,0]
        y2 = workspace[ i,1]
        z2 = workspace[ i,2]
        distance = np.sqrt(np.power((x2 - x1), 2) + np.power((y2 - y1), 2) + np.power((z2 - z1), 2))
        if distance < dist:
            dist = distance
    return dist

def jacobian(var1,var2, var3):
    theta1=var1
    theta2=var2
    d3    =var3
    J=np.array([[-(d3+l2+l3)*np.sin(theta1)*np.sin(theta2)  , (d3+l2+l3)*np.cos(theta1)*np.cos(theta2)  , np.sin(theta2)*np.cos(theta1) ],
                [(d3+l2+l3)*np.sin(theta2)*np.cos(theta1)   , (d3+l2+l3)*np.sin(theta1)*np.cos(theta2)  , np.sin(theta1)*np.sin(theta2) ], 
                [0                                          , -(d3+l2+l3)*np.sin(theta2)                , np.cos(theta2)                ]])
    return J

p_0_ee_x = []
p_0_ee_y = []
p_0_ee_z = []
workspace=np.empty([0,3])
## workspace checken
for i in range(N1):
    var1=0+2*np.pi*i/N1
    for j in range(N1):
        var2=0+2*np.pi*j/N1
        for k in range(5):
            var3=0+1*k/5
            dh_table = getdh_table(var1,var2,var3)

            Tlist=dhTransform(dh_table,onlyee)
            total_robot=xyz(Tlist,ee,onlyee)
            p_0_ee=total_robot
            p_0_ee_x.append( float( p_0_ee[ 0 ] ) )
            p_0_ee_y.append( float( p_0_ee[ 1 ] ) )
            p_0_ee_z.append( float( p_0_ee[ 2 ] ) )
            workspace=np.vstack([workspace,np.hstack([total_robot[0],total_robot[1],total_robot[2]])])


dh_table = getdh_table(var1start,var2start,var3start)         #dhtable restten 
Tlist=dhTransform(dh_table,onlyee)
p_0_ee=xyz(Tlist,ee,onlyee)

startpunt=[p_0_ee[0][0], p_0_ee[1][0],p_0_ee[2][0]]

x1_traj = np.linspace(startpunt[0], punt1[0], num=N2)    #starten in start positie, gaan naar begin punt
y1_traj = np.linspace(startpunt[1], punt1[1], num=N2)
z1_traj = np.linspace(startpunt[2], punt1[2], num=N2)

x2_traj = np.linspace(punt1[0], punt2[0] , num=N2+1)         #pad van punt 1 naar 2 
y2_traj = np.linspace(punt1[1], punt2[1] , num=N2+1)
z2_traj = np.linspace(punt1[2], punt2[2] , num=N2+1)

fig1 = plt.figure(1)                                #workspace vergelijken met pad van 1 naar 2
ax1 = fig1.add_subplot(1,1,1, projection='3d')
ax1.plot( p_0_ee_x, p_0_ee_y, p_0_ee_z, 'b.' )
ax1.plot( x1_traj, y1_traj, z1_traj, 'r-' )
ax1.plot( x2_traj, y2_traj, z2_traj, 'g-' )
ax1.set_xlabel('xx')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
plt.show()

var1 = np.zeros(N2*2)
var2 = np.zeros(N2*2)
var3 = np.zeros(N2*2)

var1[0]=var1start   
var2[0]=var2start      
var3[0]=var3start

for i in range( 1, N2 ):
    dxyz = np.array([  [x1_traj[i] - x1_traj[i-1]] , [y1_traj[i] - y1_traj[i-1]],[z1_traj[i] - z1_traj[i-1] ]])
    jointDiff = np.linalg.pinv ( jacobian (var1[i-1],var2[i-1], var3[i-1]) ) @dxyz 
    var1[i] = var1[i-1] + jointDiff[0]
    var2[i] = var2[i-1] + jointDiff[1]
    var3[i] = var3[i-1] + jointDiff[2]

for i in range( 0, N2 ):
    dxyz = np.array([  [x2_traj[i+1] - x2_traj[i]] , [y2_traj[i+1] - y2_traj[i]],[z2_traj[i+1] - z2_traj[i] ]])
    jointDiff = np.linalg.pinv ( jacobian (var1[i+N2-1],var2[i+N2-1], var3[i+N2-1]) ) @dxyz 
    var1[i+N2] = var1[i+N2-1] + jointDiff[0]
    var2[i+N2] = var2[i+N2-1] + jointDiff[1]
    var3[i+N2] = var3[i+N2-1] + jointDiff[2]

p_1_ee_x = []
p_1_ee_y = []
p_1_ee_z = []
fig2 = plt.figure(2)
ax2 = fig2.add_subplot(1,1,1, projection='3d')

for i in range (0, N2*2):
    dh_table = getdh_table(var1[i],var2[i],var3[i])
    Tlist=dhTransform(dh_table,False)
    total_robot=xyz(Tlist,ee,False)
    ax2.plot(total_robot[0],total_robot[1],total_robot[2])
    dh_table = getdh_table(var1[i],var2[i],var3[i])
    p_1_ee = posee(dh_table,ee)
    p_1_ee_x.append( float( p_1_ee[ 0 ] ) )
    p_1_ee_y.append( float( p_1_ee[ 1 ] ) )
    p_1_ee_z.append( float( p_1_ee[ 2 ] ) )


ax2.plot( p_1_ee_x, p_1_ee_y, p_1_ee_z, 'b.' )
ax2.plot( x1_traj, y1_traj, z1_traj, 'r-' )
ax2.plot( x2_traj, y2_traj, z2_traj, 'g-' )
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')
plt.show()

force = np.array([[-2], [-1],[0]])
theta1 = 0.0
d2 = 0.2

torque = np.dot(np.transpose(jacobian(var1[0],var2[0], var3[0])),force)
print (torque)


##distance trajectory to closest workspace point plot
if(onlyee==True):
    distances_vec = np.zeros(shape=(N2, 1))
    for i in range(0, N2):
        distances_vec[i] = dist_closest(np.array([x1_traj[i], y1_traj[i], z1_traj[i]]), workspace)
    ax1.plot(x1_traj, y1_traj, z1_traj, '-k')
    ax1.plot(workspace[:, 0], workspace[:, 1], workspace[:, 2],'.')

print (datetime.now()-start)

fig = plt.figure(1)
ax = fig.gca()
ax.plot(np.linspace(0, N2, num=N2), distances_vec)
ax.set_xlabel('Index')
ax.set_ylabel('Distance')
plt.show()
