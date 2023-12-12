import numpy as np
import matplotlib.pyplot as plt
import sys
from datetime import datetime
start=datetime.now()

D2=False
onlyee=True

if (D2==False):
    fig1 = plt.figure(1)
    ax1 = fig1.add_subplot(projection='3d')

def getdh_table(theta1,d2):
    dh_table = np.array([   [   0 ,    0      ,   l0 ,   theta1  ],
                            [   l1,  -np.pi/2 ,   d2 ,   0       ]  ])
    return dh_table

def posee(theta1, d2,ee):
    dh_table = getdh_table(theta1,d2)
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

def jacobian( theta1, d2 ):
    #robot parameters
    l0 = 0.4
    l1 = 0.2
    s1 = np.sin(theta1)
    c1 = np.cos(theta1)
    J = np.array ([  [ (-s1*l1 - d2*c1) ,(-s1)  ],       #computation of the Jacobian matrix
                     [ ( c1*l1 - d2*s1) ,  c1   ],  
                     [ (0) ,             (0)    ]])
    return J

def frange(start, stop=None, step=None):
    # if stop and step argument is None set start=0.0 and step = 1.0
    start = float(start)
    if stop == None:
        stop = start + 0.0
        start = 0.0
    if step == None:
        step = 1.0

    count = 0
    while True:
        temp = float(start + count * step)
        if step > 0 and temp >= stop:
            break
        elif step < 0 and temp <= stop:
            break
        yield temp
        count += 1


##variables
l1= 0.2
l0=0.4
d2=0
theta1=0
# theta2= np.linspace(-np.pi,np.pi,N1)
# theta3= np.linspace(0,np.pi,N1)
ee=[0.0,0,0,1]
##path to follow

# Create points on a line between point A and point B
p_0_ee_x = []
p_0_ee_y = []
p_0_ee_z = []

## workspace checken
for theta1 in frange( 0, 2*np.pi, np.pi/100 ):
    for d2 in frange( 0, 1, 0.25 ):
        dh_table = getdh_table(theta1,d2)

        Tlist=dhTransform(dh_table,onlyee)
        total_robot=xyz(Tlist,ee,onlyee)
        p_0_ee=total_robot
        p_0_ee_x.append( float( p_0_ee[ 0 ] ) )
        p_0_ee_y.append( float( p_0_ee[ 1 ] ) )
        p_0_ee_z.append( float( p_0_ee[ 2 ] ) )

N2 = 100
theta1start=np.pi/2                                 #initele condities
d2start=0.1  
dh_table = getdh_table(theta1start,d2start)         #dhtable restten 
Tlist=dhTransform(dh_table,onlyee)
total_robot=xyz(Tlist,ee,onlyee)
p_0_ee=total_robot

x1_traj = np.linspace(p_0_ee[0][0], 0.5, num=N2)    #starten in start positie, gaan naar begin punt
y1_traj = np.linspace(p_0_ee[1][0], 0.5, num=N2)
z1_traj = np.linspace(p_0_ee[2][0], 0.4, num=N2)


x2_traj = np.linspace(0.5, -0.1 , num=N2+1)         #pad van punt 1 naar 2 
y2_traj = np.linspace(0.5, 0.4  , num=N2+1)
z2_traj = np.linspace(0.4, 0.4  , num=N2+1)

fig1 = plt.figure(1)                                #workspace vergelijken met pad van 1 naar 2
ax1 = fig1.add_subplot(1,1,1, projection='3d')
ax1.plot( p_0_ee_x, p_0_ee_y, p_0_ee_z, 'b.' )
ax1.plot( x1_traj, y1_traj, z1_traj, 'r-' )
ax1.set_xlabel('xx')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
plt.show()

theta1 = np.zeros(N2*2)
d2 = np.zeros(N2*2)
theta1[0]=theta1start           
d2[0]=d2start

for i in range( 1, N2 ):
    dxyz = np.array([  [x1_traj[i] - x1_traj[i-1]] , [y1_traj[i] - y1_traj[i-1]],[z1_traj[i] - z1_traj[i-1] ]])
    jointDiff = np.linalg.pinv ( jacobian (theta1[i-1], d2[i-1]) ) @dxyz 
    theta1[i] = theta1[i-1] + jointDiff[0]
    d2[i]     = d2 [i-1] + jointDiff[1]
    
for i in range( 0, N2 ):
    dxyz = np.array([  [x2_traj[i+1] - x2_traj[i]] , [y2_traj[i+1] - y2_traj[i]],[z2_traj[i] - z2_traj[i-1] ]])
    jointDiff = np.linalg.pinv ( jacobian (theta1[i+N2-1], d2[i+N2-1]) ) @dxyz 
    theta1[i+N2] = theta1[i+N2-1] + jointDiff[0]
    d2[i+N2]     = d2    [i+N2-1] + jointDiff[1]

p_1_ee_x = []
p_1_ee_y = []
p_1_ee_z = []
fig2 = plt.figure(2)
ax2 = fig2.add_subplot(1,1,1, projection='3d')

for i in range (0, N2*2):
    dh_table = getdh_table(theta1[i],d2[i])
    Tlist=dhTransform(dh_table,False)
    total_robot=xyz(Tlist,ee,False)
    ax2.plot(total_robot[0],total_robot[1],total_robot[2])
    p_1_ee = posee(theta1[i],d2[i],ee)
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

torque = np.dot(np.transpose(jacobian(theta1, d2)),force)
# print (torque)

# if(onlyee==True):
#     distances_vec = np.zeros(shape=(N2, 1))
#     for i in range(0, N2):
#         distances_vec[i] = dist_closest(np.array([x1_traj[i], y_traj[i], z_traj[i]]), workspace)
#     ax1.plot(x1_traj, y_traj, z_traj, '-k')
#     ax1.plot(workspace[:, 0], workspace[:, 1], workspace[:, 2],'.')
##assen juist schalen
# min=-0.5
# max=0.5
# minz=0   
# maxz=1
# assen=np.linspace(min,max,6)
# assenz=np.linspace(minz,maxz,6)
# ax1.set_xticks(assen)
# ax1.set_yticks(assen)
# ax1.set_zticks(assenz)
# ax1.axes.set_xlim3d(left=  min, right=max) 
# ax1.axes.set_ylim3d(bottom=min, top=max) 
# ax1.axes.set_zlim3d(bottom=minz, top=maxz)

ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
print (datetime.now()-start)
# plt.show()

# fig = plt.figure(1)
# ax = fig.gca()
# ax.plot(np.linspace(0, N2, num=N2), distances_vec)
# ax.set_xlabel('Index')
# ax.set_ylabel('Distance')
# plt.show()
