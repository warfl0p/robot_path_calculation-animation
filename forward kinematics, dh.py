import numpy as np
import matplotlib.pyplot as plt
import sys
from datetime import datetime
start=datetime.now()

D2=False
onlyee=False

if (D2==False):
    fig1 = plt.figure(1)
    ax1 = fig1.add_subplot(projection='3d')

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

N1=5
N2 = 20
workspace=np.empty([0,3])
##variables
l2=0.1
d3= np.linspace(0.1,0.2,N1)
theta1= np.linspace(0,np.pi,N1)
theta2= np.linspace(0,np.pi,N1)
print(theta2)
ee=[0,0,0.1,1]
##path to follow
x_traj = np.linspace(-0.2335, 0.208, num=N2)
y_traj = np.linspace(0.03987, -0.1127, num=N2)
z_traj = np.linspace(0.5843, 0.879, num=N2)

for i in range(0,N1,1):
    for j in range(0,N1,1):
        for k in range(0,N1,1):
            dh_table = np.array([   [   0 ,    0      ,   0        ,   theta1[i]  ],
                                    [   0 ,   - np.pi/2,   0        ,   theta2[j]  ],
                                    [   0 ,   np.pi/2,   d3[k]+l2 ,   0       ]  ])
            
            Tlist=dhTransform(dh_table,onlyee)
            total_robot=xyz(Tlist,ee,onlyee)
            
            if (D2==True):
                plt.plot(total_robot[0],total_robot[2])
            elif(onlyee==False):
                ax1.plot(total_robot[0],total_robot[1],total_robot[2])
            elif(onlyee==True):
                workspace=np.vstack([workspace,np.hstack([total_robot[0],total_robot[1],total_robot[2]])]) 
               
if(onlyee==True):
    distances_vec = np.zeros(shape=(N2, 1))
    for i in range(0, N2):
        distances_vec[i] = dist_closest(np.array([x_traj[i], y_traj[i], z_traj[i]]), workspace)
    ax1.plot(x_traj, y_traj, z_traj, '-k')
    ax1.plot(workspace[:, 0], workspace[:, 1], workspace[:, 2],'.')
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
plt.show()

# fig = plt.figure(1)
# ax = fig.gca()
# ax.plot(np.linspace(0, N2, num=N2), distances_vec)
# ax.set_xlabel('Index')
# ax.set_ylabel('Distance')
# plt.show()
