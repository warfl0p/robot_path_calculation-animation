import numpy as np
import sys
# import shape
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def dhTransform(dh_table): #function to generate homogenous transformation matrixes for donovit hartenberg table (a, alpha, d, theta)
    T_list = []

    for i in range(dh_table.shape[0]):  #so it goes from zero till 2 here
        a = dh_table[i,0]
        alpha = dh_table[i,1]
        d = dh_table[i,2]
        theta = dh_table[i,3]

        Ti = np.array([ [np.cos(theta),                 -np.sin(theta),                 0,              a],
                        [np.sin(theta)*np.cos(alpha),   np.cos(theta)*np.cos(alpha),    -np.sin(alpha), -np.sin(alpha)*d],
                        [np.sin(theta)*np.sin(alpha),   np.cos(theta)*np.sin(alpha),    np.cos(alpha),  np.cos(alpha)*d],
                        [0,                             0,                              0,                 1]
                        ])
        T_list.append(Ti)
    return T_list

# List with stepsize for float(because range is only for integers
def frange(start, stop, step):
    i = start
    while i < stop:
        yield i
        i += step

#generate unit length joint velocities (2 DOF, so we sample over a circle
theta1_dot_list = []
d3_dot_list = []

circle_radius = 2 #unit-length velocities (you can scale here if needed)
for circle_angle in frange(0,2*np.pi, np.pi/10):
    theta1_dot_list.append(circle_radius * np.cos(circle_angle))
    d3_dot_list.append(circle_radius * np.sin(circle_angle))

#plot joint velocity circle
fig = plt.figure(1)
plt.plot(theta1_dot_list,d3_dot_list,"b.")
plt.axis("equal")
plt.xlabel( "theta1_dot")
plt.ylabel("d3_dot")
plt.grid()
plt.show()

#compute workspace with corresponding jacobians
# Set constants
l1 = 0.05
l2 = 0.1
l3 = 0.1
p_3_ee = np.array([[0],[0],[l3],[1]])
theta2 = -np.pi/2  #!!!!!

p_0_ee_x = []
p_0_ee_y = []
p_0_ee_z = []
J_list = [] #list containing the jacobians computed for a selection of joint positions
Jinv_list=[]
for theta1 in frange(0, np.pi, np.pi/3):  #you can choose how many points you want to see
    for d3 in frange(0,0.2,0.05):
        dh_Table = np.array([
            [0, 0, 0, theta1],
            [0, np.pi / 2, 0, theta2],
            [0, -np.pi / 2, d3 + l2, 0],
        ])
        T_list = dhTransform(dh_Table)
        T_0_1 = T_list[0]
        T_1_2 = T_list[1]
        T_2_3 = T_list[2]

        T_0_3 = np.dot(T_0_1, np.dot(T_1_2, T_2_3, ))
        p_0_ee = np.dot(T_0_3, p_3_ee)

        p_0_ee_x.append(float(p_0_ee[0]))
        p_0_ee_y.append(float(p_0_ee[1]))
        p_0_ee_z.append(float(p_0_ee[2]))

        #also compute the jacobian for this specific joint configuration, see ex 4.3 of robotics for this
        J = np.array([[-np.sin(theta1)*(d3+l2+l3), np.cos(theta1)],
                     [np.cos(theta1)*(d3+l2+l3),np.sin(theta1)]])
        Jinv=np.linalg.pinv(J)
        Jinv_list.append(Jinv)
        J_list.append(J)

fig1 = plt.figure(1)
ax1 = fig1.add_subplot(projection = '3d')
ax1.plot(p_0_ee_x,p_0_ee_y,p_0_ee_z, 'b.')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
plt.show()
#note in result it seems that there is also a z direction,
# but look at index, e-17 => soo it's all basically 0!!

#now for slide 5 point 4
#now iterate over the workspace, and apply the unit length joint velocities at every point
list_of_all_ellipsoid_x_positions = []
list_of_all_ellipsoid_y_positions = []
list_of_all_ellipsoid_z_positions = []
list_of_all_ellipsoid_x_positionsinv = []
list_of_all_ellipsoid_y_positionsinv = []
list_of_all_ellipsoid_z_positionsinv = []
dt = 0.03 # [seconds], so how long each velocity

for i in range(0,len(p_0_ee_x)): #so for every point in the workspace, we will use the velocities in all directions
    #get cartesian positions
    x = p_0_ee_x[i]
    y = p_0_ee_y[i]
    z = p_0_ee_z[i]

    #get correspondin Jacobian
    J = J_list[i]
    Jinv=Jinv_list[i]
    for j in range(0, len(theta1_dot_list)):  #velocities in all directions
        #joint velocity vector
        vel_joint = np.array([theta1_dot_list[j],
                             d3_dot_list[j]])

        #multiply jacobian with unit-lenth joint velocities
        vel_cartesian = np.dot(J,vel_joint)
        force_cartesian=np.dot(Jinv,vel_joint)
        #apply the cartesian velocity at the end effector position for a chosen time interval
        list_of_all_ellipsoid_x_positions.append(x + vel_cartesian[0]*dt)
        list_of_all_ellipsoid_y_positions.append(y + vel_cartesian[1]*dt)
        list_of_all_ellipsoid_z_positions.append(z)
        list_of_all_ellipsoid_x_positionsinv.append(x + force_cartesian[0]*dt)
        list_of_all_ellipsoid_y_positionsinv.append(y + force_cartesian[1]*dt)
        list_of_all_ellipsoid_z_positionsinv.append(z)

#plot workspace and manipulability ellipsoids
fig = plt.figure(2)
ax = fig.add_subplot(1,1,1, projection = '3d')
ax.plot(p_0_ee_x, p_0_ee_y, p_0_ee_z, "g.")
ax.plot(list_of_all_ellipsoid_x_positions,list_of_all_ellipsoid_y_positions,list_of_all_ellipsoid_z_positions, "b.")
ax.plot(list_of_all_ellipsoid_x_positionsinv,list_of_all_ellipsoid_y_positionsinv,list_of_all_ellipsoid_z_positionsinv, "r.")
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#plt.axis("equal")
plt.show()

#we see that in d3, we have more translation(so more mobility), but less resolution
#in theta, we have low mobility, but high precision


