import numpy as np
import sys
import matplotlib.pyplot as plt

#function to generate homogenous transformation matrixes for donovit hartenberg table (a, alpha, d, theta) (We compute all Transformation matrixes from the whole DH table here, the guy in the video has a method to transform line per line, so he has to call this function multiple times)
def dhTransform(dh_table):
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

def invKinSlave(x,y,z):   #function to quickly calculate joint values!
    th1_slave = np.arctan(y/x)
    th2_slave = np.arctan(y/(-np.sin(th1_slave)*z))
    d3_slave = (z/np.sin(th2_slave))-l1_slave-l2_slave

    return th1_slave, th2_slave, d3_slave

#Actual Excercise
#constants
l1_master = 0.2
l2_master = 0.2
l1_slave = 0.05
l2_slave = 0.05

p_3_ee_master = np.array([[l2_master],[0],[0],[1]])
p_3_ee_slave = np.array([[0],[0],[l2_slave],[1]])

#the cartesian position of the slave equals the cartesian position of the master
p_0_ee_master = np.array([[0.3],[0.1],[0.1],[1]])
p_0_ee_slave = p_0_ee_master
##krijg joint values zodat cartesian coordinate van slave en master hetzelfde zijn
#compute joint positions of slave with inverse kinematics
th1_slave, th2_slave, d3_slave = invKinSlave(p_0_ee_slave[0,0],p_0_ee_slave[1,0],p_0_ee_slave[2,0])

#compute slave Jacobian
J_slave = np.array([
    [np.sin(th1_slave)*np.sin(th2_slave)*(d3_slave + l1_slave + l2_slave), -np.cos(th1_slave)*np.cos(th2_slave)*(d3_slave + l1_slave + l2_slave), -np.cos(th1_slave)*np.sin(th2_slave)],
    [-np.cos(th1_slave)*np.sin(th2_slave)*(d3_slave + l1_slave + l2_slave), -np.sin(th1_slave)*np.cos(th2_slave)*(d3_slave + l1_slave + l2_slave), -np.sin(th1_slave)*np.sin(th2_slave)],
    [0                                                                    , -np.sin(th2_slave)*(d3_slave + l1_slave + l2_slave)                  , np.cos(th2_slave)],
    ])
##alles gebeurd hier: define velocity master,=vel slave, inverteer jacobian en doe maal vel-> krijgen joint values
#Compute slave joint velocities
cart_vel_master = np.array([[0.13],[0.15], [0.05]])
cart_vel_slave = cart_vel_master
joint_vel_slave = np.dot(np.linalg.inv(J_slave),cart_vel_slave)

print("joint_vel_slave:")
print(joint_vel_slave)


#you can put the code between line 64 and 72 in loops if you wan tit for multiple positions!(see picture)
#extras:
# 1.we would also need forward kinematics of the master, we skipped that for now, for new positions!
# 2.maybe you are interested in force feedback