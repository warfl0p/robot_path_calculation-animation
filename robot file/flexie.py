import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from datetime import datetime
from matplotlib import animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from robot_path_info import *
start=datetime.now()

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

def checkWorkspace():
    global workspace
    workspace=np.empty([0,3])
    def calculateWorkspace():
        global workspace
        dh_table = getdh_table(varList)
        Tlist=dhTransform(dh_table,True)
        total_robot=xyz(Tlist,ee,True)
        workspace=np.vstack([workspace,np.hstack([total_robot[0],total_robot[1],total_robot[2]])])
    if dof>=1:
        for i in range(N1):
            varList[0]=var1range[0]+(var1range[1]-var1range[0])*i/N1
            if dof>=2:
                for j in range(N1):
                    varList[1]=var2range[0]+(var2range[1]-var2range[0])*j/N1
                    if dof>=3:
                        for k in range(N1):
                            varList[2]=var3range[0]+(var3range[1]-var3range[0])*k/N1
                            if dof>=4:
                                for l in range(N1):
                                    varList[3]=var4range[0]+(var4range[1]-var4range[0])*l/N1
                                    if dof>=5:
                                        for m in range(N1):
                                            varList[4]=var5range[0]+(var5range[1]-var5range[0])*m/N1
                                            if dof>=6:
                                                for o in range(N1):
                                                    varList[5]=var6range[0]+(var6range[1]-var6range[0])*o/N1
                                            else:
                                                calculateWorkspace()
                                    else:
                                        calculateWorkspace()
                            else:
                                calculateWorkspace()
                    else:
                        calculateWorkspace()
            else:
                calculateWorkspace()
    else:
        calculateWorkspace()
    checkWorkspace.__code__ = (lambda:None).__code__                                #zet de functie na 1 keer runnen naar None, omdat workspace global variable is en dus niet veranderd, moet maar 1 keer uitgerekend worden

dh_table = getdh_table(varStartList)           #dhtable restten 
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
    x2_traj = np.linspace(punt1[0], punt2[0] , num=N2)                              #pad van punt 1 naar 2 
    y2_traj = np.linspace(punt1[1], punt2[1] , num=N2)
z2_traj     = np.linspace(punt1[2], punt2[2] , num=N2)                              #cirkel varieert in hoogte==pog
path=np.hstack([np.vstack([x1_traj, y1_traj, z1_traj]),np.vstack([x2_traj, y2_traj, z2_traj])]).T

def dist_closest():
    checkWorkspace()
    tree=KDTree(workspace)
    dist=tree.query(path)
    return dist

def plotDistWorkspaceTraj(window):
    fig=plt.figure()
    placePlotInFrame(fig,window)
    plt.plot(np.linspace(0, N2*2, num=N2*2), dist_closest()[0])
    plt.title('distance workspace-trajectory')
    plt.savefig("output.jpg")

def plotWorkspace(window):
    checkWorkspace()
    fig1 = plt.figure() 
    placePlotInFrame(fig1,window)                   #workspace vergelijken met pad van 1 naar 2
    ax1 = fig1.add_subplot(1,1,1, projection='3d',title='workspace')
    ax1.plot( workspace[:,0], workspace[:,1], workspace[:,2], 'b.' )
    ax1.plot( x1_traj, y1_traj, z1_traj, 'r-' )
    ax1.plot( x2_traj, y2_traj, z2_traj, 'g-' )
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')

varListTotal=[0]*dof
for i in range(dof):
    varListTotal[i] = np.append(np.array(varStartList[i]),np.zeros(N2*2-1))

def computePathRobot():
    global varListTotal
    for i in range( 1, N2 ):
        dxyz = np.array([  [x1_traj[i] - x1_traj[i-1]] , [y1_traj[i] - y1_traj[i-1]],[z1_traj[i] - z1_traj[i-1] ]])
        for j in range(dof):
            varList[j]=varListTotal[j][i-1]
            jointDiff = np.linalg.pinv ( jacobian (varList) ) @dxyz
        for j in range(dof):
            varListTotal[j][i] = varListTotal[j][i-1] + jointDiff[j]

    for i in range( 0, N2-1):
        dxyz = np.array([  [x2_traj[i+1] - x2_traj[i]] , [y2_traj[i+1] - y2_traj[i]],[z2_traj[i+1] - z2_traj[i] ]])
        for j in range(dof):
            varList[j]=varListTotal[j][i+N2-1]
        jointDiff = np.linalg.pinv ( jacobian (varList) ) @dxyz
        for j in range(dof):
            varListTotal[j][i+N2] = varListTotal[j][i+N2-1] + jointDiff[j]
    computePathRobot.__code__ = (lambda:None).__code__

def plotPathRobot(window):
    global eeList
    eeList= []
    computePathRobot()
    fig2 = plt.figure()
    placePlotInFrame(fig2,window)
    ax2 = fig2.add_subplot(1,1,1, projection='3d',title='path')
    varList=[0]*dof
    for i in range (0, N2*2-1):
        for j in range(dof):
            varList[j]=varListTotal[j][i]
        dh_table = getdh_table(varList)
        Tlist=dhTransform(dh_table,False)
        total_robot=xyz(Tlist,ee,False)
        p_1_ee=Tlist=np.dot(dhTransform(dh_table,True),ee)
        if (i/30).is_integer():
            ax2.plot(total_robot[0],total_robot[1],total_robot[2])                   
            distancePathee(p_1_ee)      #voor zelfde punten wordt afstand tot eerste pad berekend
            ax2.plot( p_1_ee[0], p_1_ee[1], p_1_ee[2], 'b.')
    ax2.plot(x1_traj, y1_traj, z1_traj, 'r-')
    ax2.plot(x2_traj, y2_traj, z2_traj, 'g-')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('z')
    
    plt.savefig("output.jpg")
    

def animatePathRobot(window):
    computePathRobot()
    fig3 = plt.figure()
    placePlotInFrame(fig3,window)
    ax3 = fig3.add_subplot( projection='3d',title='animation path')
    sct, = ax3.plot([], [], [], "or--", markersize=2)
    steps=int(N2*2/stepsVerlaging)
    def generateFrame(ifrm):
        for j in range(dof):
            varList[j]=varListTotal[j][ifrm*stepsVerlaging]
        dh_table = getdh_table(varList)
        Tlist=dhTransform(dh_table,False)
        total_robot=xyz(Tlist,ee,False)
        sct.set_data(total_robot[0], total_robot[1])
        sct.set_3d_properties(total_robot[2])
    ax3.plot(x1_traj, y1_traj, z1_traj, 'r-')
    ax3.plot(x2_traj, y2_traj, z2_traj, 'g-')
    ani = animation.FuncAnimation(fig3, generateFrame, steps, interval=1000/30*stepsVerlaging)
    ani.save('pen.gif',writer='pillow',fps=30/stepsVerlaging)
    window.mainloop()

def placePlotInFrame(figure, window):
    canvas = FigureCanvasTkAgg(figure, master = window)  
    canvas.draw()
    toolbar =   NavigationToolbar2Tk(canvas, window, pack_toolbar =False)
    toolbar.update()
    canvas.get_tk_widget().grid(sticky='W',column=0, row=2,padx=10, pady=10)


def distancePathee(ee):       #werkt enkel wanneer circle= False, want path is niet van start naar punt1 als circle=true
        x, y, z, _ = ee     #berekent enkel afstand van elk punt naar pad van start naar punt1
        v = np.array([punt1[0] - startpunt[0], punt1[1] - startpunt[1], punt1[2] - startpunt[2]])
        u = np.array([x - startpunt[0], y - startpunt[1], z - startpunt[2]])
        distance = np.linalg.norm(np.cross(v, u)) / np.linalg.norm(v)
        print(distance)

print ('runtime: ',datetime.now()-start)



