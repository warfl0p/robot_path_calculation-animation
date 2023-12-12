import numpy as np
import matplotlib.pyplot as plt
from sympy import Derivative
from numpy.linalg import inv
import sympy as sym
import re

##implicit multiplication matrix
d1 =sym.Symbol('d1')            # definieer hier alle variabelen die in dh tabel en in end effector voorkomen
d2 =sym.Symbol('d2')
d3 =sym.Symbol('d3')
d4 =sym.Symbol('d4')
d5 =sym.Symbol('d5')
theta1=sym.Symbol('theta1')
theta2=sym.Symbol('theta2')
theta3=sym.Symbol('theta3')
theta4=sym.Symbol('theta4')
l1=sym.Symbol('l1')
l2=sym.Symbol('l2')
a1=sym.Symbol('a1')
a2=sym.Symbol('a2')
a3=sym.Symbol('a3')
getalpi=sym.pi

def DHtoFK2(a,alpha,d,theta):
    T=sym.Matrix([[sym.cos(theta),-sym.sin(theta),0,a],
                  [sym.sin(theta)*sym.cos(alpha),sym.cos(theta)*sym.cos(alpha),-sym.sin(alpha),-sym.sin(alpha)*d],
                  [sym.sin(theta)*sym.sin(alpha),sym.cos(theta)*sym.sin(alpha),sym.cos(alpha),sym.cos(alpha)*d],
                  [0,0,0,1]])
    return T

T01=DHtoFK2(  0  ,  0          , d1,  theta1)       #dit is de dh tabel volgorde is, a apha d theta 
T12=DHtoFK2(  0  ,  -getalpi/2 ,  0,  theta2)
T23=DHtoFK2(  a2 ,  0          , d3,  theta3)
T34=DHtoFK2(  0  ,  getalpi/2  , d4,  theta4)       
T45=DHtoFK2(  a3 ,  0          , d5,  0     )
ee=np.array([0,0,0,1])

T0e=np.array(np.dot(T01,np.dot(T12,np.dot(T23,np.dot(T34,np.dot(T45,ee))))))        # hier wordt transofrmatie matrix van 0 naar end effector berekend, afhangelijk van hoeveel rijen de dh tabel is moet er een dot product verwijderd worden
# T0e=np.array(np.dot(T01,np.dot(T12,np.dot(T23,ee))))
print(T0e)                                                                          #print T0e, hier zullen sinus en cos gewoon als sin en cos geschreven zijn
                                                                                    #in volgende lijnen code voeg ik .np toe voor elke sin en cos zodat in ander je deze output gewoon kan  copy pasten
string=str(T0e)
def find(string, char):
    for i, c in enumerate(string):
        if c == char:
            yield i

indices = list(find(string, 'i'))

for i in range(len(indices)):
    string = string[:(indices[i]-1+3*i)] + 'np.' + string[(indices[i]-1+3*i):]

indices = list(find(string, 'o'))

for i in range(len(indices)):
    sstringtr = string[:(indices[i]-1+3*i)] + 'np.' + string[(indices[i]-1+3*i):]
print('T0e=',string)                                                                #T0e met np. voor cos en sin

##T0e afleiden naar jmatrix
jmatrix=[[],[],[]]

print(T0e.shape)
for i in range(T0e.shape[0]-1):
    jmatrix[i].append(sym.simplify(T0e[i].diff(theta1)  ))
    jmatrix[i].append(sym.simplify(T0e[i].diff(theta2)  ))
    jmatrix[i].append(sym.simplify(T0e[i].diff(theta3)  ))                      #in dit geval waren er 5 rijen in dh tabel en dus ook in T0e,  
    jmatrix[i].append(sym.simplify(T0e[i].diff(theta4)  ))                      #verwijder hier rij dat te veel is, en verander variabele bij  .diff() naar de variabele die veranderd in die rij       
    jmatrix[i].append(sym.simplify(T0e[i].diff(d5)      ))
                                                                                #voegen opnieuw np. voor elke cos en sin toe zodat de output makkelijk gebruikt kan worden in volgend programma
string=str(jmatrix)
def find(string, char):
    for i, c in enumerate(string):
        if c == char:
            yield i

indices = list(find(string, 'i'))

for i in range(len(indices)):
    string = string[:(indices[i]-1+3*i)] + 'np.' + string[(indices[i]-1+3*i):]

indices = list(find(string, 'o'))

for i in range(len(indices)):
    string = string[:(indices[i]-1+3*i)] + 'np.' + string[(indices[i]-1+3*i):]

print('jmatrix=',string)

#open text file
text_file = open("jacobian.txt", "w")
 
#write string to file
text_file.write(string)
 
#close file
text_file.close()

#code om na te kijken of een rotatie matrix juist is door determinant uit te rekenen, heeft niks te maken met code hier boven
## determinant
# A=np.array([
#                 [-.90099,   -.31765,    .2955],
#                 [0.064356,  -.77144,   -.63304],
#                 [.42904,   -.55135,    .7155],])

# print(np.dot(A[:,0],A[:,1]))
# print(np.linalg.det(A))

# A=np.array([
#                 [-.90099,       .2955 ,-.31765],
#                 [0.064356,     -.63304,-.77144],
#                 [.42904,       .7155  ,.55135 ],])

# print(np.dot(A[:,0],A[:,1]))
# print(np.linalg.det(A))
