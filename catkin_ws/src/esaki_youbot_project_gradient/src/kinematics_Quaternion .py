
from cmath import pi
import numpy as np
import math
import random

from torch import arcsin, atan2
def FK(js):
    Ts = []

    c1 = math.cos(js[0])
    s1 = math.sin(js[0])
    c2 = math.cos(js[1])
    s2 = math.sin(js[1])
    c3 = math.cos(js[2])
    s3 = math.sin(js[2])
    c4 = math.cos(js[3])
    s4 = math.sin(js[3])
    c5 = math.cos(js[4])
    s5 = math.sin(js[4])

    a2 = 0.155
    a3 = 0.135
    d1 = 0.147
    d5 = 0.218

    Ts.append(np.matrix([[c1, 0, s1, 0], [s1, 0, -c1, 0], [0, 1, 0, d1], [0, 0, 0, 1]]))
    Ts.append(np.matrix([[c2, -s2, 0, a2*c2], [s2, c2, 0, a2*s2], [0, 0, 1,0], [0, 0, 0, 1]]))
    Ts.append(np.matrix([[c3, -s3, 0, a3*c3], [s3, c3, 0, a3*s3], [0, 0, 1, 0], [0, 0, 0, 1]]))
    Ts.append(np.matrix([[c4, 0, s4, 0], [s4, 0, -c4, 0], [0, 1, 0, 0], [0, 0, 0, 1]]))
    Ts.append(np.matrix([[c5, -s5, 0, 0], [s5, c5, 0, 0], [0, 0, 1, d5], [0, 0, 0, 1]]))

    return [Ts[0], Ts[0]*Ts[1], Ts[0]*Ts[1]*Ts[2], Ts[0]*Ts[1]*Ts[2]*Ts[3], Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4]]

def make_q(e, theta):
    return [e[0]*math.sin(theta/2), e[1]*math.sin(theta/2), e[2]*math.sin(theta/2), math.cos(theta/2)]

def check_q(Theta):

    s321 = math.sin(Theta[3]/2 + Theta[2]/2 + Theta[1]/2)
    c321 = math.cos(Theta[3]/2 + Theta[2]/2 + Theta[1]/2)
    s04 = math.sin(Theta[0]/2 + Theta[4]/2)
    c04 = math.cos(Theta[0]/2 + Theta[4]/2)

    return [s321*c04,
            c321*s04,
            s321*s04,
            c321*c04]


def my_cross(q, p):
    return [q[3]*p[0] - q[2]*p[1] + q[1]*p[2] + q[0]*p[3],
            q[2]*p[0] + q[3]*p[1] - q[0]*p[2] + q[1]*p[3],
            -q[1]*p[0] + q[0]*p[1] + q[3]*p[2] + q[2]*p[3],
            -q[0]*p[0] - q[1]*p[1] - q[2]*p[2] + q[3]*p[3]]

def fixPi(Js):
    js = Js
    for j in range(len(Js)):
      if js[j] > math.pi:
        js[j] = js[j] - int((js[j] + math.pi)/(2*math.pi))*2*math.pi
      elif js[j] < -math.pi:
        js[j] = js[j] - int((js[j]-math.pi)/(2*math.pi))*2*math.pi
    return js




def AnalyticSolution(x,y,z,quaternion):
    
    L = [0.147,0.155,0.135,0.218]
    q = quaternion
    vx = 2*(q[0]*q[2] + q[3]*q[1])
    vy = 2*(q[1]*q[2] - q[3]*q[0])
    vz = q[3]**2 - q[0]**2 - q[1]**2 + q[2]**2

    x3 = x - L[3]*vx
    y3 = y - L[3]*vy
    z3 = z - L[3]*vz
    
    Theta0 = theta0([x,y,z])
    
    x1 = 0
    y1 = 0
    z1 = 0.147

    L13 = math.sqrt((x3 - x1)**2 + (y3 - y1)**2 + (0.146+0.155+0.135 - L[0])**2)

    Theta2 = math.acos(-(L[1]**2+L[2]**2 - L13**2)/(2*L[1]*L[2]))
    L03 = math.sqrt((x3 - x1)**2 + (y3 - y1)**2 + (z3)**2)
    Theta1_1 = math.acos((L[1]**2 + L13**2 - L[2]**2)/(2*L[1]*L13))
    Theta1_2 = math.acos((L[0]**2 + L13**2 - L03**2)/2*L[0]*L13)
    Theta1 = math.pi - (Theta1_1 + Theta1_2)

    xyz2 = FK([Theta0, Theta1, 0, 0, 0])
    x2 = xyz2[2][0, 3]
    y2 = xyz2[2][1, 3]
    z2 = xyz2[2][2, 3]
    print(x2,y2,z2)
    L26_2 = (x - x2)**2 + (y - y2)**2 + (z - z2)**2
    Theta3 = math.acos(-(L[2]**2 + L[3]**2 - L26_2)/(2*L[2] *L[3]))

    my_q = []
    my_q.append(make_q([0,0,1],Theta0))
    my_q.append(make_q([1,0,0],Theta1))
    my_q.append(make_q([1,0,0],Theta2))
    my_q.append(make_q([1,0,0],Theta3))

    my_Q = []
    my_Q.append(my_cross(my_q[0],my_q[1]))
    my_Q.append(my_cross(my_Q[0],my_q[2]))
    my_Q.append(my_cross(my_Q[1],my_q[3]))

    qx03 = my_Q[2][0]
    qy03 = my_Q[2][1]
    qz03 = my_Q[2][2]
    qw03 = my_Q[2][3]

    qx04 = q[0]
    qy04 = q[1]
    qz04 = q[2]
    qw04 = q[3]

    C5 = (qx03*qx04 + qy03*qy04)/(qx03**2 + qy03**2)
    S5 = (qx04*qx03 - qy04*qy03)/(qx03**2 + qy03**2)

    Theta4 = 2*math.atan2(S5,C5)

    js = fixPi([Theta0, Theta1, Theta2, Theta3, Theta4])

    return js












def solve(x,y,z,target_angle):

    L = [0.147,0.155,0.135,0.218]
    
    Theta0 = theta0([x,y,z])
    
    ik_l ,ik_z = getlz_1(x,y,z,target_angle,L)
    Theta1 = getAngle1_1(ik_l, ik_z,L)
    #Theta1 = math.pi/2
    Theta2 = getAngle2_1(ik_l, ik_z,L)
    Theta3 = target_angle - Theta2 -Theta1
    return [Theta0,Theta1,Theta2,Theta3]

def getlz(x,y,z,target_angle,L):
    raw_l = np.sqrt(x**2+y**2)
    raw_z = z
    
    ik_l = raw_l - L[3] * math.cos(target_angle)
    ik_z = raw_z - L[0] -L[3]* math.sin(target_angle)

    print(raw_l)
    print(raw_z)
    print(ik_l)
    print(ik_z)
    return ik_l,ik_z


def getlz_1(x,y,z,target_angle,L):
    raw_l = math.sqrt(x*x+y*y)
    raw_z = z

    ik_l = raw_l - L[3]*math.sin(math.pi - target_angle)
    ik_z = raw_z - L[3]*math.cos(math.pi - target_angle)
    print(ik_l,ik_z)
    ik_l = 0
    ik_z = 0.437
    return ik_l,ik_z

def getAngle2_1(ik_l,ik_z,L):
    v = math.sqrt(ik_l*ik_l + (ik_z-L[0])*(ik_z-L[0]))
    print(math.acos((L[1]*L[1]+L[2]*L[2]-v*v)/2*L[1]*L[2]) )
    theta = math.pi - math.acos((L[1]*L[1]+L[2]*L[2]-v*v)/2*L[1]*L[2]) 
    return theta



def getAngle1_1(ik_l,ik_z,L):
    v1_3 = math.sqrt(ik_l*ik_l + (ik_z-L[0])*(ik_z-L[0]))
    v0_3 = math.sqrt(ik_l*ik_l + ik_z*ik_z)
    theta1_3 = math.acos((L[1]*L[1]+v1_3*v1_3-L[2]*L[2])/2*L[1]*v1_3)
    theta0_3 = math.acos((L[0]*L[0]+v0_3*v0_3-v1_3*v1_3)/2*L[0]*v0_3)
    
    theta = math.pi - (theta1_3 + theta0_3)
    return theta

def getAngle1(ik_l,ik_z,L):
    v = math.sqrt(ik_l*ik_l + ik_z * ik_z)
    return math.pi/2 - getAngle(L[1],v,L[2]) - math.atan2(ik_z, ik_l)

def getAngle2(ik_l,ik_z,L):
    v = math.sqrt(ik_l * ik_l + ik_z * ik_z)
    return math.pi/2 - getAngle(L[1],L[2],v)

def getAngle(a,b,c):
    cos_n = a * a + b * b - c * c
    cos_d = 2.0 * a * b
    return math.acos(-cos_n/cos_d)

def theta0(p):

    if p[0]== 0 and p[1] == 0:
        theta = 0
    else:
        theta = math.atan2(p[1],p[0])
    return theta






theta = []
theta.append(0)
theta.append(math.pi/2)
theta.append(0)
theta.append(math.pi/2)
theta.append(0)
fk = FK(theta)
L = [0.147,0.155,0.135,0.218]
print(0.147+0.155+0.135)
random_js = []
random_js.append(random.uniform(-math.pi*17/18, math.pi*17/18))
random_js.append(random.uniform(-math.pi*100/180, math.pi*135/180))
random_js.append(random.uniform(-math.pi*12/18, math.pi*153/180))
random_js.append(random.uniform(-math.pi*27/18, math.pi*27/18))
random_js.append(random.uniform(-math.pi*12/18, math.pi*12/18))


TTs = FK(theta)
qtn = check_q(theta)
x = TTs[4][0, 3]
y = TTs[4][1, 3]
z = TTs[4][2, 3]
x1 = TTs[0][0, 3]
y1 = TTs[0][1, 3]
z1 = TTs[0][2, 3]
x4 = TTs[3][0, 3]
y4 = TTs[3][1, 3]
z4 = TTs[3][2, 3]


inverce = solve(x,y,z,math.pi/2)
inverce.append(0)
IK = FK(inverce)
x_result = IK[4][0, 3]
y_result = IK[4][1, 3]
z_result= IK[4][2, 3]

#逆運動学で各関節角度calc_jsを求める
#calc_js = AnalyticSolution(x, y, z, qtn)

#calc_jsから、手先座標[calc_x, calc_y, calc_z], 姿勢calc_qtnを導出
#calc_TTs = FK(calc_js)
#calc_qtn = check_q(calc_js)
#calc_x = calc_TTs[4][0, 3]
#calc_y = calc_TTs[4][1, 3]
#calc_z = calc_TTs[4][2, 3]

print("x**2+y**2+z**2",x**2+y**2+z**2)
print("      xyz:", [x, y, z])
print("      xyz1:", [x1, y1, z1])
print("      xyz4:", [x4, y4, z4])
print("      forward",theta)
print("      inverce",inverce)
print("      xyz:",[x_result,y_result,z_result])

#print("      calc_xyz:", [calc_x, calc_y, calc_z])
#print("      Quaternion:", qtn)
#print("      calc_Quaternion:", calc_qtn)


"""
L = [0.147,0.155,0.135,0.218]
L14 = math.sqrt(x4**2 + y4**2 + (z4 - z1)**2)
print("L14",L14)
Theta2 = math.acos(-(L[1]**2 + (L[2])**2 - L14**2)/(2*L[1]*(L[2])))
'''print( L14**2)
print(-(L[1]**2 + (L[2])**2 - L14**2))
print((2*L[1]*(L[2])))
print("Theta2",Theta2)'''

Theta1_1 = math.acos((L[1]**2 + L14**2 - L[2]**2)/2*L[1]*L14)

print(L[1]**2+L14**2 -L[2]**2)
L04_ = math.sqrt((x4)**2 + (y4)**2 + (z4)**2)

Theta1_2 = math.acos((L[0]**2 + L14**2 - L04_**2)/2*L[0]*L14)

Theta = math.asin((L[2]/L14)*math.sin(math.pi-Theta2))
Theta1_ = math.atan2(z4-z1,math.sqrt(x*x+y*y))
'''print("Theta1_",Theta1_1)
print("Theta",Theta1_2)
print(Theta,Theta1_)
print(Theta1_1 + Theta1_2)'''
Theta1 = math.pi/2 - (Theta+ Theta1_)

print("Theta1",Theta1)
"""

px = x
py = y
pz = z

L = [0.147,0.155,0.135,0.218]
print(0.135/2)

xc = math.sqrt(px*px+py*py)
zc = pz - L[0]

phi_c = 0

xw = xc - L[3]*math.cos(phi_c)
zw = zc - L[3]*math.cos(phi_c)

alpha = math.atan2(zw,xw)

cos_beta = (L[1]*L[1] + L[2]*L[2] - xw*xw -zw*zw)/(2*L[1]*L[2])
sin_beta = math.sqrt(abs(1-(cos_beta*cos_beta)))
theta_link_2 = math.pi - math.atan2(sin_beta,cos_beta)

cos_gama = (xw*xw + zw*zw + L[1]*L[1] - L[2]*L[2])/(2*L[1]*math.sqrt(xw*xw + zw*zw))
sin_gama = math.sqrt(abs(1 - (cos_gama*cos_gama)))

theta_link_1 = alpha - math.atan2(sin_gama,cos_gama)

theta_link_1p = theta_link_1 + 2* math.atan2(sin_gama,cos_gama)
theta_link_2p = - theta_link_2

theta_2 = theta_link_1p
theta_3 = theta_link_2p
theta_4 = theta_2 + theta_3

print("theta_2",theta_2)
print("theta_3",theta_3)
print("theta_4",theta_4)


rad_2 = theta_2 * (180/math.pi)
rad_3 = theta_3 * (180/math.pi)
rad_4 = theta_4 * (180/math.pi)

print(rad_2,rad_3,rad_4)







