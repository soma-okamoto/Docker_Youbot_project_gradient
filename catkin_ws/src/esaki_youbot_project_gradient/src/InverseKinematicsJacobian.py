import math
import numpy as np


def fk(L,theta0,theta1,theta2):
    x = L[0]*math.cos(theta0) + L[1]*math.cos(theta0+theta1) + L[2]*math.cos(theta0+theta1+theta2)
    y = L[0]*math.sin(theta0) + L[1]*math.sin(theta0+theta1) + L[2]*math.sin(theta0+theta1+theta2)
    theta = theta0 + theta1 + theta2
    p = [x,y,theta]
    return p

def velocity(pd,p):
    k = 0.1
    vx = k * (pd[0] - p[0])
    vy = k * (pd[1] - p[1])
    omega = k * (pd[2] - p[2])
    
    v = np.array([[vx], [vy],[omega]])
    return v

def jacobian(L,theta0,theta1,theta2):

    r11 = -L[0]*math.sin(theta0)-L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r12 = -L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r13 = -L[2]*math.sin(theta0+theta1+theta2)

    r21 = L[0]*math.cos(theta0)+L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r22 = L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r23 = L[2]*math.cos(theta0+theta1+theta2)
    r31 = 1
    r32 = 1
    r33 = 1

    J = np.matrix(
        [
            [r11,r12,r13],
            [r21,r22,r23],
            [r31,r32,r33]

        ]
    )

    return J


def inversekinematics(L,pd,p0,ramda):
    p_fk = fk(L,p0[0],p0[1],p0[2])
    v = velocity(pd,p_fk)


    J = jacobian(L,p0[0],p0[1],p0[2])

    i = np.identity(3)
    J_T = np.transpose(J)

    SR = (np.dot(J_T,J) + ramda*i)

    SR_1 = np.linalg.pinv(SR)
    
    J_SR = np.dot(SR_1,J_T)

    angular_velocity = np.dot(J_SR,v)
    
    theta = 0.1 * angular_velocity
    
    return theta




pd = [0.419,0.318-0.147,0]

p0 = [0.1,0.2,0.3]
L = [0.155,0.135,0.218]

theta0 = 0.1
theta1 = 0.2
theta2 = 0.3

w = np.diag([0.1,0.1])


for i in range(1000):

    p_old = fk(L,p0[0],p0[1],p0[2])
    error = np.array([[pd[0] - p_old[0]], [pd[1] - p_old[1]]])
    error_T = np.transpose(error)
    
    squarederror = np.dot(error_T,w)
    e = np.dot(squarederror,error)


    ramda = e + 0.002

    theta_np = inversekinematics(L,pd,p0,ramda)

    theta0 += float(theta_np[0,0])
    theta1 += float(theta_np[1,0])
    theta2 += float(theta_np[2,0])
    
    p0[0] = theta0
    p0[1] = theta1
    p0[2] = theta2
    

theta = [p0[0],p0[1],p0[2]]




print("theta",theta)
print("pd",pd)
print("p",fk(L,theta[0],theta[1],theta[2]))