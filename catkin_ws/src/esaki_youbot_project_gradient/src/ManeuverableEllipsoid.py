#!/usr/bin/env python3

from ast import Lambda
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat


fig, ax = plt.subplots(figsize = (10,8))

def draw_ellipsoid(ax,a,b,x=0,y=0,deg=0):
    ellips = pat.Ellipse(xy = (x,y), width=a,height=b,alpha = 0.6,angle=deg,color="red",label="")
    ax.add_patch(ellips)


def fk(L,theta0,theta1,theta2):

    x1 = L[0]*math.sin(theta0)
    y1 = L[0]*math.cos(theta0)
    x2 = L[0]*math.sin(theta0) + L[1]*math.sin(theta0+theta1)
    y2 = L[0]*math.cos(theta0) + L[1]*math.cos(theta0+theta1)
    x3 = L[0]*math.sin(theta0) + L[1]*math.sin(theta0+theta1) + L[2]*math.sin(theta0+theta1+theta2)
    y3 = L[0]*math.cos(theta0) + L[1]*math.cos(theta0+theta1) + L[2]*math.cos(theta0+theta1+theta2)
    
    return x1,y1,x2,y2,x3,y3

def velocity(pd,p):
    k = 0.1
    vx = k * (pd[0] - p[0])
    vz = k * (pd[1] - p[1])
    omega = k * (pd[2] - p[2])
    
    v = np.array([[vx], [vz],[omega]])
    return v

def jacobian(L,theta0,theta1,theta2):

    r11 = L[0]*math.cos(theta0)+L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r12 = L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r13 = L[2]*math.cos(theta0+theta1+theta2)

    r21 = -L[0]*math.sin(theta0)-L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r22 = -L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r23 = -L[2]*math.sin(theta0+theta1+theta2)

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

def calculate_Ellipsoid_from_Jacobian(J):
    A = J * J.T

    values,vectors = np.linalg.eig(A)
    print(values,vectors)

    Lambda_1 = np.sqrt(values[0])
    Lambda_2 = np.sqrt(values[1])

    a = Lambda_2
    b = Lambda_1

    vector_1 = vectors[:,0]
    vector_2 = vectors[:,1]

    rad = - np.arctan2(vector_1,vector_2)
    rad = np.arctan2(vector_2[1],vector_1[1])

    return a,b,rad


def draw_ellipsoid_from_robotarm(L,theta0,theta1,theta2):
    J = jacobian(L,theta0,theta1,theta2)
    x1,y1,x2,y2,x3,y3 = fk(L,theta0,theta1,theta2)
    a,b,rad = calculate_Ellipsoid_from_Jacobian(J)

    draw_ellipsoid(ax,a*0.5,b*0.5,x3,y3,np.rad2deg(rad))

    ax.plot([0,x1],[0,y1],'k',linewidth=10.0,zorder=1)
    ax.plot([x1,x2],[y1,y2],'k',linewidth=10.0,zorder=1)
    ax.plot([x2,x3],[y2,y3],'k',linewidth=10.0,zorder=1)

    c0 = pat.Circle(xy =(0,0),radius=.1,ec='w',fill=True,zorder=2)
    c1 = pat.Circle(xy =(x1,y1),radius=.1,ec='w',fill=True,zorder=2)
    c2 = pat.Circle(xy =(x2,y2),radius=.1,ec='w',fill=True,zorder=2)
    c3 = pat.Circle(xy =(x3,y3),radius=.1,ec='w',fill=True,zorder=2)

    ax.add_patch(c0)
    ax.add_patch(c1)
    ax.add_patch(c2)
    ax.add_patch(c3)

    ax.set_xlabel('x')
    ax.set_xlabel('y')
    ax.set_aspect('equal')
    ax.grid()
    plt.xlim([-3,4])
    plt.ylim([-0.5,4])

if __name__ == "__main__":
    L = [2.0,1.5,1.0]
    theta0 = np.deg2rad(90-30)
    theta1 = np.deg2rad(-120)
    theta2 = np.deg2rad(75)

    draw_ellipsoid_from_robotarm(L,theta0,theta1,theta2)
    plt.show()


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

import tf

def quaternion_to_euler(quaternion):

    e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    euler = [e[0],e[1],e[2]]

    return euler



def DegToRad(th):
    rad = (np.pi/180)*th
    return rad

def RadToDeg(th):
    Deg = (180/np.pi)*th
    return Deg
