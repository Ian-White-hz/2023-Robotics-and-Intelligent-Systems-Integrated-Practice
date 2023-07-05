import numpy as np

def cal_theta1(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz):
    """
    Calculate theta1
    """
    theta1 = []
    T1 = px-0.12*ax
    T2 = 0.12*ay-py
    theta1.append(np.arcsin(-0.3/pow((pow(T1,2)+pow(T2,2)),0.5))-np.arctan2(T2,T1))
    theta1.append(np.pi - np.arcsin(-0.3/pow((pow(T1,2)+pow(T2,2)),0.5))-np.arctan2(T2,T1))
    return theta1


def cal_theta6(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1):
    """
    Calculate theta6
    """
    theta6 = []
    s6 = -ax*np.sin(theta1)+ay*np.cos(theta1)
    theta6.append(np.arcsin(s6))
    theta6.append(np.pi-np.arcsin(s6))
    return theta6

def cal_theta5(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1, theta3, theta4, theta6):
    """
    Calculate theta5
    """
    theta5 = []
    c6 = np.cos(theta6)
    c345 = -az / c6
    s345 = (1-c345 ** 2) ** 0.5
    A1 = 1/0.4*(-py*np.cos(theta1)-px*np.sin(theta1)-0.12*s345*c6+0.1*c345+0.1)
    B1 = 1/0.4*(pz-0.12*c345*c6+0.1*s345-0.12)
    theta5 = np.arccos(c345)-theta3-theta4
    return theta5

def cal_theta7(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1, theta6):
    """
    Calculate theta7
    """
    s6 = -ax*np.sin(theta1)+ay*np.cos(theta1)
    c6 = np.cos(theta6)
    c345 = -az / c6
    s345 = (1-c345 ** 2) ** 0.5
    theta7 = []
    theta7.append(np.arcsin(-oz/((s6 ** 2*c345 ** 2) ** 2+c345 ** 2) ** 0.5)+np.arctan2(s345,c345*s6))
    return theta7

def cal_theta3(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz,theta1, theta6):
    """
    Calculate theta3
    """
    theta3 = []
    s6 = -ax*np.sin(theta1)+ay*np.cos(theta1)
    c6 = np.cos(theta6)
    c345 = -az / c6
    s345 = (1-c345 ** 2) ** 0.5
    A1 = 1/0.4*(-py*np.cos(theta1)-px*np.sin(theta1)-0.12*s345*c6+0.1*c345+0.1)
    B1 = 1/0.4*(pz-0.12*c345*c6+0.1*s345-0.12)
    theta3.append(np.arccos((A1 ** 2+B1 ** 2) ** 0.5/2)-np.arctan2(A1,B1))
    theta3.append(-np.arccos((A1 ** 2+B1 ** 2) ** 0.5/2)-np.arctan2(A1,B1))
    return theta3

def cal_theta4(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1, theta3, theta6):
    """
    Calculate theta4
    """
    theta4 = []
    s6 = -ax*np.sin(theta1)+ay*np.cos(theta1)
    c6 = np.cos(theta6)
    c345 = -az / c6
    s345 = (1-c345 ** 2) ** 0.5
    A1 = 1/0.4*(-py*np.cos(theta1)-px*np.sin(theta1)-0.12*s345*c6+0.1*c345+0.1)
    B1 = 1/0.4*(pz-0.12*c345*c6+0.1*s345-0.12)
    c34=np.cos(theta3)-B1
    theta4.append(np.arccos(c34)-theta3)
    return theta4

def cal_theta2(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1, theta3, theta4):
    """
    Calculate theta2
    """
    theta2 = []
    theta2.append(np.arctan2((py-0.12*np.cos(theta1)-0.1*np.sin(theta1)),(px-0.12*np.sin(theta1)+0.1*np.cos(theta1)))-theta1-theta3-theta4)
    theta2.append(np.arctan2((py-0.12*np.cos(theta1)-0.1*np.sin(theta1)),(px-0.12*np.sin(theta1)+0.1*np.cos(theta1)))-theta1+theta3+theta4)
    return theta2