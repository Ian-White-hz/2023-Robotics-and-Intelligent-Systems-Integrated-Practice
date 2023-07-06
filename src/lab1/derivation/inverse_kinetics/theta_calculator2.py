import sys
sys.path.append("..")
from derivation.RoboticArm_sym import RoboticArm
import sympy as sp
import numpy as np

arm = RoboticArm()
T_0_7 = sp.Matrix([
    [sp.Symbol('n_x'), sp.Symbol('s_x'), sp.Symbol('a_x'), sp.Symbol('p_x')],
    [sp.Symbol('n_y'), sp.Symbol('s_y'), sp.Symbol('a_y'), sp.Symbol('p_y')],
    [sp.Symbol('n_z'), sp.Symbol('s_z'), sp.Symbol('a_z'), sp.Symbol('p_z')],
    [0, 0, 0, 1]
])
T_0_1 = arm.T_i_ip1[0]
T_1_2 = arm.T_i_ip1[1]
T_2_3 = arm.T_i_ip1[2]
T_3_4 = arm.T_i_ip1[3]
T_4_5 = arm.T_i_ip1[4]
T_5_6 = arm.T_i_ip1[5]
T_6_7 = arm.T_i_ip1[6]
LHS = sp.simplify(T_0_1.inv() * T_0_7)
RHS = sp.simplify(T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6 * T_6_7)
from IPython.core.interactiveshell import InteractiveShell
InteractiveShell.ast_node_interactivity = "all"

equations = []

for i in range(3):
    for j in range(4):
        eq = sp.Eq(LHS[i, j], RHS[i, j])
        equations.append(eq)
lhs1 = sp.simplify(LHS[0, 0] * sp.cos(sp.Symbol('theta_2')) - LHS[2, 0] * sp.sin(sp.Symbol('theta_2')))
rhs1 = sp.simplify(RHS[0, 0] * sp.cos(sp.Symbol('theta_2')) - RHS[2, 0] * sp.sin(sp.Symbol('theta_2')))
sp.Eq(lhs1, rhs1)

lhs2 = sp.simplify(LHS[0, 2] * sp.cos(sp.Symbol('theta_2')) - LHS[2, 2] * sp.sin(sp.Symbol('theta_2')))
rhs2 = sp.simplify(RHS[0, 2] * sp.cos(sp.Symbol('theta_2')) - RHS[2, 2] * sp.sin(sp.Symbol('theta_2')))
sp.Eq(lhs2, rhs2)

lhs3 = sp.simplify(LHS[2, 1] * sp.sin(sp.Symbol('theta_2')) - LHS[0, 1] * sp.cos(sp.Symbol('theta_2')))
rhs3 = sp.simplify(RHS[2, 1] * sp.sin(sp.Symbol('theta_2')) - RHS[0, 1] * sp.cos(sp.Symbol('theta_2')))
sp.Eq(lhs3, rhs3)

lhs4 = sp.simplify(LHS[2, 3] * sp.sin(sp.Symbol('theta_2')) - LHS[0, 3] * sp.cos(sp.Symbol('theta_2')))
rhs4 = sp.simplify(RHS[2, 3] * sp.sin(sp.Symbol('theta_2')) - RHS[0, 3] * sp.cos(sp.Symbol('theta_2')))
sp.Eq(lhs4, rhs4)
a = (lhs4 + 0.12 * lhs2 + 0.3).coeff(sp.sin(sp.Symbol('theta_2')))
b = (lhs4 + 0.12 * lhs2 + 0.3).coeff(sp.cos(sp.Symbol('theta_2')))
Phi = sp.atan2(b, a)
phi = sp.asin(-0.3/sp.sqrt(a**2 + b**2))
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


def cal_theta6(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1, theta2):
    """
    Calculate theta6
    """
    theta6 = []
    s6 = -ax*np.sin(theta1)+ay*np.cos(theta1)
    theta6.append(sp.asin(lhs2).subs(sp.Symbol('theta_2'), theta2).subs(sp.Symbol('theta_1'), theta1))
    theta6.append(-sp.asin(lhs2).subs(sp.Symbol('theta_2'), theta2).subs(sp.Symbol('theta_1'), theta1))
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

def cal_theta7(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1, theta2):
    """
    Calculate theta7
    """
    theta7 = []
    theta7.append(sp.atan2(lhs3, lhs1).subs(sp.Symbol('theta_2'), theta2).subs(sp.Symbol('theta_1'), theta1))
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
    s345 = (1 - c345 ** 2) ** 0.5
    A1 = 1/0.4*(-py*np.cos(theta1)-px*np.sin(theta1)-0.12*s345*c6+0.1*c345+0.1)
    B1 = 1/0.4*(pz-0.12*c345*c6+0.1*s345-0.12)
    c34=np.cos(theta3)-B1
    theta4.append(np.arccos(c34)-theta3)
    return theta4

def cal_theta2(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta1):
    """
    Calculate theta2
    """
    theta2 = []
    val1 = phi-Phi
    val2 = np.pi - phi - Phi
    val1 = val1.subs(sp.Symbol('theta_1'),theta1).subs(sp.Symbol('a_z'), az).subs(sp.Symbol('p_z'), pz).subs(sp.Symbol('a_x'),ax).subs(sp.Symbol('a_y'),ay).subs(sp.Symbol('p_x'),px).subs(sp.Symbol('p_y'),py)
    val2 = val2.subs(sp.Symbol('theta_1'),theta1).subs(sp.Symbol('a_z'), az).subs(sp.Symbol('p_z'), pz).subs(sp.Symbol('a_x'),ax).subs(sp.Symbol('a_y'),ay).subs(sp.Symbol('p_x'),px).subs(sp.Symbol('p_y'),py)
    theta2.append(val1)
    theta2.append(val2)
    return theta2