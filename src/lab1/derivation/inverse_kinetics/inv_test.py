import numpy as np
from theta_calculator2 import cal_theta1, cal_theta2, cal_theta3, cal_theta4, cal_theta5, cal_theta7, cal_theta6

class RoboticArm:
    def __init__(self):
        
        self.joint_num = 7
        
        self.a = np.array([0, 0, 0, 400, 400, 0, 0, 0])
        self.alpha = np.array([0, -np.pi/2, -np.pi/2, 0, 0, np.pi/2, np.pi/2, 0])
        self.d = np.array([0, 120, 100, 100, 100, 100, 100, 120])
        self.theta = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.offset = np.array([0, np.pi/2, -np.pi/2, 0, np.pi, 0, np.pi/2, 0])
        
        self.DH = np.vstack((self.theta[1:], self.d[1:], self.a[:-1], self.alpha[:-1], self.offset[1:])).T
    
        self.T_i_ip1 = np.zeros((self.joint_num, 4, 4), dtype=np.float32)
        
        self.T_0_7 = np.eye(4)
        
        for i in range(self.joint_num):
            self.T_i_ip1[i] = self.calc_transform_matrix(i)
            self.T_0_7 = np.matmul(self.T_0_7, self.T_i_ip1[i])
            
            # print(self.T_0_7, '\n')
            
    def calc_transform_matrix(self, index):
        """
        Initialize the transformation matrix from frame i to frame i+1
        """
        ct = np.cos(self.theta[index+1] + self.offset[index+1])
        st = np.sin(self.theta[index+1] + self.offset[index+1])
        ca = np.cos(self.alpha[index])
        sa = np.sin(self.alpha[index])
        a = self.a[index]
        d = self.d[index+1]
        
        T_i_ip1 = np.array([
            [ct, -st, 0, a], 
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa, ca, ca*d],
            [0, 0, 0, 1]
        ])
        
        
        return T_i_ip1
    
    def forward_kinetic(self, theta):
        T = np.eye(4)
        for i in range(len(theta)):
            T = np.dot(self.transform_matrix(theta[i], self.DH[i]), T)
        return T
    
    def cal_theta(self, nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz):
        # num = 0
        # solutions = []
        # theta_2 = 0
        # theta1s = cal_theta1(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz)
        # for theta_1 in theta1s:
        #     theta6s = cal_theta6(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1)
        #     for theta_6 in theta6s:
        #         theta3s = cal_theta3(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_6)
        #         theta_7 = cal_theta7(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_6)
        #         for theta_3 in theta3s:
        #             theta_4 = cal_theta4(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_3, theta_6)
        #             theta_5 = cal_theta5(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_3, theta_4, theta_6)
        #             print(f'Solution {num}, theta_1: {theta_1}, theta_2: {theta_2}, theta_3: {theta_3}, theta_4: {theta_4}, theta_5: {theta_5}, theta_6: {theta_6}, theta_7: {theta_7}')
        #             solutions.append([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
        #             num += 1
        # return solutions
        num = 0
        solutions = []
        theta_1 = 0
        theta2s = cal_theta2(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz ,theta_1)
        for theta_2 in theta2s:
            theta6s = cal_theta6(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1)
            for theta_6 in theta6s:
                theta3s = cal_theta3(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_6)
                theta_7 = cal_theta7(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_6)
                for theta_3 in theta3s:
                    theta_4 = cal_theta4(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_3, theta_6)
                    theta_5 = cal_theta5(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, theta_1, theta_3, theta_4, theta_6)
                    print(f'Solution {num}, theta_1: {theta_1}, theta_2: {theta_2}, theta_3: {theta_3}, theta_4: {theta_4}, theta_5: {theta_5}, theta_6: {theta_6}, theta_7: {theta_7}')
                    solutions.append([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
                    num += 1
        return solutions
    
    def inverse_kinetics(self,nx,ny,nz,ox,oy,oz,ax,ay,az,px,py,pz):
        """
        Calculate the inverse kinetics of the robotic arm
        """
        result=[]
        theta = self.cal_theta(nx,ny,nz,ox,oy,oz,ax,ay,az,px,py,pz)
        # theta = np.zeros([7,4])
        # theta[1] = 0
        # T1 = px-0.12*ax
        # T2 = 0.12*ay-py
        # theta[0,0] = np.arcsin(-0.3/pow((pow(T1,2)+pow(T2,2)),0.5))-np.arctan2(T2,T1)
        # theta[0,1] = 3.14 - np.arcsin(-0.3/pow((pow(T1,2)+pow(T2,2)),0.5))-np.arctan2(T2,T1)
        # for i in range(2):
        #     s6 = -ax*np.sin(theta[0,i])+ay*np.cos(theta[0,i])
        #     c6 = (1-s6 ** 2) ** 0.5
        #     print(s6)
        #     print(np.arcsin(s6))
        #     theta[5,2*i] = np.arcsin(s6)
        #     theta[5,2*i+1] = -np.arcsin(s6)
        #     for i in range(2):
        #         theta[4,2*i] = np.arccos(-oz/((s6 ** 2*c6 ** 2) ** 2+c6 ** 2) ** 0.5)+np.arctan2(s6,c6*oz)
        #         theta[4,2*i+1] = -np.arccos(-oz/((s6 ** 2*c6 ** 2) ** 2+c6 ** 2) ** 0.5)+np.arctan2(s6,c6*oz)
        #         for i in range(2):
        #             theta[3,2*i] = np.arccos((np.cos(theta[4,2*i])*np.cos(theta[5,2*i])*ax+np.sin(theta[4,2*i])*np.cos(theta[5,2*i])*ay-np.sin(theta[5,2*i])*az)/c6)
        #             theta[3,2*i+1] = -np.arccos((np.cos(theta[4,2*i+1])*np.cos(theta[5,2*i+1])*ax+np.sin(theta[4,2*i+1])*np.cos(theta[5,2*i+1])*ay-np.sin(theta[5,2*i+1])*az)/c6)
        #             for i in range(2):
        #                 theta[2,2*i] = np.arccos((np.cos(theta[4,2*i])*np.cos(theta[5,2*i])*ax+np.sin(theta[4,2*i])*np.cos(theta[5,2*i])*ay-np.sin(theta[5,2*i])*az)/c6)
        #                 theta[2,2*i+1] = -np.arccos((np.cos(theta[4,2*i+1])*np.cos(theta[5,2*i+1])*ax+np.sin(theta[4,2*i+1])*np.cos(theta[5,2*i+1])*ay-np.sin(theta[5,2*i+1])*az)/c6)
        #                 for i in range(2):
        #                     theta[1,2*i] = np.arccos((np.cos(theta[4,2*i])*np.cos(theta[5,2*i])*ax+np.sin(theta[4,2*i])*np.cos(theta[5,2*i])*ay-np.sin(theta[5,2
        # c345 = -az/c6
        # s345 = (1-c345 ** 2) ** 0.5
        # A1=1/0.4*(-py*np.cos(theta[0])-px*np.sin(theta[0])-0.12*s345*c6+0.1*c345+0.1)
        # B1=1/0.4*(pz-0.12*c345*c6+0.1*s345-0.12)
        # theta[2,0] = np.arccos((A1 ** 2+B1 ** 2) ** 0.5/2)-np.arctan2(A1,B1)
        # theta[2,1] = -np.arccos((A1 ** 2+B1 ** 2) ** 0.5/2)-np.arctan2(A1,B1)
        # c34=np.cos(theta[2])-B1
        # theta[3,0] = np.arccos(c34)-theta[2]
        # theta[3,1] = -np.arccos(c34)-theta[2]
        # theta[4] = np.arccos(c345)-theta[2]-theta[3]
        # theta[6] = np.arcsin(-oz/((s6 ** 2*c345 ** 2) ** 2+c345 ** 2) ** 0.5)+np.arctan2(s345,c345*s6)


        return theta

            
if __name__ == '__main__':
    
    np.set_printoptions(precision=2)
    
    robot = RoboticArm()
    #theta=robot.inverse_kinetics(-0.9267767,-0.28033009,-0.25,0.28033009,-0.07322332,-0.95710678,0.24999999,-0.95710678 ,0.14644663 ,-0.32961941,0.03765981,0.07544156)
    theta_2=cal_theta2(-0.9267767,-0.28033009,-0.25,0.28033009,-0.07322332,-0.95710678,0.24999999,-0.95710678 ,0.14644663 ,-0.32961941,0.03765981,0.07544156, np.pi/4)
    print(theta_2)

