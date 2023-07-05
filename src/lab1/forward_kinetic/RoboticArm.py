import numpy as np

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
            
if __name__ == '__main__':
    
    np.set_printoptions(precision=2)
    
    robot = RoboticArm()

    print(robot.T_0_7)