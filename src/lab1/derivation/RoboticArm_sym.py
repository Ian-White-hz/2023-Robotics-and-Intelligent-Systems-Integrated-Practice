import sympy as sp

class RoboticArm:
    def __init__(self):
        
        self.joint_num = 7

        self.a = sp.Matrix([0, 0, 0, 0.400, 0.400, 0, 0, 0])
        self.alpha = sp.Matrix([0, -sp.pi/2, -sp.pi/2, 0, 0, sp.pi/2, sp.pi/2, 0])
        self.d = sp.Matrix([0, 0.120, 0.100, 0.100, 0.100, 0.100, 0.100, 0.120])
        # self.theta = sp.Matrix([None, 0, 0, 0, 0, 0, 0, 0])
        
        self.theta = sp.Matrix([
            0, 
            sp.Symbol('theta_1'), 
            sp.Symbol('theta_2'), 
            sp.Symbol('theta_3'), 
            sp.Symbol('theta_4'), 
            sp.Symbol('theta_5'), 
            sp.Symbol('theta_6'), 
            sp.Symbol('theta_7'), 
            ])
        
        self.offset = sp.Matrix([0, sp.pi/2, -sp.pi/2, 0, sp.pi, 0, sp.pi/2, 0])
        self.sign = sp.Matrix([1, 1, 1, 1, 1, 1, 1, 1])
        
        
        self.DH = sp.Matrix([self.theta[1:], self.d[1:], self.a[:-1], self.alpha[:-1], self.offset[1:]]).T
    
        self.T_i_ip1 = []
        
        self.T_0_7 = sp.eye(4)
        
        for i in range(self.joint_num):
            self.T_i_ip1.append(self.calc_transform_matrix(i))
            self.T_0_7 = self.T_0_7 * self.T_i_ip1[i]
            
    def calc_transform_matrix(self, index):
        """
        Initialize the transformation matrix from frame i to frame i+1
        """
        ct = sp.cos(self.theta[index+1] * self.sign[index+1] + self.offset[index+1])
        st = sp.sin(self.theta[index+1] * self.sign[index+1] + self.offset[index+1])
        ca = sp.cos(self.alpha[index])
        sa = sp.sin(self.alpha[index])
        a = self.a[index]
        d = self.d[index+1]
        
        T_i_ip1 = sp.Matrix([
            [ct, -st, 0, a], 
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa, ca, ca*d],
            [0, 0, 0, 1]
        ])
        
        return T_i_ip1
    
    def forward_kinetic(self, theta):
        T = sp.eye(4)
        for i in range(len(theta)):
            T = sp.dot(self.transform_matrix(theta[i], self.DH[i]), T)
        return T
            
if __name__ == '__main__':
    
    robot = RoboticArm()
    
