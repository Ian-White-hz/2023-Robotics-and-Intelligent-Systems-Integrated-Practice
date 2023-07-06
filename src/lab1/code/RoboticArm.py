import numpy as np
from Tool import pos2trans
from ThetaCalculator import ThetaCalculator

class RoboticArm:
    def __init__(self):
        
        self.joint_num = 7
        
        self.a = np.array([0, 0, 0, 0.400, 0.400, 0, 0, 0])
        self.alpha = np.array([0, -np.pi/2, -np.pi/2, 0, 0, np.pi/2, np.pi/2, 0])
        self.d = np.array([0, 0.120, 0.100, 0.100, 0.100, 0.100, 0.100, 0.120])
        
        self.offset = np.array([0, np.pi/2, -np.pi/2, 0, np.pi, 0, np.pi/2, 0])
        self.sign = np.array([1, 1, 1, 1, 1, -1, -1, -1])
        
        self.tc = ThetaCalculator()
        
    def calc_transform_matrix(self, index):
        """
        Initialize the transformation matrix from frame i to frame i+1
        """
        ct = np.cos(self.theta[index+1] * self.sign[index+1] + self.offset[index+1])
        st = np.sin(self.theta[index+1] * self.sign[index+1] + self.offset[index+1])
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
        
        self.theta = [0] + theta
        
        self.T_i_ip1 = np.zeros((self.joint_num, 4, 4), dtype=np.float32)
        
        self.T_0_7 = np.eye(4)
        
        for i in range(self.joint_num):
            self.T_i_ip1[i] = self.calc_transform_matrix(i)
            self.T_0_7 = np.matmul(self.T_0_7, self.T_i_ip1[i])
            
        return self.T_0_7
    
    def inverse_kinetics(self, T_target, theta_current=None):
        """
        Calculate the inverse kinetics of the robotic arm
        """
        if theta_current is None:
            theta_current = np.zeros((1, 7))
        
        results = self.tc.calc_theta(T_target, theta_base=0.0)
        
        # print(results)
        
        def find_closest_row(array, target_row):
            distances = np.linalg.norm(array - target_row, axis=1)
            closest_index = np.argmin(distances)
            closest_row = array[closest_index]
            return closest_row
        
        theta = find_closest_row(results, theta_current)
        
        print(results, theta, theta_current)
        
        return theta
        
                
if __name__ == '__main__':
    
    np.set_printoptions(precision=2)
    np.set_printoptions(linewidth=90)
    
    robot = RoboticArm()
    
    theta = [np.pi/4, 0, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4]
    
    T = robot.forward_kinetic(theta)

    theta = robot.inverse_kinetics(T, theta)
    print(theta)