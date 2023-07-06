import numpy as np
from Tool import norm_rad

class ThetaCalculator:
    
    def __init__(self):
           
        self.results = []
        
    def calc_theta(self, T: np.ndarray, theta_base = 0):
        
        self.nx, self.sx, self.ax, self.px = T[0, :]
        self.ny, self.sy, self.ay, self.py = T[1, :]
        self.nz, self.sz, self.az, self.pz = T[2, :]
        
        self.results = []
        
        theta_2s = self.calc_theta_2(theta_base)
        for theta_2 in theta_2s:
            theta_1s = self.calc_theta_1(theta_2)
            for theta_1 in theta_1s:
                theta_6s = self.calc_theta_6(theta_1, theta_2)
                for theta_6 in theta_6s:
                    theta_7s = self.calc_theta_7(theta_1, theta_2, theta_6)
                    for theta_7 in theta_7s:
                        theta_4s = self.calc_theta_4(theta_1, theta_2, theta_7)
                        for theta_4 in theta_4s:
                            theta_3s = self.calc_theta_3(theta_1, theta_2, theta_4, theta_6, theta_7)
                            for theta_3 in theta_3s:
                                theta_5s = self.calc_theta_5(theta_1, theta_3, theta_4, theta_6, theta_7)
                                for theta_5 in theta_5s:
                                    self.results.append(
                                        [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7])
        
        return np.array(self.results)
               
    def calc_theta_2(self, theta_0):
        """
        theta 1 customized as 0
        """
        return [norm_rad(theta_0)]
        
    def calc_theta_1(self, theta_2):
        """
        theta 2 depends on theta 1
        """
        nx, sx, ax, px = self.nx, self.sx, self.ax, self.px
        ny, sy, ay, py = self.ny, self.sy, self.ay, self.py
        nz, sz, az, pz = self.nz, self.sz, self.az, self.pz
        
        # a = -0.12*az + pz - 0.12
        # b = -0.12*ax*np.sin(theta_1) + 0.12*ay*np.cos(theta_1) + px*np.sin(theta_1) - py*np.cos(theta_1)
        
        a = -0.12*ax*np.cos(theta_2) + px*np.cos(theta_2)
        b = 0.12*ay*np.cos(theta_2) - py*np.cos(theta_2)
        c = -0.12*az*np.sin(theta_2) + pz*np.sin(theta_2) - 0.12*np.sin(theta_2) + 0.3
        
        Phi = np.arctan2(b, a)
        phi = np.arcsin(-c/np.sqrt(a**2 + b**2))
            
        return [norm_rad(phi - Phi), norm_rad(np.pi - phi - Phi)]
            
    def calc_theta_6(self, theta_1, theta_2):
        """
        theta 6 depends on theta 1, 2
        """
        nx, sx, ax, px = self.nx, self.sx, self.ax, self.px
        ny, sy, ay, py = self.ny, self.sy, self.ay, self.py
        nz, sz, az, pz = self.nz, self.sz, self.az, self.pz
        
        s6 = az*np.sin(theta_2) + (ax*np.sin(theta_1) - ay*np.cos(theta_1))*np.cos(theta_2)
        
        return [norm_rad(np.arcsin(s6)), norm_rad(-np.arcsin(s6))]
    
    def calc_theta_7(self, theta_1, theta_2, theta_6):
        """
        theta 7 depends on theta 1, 2, 6
        """
        nx, sx, ax, px = self.nx, self.sx, self.ax, self.px
        ny, sy, ay, py = self.ny, self.sy, self.ay, self.py
        nz, sz, az, pz = self.nz, self.sz, self.az, self.pz
        
        c6c7 = -nz*np.sin(theta_2) - (nx*np.sin(theta_1) - ny*np.cos(theta_1))*np.cos(theta_2)
        c6s7 = -sz*np.sin(theta_2) - (sx*np.sin(theta_1) - sy*np.cos(theta_1))*np.cos(theta_2)
        
        return [norm_rad(np.arctan2(c6s7, c6c7))]
        
        
    def calc_theta_4(self, theta_1, theta_2, theta_7):
        """
        theta 4 depends on theta 1, 2, 6, 7
        """
        nx, sx, ax, px = self.nx, self.sx, self.ax, self.px
        ny, sy, ay, py = self.ny, self.sy, self.ay, self.py
        nz, sz, az, pz = self.nz, self.sz, self.az, self.pz
                        
        A = 0.12*ax*np.sin(theta_1)*np.sin(theta_2) - \
            0.12*ay*np.sin(theta_2)*np.cos(theta_1) - \
            0.12*az*np.cos(theta_2) - \
            px*np.sin(theta_1)*np.sin(theta_2) + \
            py*np.sin(theta_2)*np.cos(theta_1) + \
            pz*np.cos(theta_2) + \
            0.1*(-nx*np.sin(theta_1)*np.sin(theta_2) + ny*np.sin(theta_2)*np.cos(theta_1) + nz*np.cos(theta_2))*np.sin(theta_7) - \
            0.1*(-sx*np.sin(theta_1)*np.sin(theta_2) + sy*np.sin(theta_2)*np.cos(theta_1) + sz*np.cos(theta_2))*np.cos(theta_7) - \
            0.12*np.cos(theta_2)
        B = 0.12*ax*np.cos(theta_1) - px*np.cos(theta_1) + \
            0.12*ay*np.sin(theta_1) - py*np.sin(theta_1) - \
            0.1*(nx*np.cos(theta_1) + ny*np.sin(theta_1))*np.sin(theta_7) + \
            0.1*(sx*np.cos(theta_1) + sy*np.sin(theta_1))*np.cos(theta_7) - 0.1   
        
        c4 = 1 - (A**2 + B**2)/0.32

        return [norm_rad(np.arccos(c4)), norm_rad(-np.arccos(c4))]

    def calc_theta_3(self, theta_1, theta_2, theta_4, theta_6, theta_7):
        """
        theta 3 depends on theta 1, 2, 4, 6, 7
        """
        nx, sx, ax, px = self.nx, self.sx, self.ax, self.px
        ny, sy, ay, py = self.ny, self.sy, self.ay, self.py
        nz, sz, az, pz = self.nz, self.sz, self.az, self.pz
        
        A = 0.12*ax*np.sin(theta_1)*np.sin(theta_2) - \
            0.12*ay*np.sin(theta_2)*np.cos(theta_1) - \
            0.12*az*np.cos(theta_2) - \
            px*np.sin(theta_1)*np.sin(theta_2) + \
            py*np.sin(theta_2)*np.cos(theta_1) + \
            pz*np.cos(theta_2) + \
            0.1*(-nx*np.sin(theta_1)*np.sin(theta_2) + ny*np.sin(theta_2)*np.cos(theta_1) + nz*np.cos(theta_2))*np.sin(theta_7) - \
            0.1*(-sx*np.sin(theta_1)*np.sin(theta_2) + sy*np.sin(theta_2)*np.cos(theta_1) + sz*np.cos(theta_2))*np.cos(theta_7) - \
            0.12*np.cos(theta_2)
        B = 0.12*ax*np.cos(theta_1) - px*np.cos(theta_1) + \
            0.12*ay*np.sin(theta_1) - py*np.sin(theta_1) - \
            0.1*(nx*np.cos(theta_1) + ny*np.sin(theta_1))*np.sin(theta_7) + \
            0.1*(sx*np.cos(theta_1) + sy*np.sin(theta_1))*np.cos(theta_7) - 0.1   
        
        c3 = (2.5*A*(1-np.cos(theta_4)) + 2.5*B*np.sin(theta_4)) / (2 - 2*np.cos(theta_4))
        s3 = (2.5*A*np.sin(theta_4) - 2.5*B*(1-np.cos(theta_4))) / (2 - 2*np.cos(theta_4))
        
        return [norm_rad(np.arctan2(s3, c3))]
    
    def calc_theta_5(self, theta_1, theta_3, theta_4, theta_6, theta_7):
        """
        theta 5 depends on theta 1, 3, 4, 6, 7
        """
        nx, sx, ax, px = self.nx, self.sx, self.ax, self.px
        ny, sy, ay, py = self.ny, self.sy, self.ay, self.py
        nz, sz, az, pz = self.nz, self.sz, self.az, self.pz
        
        s345 = -(ax*np.cos(theta_1) + ay*np.sin(theta_1))*np.cos(theta_6) - \
                (nx*np.cos(theta_1) + ny*np.sin(theta_1))*np.sin(theta_6)*np.cos(theta_7) - \
                (sx*np.cos(theta_1) + sy*np.sin(theta_1))*np.sin(theta_6)*np.sin(theta_7) 
        c345 = -(nx*np.cos(theta_1) + ny*np.sin(theta_1))*np.sin(theta_7) + \
                (sx*np.cos(theta_1) + sy*np.sin(theta_1))*np.cos(theta_7)
                
        return [norm_rad(theta_4 + theta_3 - np.arctan2(s345, c345))]
                         
# if __name__ == "__main__":
    
#     robot = RoboticArm()
    
#     robot.forward_kinetic(
#         np.array([None, 0, np.pi/3, np.pi/3, np.pi/3, np.pi/3, np.pi/3, np.pi/3])
#     )
    
#     T = robot.T_0_7
    
#     tc = ThetaCalculator()
    
#     results = tc.calc_theta(T)
    
#     np.set_printoptions(linewidth=90)
    
#     print(results)    