from RoboticArm import RoboticArm
from Tool import pos2trans
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import numpy as np 

class RobotController:
    def __init__(self) -> None:
        
        self.robot = RoboticArm()
        # self.theta = [0, 0, 0, 0, 0, 0, 0]
        self.theta = [np.pi/4, 0, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4]

        # self.T_W = pos2trans(x=0.650, y=0, z=0.235, alpha=0, beta=0, gamma=0, is_deg=False)
        
        self.T_0 = self.robot.forward_kinetic(self.theta)
        self.T_1 = pos2trans(x=0, y=0.3, z=0.15, alpha=np.pi, beta=0, gamma=-np.pi/2, is_deg=False)
        
        self.T = self.T_0
        
        """
        Remote API
        """
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        self.defaultIdleFps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)

        self.client.setStepping(True)
        self.sim.startSimulation()

        joint1 = self.sim.getObject('./L_Joint1')
        joint2 = self.sim.getObject('./L_Joint2')
        joint3 = self.sim.getObject('./L_Joint3')
        joint4 = self.sim.getObject('./Joint4')
        joint5 = self.sim.getObject('./R_Joint3')
        joint6 = self.sim.getObject('./R_Joint2')
        joint7 = self.sim.getObject('./R_Joint1')

        self.joints = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    
    def set_init(self):
        
        self.theta = [0, 0, 0, 0, 0, 0, 0]
        
        for i in range(7):
            self.sim.setJointPosition(self.joints[i], self.theta[i])
    
    def set_state(self):
        
        self.theta = self.robot.inverse_kinetics(self.T_1, self.theta)
        
        for i in range(7):
            self.sim.setJointPosition(self.joints[i], self.theta[i])

    def shut_down(self):
        
        # Stop simulation
        self.sim.stopSimulation()

        # Restore the original idle loop frequency:
        self.sim.setInt32Param(self.sim.intparam_idle_fps, self.defaultIdleFps)
        
if __name__ == "__main__":
    
    np.set_printoptions(precision=2, linewidth=90)
    
    rc = RobotController()
    
    # rc.set_state()
    rc.set_init()
    
    rc.shut_down()