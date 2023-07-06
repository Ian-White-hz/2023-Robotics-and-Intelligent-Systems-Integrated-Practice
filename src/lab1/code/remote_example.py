import time
import numpy as np

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from RoboticArm import RoboticArm
from tool import pos2trans

if __name__ == "__main__":
    
    """
    Remote API init:
    """
    
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)

    client.setStepping(True)
    sim.startSimulation()

    joint1 = sim.getObject('./L_Joint1')
    joint2 = sim.getObject('./L_Joint2')
    joint3 = sim.getObject('./L_Joint3')
    joint4 = sim.getObject('./Joint4')
    joint5 = sim.getObject('./R_Joint3')
    joint6 = sim.getObject('./R_Joint2')
    joint7 = sim.getObject('./R_Joint1')

    joints = [ joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    
    """
    Control the robot:
    """
    np.set_printoptions(precision=2, linewidth=90)
    
    # 控制到指定位置
    thetas = [0, np.pi/4, np.pi/3, np.pi/4, np.pi/4, np.pi/4, np.pi/4]
    for i in range(7):
        sim.setJointPosition(joints[i], thetas[i])
        
    # 获取末端真实位姿
    end = sim.getObject('./R_Base/Dummy')
    x, y, z = sim.getObjectPosition(end, -1)
    a, b, g = sim.getObjectOrientation(end, -1)
    T_0 = pos2trans(x, y, z, a, b, g, is_deg=False)
    print("Acutual pose:")
    print(T_0)
    
    # 计算末端理论位姿
    arm = RoboticArm()
    T_1 = arm.forward_kinetic(thetas)
    print("Calculated pose:")
    print(T_1)
    
    result = arm.inverse_kinetics(T_1)
    
    print(result)
    
    
    
    # # Change the parent
    # # sim.setObjectParent(int objectHandle, int parentObjectHandle, bool keepInPlace)

    # # Stop simulation
    # sim.stopSimulation()

    # # Restore the original idle loop frequency:
    # sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)




    # while (t := sim.getSimulationTime()) < 3:
#     sim.setJointPosition(joint1, 2 * np.pi * t / 3)

#     # message = f'Simulation time: {t:.2f} s'
#     # print(message)
    
#     sim.addLog(sim.verbosity_scriptinfos, message)
#     client.step()  # triggers next simulation step
#     time.sleep(0.01)


