import sim

def i5_robot_arm(clientID, handlname, joint_angles=None, rpy_orientation=None, position=None, relative_handle=-1):
    sim.simxGetPingTime(clientID=clientID)
    _, tip = sim.simxGetObjectHandle(clientID, handlname, sim.simx_opmode_blocking)
    print(tip)
    
    if joint_angles is not None:
        # 设置每个关节的角度
        for i in range(len(joint_angles)):
            _, joint_handle = sim.simxGetObjectHandle(clientID, f'{handlname}/i5_joint{i+1}', sim.simx_opmode_blocking)
            sim.simxSetJointPosition(clientID, joint_handle, joint_angles[i], sim.simx_opmode_oneshot)
    
    if position is not None and rpy_orientation is not None:
        # 设置末端执行器的位置和姿态
        _, tip_position = sim.simxGetObjectPosition(clientID, tip, relative_handle, sim.simx_opmode_blocking)
        _, tip_orientation = sim.simxGetObjectOrientation(clientID, tip, relative_handle, sim.simx_opmode_blocking)
        
        sim.simxSetObjectPosition(clientID, tip, relative_handle, position, sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(clientID, tip, relative_handle, rpy_orientation, sim.simx_opmode_oneshot)
        
        print(f"设置位置: {position}, 设置姿态: {rpy_orientation}")

def main():

    sim.simxFinish(-1) # just in case, close all opened connections

    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
    if clientID !=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')

    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
    print("simulation start")

    i5_robot_arm(clientID,'/1axis',[0,0,0,0,0,0],[0,0,0],[0,0,0])
    # i5_robot_arm(clientID,'/base_link/link1_link')
    # i5_robot_arm(clientID,'/base_link/link2_link')
    # i5_robot_arm(clientID,'/base_link/link3_link')
    # i5_robot_arm(clientID,'/base_link/link4_link')
    # i5_robot_arm(clientID,'/base_link/link5_link')
    # i5_robot_arm(clientID,'/base_link/link6_link')
    # i5_robot_arm(clientID,'/base_link/i5_finger')

    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
    print("simulation stop")
    sim.simxFinish(clientID)
    
if __name__ == '__main__':
    main()

