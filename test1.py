import sim

def i5_robot_arm(clientID,handlname,move_distance=0.1,relative_handle = -1):
    sim.simxGetPingTime(clientID=clientID)
    _,tip = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_blocking)
    _,tip_position = sim.simxGetObjectPosition(clientID,tip,relative_handle,sim.simx_opmode_blocking)
    _,tip_orientation = sim.simxGetObjectOrientation(clientID,tip,relative_handle,sim.simx_opmode_blocking)
    # tip_orientation[2] = move_distance
    # sim.simxSetObjectPosition(clientID,tip,relative_handle,(tip_position[0]+move_distance,
    #                                                         tip_position[1],
    #                                                         tip_position[2]),
    #                                                         sim.simx_opmode_oneshot)
    sim.simxSetObjectOrientation(clientID,tip,relative_handle,(tip_orientation[0],
                                                               tip_orientation[1],
                                                               tip_orientation[2]+move_distance),
                                                               sim.simx_opmode_oneshot)
    
    print("tip opsition = {}\ntip orientation = {}".format(tip_position,tip_orientation))

def main():

    sim.simxFinish(-1) # just in case, close all opened connections

    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID !=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')

    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
    print("simulation start")

    i5_robot_arm(clientID,'/base_link')
    i5_robot_arm(clientID,'/base_link/link1_link')
    i5_robot_arm(clientID,'/base_link/link2_link')
    i5_robot_arm(clientID,'/base_link/link3_link')
    i5_robot_arm(clientID,'/base_link/link4_link')
    i5_robot_arm(clientID,'/base_link/link5_link')
    i5_robot_arm(clientID,'/base_link/link6_link')
    i5_robot_arm(clientID,'/base_link/i5_finger')

    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
    print("simulation stop")
    sim.simxFinish(clientID)
    
if __name__ == '__main__':
    main()

