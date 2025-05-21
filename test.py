from robotcontrol import *
import numpy as np
import sim

def i5_robot():
    robot = Auboi5Robot()

    ret = robot.initialize()

    handle = robot.create_context()

    ip = "192.168.24.129"
    port = 8899
    result = robot.connect(ip,port)

    if result == 0:
        # robot.robot_startup()
        real_pos = robot.get_current_waypoint()
        print(real_pos)

        # joint_radian = (math.radians(0), math.radians(10), math.radians(90), math.radians(10), math.radians(90), math.radians(0))  # 或者 [0, 0, 0, 0, 0, 0]
        # move1 = robot.move_joint(joint_radian)
        # print("move over")
        # move2 = robot.move_line(joint_radian)
        #正解
        # fk=robot.forward_kin(joint_radian)
        fk=robot.forward_kin(real_pos['joint'])
        np.set_printoptions(suppress=True)
        rpy=robot.quaternion_to_rpy(fk['ori'])
        # rpy=np.array(rpy)*180/pi
        print(fk)
        print(rpy)
        #逆解
        # ik=robot.inverse_kin(joint_radian,fk['pos'],fk['ori'])
        ik=robot.inverse_kin(real_pos['joint'],fk['pos'],fk['ori'])
        joint_radian=ik['joint']
        joint_deg=np.array(ik['joint'])*180/pi
        print(joint_radian)
        print(joint_deg)
        return rpy

    else:
        print("failed connet")

def i5_robot_arm(clientID,handlname,rpy_orientation,relative_handle = -1):
    sim.simxGetPingTime(clientID=clientID)
    _,tip = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_blocking)
    _,tip_position = sim.simxGetObjectPosition(clientID,tip,relative_handle,sim.simx_opmode_blocking)
    _,tip_orientation = sim.simxGetObjectOrientation(clientID,tip,relative_handle,sim.simx_opmode_blocking)
    # sim.simxSetObjectPosition(clientID,tip,relative_handle,(tip_position[0]+move_per_distance,
    #                                                         tip_position[1],
    #                                                         tip_position[2]),
    #                                                         sim.simx_opmode_oneshot)
    sim.simxSetObjectOrientation(clientID,tip,relative_handle,(tip_orientation[0],
                                                               tip_orientation[1],
                                                               tip_orientation[2]),
                                                               sim.simx_opmode_oneshot)
    
    print("tip opsition = {}\ntip orientation = {}".format(tip_position,tip_orientation))

def main():

    sim.simxFinish(-1) # just in case, close all opened connections

    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
    if clientID !=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')

    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
    print("simulation start")

    ori = i5_robot()
    #base基座
    i5_robot_arm(clientID,'/base_link',ori)
    print('run over')
    # #一轴
    # i5_robot_arm(clientID,'/base_link/link1_link')
    # #二轴
    # i5_robot_arm(clientID,'/base_link/link2_link')
    # #三轴
    # i5_robot_arm(clientID,'/base_link/link3_link')
    # #四轴
    # i5_robot_arm(clientID,'/base_link/link4_link')
    # #五轴
    # i5_robot_arm(clientID,'/base_link/link5_link')
    # #六轴
    # i5_robot_arm(clientID,'/base_link/link6_link')
    # #末端执行器
    # i5_robot_arm(clientID,'/base_link/i5_finger')

    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)

    print("simulation stop")
    sim.simxFinish(clientID)


if __name__ == '__main__':
    main()