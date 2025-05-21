from robotcontrol import *
import numpy as np
import sim
import math
import time

class AuboRobot:
    def __init__(self, ip="192.168.24.129", port=8899):
        self.robot = Auboi5Robot()
        self.ip = ip
        self.port = port
        self.connected = False
        
    def connect(self):
        ret = self.robot.initialize()
        handle = self.robot.create_context()
        result = self.robot.connect(self.ip, self.port)
        self.connected = (result == 0)
        return self.connected
    
    def get_rpy_orientation(self):
        if not self.connected:
            print("Not connected to robot")
            return None
            
        real_pos = self.robot.get_current_waypoint()
        print(real_pos)
        
        fk = self.robot.forward_kin(real_pos['joint'])
        np.set_printoptions(suppress=True)
        rpy = self.robot.quaternion_to_rpy(fk['ori'])
        # rpy[0] -= np.pi/2
        
        print(fk)
        print(rpy)
        
        ik = self.robot.inverse_kin(real_pos['joint'], fk['pos'], fk['ori'])
        joint_radian = ik['joint']
        joint_deg = np.array(ik['joint'])*180/np.pi
        
        print(joint_radian)
        print(joint_deg)
        return rpy
    
    def get_position(self):
        if not self.connected:
            print("Not connected to robot")
            return None

        real_pos = self.robot.get_current_waypoint()
        # print(real_pos)
        return real_pos['pos']
    
    def get_joint_angles(self):
        if not self.connected:
            print("Not connected to robot")
            return None
            
        real_pos = self.robot.get_current_waypoint()
        return real_pos['joint']  # 返回关节角度数组
        
    def disconnect(self):
        if self.connected:
            self.robot.disconnect()
            self.connected = False

def i5_robot_arm(clientID, handlname, joint_angles=None, rpy_orientation=None, position=None, relative_handle=-1):
    sim.simxGetPingTime(clientID=clientID)
    _, tip = sim.simxGetObjectHandle(clientID, handlname, sim.simx_opmode_blocking)
    
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

    robot = AuboRobot()
    if robot.connect():
        # 启用同步模式
        sim.simxSynchronous(clientID, True)
        while True:
            # 获取真实机械臂数据
            joint_angles = robot.get_joint_angles()
            ori = robot.get_rpy_orientation()
            pos = robot.get_position()
            
            # 同步到仿真机械臂
            i5_robot_arm(clientID, '/base_link', joint_angles, ori, pos)
            
            # 触发仿真步进
            sim.simxSynchronousTrigger(clientID)
            
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    print("simulation stop")
    sim.simxFinish(clientID)


if __name__ == '__main__':
    main()