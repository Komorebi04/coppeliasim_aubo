from robotcontrol import *
import numpy as np
import sim
import math
import time
import cv2
import matplotlib.pyplot as plt


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

def set_joint_angles(clientID, handlname, joint_angles):
    """设置机械臂关节角度"""
    sim.simxGetPingTime(clientID=clientID)
    for i in range(len(joint_angles)):
        _, joint_handle = sim.simxGetObjectHandle(clientID, f'{handlname}/Revolute_joint{i+1}', sim.simx_opmode_blocking)
        print("关节节点句柄:",joint_handle)
        sim.simxSetJointPosition(clientID, joint_handle, joint_angles[i], sim.simx_opmode_oneshot)

def set_position(clientID, handlname, position, relative_handle=-1):
    """设置机械臂末端位置"""
    sim.simxGetPingTime(clientID=clientID)
    _, tip = sim.simxGetObjectHandle(clientID, handlname, sim.simx_opmode_blocking)
    sim.simxSetObjectPosition(clientID, tip, relative_handle, position, sim.simx_opmode_oneshot)
    print(f"设置位置: {position}")

def set_orientation(clientID, handlname, rpy_orientation, relative_handle=-1):
    """设置机械臂末端姿态"""
    sim.simxGetPingTime(clientID=clientID)
    _, tip = sim.simxGetObjectHandle(clientID, handlname, sim.simx_opmode_blocking)
    sim.simxSetObjectOrientation(clientID, tip, relative_handle, rpy_orientation, sim.simx_opmode_oneshot)
    print(f"设置姿态: {rpy_orientation}")

# def encode_visionsensorImage(raw_image,resolution):
#     img = np.array(raw_image,dtype=np.uint8)
#     img.resize([resolution[1],resolution[0],3])
#     img = cv2.flip(img,0)
#     return img

# def get_vs_img(clientID,handlname,mode=0):
#     if mode == 0:
#         _,vs_handle = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_blocking)
#         ret,resolution,raw_image=sim.simxGetVisionSensorImage(clientID,vs_handle,0,sim.simx_opmode_streaming)
#         time.sleep(0.5)
#         print("ret0",ret)
#         print("res0",resolution)

#     elif mode == 1:
#         _,vs_handle = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_blocking)
#         ret,resolution,raw_image = sim.simxGetVisionSensorImage(clientID,vs_handle,0,sim.simx_opmode_buffer)
#         print(ret)
#         print(resolution)
#         img = encode_visionsensorImage(raw_image,resolution)
#         print(img.shape)
#         return img
#     else:
#         print('Error mode')


        

def main():

    sim.simxFinish(-1) # just in case, close all opened connections

    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
    if clientID !=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')

    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
    print("simulation start")

    robot = AuboRobot()
    if robot.connect():
        # 启用同步模式
        sim.simxSynchronous(clientID, True)
        # get_vs_img(clientID,'/camera',mode=0)
        # plt.ion()
        # fig = plt.figure("vs_img")
        sim.simxSetInt32Signal(clientID,'RG2_open', 0, sim.simx_opmode_blocking)
        while True:
            # 获取真实机械臂数据
            joint_angles = robot.get_joint_angles()
            ori = robot.get_rpy_orientation()
            pos = robot.get_position()
            
            
            # 同步到仿真机械臂
            # 在 main 函数中替换原来的 i5_robot_arm 调用
            set_joint_angles(clientID, '/1axis', joint_angles)
            # time.sleep(2)  # 等待 2 秒

            # sim.simxSetInt32Signal(clientID,'RG2_open', 0,sim.simx_opmode_blocking)
            # print("夹爪开始抓取...")
            # time.sleep(2)  # 等待夹爪动作完成


            # img = get_vs_img(clientID,'/camera',mode=1)
            # savefile = './Images/'+str(time.time())+'.jpg'
            # img_translation = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # cv2.imwrite(savefile,img_translation)
            # ax = fig.add_subplot(111)
            # plt.imshow(img)
            # plt.pause(0.1)
            # fig.clf()

            # set_position(clientID, '/base_link', pos)
            # set_orientation(clientID, '/base_link', ori)
            
            # 触发仿真步进
            sim.simxSynchronousTrigger(clientID)
            sim.simxAddStatusbarMessage(clientID,'over',sim.simx_opmode_oneshot)
            sim.simxGetPingTime(clientID)

    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    print("simulation stop")
    plt.ioff()
    sim.simxFinish(clientID)


if __name__ == '__main__':
    main()