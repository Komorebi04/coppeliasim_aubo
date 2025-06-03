from robotcontrol import *
import numpy as np
import sim
import math
import time
import cv2
import matplotlib.pyplot as plt
import threading
from collections import deque
import socket
import queue
from queue import Empty,Queue  # 导入Empty异常类

# 在AuboRobot类中添加
class AuboRobot:
    def __init__(self, ip="192.168.24.130", port=8899):
        self.robot = Auboi5Robot()
        self.ip = ip
        self.port = port
        self.connected = False
        self.command_queue = deque(maxlen=1)  # 只保留最新指令

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
        self.command_queue.append(real_pos['joint'])
        return self.command_queue[0] if self.command_queue else None

    def disconnect(self):
        if self.connected:
            self.robot.disconnect()
            self.connected = False


def set_joint_angles(clientID, handlname, joint_angles):
    """设置机械臂关节角度"""
    sim.simxGetPingTime(clientID=clientID)
    for i in range(len(joint_angles)):
        _, joint_handle = sim.simxGetObjectHandle(clientID, f'{handlname}/Revolute_joint{i+1}', sim.simx_opmode_oneshot)
        # print("关节节点句柄:",joint_handle)
        sim.simxSetJointPosition(clientID, joint_handle, joint_angles[i], sim.simx_opmode_oneshot)


def encode_visionsensorImage(raw_image, resolution):
    img = np.array(raw_image, dtype=np.uint8)
    if len(resolution) < 2:
        print("Invalid resolution data")
        return None
    
    try:
        img.resize([resolution[1], resolution[0], 3])
        img = cv2.flip(img, 0)
        return img
    except ValueError:
        print("Error resizing image with resolution:", resolution)
        return None

def get_vs_img(clientID,handlname,mode=0):
    if mode == 0:
        _,vs_handle = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_oneshot)
        ret,resolution,raw_image=sim.simxGetVisionSensorImage(clientID,vs_handle,0,sim.simx_opmode_streaming)
        time.sleep(0.5)
        print("ret0",ret)
        print("res0",resolution)

    elif mode == 1:
        _,vs_handle = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_oneshot)
        ret,resolution,raw_image = sim.simxGetVisionSensorImage(clientID,vs_handle,0,sim.simx_opmode_buffer)
        print(ret)
        print(resolution)
        img = encode_visionsensorImage(raw_image,resolution)
        if img is None:  # 添加检查
            print("Failed to encode image")
            return None
        print(img.shape)
        return img
    else:
        print('Error mode')


def tcp_server(host, port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 添加端口复用
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"服务端已开始监听，正在等待客户端连接...")
    
    while True:  # 外层循环保持服务端持续运行
        try:
            conn, address = server_socket.accept()
            print(f"接收到了客户端的连接，客户端的信息：{address}")
            
            while True:  # 内层循环处理单个连接
                try:
                    data = conn.recv(1024).decode("UTF-8")
                    if not data:
                        print("客户端断开连接")
                        break
                    print(f"客户端发来的消息是：{data}")
                    yield data  # 使用生成器持续返回接收到的信号
                except ConnectionResetError:
                    print("客户端异常断开")
                    break
                    
    
        except Exception as e:
            print(f"服务端异常: {str(e)}")
            time.sleep(1)  # 防止异常时CPU占用过高

# 修改信号处理线程
def signal_handler(signal_generator, queue):
    for signal in signal_generator:
        print(f"信号处理器接收到信号: {signal}")  # 添加调试输出
        queue.put(signal)
        time.sleep(0.1)  # 添加短暂延迟确保信号被处理

# 修改控制线程
def control_robot_motion(robot, clientID, queue):
    while True:
        # 获取真实机械臂数据
        joint_angles = robot.get_joint_angles()
        if joint_angles is not None:
            set_joint_angles(clientID, '/1axis', joint_angles)
        
        # 检查信号队列
        try:
            signal = queue.get_nowait()
            print(f"控制线程接收到信号: {signal}")  # 修改调试输出
            if signal.strip().lower() == 'catch_ok':
                print("开始处理catch_ok信号")
                ret1 = sim.simxSetInt32Signal(clientID, 'RG2_open', 0, sim.simx_opmode_blocking)
                ret2 = sim.simxAddStatusbarMessage(clientID,"catch ok",sim.simx_opmode_blocking)
                print(f"设置信号返回值: {ret1}, 状态栏消息返回值: {ret2}")
                print("抓取完成")
            elif signal.strip().lower() == 'release_ok':
                print("开始处理release_ok信号")  # 添加调试输出
                ret1 = sim.simxSetInt32Signal(clientID, 'RG2_open', 1, sim.simx_opmode_blocking)
                ret2 = sim.simxAddStatusbarMessage(clientID,"release ok",sim.simx_opmode_blocking)
                print(f"设置信号返回值: {ret1}, 状态栏消息返回值: {ret2}")  # 添加调试输出
                print("释放完成")
        except Empty:
            print("队列为空")  # 添加调试输出
            pass
        
        time.sleep(0.1)

# 修改相机处理线程
def process_camera(clientID, queue):
    # 使用Agg后端避免GUI线程问题
    # import matplotlib
    # matplotlib.use('Agg')
    # plt.ioff()
    
    # fig = plt.figure("vs_img")
    try:
        while True:
            try:
                signal = queue.get_nowait()
                if signal.strip().lower() == 'capture':
                    get_vs_img(clientID, '/visionSensor', mode=0)
                    print("初始化完成")
                    time.sleep(0.5)
                    img = get_vs_img(clientID, '/visionSensor', mode=1)
                    print("捕获到图像")
                    if img is not None:
                        savefile = './Images/' + time.strftime("%Y%m%d_%H%M%S") + '.jpg'
                        img_translation = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        cv2.imwrite(savefile, img_translation)
                        # ax = fig.add_subplot(111)
                        # plt.imshow(img)
                        # plt.pause(0.1)
                        # fig.clf()
            except Empty:
                time.sleep(0.1)
    except Exception as e:
        print(f"相机处理异常: {str(e)}")


def main():
    sim.simxFinish(-1) # Close all opened connections

    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to CoppeliaSim
    if clientID != -1:
        print('Connected to remote API server')
    else:
        print('Failed connecting to remote API server')
        return

    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    print("Simulation started")

    # 初始化真实机械臂
    robot = AuboRobot()
    if not robot.connect():
        print("Failed to connect to real robot")
        return

    # 启用同步模式
    sim.simxSynchronous(clientID, True)

    # 创建TCP信号生成器
    signal_generator = tcp_server('192.168.113.66', 8888)  # 使用您想要的端口
    
    # 创建线程安全的队列用于信号传递
    signal_queue = Queue()
    
    # 创建线程
    signal_thread = threading.Thread(target=signal_handler, args=(signal_generator, signal_queue), daemon=True)
    motion_thread = threading.Thread(target=control_robot_motion, args=(robot, clientID, signal_queue), daemon=True)
    camera_thread = threading.Thread(target=process_camera, args=(clientID, signal_queue), daemon=True)
    
    # 启动线程
    signal_thread.start()
    motion_thread.start()
    camera_thread.start()

    try:
        last_time = time.time()
        while True:
            current_time = time.time()
            # 控制更新频率在50Hz左右
            if current_time - last_time >= 0.02:
                # 只处理最新数据
                joint_angles = robot.get_joint_angles()
                if joint_angles is not None:
                    set_joint_angles(clientID, '/1axis', joint_angles)
                
                # 触发仿真步进
                sim.simxSynchronousTrigger(clientID)
                last_time = current_time
                
            # 保持循环运行但不阻塞
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass

    # 确保线程能够终止
    # 注意：这里可能需要更复杂的线程终止机制，如使用标志位或事件
    signal_thread.join(timeout=0.5)
    motion_thread.join(timeout=0.5)
    camera_thread.join(timeout=0.5)

    # 仿真结束
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    print("Simulation stopped")

    # 释放资源
    # plt.ioff()
    sim.simxFinish(clientID)
    robot.disconnect()

if __name__ == '__main__':
    main()