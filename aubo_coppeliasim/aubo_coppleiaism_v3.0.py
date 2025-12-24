from robotcontrol import *
import sim
import threading
import socket
import cv2
import numpy as np


class AuboRobot:
    def __init__(self, ip="192.168.24.130", port=8899):
        self.robot = Auboi5Robot()
        self.ip = ip
        self.port = port
        self.connected = False

    def connect(self):
        """连接机器人"""
        ret = self.robot.initialize()
        handle = self.robot.create_context()
        result = self.robot.connect(self.ip, self.port)
        self.connected = (result == 0)  # 连接成功返回0
        return self.connected
    
    def get_joint_angles(self):
        real_pos = self.robot.get_current_waypoint()
        # joint_deg = np.array(real_pos['joint'])*180/pi
        # print(joint_deg)
        return real_pos['joint']
    
class coppliasim_connet:
    def __init__(self,ip='127.0.0.1',port=19997):
        self.ip = ip
        self.port = port

    def connect(self):
        """连接CoppeliaSim"""
        clientID = sim.simxStart(self.ip, self.port, True, True, 5000, 5)
        if clientID != -1:
            print('连接成功')
        else:
            print('连接失败')
        return clientID 

    def simulation_start(self,clientID):
        """连接simulation"""
        sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
        print("仿真启动")

    def simulation_stop(self,clientID):
        """关闭simulation"""
        sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot)
        print("仿真关闭")

    def set_joint_angels(self,clientID,handlename,joint_angles):
        sim.simxGetPingTime(clientID=clientID)
        for i in range (len(joint_angles)):
            _, joint_handle = sim.simxGetObjectHandle(clientID, f'{handlename}/Revolute_joint{i+1}', sim.simx_opmode_oneshot)
            sim.simxSetJointPosition(clientID,joint_handle,joint_angles[i],sim.simx_opmode_oneshot)

    def get_images_init(self,clientID,handlename):
            _,handle = sim.simxGetObjectHandle(clientID,handlename,sim.simx_opmode_oneshot)
            ret,resolution,raw_image = sim.simxGetVisionSensorImage(clientID,handle,0,sim.simx_opmode_streaming)

    def get_images(self,clientID,handlename):
            _,handle = sim.simxGetObjectHandle(clientID,handlename,sim.simx_opmode_oneshot)
            max_retries = 10  # 最大重试次数
            retry_count = 0
            
            while retry_count < max_retries:
                ret,resolution,raw_image = sim.simxGetVisionSensorImage(clientID,handle,0,sim.simx_opmode_buffer)
                print(f"返回码: {ret} (尝试 {retry_count+1}/{max_retries})")
                if ret == 0:
                    img = encode_visionsensorImage(raw_image,resolution)
                    if img is not None:  # 确保图像解码成功
                        print(img.shape)
                        print("完成拍照")
                        return img
                retry_count += 1
                time.sleep(0.1)
            
            print("警告：达到最大重试次数仍未获取有效图像")
            return None

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


def conctorl_robot_motion(robot,cc,clientid):
    while True:
        joint_angeles = robot.get_joint_angles()
        cc.set_joint_angels(clientid,'/1axis',joint_angeles)
        

def camera_capture(ip,port,cc,clientid):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((ip, port))
    server_socket.listen(1)
    print("服务端启动")
    while True:
        img = cc.get_images_init(clientid,'/1axis/camera')
        print("初始化完成")
        conn,addr = server_socket.accept()
        print(f"客户端{addr}已连接")
        try:
            while True:
                try:
                    data = conn.recv(1024).decode("UTF-8")
                    if not data:
                        break
                    print(f"收到客户端{addr}的数据: {data}")
                    if data == 'capture':
                        time.sleep(0.5)
                        img = cc.get_images(clientid,'/1axis/camera')
                        print("拍照一次")
                        if img is not None:
                            savefile = 'aubo_coppeliasim/images/' + time.strftime("%Y%m%d_%H%M%S") + '.png' 
                            # savefile = '/images/' + time.strftime("%Y%m%d_%H%M%S") + '.png' 
                            img_translation = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            cv2.imwrite(savefile, img_translation)
                        else:
                            print("未能获取有效图像，跳过保存")

                except (ConnectionResetError, ConnectionAbortedError, socket.error) as e:
                    print(f"接收数据时连接异常: {e}")
                    break
        finally:
            conn.close()
            print("连接关闭,等待新连接")



def main():
    sim.simxFinish(-1)
    cc = coppliasim_connet()
    clientid=cc.connect()

    robot = AuboRobot()
    if not robot.connect():
        print("连接机器人失败")
        return
    print("连接机器人成功")

    cc.simulation_start(clientid)
    
    

    camera_thread = threading.Thread(target=camera_capture,args=('192.168.218.1',8888,cc,clientid), daemon=True)
    camera_thread.start()
    if not camera_thread.is_alive():
        print("警告：相机服务端线程启动失败")
    # camera_thread.join()

    conctorl_robot_motion(robot,cc,clientid)

    cc.simulation_stop(clientid)

    sim.simxFinish(clientid)


if __name__ == '__main__':
    main()
