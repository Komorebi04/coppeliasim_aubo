import sim
import time
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt

def encode_visionsensorImage(raw_image,resolution):
    img = np.array(raw_image,dtype=np.uint8)
    img.resize([resolution[1],resolution[0],3])
    img = cv2.flip(img,0)
    return img

def get_vs_img(clientID,handlname,mode=0):
    if mode == 0:
        _,vs_handle = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_blocking)
        ret,resolution,raw_image=sim.simxGetVisionSensorImage(clientID,vs_handle,0,sim.simx_opmode_streaming)
        time.sleep(0.5)
        print("ret0",ret)
        print("res0",resolution)

    elif mode == 1:
        _,vs_handle = sim.simxGetObjectHandle(clientID,handlname,sim.simx_opmode_blocking)
        ret,resolution,raw_image = sim.simxGetVisionSensorImage(clientID,vs_handle,0,sim.simx_opmode_buffer)
        print(ret)
        print(resolution)
        img = encode_visionsensorImage(raw_image,resolution)
        print(img.shape)
        return img
    else:
        print('Error mode')


#关闭之前的连接
sim.simxFinish(-1)

# 获得客户端ID
clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
print("Connection success!!!")

if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

# 启动仿真
sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
print("Simulation start")

# 使能同步模式
sim.simxSynchronous(clientID,True)

get_vs_img(clientID,'/camera',mode=0)
plt.ion()
fig = plt.figure("vs_img")
while True:
    ax = fig.add_subplot(111)
    img = get_vs_img(clientID,'/camera',mode=1)
    plt.imshow(img)
    plt.pause(0.1)
    fig.clf()
    
plt.ioff()
# 退出
sim.simxFinish(clientID)
print('Program end')
