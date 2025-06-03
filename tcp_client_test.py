from robotcontrol import *
import socket

# 初始化机器人对象
robot = Auboi5Robot()
# 正确的连接顺序
handle = robot.create_context()  # 先创建上下文
robot.initialize()               # 然后初始化
robot.connect(ip="192.168.24.130", port=8899)  # 最后连接
# 初始化机器人
robot.initialize()
# 获取当前位置信息
real_pos=robot.get_current_waypoint()
init_pos = real_pos['pos']
init_ori = real_pos['ori']
init_joint = real_pos['joint']
print(init_pos)
print(init_ori)
print(init_joint)

# 创建TCP客户端socket
socket_client = socket.socket()
# 连接到服务器
socket_client.connect(("192.168.24.1", 8000))
 
while True:
    send_msg = input("请输入要发送给服务端的消息：")
    if send_msg == "exit":
        break
    # 发送消息
    socket_client.send(send_msg.encode("UTF-8"))
    # 接收服务端回复
    recv_data = socket_client.recv(1024).decode("UTF-8")    # 1024字节缓冲区大小
    print(f"服务端回复的消息是：{recv_data}")
    # 处理接收到的数据
    data_table = []
    for i in recv_data.split(";"):
        data_table.append(float(i))
    print(data_table)
    
# 更新位置坐标
# real_pos['pos'][0] = data_table[0]/1000 + real_pos['pos'][0]
# real_pos['pos'][1] = data_table[1]/1000 + real_pos['pos'][1]

real_pos['pos'][0] = data_table[2]/1000 + real_pos['pos'][0]
real_pos['pos'][1] = data_table[3]/1000 + real_pos['pos'][1]

# 计算逆运动学并移动机械臂
# 打印输入参数检查
print("当前关节角:", real_pos['joint'], "长度:", len(real_pos['joint']))
print("目标位置:", real_pos['pos'])
print("目标姿态:", real_pos['ori'])

# 修正参数顺序：pos → ori → joint
joint1 = robot.inverse_kin(joint_radian=real_pos['joint'],pos=real_pos['pos'], ori=real_pos['ori'], )
print(joint1)
# robot.move_joint(joint1['joint'])
# robot.move_joint(init_joint)
# 关闭socket连接
socket_client.close()


