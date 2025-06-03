from robotcontrol import *
import numpy as np
import math


def i5_robot():
    robot = Auboi5Robot()

    ret = robot.initialize()

    handle = robot.create_context()

    ip = "192.168.24.130"
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
        # fk=robot.forward_kin(real_pos['joint'])
        # np.set_printoptions(suppress=True)
        # rpy=robot.quaternion_to_rpy(fk['ori'])
        # rpy=np.array(rpy)*180/pi
        # print('正解:',fk)
        # print('正解欧拉角：',rpy)
        #逆解
        # ik=robot.inverse_kin(joint_radian,fk['pos'],fk['ori'])
        pos = [-0.44, -0.78, 0.5]
        rpy1 = [0.0, 0.0, 0.0]
        ori = robot.rpy_to_quaternion(rpy1)
        print(ori)
        ik=robot.inverse_kin(real_pos['joint'],real_pos['pos'],real_pos['ori'])
        # joint_radian=ik['joint_radian']
        # joint_deg=np.array(ik['joint'])*180/pi
        # print('逆解弧度:',joint_radian)
        # print('逆解角度:',joint_deg)
        print(ik)

    else:
        print("failed connet")

if __name__ == '__main__':
    i5_robot()