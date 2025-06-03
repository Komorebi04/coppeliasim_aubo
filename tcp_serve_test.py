import socket

def tcp_server(host, port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"服务端已开始监听，正在等待客户端连接...")
    
    while True:
        try:
            conn, address = server_socket.accept()
            print(f"接收到了客户端的连接，客户端的信息：{address}")
            
            while True:
                data = conn.recv(1024).decode("UTF-8")
                if not data:  # 客户端断开连接
                    break
                print(f"客户端发来的消息是：{data}")
                
                # 可以添加回复逻辑
                # data1 = input("请输入要发送给客户端的消息：")
                # conn.send(data1.encode("UTF-8"))
                
        except Exception as e:
            print(f"连接异常: {e}")
        finally:
            if 'conn' in locals():
                conn.close()
            print("等待新的客户端连接...")


if __name__ == "__main__":
    tcp_server("192.168.6.66", 8888) 