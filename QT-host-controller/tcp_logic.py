import socket
import threading
import time
import ctypes
import inspect

# 强制关闭线程的方法
from stopThreading import stop_thread

class TcpLogic():
    def __init__(self):
        self.tcp_socket = None
        self.sever_th = None
        self.client_th = None
        self.client_socket_list = list()
        self.link = False  # 用于标记是否开启了连接

    def tcp_server_start(self, port):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)# 取消主动断开连接四次握手后的TIME_WAIT状态
        self.tcp_socket.setblocking(False)# 设定套接字为非阻塞式
        try:
            self.tcp_socket.bind(('', port))
        except Exception as ret:
            print('请检查端口号\n')
        else:
            self.tcp_socket.listen()
            self.sever_th = threading.Thread(target=self.tcp_server_concurrency)
            self.sever_th.start()
            print('TCP服务端正在监听端口:%s\n' % str(port))

    def tcp_server_concurrency(self):
        while True:
            try:
                client_socket, client_address = self.tcp_socket.accept()
            except Exception as ret:
                time.sleep(0.001)
            else:
                client_socket.setblocking(False)
                self.client_socket_list.append((client_socket, client_address))
                print('TCP服务端已连接IP:%s端口:%s\n' % client_address)
            for client, address in self.client_socket_list:
                try:
                    recv_msg = client.recv(1024)
                except Exception as ret:
                    pass
                else:
                    if recv_msg:
                        msg = recv_msg.decode('utf-8')
                        print('来自IP:{}端口:{}:\n{}\n'.format(address[0], address[1], msg))
                    else:
                        client.close()
                        self.client_socket_list.remove((client, address))

    def tcp_client_start(self, ip, port):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            address = (ip, port)
        except Exception as ret:
            print('请检查目标IP，目标端口\n')
        else:
            try:
                print('正在连接目标服务器\n')
                self.tcp_socket.connect(address)
                self.link=False
            except Exception as ret:
                print('无法连接目标服务器\n')
                self.link=False
            else:
                self.client_th = threading.Thread(target=self.tcp_client_concurrency, args=(address,))
                self.client_th.start()
                print('TCP客户端已连接IP:%s端口:%s\n' % address)
                self.link=True

    def tcp_client_concurrency(self, address):
        while True:
            recv_msg = self.tcp_socket.recv(1024)
            if recv_msg:
                msg = recv_msg.decode('utf-8')
                print('来自IP:{}端口:{}:\n{}\n'.format(address[0], address[1], msg))
            else:
                self.tcp_socket.close()
                print('从服务器断开连接\n')
                break

    def tcp_send(self, send_msg):
        if self.link is False:
            print('请选择服务，并点击连接网络\n')
        else:
            try:
                send_msg = send_msg.encode('utf-8')
                for client, address in self.client_socket_list:
                    client.send(send_msg)
                print('TCP服务端已发送\n')
                self.tcp_socket.send(send_msg)
                print('TCP客户端已发送\n')
            except Exception as ret:
                print('发送失败\n')

    def tcp_close(self):
        try:
            for client, address in self.client_socket_list:
                client.close()
            self.tcp_socket.close()
            if self.link is True:
                print('已断开网络\n')
                self.link=False
        except Exception as ret:
            pass
        try:
            stop_thread(self.sever_th)
        except Exception:
            pass
        try:
            stop_thread(self.client_th)
        except Exception:
            pass