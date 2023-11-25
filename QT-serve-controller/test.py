import socket

def client_program():
    host = 'localhost'
    port = 2333

    client_socket = socket.socket()
    client_socket.connect((host, port))
    client_socket.settimeout(0.5)  # 设置超时时间为0.5秒

    while True:
        message = input(" -> ")  # take input
        if not message:
            break
        client_socket.send(message.encode())  # send message
        try:
            data = client_socket.recv(1024).decode()  # receive response
            print('Received from server: ' + data)  # show in terminal
        except socket.timeout:
            print('No response from server, continue sending')

    client_socket.close()  # close the connection

if __name__ == '__main__':
    client_program()