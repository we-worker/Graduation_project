import socket
import threading
from start import run_command, stop_command,commands
import time

def handle_client(client_socket):
	while True:
		request = client_socket.recv(1024).decode()
		if not request:
			break
		print(f'Received: {request}')
		if request.startswith('START'):
			cmd_id = request.split(' ')[1]
			if cmd_id == 'ALL':
				for cmd_id in commands.keys():
					threading.Thread(target=run_command, args=(cmd_id, client_socket)).start()
					time.sleep(4)
			else:
				threading.Thread(target=run_command, args=(cmd_id, client_socket)).start()
		elif request.startswith('STOP'):
			cmd_id = request.split(' ')[1]
			if cmd_id == 'ALL':
				for cmd_id in commands.keys():
					threading.Thread(target=stop_command, args=(cmd_id, client_socket)).start()
					time.sleep(5)
			else:
				threading.Thread(target=stop_command, args=(cmd_id, client_socket)).start()
	client_socket.close()

def server_program():
    host = '0.0.0.0'
    port = 2333

    server_socket = socket.socket()
    server_socket.bind((host, port))

    server_socket.listen(2)
    print('Server started. Waiting for connections...')

    while True:
        client_socket, address = server_socket.accept()
        print(f'Connection from: {address}')
        threading.Thread(target=handle_client, args=(client_socket,)).start()

def debug_msg(client,msg):
    client.sendall(f'DEBUG {msg}'.encode())

if __name__ == '__main__':
    server_program()