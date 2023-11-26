import re
import time

message_handler = None

def get_message_handler(runtime_stylesheets):
	return MessageHandler(runtime_stylesheets)

def tcp_client_concurrency(tcp_socket, client_socket_list):
	while True:
		recv_msg = tcp_socket.recv(1024)
		if recv_msg:
			msg = recv_msg.decode('utf-8')
			message_handler.handle_message(msg)
			print(msg)
			# print('来自IP:{}端口:{}:\n{}\n'.format(address[0], address[1], msg))
		else:
			tcp_socket.close()
			print('从服务器断开连接\n')
			break

class MessageHandler:
	def __init__(self, runtime_stylesheets):
		self.handlers = {
			'DEBUG': self.debug_message,
			# 添加更多的消息类型和处理函数
		}
		self.runtime_stylesheets = runtime_stylesheets


	def handle_message(self, message):
		for msg_type, handler in self.handlers.items():
			if message.startswith(msg_type):
				return handler(message)
		return None

	def debug_message(self, message):
		match = re.search(r'DEBUG (.*)', message)
		if match:
			debug_msg = match.group(1)
			print(debug_msg)
			self.runtime_stylesheets.write_debug(debug_msg)
			return debug_msg
		return None
