import socket
import time

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('127.0.0.1',5555))
server.listen(1)

while True:
    time.sleep(1)