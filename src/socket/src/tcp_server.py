# -*- coding: utf-8 -*-
import socket
HOST = '192.168.225.20'
# HOST = '10.1.1.2'
PORT = 9930

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
server.bind((HOST, PORT))
server.listen(10)

while True:
    
    conn, addr = server.accept()
    clientMessage = str(conn.recv(1024), encoding='utf-8') # python3
    # clientMessage = str(conn.recv(1024).encode('utf-8')) # python2


    print('Client message is:', clientMessage)

    serverMessage = 'I\'m here!'
    conn.sendall(serverMessage.encode())
    conn.close()