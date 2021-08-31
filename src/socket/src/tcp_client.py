
# python 3
import socket
import time

HOST = '127.0.0.1'
PORT = 9930
clientMessage = 'Hello!'
loop = 0

while 1:
    
    # 建立連線
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((HOST, PORT))

    # 傳送
    print("send:",(clientMessage + str(loop)).encode())
    client.sendall((clientMessage + str(loop)).encode())

    # 接收
    serverMessage = str(client.recv(1024), encoding='utf-8')
    print('Server:', serverMessage)

    # 中止
    client.close()

    time.sleep(1)
    loop += 1