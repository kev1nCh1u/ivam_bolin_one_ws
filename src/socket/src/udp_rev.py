import socket

# UDP_IP = "127.0.0.1"
# UDP_IP = "172.20.10.3"
UDP_IP = ""
UDP_PORT = 9930

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("received message: %s" % data)