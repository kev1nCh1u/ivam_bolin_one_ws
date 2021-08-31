
# py3
import socket

UDP_IP = "127.0.0.1"
# UDP_IP = "192.168.72.152"
# UDP_IP = "221.120.83.108"

UDP_PORT = 9930

MESSAGE = b"Mr;\
2,0, -0.19, -3.57, -0.02,diff,0,1,two_part_1;\
4,13, 3.75, -3.54, 0.00,diff,0,1,two_part_1;\
5,13, 0.68, 0.16, -0.05,diff,0,1,two_part_2;\
6,3, 2.99, 0.44, 0.00,diff,0,1,two_part_2,2;\
E"

# MESSAGE = b"Mr;\
# 2,0, -0.19, -3.57, -0.02,diff,0,1,two_part_1;\
# 4,3, 3.75, -3.54, -0.00,diff,0,1,two_part_1,2;\
# E"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))