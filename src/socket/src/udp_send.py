
# py3
import socket

UDP_IP = "127.0.0.1"
# UDP_IP = "192.168.72.152"
# UDP_IP = "221.120.83.108"

UDP_PORT = 9930

# MESSAGE = b"Mr;\
# 0,0, 0, 0, 0,diff,0,1,two_part_1;\
# 1,3, -0.19, -3.57, -0.02,diff,0,1,two_part_1,0;\
# 2,1, 1.38, -3.42, 0,diff,0,1,two_part_1;\
# 4,13, 3.75, -3.54, 0.00,diff,0,1,two_part_1;\
# 5,13, 0.68, 0.16, -0.05,diff,0,1,two_part_2;\
# 6,3, 2.99, 0.44, 0.00,diff,0,1,two_part_2,2;\
# E"

MESSAGE = b"Mr;\
1,0, -0.19, -3.57, -0.02,diff,0,1,two_part_1;\
4,13, 3.75, -3.54, 0.00,diff,0,1,two_part_1;\
5,13, 0.68, 0.16, -0.05,diff,0,1,two_part_2;\
810,18, 2.99, 0.44, 0.00,diff,0,1,two_part_2,0-9.1-0.78\
6,3, 4.99, 0.44, 0.00,diff,0,1,two_part_2,2;\
E"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))