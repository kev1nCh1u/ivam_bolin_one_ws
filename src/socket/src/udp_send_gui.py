#!/usr/bin/env python

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QKeySequence, QPalette, QColor
from PyQt5.QtCore import Qt
import socket


def SocketFuc(MESSAGE):
    UDP_IP = "127.0.0.1"
    # UDP_IP = "192.168.72.152"
    UDP_PORT = 9930

    # MESSAGE = b"Hello, World!"
    # MESSAGE = b"Mr;1,0,1.73,0.43,1.74,diff,0,0.5,ivam_3F;2,3,2.48,-5.01,1.74,diff,0,0.5,ivam_3F,2;E" #line
    # MESSAGE = b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,0,0.5,ivam_3F;3,3,4.01,-3.44,1.68,diff,0,0.5,ivam_3F,2;E" #one turn
    # MESSAGE = b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,1,0.5,ivam_3F;3,1,4.01,-3.44,1.68,diff,0,0.5,ivam_3F;4,3,1.31,-3.73,0.12,diff,0,0.5,ivam_3F,2;E" #two turn
    # MESSAGE = b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,0,0.5,ivam_3F;3,1,3.77,-0.85,-1.46,diff,1,0.5,ivam_3F;4,1,4.01,-3.44,1.68,diff,1,0.5,ivam_3F;5,1,1.31,-3.73,0.12,diff,0,0.5,ivam_3F;6,3,0.72,-0.358,-1.45,diff,0,0.5,ivam_3F,2;E"  # o turn
    # "Mr;1,0,1.68822,0.432922,1.71258,diff,0,0.5,ivam_3F;2,3,2.46556,-4.44588,1.66535,diff,0,0.5,ivam_3F,2;E"

    print("UDP target IP: %s" % UDP_IP)
    print("UDP target port: %s" % UDP_PORT)
    print("message: %s" % MESSAGE)

    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print()

# back
def SendBack():
    SocketFuc(b"F;10;E")

# front
def SendFront():
    SocketFuc(b"F;11;E")

# square
def SendSquPath():
    SendBack()
    # SocketFuc(b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,0,0.5,ivam_3F;3,1,3.77,-0.85,-1.46,diff,1,0.5,ivam_3F;4,1,4.01,-3.44,1.68,diff,1,0.5,ivam_3F;5,1,1.31,-3.73,0.12,diff,0,0.5,ivam_3F;6,3,0.72,-0.358,-1.45,diff,0,0.5,ivam_3F,2;E")
    SocketFuc(b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,0,0.5,ivam_3F;3,1,3.77,-0.85,-1.46,diff,1,0.5,ivam_3F;4,1,4.01,-3.44,1.68,diff,1,0.5,ivam_3F;5,1,1.31,-3.73,0.12,diff,0,0.5,ivam_3F;6,3,1.14,--2.10,-1.45,diff,0,0.5,ivam_3F,2;E")

# S back
def SendSBack():
    SendBack()
    SocketFuc(b"Mr;1,0,2.11,0.57,1.655,diff,1,0.5,ivam_3F;2,1,3.8,-0.48,1.74,diff,0,0.5,ivam_3F;3,1,2.27,-2.0,1.08,diff,1,0.5,ivam_3F;4,1,1.18,-2.9,1.74,diff,0,0.5,ivam_3F;5,3,2.25556,-4.49588,1.66535,diff,0,0.5,ivam_3F,2;E")

# S front
def SendSFront():
    SendFront()
    SocketFuc(b"Mr;1,0,2.25556,-4.49588,1.66535,diff,1,0.5,ivam_3F;2,1,1.18,-2.9,1.74,diff,0,0.5,ivam_3F;3,1,2.27,-2.0,1.08,diff,1,0.5,ivam_3F;4,1,3.8,-0.48,1.74,diff,0,0.5,ivam_3F;5,3,2.11,0.57,1.655,diff,0,0.5,ivam_3F,2;E")

# home to cnc
def SendHtoC():
    SendFront()
    # SocketFuc(b"Mr;1,0,-0.95, -0.30,-3.02,diff,0,0.5,ivam_3F;3,19,1.49,-0.90,-0.32,diff,0,0.5,ivam_3F;6,3, 1.33, -5.12, 0.99,diff,0,0.5,ivam_3F,2;E")
    SocketFuc(b"Mr;1,0,  -2.68, -0.32, 0.11,diff,0,0.5,ivam_3F;2,19,1.57,-0.06,1.09,diff,0,0.5,ivam_3F;3,1,0.92,-1.92,1.66,diff,0,0.5,ivam_3F;4,1,2.06, -3.88,1.04,diff,0,0.5,ivam_3F;5,3, 1.29, -5.20, 1.01,diff,0,0.5,ivam_3F,2;E")

# cnc to home
def SendCtoH():
    SendFront()
    SocketFuc(b"Mr;1,0, 1.29, -5.20, 1.01,diff,0,0.5,ivam_3F;2,1,2.06, -3.88,1.04,diff,0,0.5,ivam_3F;3,1,0.92,-1.92,1.66,diff,0,0.5,ivam_3F;4,19,1.57,-0.06,1.09,diff,0,0.5,ivam_3F;5,3, -2.68, -0.32, 0.11,diff,0,0.5,ivam_3F,2;E")

# home to turnplace
def SendHtoT():
    SendFront()
    SocketFuc(b"Mr;1,0,-0.95, -0.30,-3.02,diff,0,0.5,ivam_3F;2,3, 1.49,-0.90,-0.32,diff,0,0.5,ivam_3F,2;E")

# turnplace to cnc
def SendTtoC():
    SendBack()
    SocketFuc(b"Mr;1,0,1.49,-0.90,-0.32,diff,0,0.5,ivam_3F;2,3, 1.01,-2.98,1.69,diff,0,0.5,ivam_3F,2;E")

# cnc to turnplace
def SendCtoT():
    SendFront()
    SocketFuc(b"Mr;1,0, 0.92,-1.92,1.66,diff,0,0.5,ivam_3F;2,3, 0.70,0.10,1.68,diff,0,0.5,ivam_3F,2;E")

# turnplace to home
def SendTtoH():
    SendBack()
    SocketFuc(b"Mr;1,0, -1.93,-0.44,-3.06,diff,0,0.5,ivam_3F;2,3, -1.94, -0.39,0.09,diff,0,0.5,ivam_3F,2;E")

def main():
    app = QApplication([])

    app.setStyle("Fusion")

    # Now use a palette to switch to dark colors:
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)

    # The rest of the code is the same as for the "normal" text editor.
    app.setApplicationName("IVAM")
    text = QPlainTextEdit()

    window = QWidget()
    layout = QGridLayout()

    backButton = QPushButton("Back")
    backButton.clicked.connect(SendBack)
    layout.addWidget(backButton, 0, 0)

    frontBotton = QPushButton("Front")
    frontBotton.clicked.connect(SendFront)
    layout.addWidget(frontBotton, 0, 1)

    squButton = QPushButton("Square Back")
    squButton.clicked.connect(SendSquPath)
    layout.addWidget(squButton, 1, 0)

    sBack = QPushButton("S Back")
    sBack.clicked.connect(SendSBack)
    layout.addWidget(sBack, 2, 0)

    sFront = QPushButton("S Front")
    sFront.clicked.connect(SendSFront)
    layout.addWidget(sFront, 2, 1)
    
    HometoCNC = QPushButton("HometoCNC")
    HometoCNC.clicked.connect(SendHtoC)
    layout.addWidget(HometoCNC, 3, 0)

    CNCtoHome = QPushButton("CNCtoHome")
    CNCtoHome.clicked.connect(SendCtoH)
    layout.addWidget(CNCtoHome, 3, 1)

    HtoT = QPushButton("HtoT")
    HtoT.clicked.connect(SendHtoT)
    layout.addWidget(HtoT, 4, 0)

    TtoC = QPushButton("TtoC")
    TtoC.clicked.connect(SendTtoC)
    layout.addWidget(TtoC, 4, 1)

    CtoT = QPushButton("CtoT")
    CtoT.clicked.connect(SendCtoT)
    layout.addWidget(CtoT, 5, 0)

    TtoH = QPushButton("TtoH")
    TtoH.clicked.connect(SendTtoH)
    layout.addWidget(TtoH, 5, 1)

    window.setLayout(layout)
    window.show()

    app.exec_()

main()