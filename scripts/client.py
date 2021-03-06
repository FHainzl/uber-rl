import socket
import numpy as np

import rospy
from rospy import sleep


class Client(object):
    def __init__(self, port, host, bufsize):
        self.address = (host, port)
        self.bufsize = bufsize
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(self.address)
        rospy.loginfo("Connected to server at " + host)
        sleep(5)

    def send_array(self, array):
        if array.size > 50:
            raise ValueError("Array too big! Increase bufsize")
        s = array.tostring()
        self.client_socket.send(s)
        while self.client_socket.recv(self.bufsize) != b"ack":
            pass
            # print "Waiting for ack..."

    def send_flush(self):
        self.client_socket.send(b"flush")

    def __del__(self):
        disconnect = b"disconnect"
        self.client_socket.send(disconnect)
        self.client_socket.close()
        print "Connection closed!"
