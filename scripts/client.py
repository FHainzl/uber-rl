import socket
import numpy as np
from rospy import sleep


class Client(object):
    def __init__(self, port, host, bufsize):
        self.address = (host, port)
        self.bufsize = bufsize
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(self.address)
        sleep(0.5)

    def send_array(self, array):
        if array.size > 50:
            raise ValueError("Array too big! Increase bufsize")
        s = array.tostring()
        self.client_socket.send(s)
        while self.client_socket.recv(self.bufsize) != b"ack":
            print "Waiting for ack..."
            sleep(0.1)

    def send_flush(self):
        self.client_socket.send(b"flush")

    def __del__(self):
        disconnect = b"disconnect"
        self.client_socket.send(disconnect)
        self.client_socket.close()
        print "Connection closed!"


if __name__ == '__main__':
    client = Client(port=47474, host="192.168.1.234", bufsize=1024)
    while True:
        x = np.random.rand(100)
        client.send_array(x)
        sleep(1)
