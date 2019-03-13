import socket
import numpy as np
from rospy import sleep


class Client(object):
    def __init__(self, port, host="192.168.1.234"):
        self.address = (host, port)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(self.address)
        sleep(0.5)

    def send_array(self, array):
        if array.size > 100:
            raise ValueError("Array too big! Increase bufsize")
        s = array.tostring()
        print 'Sent to server: '
        print array
        self.client_socket.send(s)
        while self.client_socket.recv(2048) != b"ack":
            print "Waiting for ack..."
            sleep(0.1)
        print "<ack> received!"

    def __del__(self):
        disconnect = b"disconnect"
        self.client_socket.send(disconnect)
        self.client_socket.close()
        print "Connection closed!"


if __name__ == '__main__':
    client = Client(port=47474)
    while True:
        x = np.random.rand(100)
        client.send_array(x)
        sleep(1)
