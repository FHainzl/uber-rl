import socket
import numpy as np

import rospy


class Server:
    def __init__(self, port, bufsize):
        self.address = ("", port)
        self.bufsize = bufsize
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(self.address)
        self.server_socket.listen(5)
        rospy.loginfo("Listening for client . . .")
        self.conn, self.conn_address = self.server_socket.accept()
        rospy.loginfo("Connected to client!")

    def loop(self):
        while True:
            output = self.conn.recv(self.bufsize)
            if output.strip() == b"disconnect":
                rospy.loginfo("Received disconnect message.  Shutting down.")
                break
            elif output:
                array = self.receive_array(output)
                self.callback(array)
                self.conn.send(b"ack")

    @staticmethod
    def receive_array(data):
        array = np.fromstring(data)
        return array

    @staticmethod
    def callback(data):
        rospy.loginfo("Message received from client:")
        print data

    def __del__(self):
        self.conn.close()
        rospy.loginfo("Connection closed")
