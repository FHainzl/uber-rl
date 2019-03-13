import socket
import numpy as np


class Server:
    def __init__(self, port):
        self.address = ("", port)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(self.address)
        self.server_socket.listen(5)
        print "Listening for client . . ."
        self.conn, self.conn_address = self.server_socket.accept()
        print "Connected to client at ", self.conn_address, '\n'
        self.loop()

    def loop(self):
        while True:
            output = self.conn.recv(2048)
            if output.strip() == b"disconnect":
                print "Received disconnect message.  Shutting down."
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
        print "Message received from client:"
        print data

    def __del__(self):
        self.conn.close()
        print "Connection closed"


if __name__ == '__main__':
    serv = Server(47474)
