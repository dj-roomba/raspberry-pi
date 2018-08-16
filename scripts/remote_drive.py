#!/usr/bin/env python

from websocket_server import WebsocketServer

import rospy

import drive

# Contains a websocket server listening for drive commands
# Starts when the run() function is called
class RemoteDrive(object):
    # set up a Drive object to send drive commands to the roomba
    # set up the websocket server on port 8001
    def __init__(self):
        self.drive = drive.Drive()

        self.server = WebsocketServer(8001, '0.0.0.0')
        self.server.set_fn_new_client(self.new_client)
        self.server.set_fn_message_received(self.msg_recv)

    def new_client(self, client, server):
        print 'client connected'

    # n: none, stop driving
    # f: forward
    # l: left
    # r: right
    # b: backwards
    def msg_recv(self, client, server, msg):
        print msg
        if msg == 'n':
            self.drive.drive(0, 0)
        elif msg == 'f':
            self.drive.drive(0.5, 0)
        elif msg == 'l':
            self.drive.drive(0, 2)
        elif msg == 'r':
            self.drive.drive(0, -2)
        elif msg == 'b':
            self.drive.drive(-0.5, 0)

    # start the server
    def run(self):
        self.server.run_forever()

if __name__ == '__main__':
    rospy.init_node("remote_drive", anonymous=True)
    rd = RemoteDrive()
    rd.run()
