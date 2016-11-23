#!/usr/bin/env python

#from beginner_tutorials.srv import *
from aimm_srv.srv import hksGripper
import rospy
import socket
import string
from PIL import Image,ImageFont,ImageDraw

HOST = '192.168.1.109'    # The remote host  
PORT = 14000             # The same port as used by the server  
s = None  
state = 0;

def handle_gripper_srv(req):
	print "Received the para:  %d"%(req.position)
	if req.position < 60 or req.position > 100:
		print "Received para error!"
		return -1
	position = req.position
	#print "Begin to TCP!"
	ADDR=(HOST, PORT)
	try:
		hksSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	except:
		print "TCP creat fail!"
		return -1
	hksSocket.settimeout(1)
	try:
		hksSocket.connect(ADDR)
	except:
		print "TCP connect fail!"
		return -1
	hksSocket.settimeout(1)
	try:
		hksSocket.send("ID: XDJ IDEND CMD: MOVE CMDEND MVPARA: " + str(position) + " MVEND" + "\r\n")
	except:
		print "TCP send fail!"
		hksSocket.close()
		return -1
	hksSocket.settimeout(2)
	try:
		data=hksSocket.recv(40)
		#print "Received string:  %s"%(data)
		if data.index("ANS: Start") > 10:
			return 1
		hksSocket.close()
	except:
		print "TCP receive fail!"
		hksSocket.close()
		return -1
	#not success
    #return -1

def hks_gripper_srv_main():
	rospy.init_node('hks_gripper_srv')
	s = rospy.Service('hksGpripperSrv', hksGripper, handle_gripper_srv)
	print "HKSGRIPPER Ready to receive!"
	rospy.spin()

if __name__ == "__main__":
	hks_gripper_srv_main()
	
