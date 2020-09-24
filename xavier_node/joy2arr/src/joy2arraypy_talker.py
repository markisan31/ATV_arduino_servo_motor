#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from g29_force_feedback.msg import ForceFeedback
import serial
import time



def mapf(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def callback(data): 

        #rospy.loginfo("Subscribing")
        remoteStearing = int(mapf(data.axes[0], -1, 1, 1000, 0))
        if remoteStearing > 1000:
            remoteStearing = 1000
        elif remoteStearing < 0:
            remoteStearing = 0
        if data.buttons[0] == 0:
	    remoteThrottle = int(mapf(data.axes[2], -1, 1, 515, 1000))
	elif  data.buttons[0] == 1:
	    remoteThrottle = int(mapf(data.axes[2], -1, 1, 485, 0))
        remoteBreak = int(mapf(data.axes[3], -1, 1, 0, 1000))

	if data.buttons[4] == 1 and data.buttons[5] != 1:
            turningLights = 1000
	elif data.buttons[5] == 1 and data.buttons[4] != 1:
	    turningLights = 0
	else:
	    turningLights = 500 

	
	    
	

	
        arduino.write(b"<" + str(remoteStearing) + ", " + str(remoteThrottle) +  ", "  + str(remoteBreak) + ", " + str(turningLights) + ">\n")
        


arduino = serial.Serial('/dev/ttyACM3', 460800, write_timeout=0)
rospy.init_node('joy2serialpy', anonymous=True)
rospy.Subscriber("joy", Joy, callback, queue_size = 1)
# spin() simply keeps python from exiting until this node is stopped
rate = rospy.Rate(100) 
rospy.spin()   

if __name__ == '__main__':
    pass
    

