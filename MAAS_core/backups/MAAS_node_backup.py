#!/usr/bin/env python

# imports
import sys
import numpy as np
import json

# ros imports
import rospy
from std_msgs.msg import String


class MAAS_node:
    def __init__(self):

        # subscribers
        self.mcts_sub = rospy.Subscriber("/mcts/data", String, self.data_callback)

        # publishers
        self.maas_pub = rospy.Publisher('/maas/data', String, queue_size = 10) # jsonified data

         # some other stuff
        self.json_dict = {"sample_value": []}


############################# Subscriber Callback functions ####################
    def data_callback(self, data):

        # json loads
        temp_data = json.loads(data.data)
	print(data.data)

	# get the x and y values of the point
	x = 3
	y = 4 

        # get the estimated value of a point
	estimated_value = 5.3

       
 	# create the message
 	self.json_dict['sample_value'].append({"x": x, "y": y, "estimated_value":estimated_value})
        #self.json_dict['points'].append({"x": 2, "y": 3, "estimated_value":estimated_value})
        #self.json_dict['points'].append({"x": 3, "y": 3, "estimated_value":estimated_value})
        #self.json_dict['points'].append({"x": 4, "y": 3, "estimated_value":estimated_value})
        
		
        self.publish_action()


############################# Publisher functions ##############################
    def publish_action(self):
        # publish action
        self.maas_pub.publish(json.dumps(self.json_dict))



############################# Main #############################################
def main():
    # init ros node
    rospy.init_node('MAAS', anonymous = True)

    # class instance
    MAAS_instance = MAAS_node()

    # create ros loop
    pub_rate = 1 # hertz
    rate = rospy.Rate(pub_rate)

    while (not rospy.is_shutdown()):
        # do some stuff if necessary
	print("I AM ALIVE")

        # ros sleep (sleep to maintain loop rate)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
