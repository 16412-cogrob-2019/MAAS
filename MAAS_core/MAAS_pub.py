#!/usr/bin/env python

# imports
import sys
import numpy as np
import json

# ros imports
import rospy
from std_msgs.msg import String


class Test_Sampler:

    def __init__(self):

        # publishers
        self.test_pub = rospy.Publisher('/sample/data', String, queue_size = 10) # jsonified data
 	    
        # some other stuff
        self.json_dict = {"sample": []}

	self.agent_id = 1 	
	self.x = 1
	self.y = 2
	self.z = 4
	

    # publish a sample 
    def publish_sample(self):
	
	# the point to estimate
	measurement_type = "depth"
	measurement_value = 5.5
	
	# reset the data - and send it  	
	self.json_dict = {"sample": []}
	self.json_dict['sample'].append({"agent_id":self.agent_id, "x": self.x, "y": self.y,  "z": self.z, "measurement_type":measurement_type, "measurement_value":measurement_value})
	
        # publish action
        self.test_pub.publish(json.dumps(self.json_dict))
    
    


############################# Main #############################################
def main():

    # init ros node
    rospy.init_node('test_sampler', anonymous = True)

    # class instance
    test_instance = Test_Sampler()

    # create ros loop
    pub_rate = 10 # hertz
    rate = rospy.Rate(pub_rate)

    while (not rospy.is_shutdown()):
        # pack something in a json object
        test_instance.publish_sample()

        # ros sleep (sleep to maintain loop rate)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
