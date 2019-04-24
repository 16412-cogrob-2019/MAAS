#!/usr/bin/env python

# imports
import sys
import numpy as np
import json

# ros imports
import rospy
from std_msgs.msg import String


class Test_Class:
    def __init__(self):

        # publishers
        self.test_pub = rospy.Publisher('/mcts/data', String, queue_size = 10) # jsonified data
   	
     	# subscribers
        self.mcts_sub = rospy.Subscriber("/maas/data", String, self.data_callback)

     
        # some other stuff
        self.json_dict = {"data_request": []}

    def test_publish(self):
	
	# the point to estimate 	
	x =1
	y =2

	# reset the data - and send it  	
	self.json_dict = {"data_request": []}
	self.json_dict['data_request'].append({"x": x, "y": y})
	
        # publish action
        self.test_pub.publish(json.dumps(self.json_dict))
    
    def data_callback(self, data):

        # json loads
        temp_data = json.loads(data.data)
	
	print("I got a call back - how exciting!")
	print(data.data)


############################# Main #############################################
def main():
    # init ros node
    rospy.init_node('test', anonymous = True)

    # class instance
    test_instance = Test_Class()

    # create ros loop
    pub_rate = 10 # hertz
    rate = rospy.Rate(pub_rate)

    while (not rospy.is_shutdown()):
        # pack something in a json object
        test_instance.test_publish()

        # ros sleep (sleep to maintain loop rate)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
