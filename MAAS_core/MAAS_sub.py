#!/usr/bin/env python

# imports
import sys
import numpy as np
import json

# ros imports
import rospy
from std_msgs.msg import String


class Test_Subscriber:
    def __init__(self):

         # subscribers
        self.mcts_sub = rospy.Subscriber("/maas/data", String, self.data_callback)
     
    
    def data_callback(self, data):

        # json loads
        temp_data = json.loads(data.data)
	
	print("I got a sample - how exciting!")
	print(data.data)


############################# Main #############################################
def main():

    # init ros node
    rospy.init_node('test_subscriber', anonymous = True)

    # class instance
    test_instance = Test_Subscriber()

    # create ros loop
    pub_rate = 10 # hertz
    rate = rospy.Rate(pub_rate)

    while (not rospy.is_shutdown()):
        print("Subsriber is alive")
        # ros sleep (sleep to maintain loop rate)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
