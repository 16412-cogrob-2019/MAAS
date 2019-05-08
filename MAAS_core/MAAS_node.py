#!/usr/bin/env python

# imports
import sys
import numpy as np
import json
from mission_controller.msg import ActivityDone

# ros imports
import rospy
from std_msgs.msg import String


class MAAS_node:

    def __init__(self, num_of_points, numb_of_agents):
        # subscribers
        self.sample_sub = rospy.Subscriber('/activity/done', ActivityDone, self.sampled_data_callback, queue_size=10)

        # publishers
        self.maas_pub = rospy.Publisher('/maas/data', String, queue_size=10)  # jsonified data

        # number of POI points to publish
        self.num_of_points = num_of_points

        # data samples collected so far
        self.json_dict_samples = {"sample_values": []}

        # current points of interest
        self.POIs = {"POIs": []}

        # number of POI points to publish
        self.agent_locations = {"agent_locs": []}

    ############################# Subscriber Callback functions ####################

    def sampled_data_callback(self, data):
        print("receieved sample data")
        message = json.loads(data.data)
        activity_id = message["activity_id"]
        activity_name = message["activity_name"]
        samples = message["samples"]
        samples_x_vals = message["x_vals"]
        samples_y_vals = message["y_vals"]

        for agent_id in xrange(len(samples)):
            cur_sample = samples[agent_id]
            cur_sample_x = samples_x_vals[agent_id]
            cur_sample_y = samples_y_vals[agent_id]

            self.json_dict_samples['sample_values'].append(
                {"x": cur_sample_x, "y": cur_sample_y, "sample_value": cur_sample})

        print(self.json_dict['sample_values'])

    ############################# Publisher functions ##############################
    def compute_POIs(self):
        # reset map
        self.POIs = {"POIs": []}
        print("compute")

        for i in xrange(self.num_of_points):
            x = 3.3 + i
            y = 5.2
            z = 3.4
            measurement_type = "depth"
            score = 5.5

            self.POIs['POIs'].append({"x": x, "y": y, "z": z, "measurement_type": measurement_type, "score": score})

    def publish_points(self):

        # compute k best points
        self.compute_POIs()
        # publish action
        self.maas_pub.publish(json.dumps(self.POIs))


############################# Main #############################################
def main():
    # init ros node
    rospy.init_node('MAAS', anonymous=True)

    # class instance
    MAAS_instance = MAAS_node(5, 1)

    # create ros loop
    pub_rate = 1  # hertz
    rate = rospy.Rate(pub_rate)

    while (not rospy.is_shutdown()):
        # do some stuff if necessary
        print("Publish Points of Interest")
        MAAS_instance.publish_points()

        # ros sleep (sleep to maintain loop rate)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
