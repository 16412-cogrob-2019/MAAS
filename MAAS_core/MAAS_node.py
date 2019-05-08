#!/usr/bin/env python

# imports
import sys
import numpy as np
import json
from MAAS_core import suggest_points
from mission_controller.msg import ActivityDone
from bayes_opt import UtilityFunction

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
        """
        :type data: ActivityDone
        """
        print("receieved sample data")
        # message = json.loads(data.data)
        activity_id = data.activity_id #message["activity_id"]
        activity_name = data.activity_name # message["activity_name"]
        samples = data.samples # message["samples"]
        samples_x_vals = data.x_vals #  message["x_vals"]
        samples_y_vals = data.y_vals # message["y_vals"]

        for agent_id in xrange(len(samples)):
            cur_sample = samples[agent_id]
            cur_sample_x = samples_x_vals[agent_id]
            cur_sample_y = samples_y_vals[agent_id]

            self.json_dict_samples['sample_values'].append(
                {"x": cur_sample_x, "y": cur_sample_y, "sample_value": cur_sample})

        print(self.json_dict_samples['sample_values'])

    ############################# Publisher functions ##############################
    def compute_POIs(self):
        # reset map
        self.POIs = {"POIs": []}
        # print("compute")

        dim_x, dim_y = 2, 2
        pbounds = {'x': (0, dim_x), 'y': (0, dim_y)}
        num_suggested_points = self.num_of_points
        samples = [({'x': s['x'], 'y': s['y']}, s['sample_value']) for s in self.json_dict_samples['sample_values']]
        print(samples)
        utility = UtilityFunction(kind="ucb", kappa=100, xi=0.0)

        suggested_points = suggest_points(num_suggested_points, utility, pbounds, samples)
        for i, point in enumerate(suggested_points):
            self.POIs['POIs'].append({"x": point['x'], "y": point['y'],
                                      #"z": 0,
                                      "poi_id": i, "poi_reward": point['reward']})
        # print(self.POIs)

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
