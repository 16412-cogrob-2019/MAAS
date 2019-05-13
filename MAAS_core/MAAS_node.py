#!/usr/bin/env python

# imports
import sys
import numpy as np
import json
from MAAS_core import suggest_points
from mission_controller.msg import ActivityDone
from bayes_opt import UtilityFunction
from nav_msgs.msg import OccupancyGrid
from map import Map

# ros imports
import rospy
from std_msgs.msg import String


class Position():
    def __init__(self, x, y):
        self.x = x
        self.y = y


class MAAS_node:

    def __init__(self, x_low=-0.27, x_up=1.9, y_low=-2.5, y_high=0.3, dim_x=2, dim_y=2):
        # subscribers
        self.sample_sub = rospy.Subscriber('/activity/done', ActivityDone, self.sampled_data_callback, queue_size=10)
        # subscribe to the map (which publishes occupancy grid)
        namespace = rospy.get_param('namespace')
        self.map_sub = rospy.Subscriber(namespace + "/map", OccupancyGrid, self.map_callback)

        # publishers
        self.maas_pub = rospy.Publisher('/maas/poi_data', String, queue_size=10)  # jsonified data

        # number of active agents
        self.num_of_agents = rospy.get_param('n_agents')

        # number of POI points to publish
        self.num_of_points = rospy.get_param('n_pois')

        # data samples collected so far
        self.json_dict_samples = {"sample_values": []}

        # current points of interest
        self.POIs = {"POIs": []}

        # number of POI points to publish
        self.agent_locations = {"agent_locs": []}

        # setting the sampling bounds
        self.x_low = x_low
        self.x_up = x_up
        self.y_low = y_low
        self.y_high = y_high

        # the sampling grid
        self.dim_x = dim_x
        self.dim_y = dim_y

        # the occupancy grid
        self.occ_grid = None

    ############################# Subscriber Callback functions ####################

    def sampled_data_callback(self, data):
        """
        :type data: ActivityDone
        """
        rospy.loginfo("receieved sample data")
        # message = json.loads(data.data)
        activity_id = data.activity_id  # message["activity_id"]
        activity_name = data.activity_name  # message["activity_name"]
        samples = data.samples  # message["samples"]
        samples_x_vals = data.x_vals  # message["x_vals"]
        samples_y_vals = data.y_vals  # message["y_vals"]

        for agent_id in xrange(len(samples)):
            cur_sample = samples[agent_id]
            cur_sample_x = samples_x_vals[agent_id]
            cur_sample_y = samples_y_vals[agent_id]

            self.json_dict_samples['sample_values'].append(
                {"x": cur_sample_x, "y": cur_sample_y, "sample_value": cur_sample})

        print(self.json_dict_samples['sample_values'])
        self.publish_points()

    ############################# Publisher functions ##############################
    def compute_POIs(self):
        # reset map
        self.POIs = {"POIs": []}
        # print("compute")

        pbounds = {'x': (self.x_low, self.x_up), 'y': (self.y_low, self.y_high)}
        # pbounds = {'x': (0, dim_x), 'y': (0, dim_y)}
        num_suggested_points = self.num_of_points
        samples = [({'x': s['x'], 'y': s['y']}, s['sample_value']) for s in self.json_dict_samples['sample_values']]
        print(samples)
        utility = UtilityFunction(kind="ucb", kappa=100, xi=0.0)

        suggested_points = suggest_points(num_suggested_points, utility, pbounds, samples)
        for i, point in enumerate(suggested_points):
            # test occupency grid
            occupency_val = self.map.get_cell_val(point['x'], point['y'])
            # occupency_val = -888
            rospy.loginfo('occupency_val of suggested point is: %d' % occupency_val)
            if occupency_val < 0.1:
                self.POIs['POIs'].append({"x": point['x'], "y": point['y'],
                                          # "z": 0,
                                          "poi_id": i, "poi_reward": point['reward']})
        # print(self.POIs)

    def publish_points(self):
        rospy.loginfo("computing new POI's")
        # compute k best points
        self.compute_POIs()
        # publish action
        self.maas_pub.publish(json.dumps(self.POIs['POIs']))

    # MAP DATA
    def map_callback(self, msg):
        rospy.loginfo("[MAAS] Received map data!")
        self.map = Map(msg)


############################# Main #############################################
def main():
    # init ros node
    rospy.init_node('MAAS', anonymous=True)

    # class instance
    MAAS_instance = MAAS_node()

    # create ros loop
    pub_rate = 0.5  # hertz
    rate = rospy.Rate(pub_rate)

    while (not rospy.is_shutdown()):
        # do some stuff if necessary
        rospy.logdebug("MAAS node is alive...")
        # MAAS_instance.publish_points()

        # ros sleep (sleep to maintain loop rate)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
