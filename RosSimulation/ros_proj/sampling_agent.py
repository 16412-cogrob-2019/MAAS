import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
import sys
import numpy as np
import json
import simulator

class SamplingAgent:
    """
    This is an class for a robot that goes forward and stops when it senses a bump
    """

    def __init__(self):
        """
        The __init__() function is run automatically when an object is created --
        in this case, when you call the function GoForwardAvoidBump()
        """

	# some other stuff
        self.json_dict = {"sample": []}

	self.agent_id = 1 	
	self.x = 1
	self.y = 2
	self.z = 4

        # Boolean variables for sampling taks 
        self.active_task = False
	self.POI_x = 0
	self.POI_y = 0
	self.POI_z = 0
	self.POI_type = "depth"

        # Linear movement speed
        self.lin_speed = 0.2  # m/s

        # Initiliaze
        rospy.init_node('SamplingAgent', anonymous=False)

        # Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

	# create a publisher for location	
        self.status_pub = rospy.Publisher('/update/status', String, queue_size=10)

	# create a publisher for samples	
        self.sample_pub = rospy.Publisher('/sample/data', String, queue_size=10)

	# Subscribe to move commands
        rospy.Subscriber('/sample/goto', String, self.go_to_waypoint)

        # Create a publisher which can "talk" to TurtleBot wheels and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Subscribe to queues for receiving sensory data
        #rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        self.rate = rospy.Rate(10)

    def go_to_waypoint(self, data):
	message = json.loads(data.data)
	goTo_message = message['GoTo'][0]
	
	# get the agent_id, x and y and z locations of the agent's location, measurement type (e.g. depth), and measurement value 
	agent_id = goTo_message["agent_id"]
	self.POI_x = goTo_message["x"]
	self.POI_y = goTo_message["y"]
	self.POI_z = goTo_message["z"]
	self.POI_type = goTo_message["measurement_type"]
	
	print("I got a command to move")
	print(data)
	if agent_id != self.agent_id:
	    print("This command is not for me :-(")
	else:
	    self.active_task = True
	    print("Preparing to move to: "+ str(self.POI_x) + " " + str(self.POI_y) + " " + str(self.POI_z)+ " " + self.POI_type)		

    def get_sample(self, measurement_type): 
	
	measurement_value = None
 	print("in publish sample")	
	
	if measurement_type == "depth":
	    measurement_value = simulator.caldera_sim_function(self.x, self.y)
	else:
            print("Sorry: only depth supported!")
		
	return measurement_value

    # publish a sample 
    def publish_sample(self):
	
	# when the agent is in the location it is supposed to be in, it takes the relevant sample
	#TODO: add a marginal error 
	print("Fix this !!!") 
           
	if self.x == self.POI_x and self.y == self.POI_y and self.z == self.POI_z:
	    measurement_value = get_sample(self.POI_type)
	    
	    # send a message with the data  	
	    if measurement_value is not None:
	        self.active_task = False	
	        # reset the data - and send it  	
	        self.json_dict = {"sample": []}
	        self.json_dict['sample'].append({"agent_id":self.agent_id, "x": self.x, "y": self.y,  "z": self.z, "measurement_type":measurement_type, "measurement_value":measurement_value})
	
                # publish action
                self.sample_pub.publish(json.dumps(self.json_dict))

    # publish status
    def publish_status(self):
	print("publish location")
	
        self.active_task = False	
        # reset the data - and send it  	
        self.json_dict = {"status": []}

	# Boolean variables for sampling taks 
        self.json_dict['status'].append({"agent_id":self.agent_id, "x": self.x, "y": self.y,  "z": self.z})

        # publish action
        self.status_pub.publish(json.dumps(self.json_dict))



    def run(self):
        """
        Run the control loop until Ctrl+C is pressed 
        :return: None
        """
   
        # Twist is a datatype for velocity
        move_cmd = Twist()

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            move_cmd.linear.x = self.lin_speed
            move_cmd.angular.z = 0
  	    # publish the velocity
            self.cmd_vel.publish(move_cmd)
          
	    # if the agent is on a mission, then it should check if it can sample
	    if self.active_task:
	    	self.publish_sample()	
	    self.publish_status()
            self.rate.sleep()
    
    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(5)


if __name__ == '__main__':
    """
    This runs when the program is run directly from the command line
    but it will not run if you import this module (file) in another file
    """
    try:
        robot = SamplingAgent()
        robot.run()
    except Exception, err:
        rospy.loginfo("GoForwardAvoidBump node terminated.")
        print err
