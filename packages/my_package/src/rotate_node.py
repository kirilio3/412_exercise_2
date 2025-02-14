#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
import time
import rosbag

# Robot parameters
WHEEL_RADIUS = 0.0325  # in meters
distance_between_wheels = 0.07  # in meters

# Desired turn angle (90 degrees)
TURN_ANGLE = 90  # degrees

# Compute required turn duration
angular_velocity = 3  # rad/s (adjust as needed)
turn_duration = (TURN_ANGLE * (3.14159265 / 180)) / angular_velocity

# Reduce duration for counterclockwise turn
counter_turn_duration = turn_duration * 0.95  # Adjust this factor if necessary

class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.on_shutdown(self.on_shutdown)  # Register shutdown callback
        # static parameters
        self.vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        
        # Open the rosbag file for writing
        self.bag = rosbag.Bag('csc22911_rotate_odometry.bag', 'w')
        self.bag_closed = False  # Flag to check if the bag is closed

    def turn(self, left_speed, right_speed, duration):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            message = WheelsCmdStamped(vel_left=left_speed, vel_right=right_speed)
            self._publisher.publish(message)
            
            if not self.bag_closed:
                # Write the message to the rosbag if it's open
                self.bag.write(f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd', message)
            
            rate.sleep()
        
        # Stop the robot after turn
        stop_message = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop_message)
        
        if not self.bag_closed:
            # Write the stop message to the rosbag
            self.bag.write(f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd', stop_message)

        time.sleep(2)  # Pause before returning

    def run(self):
        # Turn 90 degrees clockwise
        self.turn(0.5, -0.5, turn_duration)
        # Turn 90 degrees counterclockwise (return to original position)
        self.turn(-0.5, 0.5, counter_turn_duration)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)
        
        if not self.bag_closed:
            # Write the stop message to the rosbag
            self.bag.write(f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd', stop)
        
        # Close the rosbag file when shutting down
        self.bag.close()
        self.bag_closed = True  # Mark the bag as closed

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
