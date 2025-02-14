#!/usr/bin/env python3

import os
import rospy
import rosbag
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
import math

class StraightLineTask(DTROS):
    def __init__(self, node_name):
        super(StraightLineTask, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Get vehicle name
        self.vehicle_name = os.environ['VEHICLE_NAME']
        
        # Publisher for velocity commands
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # Encoder subscriber (left wheel as example)
        left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
        self._right_encoder_sub = rospy.Subscriber(right_encoder_topic, 
                                                  WheelEncoderStamped, 
                                                  self.cb_right_encoder)
        self._left_encoder_sub = rospy.Subscriber(left_encoder_topic, 
                                                  WheelEncoderStamped, 
                                                  self.cb_left_encoder)
        
        # Distance traveled (forward is +, backward is -)
        self._left_distance_traveled = 0.0
        self._right_distance_traveled = 0.0
        self._last_left_encoder_ticks = None
        self._last_right_encoder_ticks = None
        
        # Wheel/encoder parameters (these may vary for your Duckiebot)
        self.TICKS_PER_REV = 135    # typical for standard Duckietown wheel encoders
        self.WHEEL_RADIUS  = 0.0318 # meters (approx. radius for Duckiebot wheels)
        self.WHEEL_CIRCUM  = 2.0 * math.pi * self.WHEEL_RADIUS  # meters per revolution
        
        # Target distances
        self.FORWARD_DISTANCE  = 1.25
        self.BACKWARD_DISTANCE = -1.25
        
        # Driving speed
        self.VELOCITY = 0.3  # m/s
        self.OMEGA = -0.02    # no rotation, going straight

        # Initialize rosbag file to record the velocity commands
        self.bag = rosbag.Bag('csc22911_straight_line_odometry.bag', 'w')

    def encoder_helper(self, msg):
        self.cb_left_encoder(msg)
        self.cb_right_encoder(msg)

    def cb_left_encoder(self, msg):
        """
        Callback for left wheel encoder ticks.
        WheelEncoderStamped has:
          - msg.data: current encoder tick count (absolute, wraps around)
          - msg.resolution: ticks per revolution (sometimes duplicates self.TICKS_PER_REV)
        """
        if self._last_left_encoder_ticks is None:
            # First reading, just store and do nothing
            self._last_left_encoder_ticks = msg.data
            return
        
        # Compute how many ticks since last update
        delta_ticks = msg.data - self._last_left_encoder_ticks
        
        # Handle wrap-around (encoder can reset or roll over)
        if delta_ticks > (self.TICKS_PER_REV / 2):
            # Possibly rolled over from max -> 0
            delta_ticks -= self.TICKS_PER_REV
        elif delta_ticks < -(self.TICKS_PER_REV / 2):
            # Possibly rolled over from 0 -> max
            delta_ticks += self.TICKS_PER_REV
        
        # Update last tick reading
        self._last_left_encoder_ticks = msg.data
        
        # Convert ticks to distance (one wheel revolution = WHEEL_CIRCUM meters)
        # delta_distance = (# of revolutions) * circumference
        # # of revolutions = delta_ticks / TICKS_PER_REV
        revolutions = float(delta_ticks) / self.TICKS_PER_REV
        distance = revolutions * self.WHEEL_CIRCUM

        # Accumulate distance. Note: If your robot is truly going forward, distance is +,
        # but if your code commands negative velocity, the ticks may still go up or down
        # depending on how the encoder is physically mounted.
        # You may need to invert the sign if the measurement is opposite your commanded direction.
        self._left_distance_traveled += distance

    def cb_right_encoder(self, msg):
        """
        Callback for left wheel encoder ticks.
        WheelEncoderStamped has:
          - msg.data: current encoder tick count (absolute, wraps around)
          - msg.resolution: ticks per revolution (sometimes duplicates self.TICKS_PER_REV)
        """
        if self._last_right_encoder_ticks is None:
            # First reading, just store and do nothing
            self._last_right_encoder_ticks = msg.data
            return
        
        # Compute how many ticks since last update
        delta_ticks = msg.data - self._last_right_encoder_ticks
        
        # Handle wrap-around (encoder can reset or roll over)
        if delta_ticks > (self.TICKS_PER_REV / 2):
            # Possibly rolled over from max -> 0
            delta_ticks -= self.TICKS_PER_REV
        elif delta_ticks < -(self.TICKS_PER_REV / 2):
            # Possibly rolled over from 0 -> max
            delta_ticks += self.TICKS_PER_REV
        
        # Update last tick reading
        self._last_right_encoder_ticks = msg.data
        
        # Convert ticks to distance (one wheel revolution = WHEEL_CIRCUM meters)
        # delta_distance = (# of revolutions) * circumference
        # # of revolutions = delta_ticks / TICKS_PER_REV
        revolutions = float(delta_ticks) / self.TICKS_PER_REV
        distance = revolutions * self.WHEEL_CIRCUM

        # Accumulate distance. Note: If your robot is truly going forward, distance is +,
        # but if your code commands negative velocity, the ticks may still go up or down
        # depending on how the encoder is physically mounted.
        # You may need to invert the sign if the measurement is opposite your commanded direction.
        self._right_distance_traveled += distance

    def run(self):
        rospy.sleep(2.0)  # small pause before starting
        
        # 1) Move forward until we have traveled +1.25 m
        rospy.loginfo("Moving forward 1.25 meters...")
        forward_msg = Twist2DStamped(v=self.VELOCITY, omega=self.OMEGA)
        self._publisher.publish(forward_msg)
        
        # Write forward message to rosbag
        self.bag.write(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", forward_msg)
        
        # Wait until distance >= +1.25
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if (self._right_distance_traveled + self._left_distance_traveled)/2 >= self.FORWARD_DISTANCE:
                break
            self._publisher.publish(forward_msg)
            self.bag.write(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", forward_msg)  # Write every iteration
            rate.sleep()
        
        # Stop
        rospy.loginfo("Stopping after forward motion...")
        self.stop_robot()
        rospy.sleep(2.0)
        
        # 2) Move backward until we have traveled back to 0 - 1.25 => -1.25 from the *start* OR from the forward point
        #    Usually we measure total distance from an initial reference, so you have to decide how you measure backward distance.
        #    Option A: Reset the distance traveled to 0 and measure again.
        #    Option B: Keep the same distance traveled, so the stopping threshold is (initial + forward_distance) + backward_distance
        # 
        # Let's do Option A for clarity: reset distance, then move until -1.25
        self._right_distance_traveled = 0.0
        self._left_distance_traveled = 0.0
        self._last_left_encoder_ticks = None
        
        rospy.loginfo("Moving backward 1.25 meters...")
        backward_msg = Twist2DStamped(v=-self.VELOCITY, omega=self.OMEGA)
        self._publisher.publish(backward_msg)

        # Write backward message to rosbag
        self.bag.write(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", backward_msg)
        

        while not rospy.is_shutdown():
            # Here, if we’re going backward, the encoder might just keep increasing 
            # or decreasing—depends on hardware. 
            # We are expecting self._distance_traveled to become negative if the 
            # encoder setup is consistent with forward being positive. 
            # So we check if distance <= BACKWARD_DISTANCE (-1.25).
            rospy.loginfo(f"Distance traveled: {self._right_distance_traveled + self._left_distance_traveled}")
            if (self._right_distance_traveled + self._left_distance_traveled)/2 <= self.BACKWARD_DISTANCE:
                break
            self._publisher.publish(backward_msg)
            self.bag.write(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", backward_msg)  # Write every iteration
            rate.sleep()
        
        # Stop
        rospy.loginfo("Stopping after backward motion...")
        self.stop_robot()
        
    def stop_robot(self):
        stop_msg = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop_msg)
        self.bag.write(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", stop_msg)  # Write stop message


    def on_shutdown(self):
        # Ensure robot stops
        self.stop_robot()
        self.bag.close()  # Close the rosbag file to save it
        super(StraightLineTask, self).on_shutdown()

if __name__ == '__main__':
    node = StraightLineTask(node_name='straight_line_task')
    node.run()
    # on_shutdown() will be called automatically by DTROS on exit
