#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraProcessingNode(DTROS):

    def __init__(self, node_name):
        super(CameraProcessingNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._bridge = CvBridge()
        self._window = "camera-processor"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        
        # Create subscriber to camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        
        # Create a publisher for the annotated image
        # Change to compressed transport for annotated image
        self._publisher = rospy.Publisher(f"/{self._vehicle_name}/camera_node/image/annotated_image/compressed", CompressedImage, queue_size=10)

        # self._publisher = rospy.Publisher(f"/{self._vehicle_name}/camera_node/image/annotated_image", CompressedImage, queue_size=10)

    def callback(self, msg):
        # Convert JPEG bytes to OpenCV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Print image size (height, width)
        height, width = image.shape[:2]
        rospy.loginfo(f"Image size: {width}x{height}")
        
        # Convert to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Annotate the grayscale image
        annotation_text = f"Duck {self._vehicle_name} says, 'Cheese! Capturing {width}x{height} – quack-tastic!'"
        # rospy.loginfo(annotation_text)
        cv2.putText(gray_image, annotation_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        # Convert back to CompressedImage message
        annotated_image_msg = self._bridge.cv2_to_compressed_imgmsg(gray_image)
        
        # Publish the annotated image
        self._publisher.publish(annotated_image_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = CameraProcessingNode(node_name='camera_processing_node')
    node.run()
