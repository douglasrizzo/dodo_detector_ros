#!/usr/bin/env python

import rospy
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from dodo_detector_ros import DetectedObject, DetectedObjectArray
from dodo_detector import KeypointObjectDetector, ObjectDetector, SSDObjectDetector

class object_detection:

    def __init__(self):
        # TODO: planejar como a localização desses arquivos será passada
        frozen_graph, label_map
        
        label_map_contents  = open(label_map, 'r').read()
        num_classes = label_map_contents.count('name:')

        self.detector = SSDObjectDetector.SSDObjectDetector(frozen_graph, label_map, num_classes)

        # the following subscribers should come from the same Kinect
        # subscriber for Image messages
        image_sub = message_filters.Subscriber('/camera/rgb/image_color', Image)
        # subscriber for PointCloud2
        pc_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)

        # synchronize them both into a single callback
        ts = message_filters.TimeSynchronizer([image_sub, pc_sub], 10)
        ts.registerCallback(callback)

        self._pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=10)



    def callback(self, image, pointcloud):
        try:
            # capture image from the subscriber and convert to an OpenCV image
            scene = self.bridge.imgmsg_to_cv2(image, "rgb8")
        except CvBridgeError as e:
            print(e)

        marked_image, objects = self.detector.from_image(scene)
        msg_to_send = DetectedObjectArray()

        for obj in objects:
            mini_msg = DetectedObject()
            mini_msg.type = obj[0]
            msg_to_send.append()

        self._pub.publish(msg_to_send)