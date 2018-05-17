#!/usr/bin/env python

import rospy
import message_filters

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from dodo_detector import KeypointObjectDetector, ObjectDetector, SSDObjectDetector

class object_detection:

    def __init__(self):
        # TODO: planejar como a localização desses arquivos será passada
        frozen_graph, label_map, num_classes
        self.detector = SSDObjectDetector.SSDObjectDetector(frozen_graph, label_map, num_classes)

        # the following subscribers should come from the same Kinect
        # subscriber for Image messages
        image_sub = message_filters.Subscriber('/camera/rgb/image_color', Image)
        # subscriber for PointCloud2
        pc_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)

        # synchronize them both into a single callback
        ts = message_filters.TimeSynchronizer([image_sub, pc_sub], 10)
        ts.registerCallback(callback)


    def callback(self, image, pointcloud):
        try:
            # capture image from the subscriber and convert to an OpenCV image
            scene = self.bridge.imgmsg_to_cv2(image, "rgb8")
        except CvBridgeError as e:
            print(e)

        marked_image, objects = self.detector.from_image(scene)

        # TODO: JEFF!!!
        # objetos detectados na imagem, agora usamos a mensagem de pointcloud para fazer o tal vetor que Fafá pediu