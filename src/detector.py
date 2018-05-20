#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from dodo_detector_ros.msg import DetectedObject, DetectedObjectArray
from dodo_detector import KeypointObjectDetector, ObjectDetector, SSDObjectDetector


class Detector:

    def __init__(self):
        frozen_graph = rospy.get_param("/dodo_detector_ros/inference_graph")
        label_map = rospy.get_param("/dodo_detector_ros/label_map")

        if frozen_graph is None:
            raise ValueError('Parameter \'frozen_graph\' must be passed')
        if label_map is None:
            raise ValueError('Parameter \'label_map\' must be passed')

        rospy.loginfo(frozen_graph)
        rospy.loginfo(label_map)

        label_map_contents = open(label_map, 'r').read()
        num_classes = label_map_contents.count('name:')

        self.bridge = CvBridge()
        self.detector = SSDObjectDetector.SSDObjectDetector(frozen_graph, label_map, num_classes)

        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        rospy.Subscriber('/camera/depth_registered/image', Image, self.depth_callback)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pc_callback)

        # # the following subscribers should come from the same Kinect
        # # subscriber for Image messages
        # image_sub = message_filters.Subscriber('/camera/rgb/image_color', Image)
        # # subscriber for PointCloud2
        # pointcloud_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)
        # # synchronize them both into a single callback
        # ts = message_filters.TimeSynchronizer([image_sub, pointcloud_sub], 10)
        # ts.registerCallback(self.callback)

        self._imagepub = rospy.Publisher('detected_objects_image', Image, queue_size=10)
        self._pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=10)
        self._current_image = None
        self._current_depth = None
        self._current_pc = None

    def image_callback(self, image):
        self._current_image = image

    def depth_callback(self, depth):
        self._current_depth = depth

    def pc_callback(self, pc):
        self._current_pc = pc

    def run(self):
        while not rospy.is_shutdown():
            if self._current_image is not None:
                try:
                    # capture image from the subscriber and convert to an OpenCV image
                    scene = self.bridge.imgmsg_to_cv2(self._current_image, "rgb8")
                    marked_image, objects = self.detector.from_image(scene)
                    self._imagepub.publish(self.bridge.cv2_to_imgmsg(marked_image, "rgb8"))
                except CvBridgeError as e:
                    print(e)

                msg_to_send = DetectedObjectArray()

                for obj_class in objects.iterkeys():
                    for coordinates in objects[obj_class]:
                        ymin, xmin, ymax, xmax = coordinates
                        y_center = ymax - ((ymax - ymin) / 2)
                        x_center = xmax - ((xmax - xmin) / 2)

                        mini_msg = DetectedObject()
                        mini_msg.type.data = obj_class

                        if self._current_pc is not None:
                            rospy.loginfo(self._current_pc.height)
                            rospy.loginfo(self._current_pc.width)
                            rospy.loginfo(self._current_pc.point_step)
                            rospy.loginfo(self._current_pc.row_step)
                            rospy.loginfo(self._current_pc.is_dense)
                            for datum in self._current_pc.data:
                                rospy.loginfo(datum)

                        elif self._current_depth is not None:
                            depth = self.bridge.imgmsg_to_cv2(self._current_depth, "passthrough")
                            mini_msg.location.x = x_center
                            mini_msg.location.y = y_center
                            mini_msg.location.z = depth[x_center, y_center]

                        msg_to_send.detected_objects.append(mini_msg)

                self._pub.publish(msg_to_send)

if __name__ == '__main__':
    rospy.init_node('dodo_detector_ros', log_level=rospy.INFO)
    Detector().run()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')