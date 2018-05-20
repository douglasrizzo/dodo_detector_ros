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
        #  get label map and inference graph from params
        frozen_graph = rospy.get_param("/dodo_detector_ros/inference_graph")
        label_map = rospy.get_param("/dodo_detector_ros/label_map")

        if frozen_graph is None:
            raise ValueError('Parameter \'frozen_graph\' must be passed')
        if label_map is None:
            raise ValueError('Parameter \'label_map\' must be passed')

        rospy.loginfo('Path to inference graph: ' + frozen_graph)
        rospy.loginfo('Path to label map: ' + label_map)

        # count number of classes from label map
        label_map_contents = open(label_map, 'r').read()
        num_classes = label_map_contents.count('name:')
        rospy.loginfo('Number of classes: ' + num_classes)

        # create detector
        self.detector = SSDObjectDetector.SSDObjectDetector(frozen_graph, label_map, num_classes)
        self.bridge = CvBridge()

        # image, depth and point cloud subscribers
        # and variables that will hold their values
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        rospy.Subscriber('/camera/depth_registered/image', Image, self.depth_callback)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pc_callback)
        self._current_image = None
        self._current_depth = None
        self._current_pc = None

        # publisher for frames with detected objects
        self._imagepub = rospy.Publisher('detected_objects_image', Image, queue_size=10)
        #  publisher for object locations
        self._pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=10)

    def image_callback(self, image):
        """Image callback"""
        # Store value on a private attribute
        self._current_image = image

    def depth_callback(self, depth):
        """Depth map callback"""
        # Store value on a private attribute
        self._current_depth = depth

    def pc_callback(self, pc):
        """Point cloud callback"""
        # Store value on a private attribute
        self._current_pc = pc

    def run(self):
        # run while ROS runs
        while not rospy.is_shutdown():
            # only run if there's an image present
            if self._current_image is not None:
                try:
                    # convert image from the subscriber into an OpenCV image
                    scene = self.bridge.imgmsg_to_cv2(self._current_image, "rgb8")
                    marked_image, objects = self.detector.from_image(scene) # detect objects
                    self._imagepub.publish(self.bridge.cv2_to_imgmsg(marked_image, "rgb8")) # publish detection results
                except CvBridgeError as e:
                    print(e)

                msg_to_send = DetectedObjectArray()

                # iterate over the dictionary of detected objects
                for obj_class in objects.iterkeys():
                    for coordinates in objects[obj_class]:
                        ymin, xmin, ymax, xmax = coordinates
                        y_center = ymax - ((ymax - ymin) / 2)
                        x_center = xmax - ((xmax - xmin) / 2)

                        mini_msg = DetectedObject()
                        mini_msg.type.data = obj_class

                        # TODO the timestamp of image, depth and point cloud should be checked
                        # to make sure we are using synchronized data...

                        # if we have a point cloud, we'll try and find
                        # the location of the object in space using it
                        if self._current_pc is not None:
                            # TODO this is here for debug purposes, should be deleted later
                            # actual code is missing too
                            rospy.loginfo(self._current_pc.height)
                            rospy.loginfo(self._current_pc.width)
                            rospy.loginfo(self._current_pc.point_step)
                            rospy.loginfo(self._current_pc.row_step)
                            rospy.loginfo(self._current_pc.is_dense)

                        # else we could try and use the depth map...
                        elif self._current_depth is not None:
                            rospy.logwarn('Point cloud not found, using depth map to find object.')
                            depth = self.bridge.imgmsg_to_cv2(self._current_depth, "passthrough")
                            mini_msg.location.x = x_center
                            mini_msg.location.y = y_center
                            mini_msg.location.z = depth[x_center, y_center]

                        else:
                            rospy.logwarn('No 3D information available to position object in scene')

                        msg_to_send.detected_objects.append(mini_msg)

                self._pub.publish(msg_to_send)

if __name__ == '__main__':
    rospy.init_node('dodo_detector_ros', log_level=rospy.INFO)
    Detector().run()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')