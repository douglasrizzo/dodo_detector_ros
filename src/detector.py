#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
import tf

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from dodo_detector_ros.msg import DetectedObject, DetectedObjectArray
from dodo_detector.detection import SingleShotDetector


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
        rospy.loginfo('Number of classes: ' + str(num_classes))

        # create detector
        self.detector = SingleShotDetector(frozen_graph, label_map, num_classes)
        self.bridge = CvBridge()

        # image and point cloud subscribers
        # and variables that will hold their values
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pc_callback)
        self._current_image = None
        self._current_pc = None

        # publisher for frames with detected objects
        self._imagepub = rospy.Publisher('detected_objects_image', Image, queue_size=10)
        #  publisher for object locations
        self._pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=10)
        
        self._tfpub = tf.TransformBroadcaster()

    def image_callback(self, image):
        """Image callback"""
        # Store value on a private attribute
        self._current_image = image

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

                msg = DetectedObjectArray()

                # iterate over the dictionary of detected objects
                for obj_class in objects.iterkeys():
                    rospy.logdebug("Found " + str(len(objects[obj_class])) + " object(s) of type " + obj_class)
                    for obj_type_index, coordinates in enumerate(objects[obj_class]):
                        ymin, xmin, ymax, xmax = coordinates
                        y_center = ymax - ((ymax - ymin) / 2)
                        x_center = xmax - ((xmax - xmin) / 2)

                        detected_object = DetectedObject()
                        detected_object.type.data = obj_class

                        # TODO the timestamp of image, depth and point cloud should be checked
                        # to make sure we are using synchronized data...

                        if self._current_pc is None:
                            rospy.logwarn('No point cloud information available to track object in scene')

                        # if we have a point cloud, we'll try and find
                        # the location of the object in space using it
                        else:
                            # this function gives us a generator of points.
                            # we ask for a single point in the center of our object.
                            pc_list = list(pc2.read_points(self._current_pc,
                                skip_nans=True,
                                field_names = ("x", "y", "z"),
                                uvs=[(x_center, y_center)]))

                            if len(pc_list) > 0:
                                # this is the location of our object in space
                                detected_object.location.x, detected_object.location.y, detected_object.location.z = pc_list[0]

                                self._tfpub.sendTransform((detected_object.location.x,
                                                           detected_object.location.y,
                                                           detected_object.location.z),
                                                        tf.transformations.quaternion_from_euler(0, 0, 0),
                                                        rospy.Time.now(),
                                                        obj_class + "_" + str(obj_type_index),
                                                        "kinect")

                        msg.detected_objects.append(detected_object)

                self._pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('dodo_detector_ros', log_level=rospy.INFO)
    try:
        Detector().run()
        # rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')