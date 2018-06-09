#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf

from os.path import expanduser
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from dodo_detector.detection import SingleShotDetector, KeypointObjectDetector
from dodo_detector_ros.msg import DetectedObject, DetectedObjectArray


class Detector:

    def __init__(self):
        #  get label map and inference graph from params
        detector_type = rospy.get_param("/dodo_detector_ros/detector_type")
        frozen_graph = rospy.get_param("/dodo_detector_ros/inference_graph", '')
        label_map = rospy.get_param("/dodo_detector_ros/label_map", '')
        confidence = rospy.get_param("/dodo_detector_ros/ssd_confidence", 0.5)
        min_points = rospy.get_param("/dodo_detector_ros/sift_min_pts", 10)
        database_path = rospy.get_param("/dodo_detector_ros/sift_database_path", '')

        if detector_type == 'ssd':
            rospy.loginfo('Chosen detector type: Single Shot Detector')
            if len(frozen_graph) == 0:
                raise ValueError('Parameter \'frozen_graph\' must be passed')
            if len(label_map) == 0:
                raise ValueError('Parameter \'label_map\' must be passed')
            if confidence <= 0 or confidence > 1:
                raise ValueError('Parameter \'confidence\' must be between 0 and 1')                

            frozen_graph = expanduser(frozen_graph)
            label_map = expanduser(label_map)

            self.detector = SingleShotDetector(frozen_graph, label_map, confidence=confidence)
            rospy.loginfo('Path to inference graph: ' + frozen_graph)
            rospy.loginfo('Path to label map: ' + label_map)

            # count number of classes from label map
            label_map_contents = open(label_map, 'r').read()
            num_classes = label_map_contents.count('name:')
            rospy.loginfo('Number of classes: ' + str(num_classes))

        elif detector_type in ['sift', 'rootsift']:
            rospy.loginfo('Chosen detector type: Keypoint Object Detector')
            if min_points <= 0:
                raise ValueError('Parameter \'min_points\' must greater than 0')
            if len(database_path) == 0:
                raise ValueError('Parameter \'database_path\' must be passed')

            database_path = expanduser(database_path)

            detector_type = 'SIFT' if detector_type == 'sift' else 'RootSIFT'
            self.detector = KeypointObjectDetector(
                database_path,
                detector_type,
                min_points=min_points)
            rospy.loginfo('Database path: ' + database_path)
            rospy.loginfo('Min. points: ' + min_points)

        # create detector
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
                for obj_class in objects:
                    rospy.logdebug("Found " + str(len(objects[obj_class])) + " object(s) of type " + obj_class)

                    for obj_type_index, coordinates in enumerate(objects[obj_class]):
                        rospy.logdebug("..." + obj_class + " " + str(obj_type_index) + " at " + str(coordinates))
                        
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
                            pc_list = list(pc2.read_points(
                                self._current_pc,
                                skip_nans=True,
                                field_names = ("x", "y", "z"),
                                uvs=[(x_center, y_center)]))

                            if len(pc_list) > 0:
                                # this is the location of our object in space
                                tf_id = obj_class + "_" + str(obj_type_index)
                                detected_object.tf_id.data = tf_id

                                point_x, point_y, point_z = pc_list[0]

                                # kinect here is mapped as camera_link
                                # object tf (x, y, z) must be
                                # passed as (z,-x,-y)
                                self._tfpub.sendTransform(
                                        (point_z,
                                        -point_x,
                                        -point_y),
                                    tf.transformations.quaternion_from_euler(0, 0, 0),
                                    rospy.Time.now(),
                                    tf_id,
                                    "camera_link")

                        msg.detected_objects.append(detected_object)

                self._pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('dodo_detector_ros', log_level=rospy.DEBUG)
    try:
        Detector().run()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
