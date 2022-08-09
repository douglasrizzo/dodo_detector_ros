#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy
import tf

from os.path import expanduser
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from dodo_detector.detection import TFObjectDetectorV1, TFObjectDetectorV2, KeypointObjectDetector
from dodo_detector_ros.msg import DetectedObject, DetectedObjectArray


class Detector:

   def __init__(self):
      #  get label map and inference graph from params
      detector_type = rospy.get_param('~detector_type')
      frozen_graph = rospy.get_param('~inference_graph', '')
      saved_model_path = rospy.get_param('~saved_model', '')
      label_map = rospy.get_param('~label_map', '')
      confidence = rospy.get_param('~tf_confidence', 0.5)
      min_points = rospy.get_param('~sift_min_pts', 10)
      database_path = rospy.get_param('~sift_database_path', '')
      filters = rospy.get_param('~filters', {})
      image_topic = rospy.get_param('~image_topic')
      point_cloud_topic = rospy.get_param('~point_cloud_topic', None)

      self._global_frame = rospy.get_param('~global_frame', None)
      self._tf_prefix = rospy.get_param('~tf_prefix', rospy.get_name())

      # create a transform listener so we get the fixed frame the user wants
      # to publish object tfs related to
      self._tf_listener = tf.TransformListener()

      if detector_type in ['tf1', 'tf2']:
         if detector_type == 'tf2':
            rospy.loginfo('Chosen detector type: TensorFlow 2')
            if len(saved_model_path) == 0:
               raise ValueError('Parameter \'saved_model\' must be passed')

            saved_model_path = expanduser(saved_model_path)

            self._detector = TFObjectDetectorV2(saved_model_path, label_map, confidence=confidence)
            rospy.loginfo('Path to saved model directory: ' + saved_model_path)

         if detector_type == 'tf1':
            rospy.loginfo('Chosen detector type: TensorFlow 1')
            if len(frozen_graph) == 0:
               raise ValueError('Parameter \'frozen_graph\' must be passed')

            frozen_graph = expanduser(frozen_graph)

            self._detector = TFObjectDetectorV1(frozen_graph, label_map, confidence=confidence)
            rospy.loginfo('Path to inference graph: ' + frozen_graph)

         if len(label_map) == 0:
            raise ValueError('Parameter \'label_map\' must be passed')
         if confidence <= 0 or confidence > 1:
            raise ValueError('Parameter \'confidence\' must be between 0 and 1')

         label_map = expanduser(label_map)
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
         self._detector = KeypointObjectDetector(database_path,
                                                 detector_type,
                                                 min_points=min_points)
         rospy.loginfo('Database path: ' + database_path)
         rospy.loginfo('Min. points: ' + str(min_points))

      # create detector
      self._bridge = CvBridge()

      # image and point cloud subscribers
      # and variables that will hold their values
      rospy.Subscriber(image_topic, Image, self.image_callback)

      if point_cloud_topic is not None:
         rospy.Subscriber(point_cloud_topic, PointCloud2, self.pc_callback)
      else:
         rospy.loginfo(
            'No point cloud information available. Objects will not be placed in the scene.')

      self._current_image = None
      self._current_pc = None

      # publisher for frames with detected objects
      self._imagepub = rospy.Publisher('~labeled_image', Image, queue_size=10)

      # this package works with a dynamic list of publishers
      # if no filter is configured via parameters to the package,
      # one default, unfiltered publisher will publish every object
      if len(filters) == 0:
         rospy.loginfo('No filter configured, publishing every detected object in a single topic')
         self._publishers = {
            None: (None, rospy.Publisher('~detected', DetectedObjectArray, queue_size=10))}

      # else, for each filter created in the yaml config file, a new publisher is created
      else:
         self._publishers = {}
         for key in filters:
            cat_ok = False
            for cat in self._detector.categories:
               if cat in filters[key]:
                  cat_ok = True
                  break

            if not cat_ok:
               rospy.logwarn('Key ' + filters[key] + ' is not detected by this detector!')

            else:
               self._publishers[key] = (filters[key],
                                        rospy.Publisher('~detected_' + key,
                                                        DetectedObjectArray,
                                                        queue_size=10))
               rospy.loginfo('Created topic for filter [' + key + ']')

      self._tfpub = tf.TransformBroadcaster()
      rospy.loginfo('Ready to detect!')

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

               # if the user passes a fixed frame, we'll ask for transformation
               # vectors from the camera link to the fixed frame
               if self._global_frame is not None:
                  (trans, _) = self._tf_listener.lookupTransform('/' + self._global_frame,
                                                                 '/camera_link',
                                                                 rospy.Time(0))

               # convert image from the subscriber into an OpenCV image
               scene = self._bridge.imgmsg_to_cv2(self._current_image, 'rgb8')
               marked_image, objects = self._detector.from_image(scene)  # detect objects
               self._imagepub.publish(self._bridge.cv2_to_imgmsg(
                  marked_image, 'rgb8'))  # publish detection results

               # well create an empty msg for each publisher
               msgs = {}
               for key in self._publishers:
                  msgs[key] = DetectedObjectArray()

               # iterate over the dictionary of detected objects
               for obj_class in objects:
                  rospy.logdebug('Found ' + str(len(objects[obj_class])) + ' object(s) of type ' +
                                 obj_class)

                  for obj_type_index, coordinates in enumerate(objects[obj_class]):
                     rospy.logdebug('...' + obj_class + ' ' + str(obj_type_index) + ' at ' +
                                    str(coordinates['box']))

                     ymin, xmin, ymax, xmax = coordinates['box']

                     detected_object = DetectedObject()
                     detected_object.type.data = obj_class
                     detected_object.image_x.data = xmin
                     detected_object.image_y.data = ymin
                     detected_object.image_width.data = xmax - xmin
                     detected_object.image_height.data = ymax - ymin
                     # TODO the timestamp of image, depth and point cloud should be checked
                     # to make sure we are using synchronized data...

                     publish_tf = False
                     if self._current_pc is None:
                        rospy.loginfo(
                           'No point cloud information available to track current object in scene')

                     # if there is point cloud data, we'll try to place a tf
                     # in the object's location
                     else:
                        y_center = round(ymax - ((ymax - ymin) / 2))
                        x_center = round(xmax - ((xmax - xmin) / 2))
                        # this function gives us a generator of points.
                        # we ask for a single point in the center of our object.
                        pc_list = list(
                           pc2.read_points(self._current_pc,
                                           skip_nans=True,
                                           field_names=('x', 'y', 'z'),
                                           uvs=[(x_center, y_center)]))

                        if len(pc_list) > 0:
                           publish_tf = True
                           # this is the location of our object in space
                           tf_id = obj_class + '_' + str(obj_type_index)

                           # if the user passes a tf prefix, we append it to the object tf name here
                           if self._tf_prefix is not None:
                              tf_id = self._tf_prefix + '/' + tf_id

                           detected_object.tf_id.data = tf_id

                           point_x, point_y, point_z = pc_list[0]

                     for key in self._publishers:
                        # add the object to the unfiltered publisher,
                        # as well as the ones whose filter include this class of objects
                        if key is None or obj_class in self._publishers[key][0]:
                           msgs[key].detected_objects.append(detected_object)

                     # we'll publish a TF related to this object only once
                     if publish_tf:
                        # kinect here is mapped as camera_link
                        # object tf (x, y, z) must be
                        # passed as (z,-x,-y)
                        object_tf = [point_z, -point_x, -point_y]
                        frame = 'camera_link'

                        # translate the tf in regard to the fixed frame
                        if self._global_frame is not None:
                           object_tf = numpy.array(trans) + object_tf
                           frame = self._global_frame

                        # this fixes #7 on GitHub, when applying the
                        # translation to the tf creates a vector that
                        # RViz just can'y handle
                        if object_tf is not None:
                           self._tfpub.sendTransform((object_tf),
                                                     tf.transformations.quaternion_from_euler(
                                                        0, 0, 0),
                                                     rospy.Time.now(),
                                                     tf_id,
                                                     frame)

               # publish all the messages in their corresponding publishers
               for key in self._publishers:
                  self._publishers[key][1].publish(msgs[key])
            except CvBridgeError as e:
               print(e)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
               print(e)


if __name__ == '__main__':
   rospy.init_node('dodo_detector_ros', log_level=rospy.INFO)

   try:
      Detector().run()
   except KeyboardInterrupt:
      rospy.loginfo('Shutting down')
