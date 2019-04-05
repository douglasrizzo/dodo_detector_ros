# Object detection from images/point cloud using ROS

This ROS package creates an interface with [dodo detector](https://github.com/douglasrizzo/dodo_detector), a Python package that detects objects from images.

This package makes information regarding detected objects available in a topic, using a special kind of message. 

When using an OpenNI-compatible sensor (like Kinect) the package uses point cloud information to locate objects in the world, wrt. to the sensor.

Click the image below for a YouTube video showcasing the package at work.

[![Youtube video](https://img.youtube.com/vi/fXJYmJOaSxQ/0.jpg)](https://www.youtube.com/watch?v=fXJYmJOaSxQ)

## Installation

Check [dodo detector](https://github.com/douglasrizzo/dodo_detector)'s README file for a list of dependencies unrelated to ROS. Other dependencies are listed on `package.xml`.

## Usage

### Configuring a detector

To configure the detector that will be used by the package as well as to point the package to the artifacts each detector expects, edit `config/main_config.yaml`.

 - `global_frame`: the frame or tf that all object tfs will be published in relation to, eg `map`. Leave blank to publish wrt. camera_link
 - `tf_prefix`: a prefix for the object tfs which will be published by the package
 - `detector_type`: either 'sift', 'rootsift' or 'ssd'
 - `inference_graph`: path to TensorFlow Object Detection API frozen inference graph, the `.pb` file
 - `label_map`: path to TensorFlow Object Detection API label map, the `.pbtxt`
 - `ssd_confidence`: confidence level to report objects as detected by the neural network, between 0 and 1
 - `sift_min_pts`: minimum number of points to consider an object as present in the scene
 - `sift_database_path`: path to the database used by the keypoint object detector

### Image and point-cloud topics

The package detects objects from images provided by a topic, whose name is given through the `image_topic` parameter.

If you want the package to position the detected objects in space, pass a point cloud topic to the `point_cloud_topic` parameter. Please note that the point cloud must be provided by the same sensor that provides the images that are being used to detect the objects (such as a Kinect).

### Launch files

The package provides 3 launch files:

 - `detect_camera.launch` detects objects from the `image_raw` topic of a webcam or other camera compatible with the *uvc_camera* package;
 - `detect_kinect.launch` detects objects using a Kinect. It uses the *freenect* package to start the sensor and subscribes to `camera/rgb/image_color` for images and `/camera/depth/points` for the point cloud;
 - `detect_kinect2.launch` detects objects using a Kinect for Xbox One. It uses [libfreenect2](https://github.com/OpenKinect/libfreenect2) and [iai_kinect2](https://github.com/code-iai/iai_kinect2) to connect with the device and subscribes to `/kinect2/hd/image_color` for images and `/kinect2/hd/points` for the point cloud. You can copy the launch file and use the `sd` and `qhd` topics if you need more performance.

The topics that subscribe to point clouds publish TFs for each detected object.

## Functionality

The package works in two steps. First, it detects objects by using a video feed and then it uses a point cloud to publish TFs for each detected object.

Object detection is done via the [dodo detector](https://github.com/douglasrizzo/dodo_detector) package. In case the single-shot detector is to be used, point the `inference_graph` and `label_map` parameters to your corresponding files. These files are created when training an object detection neural network using [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection). Also, the `ssd_confidence` parameter can be changed to adjust the detection threshold of the network.

In case the keypoint-based detector is to be used (either SIFT or RootSIFT), you need to create a database directory. The procedure to do so is described [here](https://douglasrizzo.github.io/dodo_detector/#keypoint-based-detector).
