# Object detection from images/point cloud using ROS

This ROS package creates an interface with [dodo detector](https://github.com/douglasrizzo/dodo_detector), a Python package that detects objects from images.

This package makes information regarding detected objects available in a topic, using a special kind of message. 

When using an OpenNI-compatible sensor (like Kinect) the package uses point cloud information to locate objects in the world, wrt. to the sensor.

Click the image below for a YouTube video showcasing the package at work.

[![Youtube video](https://img.youtube.com/vi/fXJYmJOaSxQ/0.jpg)](https://www.youtube.com/watch?v=fXJYmJOaSxQ)

## Installation

Check [dodo detector](https://github.com/douglasrizzo/dodo_detector)'s README file for a list of dependencies unrelated to ROS. Other dependencies are listed on `package.xml`.

## Configuration

The package works in two steps. First, it detects objects by using a video feed and then it uses a point cloud to publish TFs for each detected object.

Object detection is done via the [dodo detector](https://github.com/douglasrizzo/dodo_detector) package. In case the single-shot detector is to be used, point the `inference_graph` and `label_map` parameters to your corresponding files. These files are created when training an object detection neural network using [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection). Also, the `ssd_confidence` parameter can be changed to adjust the detection threshold of the network.

In case the keypoint-based detector is to be used (either SIFT or RootSIFT), you need to create a database directory. The procedure to do so is described [here](https://douglasrizzo.github.io/dodo_detector/#keypoint-based-detector).

Edit `config/main_config.yaml` to select which type of detector you want as well as to point the package to the artifacts each detector expects.

## Usage

There are 3 launch files:

 - `detect_camera.launch` detects objects from the `image_raw` topic of a webcam or other camera compatible with the _uvc_camera_ package;
 - `detect_kinect.launch` detects objects using a Kinect. It uses the _freenect_ and subscribes to `camera/rgb/image_color` for images and `/camera/depth/points` for the point cloud;
 - `detect_kinect2.launch` detects objects using a Kinect for Xbox One. It uses [libfreenect2](https://github.com/OpenKinect/libfreenect2) and [iai_kinect2](https://github.com/code-iai/iai_kinect2) to connect with the device and subscribes to `/kinect2/hd/image_color` for images and `/kinect2/hd/points` for the point cloud. You can copy the launch file and use the `sd` and `qhd` topics if you need more perfomance.

The topics that subscribe to point clouds publish TFs for each detected object.
