# Object detection from images/point cloud using ROS

This ROS package creates an interface with [dodo detector](https://gitlab.com/douglasrizzo/dodo_detector), a Python package that detects objects from images.

This package makes information regarding detected objects available in a topic, using a special kind of message. 

_(ONGOING)_ When using an OpenNI-compatible sensor (like Kinect) the package also aims to use information from depth perception and the point cloud to position objects in the world, regarding the sensor.

## Installation

Check [dodo detector](https://gitlab.com/douglasrizzo/dodo_detector)'s README file for a list of dependencies unrelated to ROS. Other dependencies are listed on `package.xml` and `CMakeLists.txt`.

## Usage

Edit `launch/dodo_detector_ros.launch` to point the `inference_graph` and `label_map` parameters to your corresponding files. These are files created when training an object detection neural network using [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection).