# Object detection from images/point cloud using ROS

This ROS package creates an interface with [dodo detector](https://gitlab.com/douglasrizzo/dodo_detector), a Python package that detects objects from images.

This package makes information regarding detected objects available in a topic, using a special kind of message. 

When using an OpenNI-compatible sensor (like Kinect) the package uses point cloud information to locate objects in the world, wrt. to the sensor.

## Installation

Check [dodo detector](https://gitlab.com/douglasrizzo/dodo_detector)'s README file for a list of dependencies unrelated to ROS. Other dependencies are listed on `package.xml` and `CMakeLists.txt`.

## Usage

Edit `config/main_config.yaml` to select which type of detector you want.

In case the single-shot detector is to be used, point the `inference_graph` and `label_map` parameters to your corresponding files. These files are created when training an object detection neural network using [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection). Also, the `ssd_confidence` parameter can be changed to adjust the detection threshold of the network.

In case the keypoint-based detector is to be used (either SIFT OR RootSIFT), you need to create a database directory. The procedure to do so is described [here](http://douglasrizzo.gitlab.io/dodo_detector/#keypoint-based-detector).