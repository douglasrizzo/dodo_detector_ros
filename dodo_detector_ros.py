#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dodo_detector import KeypointObjectDetector, ObjectDetector, SSDObjectDetector
from openni import Context, DepthGenerator

class object_detection:

    def __init__(self):
        frozen_graph, label_map, num_classes
        self.detector = SSDObjectDetector.SSDObjectDetector(frozen_graph, label_map, num_classes)


        ctx = pyopenni.Context()
        ctx.init()

        # Create a depth generator
        depth = pyopenni.DepthGenerator()
        depth.create(ctx)

        # Set it to VGA maps at 30 FPS
        depth.set_resolution_preset(RES_VGA)
        depth.fps = 30

        # Start generating
        ctx.start_generating_all()

        rospy.Subscriber('/camera/rgb/image_color', Image, self.callback, queue_size=1)


    def callback(self, data):
        try:
            # capture image from the subscriber and convert to an OpenCV image
            scene = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        marked_image, objects = self.detector.from_image(scene)

        # Update to next frame
        nRetVal = ctx.wait_one_update_all(depth)
        depthMap = depth.map

        # Get the coordinates of the middle pixel
        x = depthMap.width / 2
        y = depthMap.height / 2
        
        # Get the pixel at these coordinates
        pixel = depthMap[x,y]

        print "The middle pixel is %d millimeters away." % pixel
