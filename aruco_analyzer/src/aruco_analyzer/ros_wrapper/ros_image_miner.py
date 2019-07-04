#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from aruco_analyzer.helper_classes.camera import Camera
from aruco_analyzer.helper_classes.camera_image import CameraImage

class ROSImageMiner(object):
    def __init__(self, image_distributor, cameras):
        self.image_distributor = image_distributor
        self.cameras = cameras
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param('/aruco_analyzer/image_topic', '/image_raw')

        self.subscribers = {}
        for camera_name in self.cameras.keys():
            topic_name = '/cameras/'+camera_name + self.image_topic
            subscriber = rospy.Subscriber(
                topic_name, Image, callback=self.image_callback)
            self.subscribers[camera_name] = subscriber

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        camera_image_container = CameraImage(self.cameras[data.header.frame_id], cv_image, data.header.stamp.to_time())
        self.image_distributor.put_image(camera_image_container)
