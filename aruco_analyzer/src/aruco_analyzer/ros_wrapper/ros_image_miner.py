#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from aruco_analyzer.util import Camera
from aruco_analyzer.util import CameraImage


class ROSImageMiner(object):
    def __init__(self, image_distributor, cameras):        
        self.image_distributor = image_distributor
        self.cameras = cameras
        self.bridge = CvBridge()
        self.transport_hint = rospy.get_param('/aruco_analyzer/transport_hint', '/image_raw')
        self.camera_ns = rospy.get_param('/aruco_analyzer/camera_ns', '/cameras')

        self.subscribers = {}
        for camera_name in self.cameras.keys():
            topic_name = self.camera_ns + '/'+ camera_name + self.transport_hint
            subscriber = rospy.Subscriber(
                topic_name, Image, callback=self.image_callback)
            self.subscribers[camera_name] = subscriber

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        for camera in self.cameras.values():
            if data.header.frame_id == camera.frame_id:
                camera_name = camera.name
                break
        if 'camera_name' not in locals():
            rospy.logerr('Error while processing image callback. Found frame_id "{}" but no corresponding camera. Try setting "frame_id" in config file.'.format(data.header.frame_id))
            return
        camera_image_container = CameraImage(self.cameras[camera_name], cv_image, data.header.stamp.to_time())
        self.image_distributor.put_image(camera_image_container)
