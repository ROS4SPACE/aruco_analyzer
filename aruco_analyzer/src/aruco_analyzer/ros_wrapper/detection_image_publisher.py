import rospy
from threading import Event
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class DetectionImagePublisher(object):
    def __init__(self, detection_images_qdic):
        self.detection_images_qdic = detection_images_qdic
        self.image_publishers = {}
        self.bridge = CvBridge()
        for camera in self.detection_images_qdic.keys():
            topic = '/cameras/'+camera+'/ar_detection_images/compressed'
            self.image_publishers[camera] = rospy.Publisher(
                topic, CompressedImage, queue_size=10
            )
        self._detection_image_available = Event()

    def notify_detection_image_available(self):
        self._detection_image_available.set()

    def run(self):
        while not rospy.is_shutdown():
            self._detection_image_available.wait()
            for camera in self.detection_images_qdic.keys():
                for _ in range(self.detection_images_qdic[camera].qsize()):
                    try:
                        self.image_publishers[camera].publish(
                            self.bridge.cv2_to_compressed_imgmsg(self.detection_images_qdic[camera].get())
                        )
                    except CvBridgeError as e:
                        print(e)
            self._detection_image_available.clear()
