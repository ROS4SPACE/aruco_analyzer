#!/usr/bin/env python
import rospy
from threading import Thread
from aruco_analyzer import ARMarkerDetector
from detection_image_publisher import DetectionImagePublisher
from ros_image_miner import ROSImageMiner
from cpumem import CpuMemMonitor
from tf_broadcaster import TFBroadCaster

class ARMarkerDetectorWrapper (object):
    def __init__(self):
        rospy.loginfo("Instantiating {}".format(type(self).__name__))
        self.system_monitor = CpuMemMonitor()
        self.system_monitor_thread = Thread(target=self.system_monitor.run)
        self.system_monitor_thread.daemon = True
        self.system_monitor_thread.start()

        config = rospy.get_param('aruco_analyzer/config')        
        self.ar_marker_detector = ARMarkerDetector(config)

        self.ar_marker_detector.set_image_miner(ROSImageMiner)
        self.ar_marker_detector.launch_detection_workers()

        broadcaster = TFBroadCaster()
        self.ar_marker_detector.launch_analyzer(broadcaster)

        self.detection_image_publisher = DetectionImagePublisher(self.ar_marker_detector.image_distributor._detection_images_qdic)
        self.publisher_thread = Thread(target=self.detection_image_publisher.run)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

        self.ar_marker_detector.set_detection_image_publisher(self.detection_image_publisher)
        