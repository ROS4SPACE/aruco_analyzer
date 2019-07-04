#!/usr/bin/env python
import logging
import sys
import threading
import time
from yaml import load
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

from helper_classes.aruco_detector import ArucoDetector
from helper_classes.analyzer import Analyzer
from helper_classes.image_distributor import ImageDistributor
from helper_classes.config import Config

class ARMarkerDetector(object):
    
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    logger.setLevel('DEBUG')

    def __init__(self, config):
        self.conf = Config()
        self.conf.read_from_dictionary(config)
        self.image_distributor = ImageDistributor()

    def set_image_miner(self, image_miner):
        self.image_miner = image_miner(self.image_distributor, self.conf.cameras)

    def set_detection_image_publisher(self, detection_image_publisher):
        self.image_distributor.register_detection_image_publisher(detection_image_publisher)

    def launch_detection_workers(self, detection_image_publisher=None):
        self.worker = []
        self.worker_threads = []
        for worker in range(0, self.conf.number_of_workers):
            self.worker.append(ArucoDetector(self.image_distributor))
            self.worker_threads.append(threading.Thread(target=self.worker[worker].run_detect))
            self.worker_threads[worker].daemon = True
            time.sleep(0.05)
            self.worker_threads[worker].start()

    def launch_analyzer(self, broadcaster=None):
        self.analyzer = Analyzer(self.image_distributor)
        self.analyzer_thread = threading.Thread(target=self.analyzer.run)
        self.analyzer_thread.daemon = True
        self.analyzer_thread.start()

        if not broadcaster is None:
            self.analyzer.set_broadcaster(broadcaster)
            self.analyzer.start_broadcasting()
