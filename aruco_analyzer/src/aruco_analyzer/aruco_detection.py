#!/usr/bin/env python
import logging
import threading
import time

from .util import ArucoDetector, Analyzer, ImageDistributor, Config


class ARMarkerDetector(object):

    def __init__(self, config):
        logging.basicConfig()
        self.logger = logging.getLogger(__name__)
        self.logger.debug('Start')

        self.conf = Config()
        self.conf.read_from_dictionary(config)
        self.image_distributor = ImageDistributor()

    def set_image_miner(self, image_miner, *miner_args):
        self.image_miner = image_miner(self.image_distributor, self.conf.cameras, *miner_args)

    def set_detection_image_listener(self, detection_image_listener):
        self.image_distributor.set_detection_image_listener(detection_image_listener)

    def launch_detection_workers(self, detection_image_listener=None):
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

        if broadcaster is not None:
            self.analyzer.set_broadcaster(broadcaster)
            self.analyzer.start_broadcasting()
