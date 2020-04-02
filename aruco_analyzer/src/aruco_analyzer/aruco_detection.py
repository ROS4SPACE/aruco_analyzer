#!/usr/bin/env python
import logging

from .util import ArucoDetector, Analyzer, ImageDistributor, Config


class ARMarkerDetector(object):

    def __init__(self, config):
        logging.basicConfig()
        self.logger = logging.getLogger(__name__)
        self.logger.info("Initializing ArUco Detection")

        self.conf = Config()
        self.conf.read_from_dictionary(config)
        self.image_distributor = ImageDistributor()

        self.worker = []
        self.worker_threads = []

    def set_image_miner(self, image_miner, *miner_args):
        self.image_miner = image_miner(self.image_distributor, self.conf.cameras, *miner_args)

    def set_detection_image_listener(self, detection_image_listener):
        self.image_distributor.set_detection_image_listener(detection_image_listener)

    def launch_detection_workers(self, detection_image_listener=None):
        for worker in range(0, self.conf.number_of_workers):
            self.worker.append(ArucoDetector(self.image_distributor))
            self.worker[-1].start()

    def launch_analyzer(self, broadcaster=None):
        self.analyzer = Analyzer(self.image_distributor)
        self.analyzer.start(broadcaster)

    def end_all(self):
        self.logger.debug('end all')
        self.conf.reset()

        for worker in self.worker:
            worker.running = False

        for worker in self.worker:
            worker.join()

        self.analyzer.running = False
        self.analyzer.join()

        self.logger.debug('all joined')
