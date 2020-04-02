#!/usr/bin/env python
import logging
import time
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np
from pyquaternion import Quaternion
from .detection_output import SingleOutput
from .quaternion_helper import averageQuaternions
from .config import Config


class Analyzer(threading.Thread):

    def __init__(self, image_distributor):
        super(Analyzer, self).__init__()
        self.daemon = True
        self.name = __name__

        self.logger = logging.getLogger(__name__)

        self.image_distributor = image_distributor
        self.broadcaster = None

        self.config = Config()
        self.space_fixed_ids = self.config.space_fixed_ids
        self.max_analyzer_list = self.config.max_analyzer_list
        self.max_age = self.config.max_detection_age

        # holds unprocessed estimations for each marker and each camera
        self.marker_detections = {}
        # holds processed estimations
        self.analyzed_targets = Queue()
        self.analyzing_threads = {}

        # self.lock = Lock()
        self.analyzed_target_available = threading.Event()

        self.running = True

    def start(self, broadcaster):
        super(Analyzer, self).start()
        self.start_broadcasting(broadcaster)

    def run(self):
        while self.running:
            detection = self.image_distributor.get_detection()
            if detection:
                if self.config.filter_by_id:
                    self.filter_by_id(detection)
                else:
                    self.analyzed_targets.put(detection)
                    self.analyzed_target_available.set()
        self.logger.info("Analyzer end")

    def filter_by_id(self, detection_output):
        for i in range(0, len(detection_output.ar_ids)):
            single = SingleOutput()
            single.pack_from_parent(detection_output, i)

            # self.logger.warn(str(single._camera_image.timestamp))

            identifier = single.unique_ar_id

            if identifier not in self.marker_detections:
                # create list
                self.logger.info('First detection of {}'.format(identifier))
                self.marker_detections[identifier] = {}

            camera_name = single.camera_image.camera.name
            if camera_name not in self.marker_detections[identifier]:
                self.marker_detections[identifier][camera_name] = (list(), threading.Event())

            single_detection_list, detection_available = self.marker_detections[identifier][camera_name]

            # self.lock.acquire()
            single_detection_list.append(single)
            detection_available.set()

            # remove oldest, i.e. first detection if list gets too big, do not remove when max_analyzer_list is set to 0
            if self.max_analyzer_list != 0 and len(single_detection_list) > self.max_analyzer_list:
                single_detection_list.pop(0)
            # self.lock.release()

            if identifier not in self.analyzing_threads:
                # start analyzing thread for each new id
                self.analyzing_threads[identifier] = self.create_analyzing_thread(single_detection_list, detection_available)
                self.analyzing_threads[identifier].start()

    def create_analyzing_thread(self, single_detection_list, detection_available):
        analyze_thread = threading.Thread(target=self.analyze_single_id, args=[single_detection_list, detection_available])
        analyze_thread.daemon = True
        return analyze_thread

    def analyze_single_id(self, single_detection_list, detection_available):
        # thread will be suspended when single_detection_list is empty
        # it is returned to unsuspended state when a new detection occurs
        suspend = False
        while True:
            if not suspend:
                detection_available.wait(0.1)
            else:
                detection_available.wait()

            if detection_available.is_set():
                suspend = False
                detection_available.clear()

            # removes detections older than max_age
            single_detection_list[:] = [x for x in single_detection_list if self.max_age == 0 or not time.time() - x.timestamp > self.max_age]

            if single_detection_list:
                if self.config.average_estimations:
                    self.logger.debug('Averaging over {} estimations for {}'.format(len(single_detection_list), single_detection_list[-1].unique_ar_id))

                    analyzed_target = SingleOutput()
                    [average_pose, average_quat, last_candidate] = self.average_result(single_detection_list)

                    analyzed_target.camera_image = last_candidate.camera_image
                    analyzed_target.ar_id = last_candidate.ar_id
                    analyzed_target.quaternion = average_quat
                    analyzed_target.position = average_pose
                    analyzed_target.timestamp = last_candidate.timestamp
                    analyzed_target.marker_type = last_candidate.marker_type
                else:
                    self.logger.debug('Not averaging.')
                    analyzed_target = single_detection_list.pop(-1)

                self.analyzed_targets.put(analyzed_target)
                self.analyzed_target_available.set()
                self.logger.debug('set analyzed_target_available')

            else:
                suspend = True

    # TODO weighted averaging?
    def average_result(self, single_detection_list):
        average_pose = sum(item.position for item in single_detection_list)
        average_pose = np.true_divide(average_pose, len(single_detection_list))

        average_quat = averageQuaternions(np.concatenate([item.quaternion for item in single_detection_list], axis=0).reshape(-1, 4))
        average_quat = Quaternion(average_quat).normalised.elements
        return [average_pose, average_quat, single_detection_list[-1]]

    def broadcast(self):
        while self.running:
            ret = self.analyzed_target_available.wait(1.0)
            if not ret:
                continue
            analyzed_target = self.analyzed_targets.get()
            if self.analyzed_targets.empty():
                self.analyzed_target_available.clear()
            self.broadcaster.broadcast(analyzed_target)

    def start_broadcasting(self, broadcaster):
        self.broadcaster = broadcaster
        if self.broadcaster is not None:
            self.broadcast_thread = threading.Thread(target=self.broadcast)
            self.broadcast_thread.daemon = True
            self.broadcast_thread.name = 'Broadcaster'
            self.broadcast_thread.start()
