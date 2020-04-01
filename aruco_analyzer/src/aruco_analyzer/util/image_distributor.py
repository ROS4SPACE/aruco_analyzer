#!/usr/bin/env python
import logging
from threading import Lock, Condition, Event
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
from .config import Config


class ImageDistributor(object):

    def __init__(self):
        self.logger = logging.getLogger(__name__)

        self.config = Config()

        self._max_detection_images = self.config.max_detection_images
        self._image_queue = Queue(maxsize=self.config.max_detection_images)
        self._detection_queue = Queue(maxsize=self._max_detection_images)
        self._detection_images_qdic = {}

        for camera in self.config.cameras.keys():
            self._detection_images_qdic[camera] = Queue(maxsize=self._max_detection_images)

        # list of next expected aruco_detector IDs
        self._order_list = []
        # storage for out-of-order data, contains list of data per ID
        self._reorder_dict = {}

        # protect _order_list
        self._order_list_lock = Lock()
        # set to True when new image is available
        self._image_available = Event()
        # set to True when new detection is available
        self._detection_available = Event()

        self._detection_image_listener = None

    def set_detection_image_listener(self, detection_image_listener):
        self._detection_image_listener = detection_image_listener

    # puts image into image queue
    def put_image(self, camera_image_container):
        if not self.config.never_drop_frames and self._image_queue.full():
            self.logger.info('Image queue is full! --> Emptying')
            for _ in range(self._image_queue.qsize()-1):
                self._image_queue.get()

        self._image_queue.put(camera_image_container)
        self._image_available.set()
        self.logger.debug('notify')

    # returns image from image queue and stores ID of requesting aruco detector
    def get_image(self, id):
        self._order_list_lock.acquire()
        while self._image_queue.empty():
            self.logger.debug('wait {}'.format(id))
            self._image_available.wait()
            self.logger.debug('resume {}'.format(id))
        self._order_list.append(id)
        self._order_list_lock.release()
        return self._image_queue.get()

    # puts detection into detection queue
    def put_detection(self, id, detection, detection_image):
        self.logger.debug('enter put_detection {}'.format(id))
        try:
            self.logger.debug('acquire lock')
            self._order_list_lock.acquire()
            self.logger.debug('lock acquired')
            # no detection
            if detection is None:
                self._put_detection(detection, detection_image)
                self._order_list.remove(id)
                self._reorder()
                return

            # check if aruco_analyer ID matches next expected
            if self._order_list[0] is id:
                # put detection into output queues
                self._put_detection(detection, detection_image)
                self._order_list.pop(0)
                # try to add previously gotten to output queues
                self._reorder()
            else:
                # put detection into temporary storage
                if id not in self._reorder_dict:
                    self._reorder_dict[id] = []
                # use list to allow multiple data from the same ID
                self._reorder_dict[id].append((detection, detection_image))
        finally:
            self._order_list_lock.release()
            self.logger.debug('release lock')
            self.logger.debug('return put_detection {}'.format(id))

    # gets detection from detection queue
    def get_detection(self):
        self._detection_available.wait()
        detection = self._detection_queue.get()
        if self._detection_queue.empty():
            self._detection_available.clear()
        return detection

    def _put_detection(self, detection, detection_image):
        if detection is not None:
            if not self.config.never_drop_frames and self._detection_queue.full():
                self.logger.debug('Detection queue is full! --> Emptying')
                for _ in range(self._image_queue.qsize()-1):
                    self._detection_queue.get()

            self._detection_queue.put(detection)
            self._detection_available.set()

        if not self.config.never_drop_frames and self._detection_images_qdic[detection_image.camera.name].full():
            for _ in range(self._detection_images_qdic[detection_image.camera.name].qsize()-1):
                self._detection_images_qdic[detection_image.camera.name].get()
        self._detection_images_qdic[detection_image.camera.name].put(detection_image.image)

        # notify detection image listeners
        if self._detection_image_listener is not None:
            self._detection_image_listener.notify_detection_image_available()

    # tries to move data from temporary storage into output queues
    def _reorder(self):
        if self._order_list:
            while self._order_list[0] in self._reorder_dict:
                id_list = self._reorder_dict[self._order_list[0]]
                detection, detection_image = id_list.pop(0)
                self._put_detection(detection, detection_image)
                if not id_list:
                    self._reorder_dict.pop(self._order_list[0])
                self._order_list.pop(0)
                if not self._order_list:
                    return
