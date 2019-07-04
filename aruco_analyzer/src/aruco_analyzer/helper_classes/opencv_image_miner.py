#!/usr/bin/env python
import logging
import time
import cv2
from threading import Thread
from .camera import Camera
from .camera_image import CameraImage

props = [
    'CAP_PROP_FRAME_WIDTH',
    'CAP_PROP_FRAME_HEIGHT',
    'CAP_PROP_FPS',
    'CAP_PROP_BRIGHTNESS',
    'CAP_PROP_CONTRAST',
    'CAP_PROP_SATURATION',
    # 'CAP_PROP_MODE',
    # 'CAP_PROP_FOURCC',
    # 'CAP_PROP_GAIN',
    # 'CAP_PROP_EXPOSURE',
]

class OpenCVImageMiner(object):
    """
    This class is only used for testing the OpenCV method of capturing video from USB cameras.
    """

    logger = logging.getLogger('aruco_analyzer.OpenCVImageMiner')

    def __init__(self, image_distributor, cameras):
        self._image_distributor = image_distributor
        self.cameras = cameras
        self.capture_threads = {}

        for camera_name, camera in self.cameras.items():
            self.capture_threads[camera_name] = Thread(target=self.run, args=[camera_name])
            self.capture_threads[camera_name].daemon = True
            self.capture_threads[camera_name].start()

    def run(self, camera_name):
        self.logger.info('initializing {}'.format(camera_name))
        if camera_name == 'C920':
            cap = cv2.VideoCapture(0)
        else:
            cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS, 30.)

        self.set_video_capture_property(cap, 'CAP_PROP_MODE', 2)
        self.set_video_capture_property(cap, 'CAP_PROP_FPS', 30)
        # self.set_video_capture_property(cap, 'CAP_PROP_BRIGHTNESS', 50.5/100.)
        # self.set_video_capture_property(cap, 'CAP_PROP_CONTRAST', 50.5/100.)
        # self.set_video_capture_property(cap, 'CAP_PROP_SATURATION', 50.5/100.)

        for prop in props:
            value = cap.get(getattr(cv2, prop))
            self.logger.info('{}: {}'.format(prop, value))

        fps = FPS()
        fps.start()

        while cap.isOpened():            
            ret, frame = cap.read()
            # cv2.imshow(camera_name, frame)
            if ret:
                camera_image_container = CameraImage(self.cameras[camera_name], frame, time.time())
                self._image_distributor.put_image(camera_image_container)
                fps.update()

    def set_video_capture_property(self, cap, property, value):
        if property in props:
            cap.set(getattr(cv2, property), value)
        else:
            self.logger.error('No property named {}'.format(property))
 
class FPS:

    logger = logging.getLogger('aruco_analyzer.OpenCVImageMiner.FPS')

    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        # start the timer
        self._start = time.time()

        thread = Thread(target=self.show_fps_thread)
        thread.start()

    def stop(self):
        # stop the timer
        self._end = time.time()

    def update(self):
        # increment the total number of frames examined during the
		# start and end intervals
        self._numFrames += 1

    def elapsed(self):
        # return the total number of seconds between the start and
		# end interval
        if self._end is None:
            return time.time() - self._start
        else:
            return self._end - self._start

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()

    def show_fps_thread(self):
        while True:
            self.logger.info(self.fps())
            time.sleep(5)
