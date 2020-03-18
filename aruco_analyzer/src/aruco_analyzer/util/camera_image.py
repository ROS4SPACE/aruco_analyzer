#!/usr/bin/env python


class CameraImage(object):
    def __init__(self, camera, image, timestamp):
        self.camera = camera
        self.image = image
        self.timestamp = timestamp

    def __del__(self):
        self.__delattr__('image')
