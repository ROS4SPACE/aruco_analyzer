#!/usr/bin/env python


class CameraImage(object):
    def __init__(self, camera, image, timestamp):
        self._camera = camera
        self._image = image
        self._timestamp = timestamp

    @property
    def camera(self):
        return self._camera

    @property
    def image(self):
        return self._image

    @property
    def timestamp(self):
        return self._timestamp
