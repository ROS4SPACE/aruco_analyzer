#!/usr/bin/env python
from abc import ABC, abstractmethod
from threading import Thread


class ImageMiner(ABC, Thread):

    def __init__(self, image_distributor):
        Thread.__init__(self)
        self.daemon = True

        self._image_distributor = image_distributor

    @abstractmethod
    def run(self):
        pass
