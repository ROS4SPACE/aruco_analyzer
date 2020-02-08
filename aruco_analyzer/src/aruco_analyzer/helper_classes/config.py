#!/usr/bin/env python
import logging
import itertools
from .camera import Camera
from .board_configs import *

class Singleton(object):
    def __new__(cls, *args, **kwds):
        it = cls.__dict__.get("__it__")
        if it is not None:
            return it
        cls.__it__ = it = object.__new__(cls)
        it.init(*args, **kwds)
        return it

    def init(self, *args, **kwds):
        pass

class Config(Singleton):

    _max_detection_images = 10
    _max_analyzer_list = 10
    _max_detection_age = 1
    _number_of_workers = 1
    _space_fixed_ids = []
    
    _marker_config = None
    _board_config = []

    _cameras = {}

    _print_in_image = False
    _draw_detected_markers = False
    _draw_axis = False

    _average_estimations = False

    logger = logging.getLogger('aruco_analyzer.config')

    def read_from_dictionary(self, config):
        for key, value in config.items():
            if key == 'board_config':
                if value is not None:
                    for board in value:
                        try:
                            if board['type'] == 'cube':
                                self._board_config.append(
                                    CubeBoardConfig(board['dictionary'], board['marker_length'], board['border'], board['marker_per_side'], board['first_marker'])
                                )
                            elif board['type'] == 'satellite':
                                self._board_config.append(
                                    SatelliteBoardConfig(board['dictionary'], board['marker_length'], board['border'], board['marker_per_side'], board['first_marker'])
                                )
                            elif board['type'] == 'grid':
                                self._board_config.append(
                                    GridBoardConfig(board['dictionary'], board['marker_length'], board['x'], board['y'], 
                                        board['separation'], board['first_marker'])
                                )
                            else:
                                self.logger.error('Unknown board type \'{}\''.format(board['type']))
                                exit()
                        except KeyError as keyError:
                            if keyError.message == 'type':
                                self.logger.error('Must specify key \'{}\' for board'.format(keyError.message))
                            else:
                                self.logger.error('Must specify key \'{}\' for board of type \'{}\''.format(keyError.message, board['type']))
                            exit()
                    
            elif key == 'marker_config':
                if value is not None:
                    self._marker_config = MarkerConfig(value['dictionary'], value['marker_length'])
            elif key == 'cameras':
                for camera in value:
                    if type(camera) is str:
                        self._cameras[camera] = Camera(camera)
                    else:
                        camera_name = list(camera.keys())[0]
                        self._cameras[camera_name] = Camera(camera_name, **camera[camera_name])
            else:
                setattr(self, '_'+key, value)
        
        self.check_valid()

    def check_valid(self):
        if self._marker_config is not None:
            for board_config in self._board_config:
                if self._marker_config.dictionary_name == board_config.dictionary_name:
                    self.logger.error('Dictionaries for single markers and boards cannot be the same!')
                    exit()

        for a, b in itertools.combinations(self._board_config, 2):
            if a.dictionary_name == b.dictionary_name and not set(a.ids).isdisjoint(b.ids):
                self.logger.error('Markers of two boards overlap!')
                self.logger.error('Type: {}, Dictionary: {}, first_marker: {}'.format(a.type, a.dictionary_name, a.first_marker))
                self.logger.error('Type: {}, Dictionary: {}, first_marker: {}'.format(b.type, b.dictionary_name, b.first_marker))
                exit()


    @property
    def max_detection_images(self):
        return self._max_detection_images

    @property
    def max_analyzer_list(self):
        return self._max_analyzer_list

    @property
    def max_detection_age(self):
        return self._max_detection_age

    @property
    def number_of_workers(self):
        return self._number_of_workers

    @property
    def space_fixed_ids(self):
        return self._space_fixed_ids

    @property
    def marker_config(self):
        return self._marker_config

    @property
    def board_config(self):
        return self._board_config

    @property
    def cameras(self):
        return self._cameras

    @property
    def print_in_image(self):
        return self._print_in_image

    @property
    def draw_detected_markers(self):
        return self._draw_detected_markers

    @property
    def draw_axis(self):
        return self._draw_axis

    @property
    def average_estimations(self):
        return self._average_estimations
