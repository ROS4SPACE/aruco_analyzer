#!/usr/bin/env python
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

    def read_from_dictionary(self, config):
        self.reset()

        for key, value in config.items():
            if key == 'markers':
                for marker in value:
                    if marker['type'] == 'marker':
                        self._marker_config = MarkerConfig(**marker)
                    elif marker['type'] == 'cube':
                        self._board_config.append(CubeBoardConfig(**marker))
                    elif marker['type'] == 'grid':
                        self._board_config.append(GridBoardConfig(**marker))
                    elif marker['type'] == 'satellite':
                        self._board_config.append(SatelliteBoardConfig(**marker))
                    else:
                        raise ValueError("Unknown marker type \'{}\'".format(marker['type']))
            elif key == 'cameras':
                for camera in value:
                    if type(camera) is str:
                        self._cameras[camera] = Camera(camera)
                    else:
                        camera_name = list(camera.keys())[0]
                        self._cameras[camera_name] = Camera(camera_name, **camera[camera_name])
            else:
                setattr(self, '_' + key, value)

        self.check_valid()

    def check_valid(self):
        # if self._marker_config is not None:
        #     for board_config in self._board_config:
        #         if self._marker_config.dictionary_name == board_config.dictionary_name:
        #             raise ValueError("Dictionaries for single markers and boards cannot be the same!")

        for a, b in itertools.combinations(self._board_config, 2):
            if a.dictionary_name == b.dictionary_name and not set(a.ids).isdisjoint(b.ids):
                raise ValueError(
                    (
                        "Markers of two boards overlap!\n"
                        "Type: {}, Dictionary: {}, first_marker: {}\n"
                        "Type: {}, Dictionary: {}, first_marker: {}"
                    ).format(a.type, a.dictionary_name, a.first_marker, b.type, b.dictionary_name, b.first_marker)
                )

    def reset(self):
        self._publish_detection_images = False
        self._filter_by_id = False
        self._never_drop_frames = False
        self._max_detection_images = 0
        self._max_analyzer_list = 0
        self._max_detection_age = 0
        self._number_of_workers = 1
        self._space_fixed_ids = []

        self._marker_config = None
        self._board_config = []

        self._cameras = {}

        self._print_in_image = False
        self._draw_detected_markers = False
        self._draw_axis = False

        self._average_estimations = False

    @property
    def publish_detection_images(self):
        return self._publish_detection_images

    @property
    def filter_by_id(self):
        return self._filter_by_id

    @property
    def never_drop_frames(self):
        return self._never_drop_frames

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
