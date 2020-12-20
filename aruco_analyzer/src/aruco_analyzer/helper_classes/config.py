#!/usr/bin/env python3
import logging
import itertools
import cv2
import numpy as np
from operator import add
from .camera import Camera
from abc import abstractmethod
from pyquaternion import Quaternion

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
                for board in value:
                    try:
                        if board['type'] == 'cube':
                            self._board_config.append(
                                CubeBoardConfig(board['dictionary'], board['marker_length'], board['border'], board['marker_per_side'], board['first_marker'])
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

class MarkerConfig(object):

    def __init__(self, dictionary, marker_length):
        self._dictionary_name = dictionary
        self._dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self._dictionary_name))
        self._marker_length = marker_length

    @property
    def dictionary_name(self):
        return self._dictionary_name

    @property
    def dictionary(self):
        return self._dictionary

    @property
    def marker_length(self):
        return self._marker_length

class BoardConfig(MarkerConfig):

    @abstractmethod
    def __init__(self, dictionary, marker_length):
        super(BoardConfig, self).__init__(dictionary, marker_length)

    @property
    def board(self):
        return self._board

    @property
    def type(self):
        return self._type

class GridBoardConfig(BoardConfig):

    def __init__(self, dictionary, marker_length, x, y, marker_separation, first_marker):
        super(GridBoardConfig, self).__init__(dictionary, marker_length)
        self._type = 'G'
        self._x = x
        self._y = y
        self._marker_separation = marker_separation
        self._first_marker = first_marker
        self._board = cv2.aruco.GridBoard_create(self._x, self._y, self._marker_length, self._marker_separation, self._dictionary, self._first_marker)
        
    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def first_marker(self):
        return self._first_marker
    
    @property
    def ids(self):
        return range(self._first_marker, self._first_marker + self._x*self._y)

    @property
    def id(self):
        return '{}{:03d}'.format(self.type, self._first_marker)

class CubeBoardConfig(BoardConfig):

    def __init__(self, dictionary, marker_length, border, marker_per_side, first_marker, ):
        super(CubeBoardConfig, self).__init__(dictionary, marker_length)
        self._type = 'C'
        self._border = border
        self._marker_per_side = marker_per_side
        self._first_marker = first_marker

        self._ids = [i for i in range(self._first_marker, self._first_marker+4*self._marker_per_side**2)]

        corners = []
        front_side_corners = []

        for j in range(self._marker_per_side, 0, -1):
            for i in range(1, self._marker_per_side+1):
                marker_corners = []
                marker_corners.append([0, (i-1) * self._marker_length + i*self._border, j*self._marker_length + j*self._border])
                marker_corners.append([0, i * self._marker_length + i*self._border, j*self._marker_length + j*self._border])
                marker_corners.append([0, i * self._marker_length + i*self._border, (j-1)*self._marker_length + j*self._border])
                marker_corners.append([0, (i-1) * self._marker_length + i*self._border, (j-1)*self._marker_length + j*self._border])
                front_side_corners.append(marker_corners)

         # transform coordinates so that centre of cube is (0,0,0)
        cube_length = self._marker_per_side*self._marker_length + (self._marker_per_side+1)*self._border
        transform = [cube_length/2, -cube_length/2, -cube_length/2]
        transformed_frontside_corners = []
        for marker_corners in front_side_corners:
            transformed_frontside_corners.append([list(map(add, transform, marker_corner)) for marker_corner in marker_corners])

        corners.extend(transformed_frontside_corners)

        # rotate coordinates for each cube side
        for marker_corners in transformed_frontside_corners:
            corners.append([transform_vector(marker_corner, (0,0,1), 90) for marker_corner in marker_corners])
        for marker_corners in transformed_frontside_corners:
            corners.append([transform_vector(marker_corner, (0,0,1), 180) for marker_corner in marker_corners])
        for marker_corners in transformed_frontside_corners:
            corners.append([transform_vector(marker_corner, (0,0,1), 270) for marker_corner in marker_corners])

        # transform into correct data format
        points = np.array([[np.array(marker_corners, 'float32')] for marker_corners in corners])
        ids = np.array([[id] for id in self._ids])
        
        self._board = cv2.aruco.Board_create(points, self._dictionary, ids)

    @property
    def first_marker(self):
        return self._first_marker

    @property
    def border(self):
        return self._border
    
    @property
    def ids(self):
        return self._ids

    @property
    def id(self):
        return '{}{:03d}'.format(self.type, self._first_marker)


def transform_vector(vec, axis, angle):
    q = Quaternion(axis=axis, degrees=angle)
    vec_ = np.zeros((4))
    vec_[1:] = vec
    vec = q * vec_ * q.inverse
    return vec.elements[1:].tolist()