#!/usr/bin/env python3
import logging
import sys
import os
import numpy as np
from yaml import load, dump
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

class Camera (object):
    """
    This class handles the camera specific data. 
    """

    logger = logging.getLogger('aruco_analyzer.camera')

    def __init__(self, name, **kwargs):
        self._name = name
        for key, value in kwargs.items():
            setattr(self, '_'+key, value)
        if not hasattr(self, '_frame_id'):
            self._frame_id = self.name
        if "calibration_location" in kwargs.keys():
            calibration_location = kwargs['calibration_location']
        else:
            calibration_location = os.path.join(os.path.expanduser("~"), 'space_rover', 'srcameras', 'calibrationdata', self.name+'.yaml')
            if not os.path.exists(calibration_location):
                calibration_location = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), '..', 'config', self.name+'.yaml')
            if not os.path.exists(calibration_location):
                self.logger.error('Cannot find calibration file for camera {}'.format(self.name))
                exit()
        self._calibration_file = load(open(calibration_location), Loader=Loader)
        self._camera_matrix = self.read_camera_matrix(self._calibration_file)
        self._dist_coefficients = self.read_distortion_coefficients(self._calibration_file)

    def read_camera_matrix(self, calibration_file):
        camera_mat_raw = calibration_file['camera_matrix']['data']
        return np.array([camera_mat_raw[x:x+3] for x in range(0, len(camera_mat_raw), 3)])

    def read_distortion_coefficients(self, calibration_file):
        return np.array(calibration_file['distortion_coefficients']['data'])

    @property
    def name(self):
        return self._name

    @property
    def frame_id(self):
        return self._frame_id

    @property
    def camera_matrix(self):
        return self._camera_matrix

    @property
    def distortion_coefficients(self):
        return self._dist_coefficients
