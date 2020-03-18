#!/usr/bin/env python
import logging
import sys
import os
import errno
import numpy as np
from yaml import load
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


class Camera (object):
    """
    This class handles the camera specific data.
    """

    def __init__(self, name, **kwargs):
        self.logger = logging.getLogger(__name__)

        self._name = name
        for key, value in kwargs.items():
            setattr(self, '_' + key, value)
        if not hasattr(self, '_frame_id'):
            self._frame_id = self.name
        if 'calibration_file' in kwargs.keys():
            calibration_file = kwargs['calibration_file']
        else:
            calibration_file = os.path.join(os.path.expanduser('~'), 'space_rover', 'srcameras', 'calibrationdata', self.name + '.yaml')
            if not os.path.exists(calibration_file):
                calibration_file = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), '..', 'config', self.name + '.yaml')
            if not os.path.exists(calibration_file):
                self.logger.error("Cannot find calibration file for camera {}".format(self.name))
                raise IOError(
                    errno.ENOENT, os.strerror(errno.ENOENT), "Cannot find calibration file for camera {}".format(self.name)
                )
        self._calibration_file = load(open(calibration_file), Loader=Loader)
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
