try:
    from abc import ABC as ABC
except ImportError:
    from abc import ABCMeta as ABC

from abc import abstractmethod

import cv2
import numpy as np
from pyquaternion import Quaternion


class BaseConfig(ABC):
    def __init__(self, **kwargs):
        self._type = kwargs['type']
        try:
            self._set_variables(**kwargs)
        except KeyError as keyError:
            raise ValueError("Must specify key \'{}\' in configuration for type \'{}\'".format(keyError.args[0], self._type))

    @abstractmethod
    def _set_variables(self, **kwargs):
        self._dictionary_name = kwargs['dictionary']
        self._dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self._dictionary_name))
        self._marker_length = kwargs['marker_length']

    @property
    def dictionary_name(self):
        return self._dictionary_name

    @property
    def dictionary(self):
        return self._dictionary

    @property
    def marker_length(self):
        return self._marker_length

    @property
    def type(self):
        return self._type

    @property
    def type_id(self):
        return self._type_id


class BoardConfig(BaseConfig):

    @abstractmethod
    def __init__(self, **kwargs):
        super(BoardConfig, self).__init__(**kwargs)

    @property
    def board(self):
        return self._board

    @property
    def id(self):
        return '{}{:03d}'.format(self.type_id, self._first_marker)


def transform_vector(vec, axis, angle):
    q = Quaternion(axis=axis, degrees=angle)
    vec_ = np.zeros((4))
    vec_[1:] = vec
    vec = q * vec_ * q.inverse
    return vec.elements[1:].tolist()
