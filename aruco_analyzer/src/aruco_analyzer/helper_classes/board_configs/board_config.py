from .marker_config import MarkerConfig
from abc import abstractmethod
from pyquaternion import Quaternion
import numpy as np

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

def transform_vector(vec, axis, angle):
    q = Quaternion(axis=axis, degrees=angle)
    vec_ = np.zeros((4))
    vec_[1:] = vec
    vec = q * vec_ * q.inverse
    return vec.elements[1:].tolist()
