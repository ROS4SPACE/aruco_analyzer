import cv2

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
