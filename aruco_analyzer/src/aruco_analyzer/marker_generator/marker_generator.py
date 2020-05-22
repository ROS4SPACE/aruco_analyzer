import logging

import cv2
import numpy as np

logger = logging.getLogger(__name__)


def generate_single_marker(dictionary, ar_id, length=500, margin=0):
    if isinstance(dictionary, str):
        dictionary = getattr(cv2.aruco, dictionary)
    if isinstance(dictionary, int):
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary)

    marker = cv2.aruco.drawMarker(dictionary, ar_id, length)
    image = np.full((length + 2 * margin, length + 2 * margin), fill_value=255, dtype=np.uint8)
    image[margin:margin+marker.shape[0], margin:margin+marker.shape[1]] = marker
    return image


def generate_grid(dictionary, first_ar_id, x, y, length, separation, margin=0, border_bits=1):
    if isinstance(dictionary, str):
        dictionary = getattr(cv2.aruco, dictionary)
    if isinstance(dictionary, int):
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary)

    width = x * length + (x + 1) * separation
    height = y * length + (y + 1) * separation

    board = cv2.aruco.GridBoard_create(x, y, length, separation, dictionary, first_ar_id)
    image = board.draw((width, height), np.array([]), margin, border_bits)
    return image


def generate_cube(dictionary_name, first_ar_id, x, length, separation, margin=0, border_bits=1):
    images = {}
    images['front'] = generate_grid(dictionary_name, first_ar_id=first_ar_id, x=x, y=x, length=length, separation=separation, margin=0, border_bits=1)
    images['right'] = generate_grid(dictionary_name, first_ar_id=first_ar_id + x**2, x=x, y=x, length=length, separation=separation, margin=0, border_bits=1)
    images['back'] = generate_grid(dictionary_name, first_ar_id=first_ar_id + 2*x**2, x=x, y=x, length=length, separation=separation, margin=0, border_bits=1)
    images['left'] = generate_grid(dictionary_name, first_ar_id=first_ar_id + 3*x**2, x=x, y=x, length=length, separation=separation, margin=0, border_bits=1)
    return images
