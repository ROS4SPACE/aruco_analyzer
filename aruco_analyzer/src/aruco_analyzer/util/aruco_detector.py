#!/usr/bin/env python
import logging
import math
import threading
from functools import reduce

import cv2
import numpy as np

from .config import Config
from .detection_output import DetectionOutput


def partition(l, p):
    return reduce(lambda x, y: x[not p(y)].append(y) or x, l, ([], []))


class ArucoDetector(threading.Thread):
    counter = 0

    def __init__(self, image_distributor):
        super(ArucoDetector, self).__init__()
        self.daemon = True

        # unique id
        self.id = type(self).counter
        type(self).counter += 1

        self.name = '{}-{}'.format(__name__, str(self.id))

        self.logger = logging.getLogger(__name__)

        self.image_distributor = image_distributor

        self.config = Config()
        self.marker_config = self.config.marker_config
        self.board_config = self.config.board_config

        # holds detected markers for this image
        self.detected_markers = {}

        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.running = True

    def detect_markers(self, camera_image, dictionary):
        # check if the markers of this dictionary were already detected in this image
        try:
            markers = self.detected_markers[dictionary]
            if markers is None:
                return None, None
            else:
                corners, ids = zip(*markers)
        except KeyError:
            corners, ids, rejected = cv2.aruco.detectMarkers(camera_image.image, dictionary)

        return corners, ids

    def detect_board(self, camera_image, board):
        corners, ids = self.detect_markers(camera_image, board.dictionary)
        text_place = 30

        if ids is None:
            # no marker detected
            return None

        # filter markers which are not part of the board
        markers, filtered_markers = partition(zip(corners, ids), lambda x: x[1][0] in board.ids)

        if not markers:
            return

        corners, ids = zip(*markers)
        ids = np.asarray(ids)

        self.detected_markers[board.dictionary] = filtered_markers

        # cv2.aruco.drawDetectedMarkers(camera_image.image, corners, ids)
        if self.config.draw_detected_markers:
            cv2.aruco.drawDetectedMarkers(camera_image.image, corners)

        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
            corners, ids, board.board, camera_image.camera.camera_matrix, camera_image.camera.distortion_coefficients,
            None, None
        )

        if retval == 0:
            return None

        rvec = rvec.reshape(-1)
        tvec = tvec.reshape(-1)

        if self.config.draw_axis:
            cv2.aruco.drawAxis(camera_image.image, camera_image.camera.camera_matrix, camera_image.camera.distortion_coefficients, rvec, tvec, board.marker_length)

        if self.config.print_in_image:
            dist = np.linalg.norm(tvec)

            strg = 'ID {} distance: {:.3f}m'.format(board.id, dist)
            strg2 = 'x: {:.3f} y: {:.3f} z: {:.3f}'.format(*tvec)
            strg3 = 'r: {:.3f} p: {:.3f} y: {:.3f}'.format(*map(math.degrees, rvec))

            cv2.putText(camera_image.image, strg, (0, text_place), self.font,
                        1, (0, 255, 0), 2, cv2.LINE_AA)
            text_place += 30
            cv2.putText(camera_image.image, strg2, (0, text_place), self.font,
                        1, (0, 0, 255), 2, cv2.LINE_AA)
            text_place += 30
            cv2.putText(camera_image.image, strg3, (0, text_place), self.font,
                        1, (0, 0, 255), 2, cv2.LINE_AA)

        detection = DetectionOutput()
        detection.pack(camera_image, np.array([board.first_marker]), [rvec], [tvec], [board.type_id])
        return detection

    def detect_single_markers(self, camera_image):
        # corners, ids, rejected = cv2.aruco.detectMarkers(camera_image.image, self.marker_config.dictionary)
        corners, ids = self.detect_markers(camera_image, self.marker_config.dictionary)
        base_offset = 30

        if ids is None:
            # no marker detected
            return None

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_config.marker_length, camera_image.camera.camera_matrix, camera_image.camera.distortion_coefficients
        )

        rvecs = rvecs.reshape(-1, 3)
        tvecs = tvecs.reshape(-1, 3)
        ids = ids.reshape(-1)

        if self.config.draw_detected_markers:
            cv2.aruco.drawDetectedMarkers(camera_image.image, corners, ids)

        for id, rvec, tvec in zip(ids, rvecs, tvecs):
            if self.config.draw_axis:
                cv2.aruco.drawAxis(
                    camera_image.image, camera_image.camera.camera_matrix,
                    camera_image.camera.distortion_coefficients, rvec, tvec, self.marker_config.marker_length / 2)

            if self.config.print_in_image:
                dist = np.linalg.norm(tvec)

                strg = 'ID: {} Distance: {:.3f} m'.format(id, dist)
                strg2 = 'x: {:.3f} y: {:.3f} z: {:.3f}'.format(*tvec)
                strg3 = 'r: {:.3f} p: {:.3f} y: {:.3f}'.format(*map(math.degrees, rvec))

                text_place = base_offset + id*(30*3+20)

                cv2.putText(camera_image.image, strg, (0, text_place), self.font,
                            1, (0, 255, 0), 2, cv2.LINE_AA)
                text_place += 30
                cv2.putText(camera_image.image, strg2, (0, text_place), self.font,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
                text_place += 30
                cv2.putText(camera_image.image, strg3, (0, text_place), self.font,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
                text_place += 30

        detection = DetectionOutput()
        detection.pack(camera_image, ids, rvecs, tvecs, ['M']*len(ids))
        return detection

    def detect(self, camera_image):
        detection = DetectionOutput()
        detection.pack_dummy(camera_image)

        # detect boards first so that boards are not confused for single markers
        for board_config in self.board_config:
            detection.append(self.detect_board(camera_image, board_config))

        if self.marker_config is not None:
            detection.append(self.detect_single_markers(camera_image))

        return detection

    def run(self):
        while self.running:
            self.detected_markers.clear()
            image = self.image_distributor.get_image(self.id)
            if image is None:
                continue
            detection = self.detect(image)
            if detection.timestamp is None:
                detection = None
            self.image_distributor.put_detection(self.id, detection, image)
        self.logger.info("{} end".format(self.name))
