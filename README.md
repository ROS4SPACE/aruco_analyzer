# Aruco Analyzer

## Description

The `aruco_analyzer` package is a ROS wrapper for the [ArUco](https://www.uco.es/investiga/grupos/ava/node/26) library based on Python.
It was originally developed by the Chair of Space Technology at [TU Berlin](www.tu.berlin)  for robotic applications (identification of objects and other robots) as well as for satellite qualification campaigns.
In case of the latter, it is intended to be used for verifying a satellite's attitude determination and control system (ADCS) without any interference with its sensors.

It shall be stressed that the detection of ArUco markers is completely done with the ArUco library, the wrapper however is designed to accomplish the following:

* Provide multi-camera support
* Generate correct transforms for detected markers in a defined parent frame
* Increase detection accuracy by utilizing multiple detections of the same marker
* Enable the differentiation between fixed and mobile markers for tests of
  * Robot navigation
  * Satellites' ADCS

The package was tested with various cameras, primarily webcams. Please note that a correct camera calibration
is essential for the detection accuracy.

## How to use

Check out the [wiki](https://github.com/ROS4SPACE/aruco_analyzer/wiki) for further information on how to use this package.

## Authors

* [**Lennart Kryza**](https://github.com/tublen) - *Initial work and design*
* [**Michael Rössler**](https://github.com/maroessler) - *Maintainer and code extensions*

## License

This project is licensed under the GNU General Public License v3.0.
