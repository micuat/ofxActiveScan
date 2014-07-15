ofxActiveScan
========

Naoto Hieda <micuat@gmail.com>, 09 July 2013


About
--------

This repository is an openFrameworks addon for active 3D scanning for
uncalibrated projector-camera system. While this repository takes ofxaddon-style,
you can start 3D scanning just by compiling the included example projects.

3D scanning algorithm is heavily relying on the software provided by
[Dr. Shuntaro Yamazaki][1] and visualization is inspired by [ProCamToolkit][2].


Dependencies
--------

* ofxOpenCv
* [ofxCv](https://github.com/kylemcdonald/ofxCv)
    * required for `example_triangulate` and Kinect examples
* ofxKinect
    * required for Kinect examples


Data Folder
--------

In ofxActiveScan examples, a data folder stores all information of
a projector-camera pair, including calibration parameters and a point cloud.

In order to specify the data folder, just drag the folder icon
to the openFrameworks window. The folder path can be specified by 
a command-line argument as well.

To begin with, only the file `config.yml` must be created in the folder.
Look for `myData` for an example.

First, use example-encode to capture structured-light images,
then, example-calibrate to solve camera parameters,
and finally, example-triangulate to reconstruct a point cloud.


### config.yml

Set parameters for example projects:

* proWidth/proHeight/camWidth/camHeight
    * specify projector/camera image size
* grayLow/grayHigh
    * *specific to example-encode*
    * black and white value for structured light (0-255; grayLow currently ignored)
    * grayHigh must be carefully adjusted to avoid saturation
* devID
    * *specific to example-encode*
    * device ID of the camera
* bufferTime
    * *specific to example-encode*
    * buffer time for structured light capturing (milliseconds)
    * set longer when camera buffer is too long
* vertical_center
    * y value of the principal point of the projector divided by image height (0: top of the image, 1: bottom)
    * can be calculated from parameters in a user manual of the projector
        * 0.83 for EMP765
        * 0.92 for EMP1735W 16:9
        * 0.87 for EMP1735W 16:10
        * 0.86 for EMP1735W 4:3
        * 1.12 for XD490U 4:3


example_encode
--------

This app projects and captures structured light patterns.
[f] to toggle fullscreen and [space] to start capturing.
The app window must be set fullscreen on the projector desktop before starting.

An image is saved for colored point cloud to `camPerspective.jpg`.

After scanning, the result is decoded and following files are saved:
`h.map`, `v.map`, `mask.png`, `reliable.png`


example_calibrate
--------

This app solves camera/projector intrinsic parameters and extrinsic paramters from
the example_encode outputs. Calibration results are saved to `calibration.yml`.


example_triangulate
--------

Finally, this app reconstructs a 3D point cloud. The point cloud is saved to `out.ply`.
Press \[1] to view the point cloud, \[2] to view the projector perspective
and \[3] to camera perspective.


License
--------

This software is provided under the terms of the GNU General Public License.


[1]: http://staff.aist.go.jp/shun-yamazaki/research/calibration/ "Self-Calibration of Projector Camera Systems"
[2]: https://github.com/YCAMInterlab/ProCamToolkit/ "ProCamToolkit"
