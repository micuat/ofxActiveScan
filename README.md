ofxActiveScan
========

Naoto Hieda <micuat@gmail.com>, 09 July 2013


About
--------

This repository is an openFrameworks addon for active 3D scanning using
uncalibrated projector-camera system. While this takes addon-style,
you can start 3D scanning just by compiling the included example projects.

3D scanning algorithm is heavily relying on the software provided by
[Mr. Shuntaro Yamazaki][1] and visualization is inspired by [ProCamToolkit][2].


Dependencies
--------

* ofxOpenCv
* [ofxCv](https://github.com/kylemcdonald/ofxCv)
* [ofxLibdc (optional)](https://github.com/kylemcdonald/ofxLibdc)
* [ofxPCL (optional)](https://github.com/satoruhiga/ofxPCL)

**Following instructions to be updated**


config.yml
--------

Set parameters for example projects:

* proWidth/proHeight/camWidth/camHeight: specify projector/camera image size
* grayLow/grayHigh: black and white value for structured light (0-255; grayLow currently ignored)
* devID: device ID of the camera when using ofVideoGrabber
* bufferTime: buffer time for structured light capturing (milliseconds)
* vertical_center: principal point of the projector (0: top of the image, 1: bottom)
* nsamples

All saved data are stored in `ofxActiveScan/SharedData`.
However, you must create `example-*/bin/data` (for openFrameworks) or 
`example-*/share/openframeworks` (for libopenframeworks) directory 
before launching the examples due to the openFrameworks implementation.


example-encode
--------

This app projects and captures structured light patterns.
[f] to toggle fullscreen and [space] to start capturing.
The app window must be set fullscreen on the projector desktop.

Comment or uncomment `#define USE_LIBDC` to choose ofxLibdc or ofVideoGrabber.

Captured images are saved to `img`. The last image is taken for
colored point cloud and saved to `camPerspective.jpg`.


example-decode
--------

This app decodes the results of example-encode and outputs
`h.map`, `v.map`, `mask.bmp` and `reliable.bmp`.


example-calibrate
--------

This app solves camera/projector intrinsic parameters and extrinsic paramters from
the example-decode outputs. Calibration result is saved to `calibration.yml`.


example-triangulate
--------

Finally, this app reconstructs a 3D point cloud. The point cloud is saved to `out.ply`.
Press [1] to view the point cloud (not working), [2] to view the projector perspective
and [3] to camera perspective.


example-recalibrate
--------

This app is for manually adjusting the point cloud and the calibration parameters.
First, press [2] and [f] to project the point cloud on to the actual scene.
Then, pick a point, press [space], and move the point that is selected and changed blue
to the corresponding actual point by pressing arrow keys. When finished, press [enter].
Iterate these steps for at least six points, and press [c] to adjust the point cloud.


example-pcl
--------

This app is a PCL segmentation demo for projection mapping. The loaded 3D point cloud
is segmented by PCL segmentation feature, and subsequently displayed.
Press [2] and [f] to start projection mapping.


License
--------

This software is provided under the terms of the GNU General Public License.


[1]: http://staff.aist.go.jp/shun-yamazaki/research/calibration/ "Self-Calibration of Projector Camera Systems"
[2]: https://github.com/YCAMInterlab/ProCamToolkit/ "ProCamToolkit"
