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

* proWidth/proHeight/camWidth/camHeight
* grayLow/grayHigh
* devID
* bufferTime
* vertical_center
* nsamples


example-encode
--------

Project and capture structured light patterns.


example-decode
--------

Decode results from example-encode.


example-calibrate
--------

Solve camera/projector intrinsic parameters and extrinsic paramters from
decoded results.


example-triangulate
--------

Finally reconstruct a 3D point cloud from calibration parameters
and decoded results.


example-recalibrate
--------

Manually adjust calibration parameters.


example-pcl
--------

PCL segmentation demo for projection mapping.


License
--------

This software is provided under the terms of the GNU General Public License.


[1]: http://staff.aist.go.jp/shun-yamazaki/research/calibration/ "Self-Calibration of Projector Camera Systems"
[2]: https://github.com/YCAMInterlab/ProCamToolkit/ "ProCamToolkit"
