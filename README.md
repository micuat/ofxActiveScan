ofxActiveScan
========

Naoto Hieda <micuat@gmail.com>, 09 July 2013


About
--------

This repository is an openFrameworks addon for active 3D scanning for
uncalibrated projector-camera system. While this repository takes ofxaddon-style,
you can start 3D scanning just by compiling the included example projects.

3D scanning algorithm is heavily relying on the software provided by
[Mr. Shuntaro Yamazaki][1] and visualization is inspired by [ProCamToolkit][2].


Dependencies
--------

* ofxOpenCv
* [ofxCv](https://github.com/kylemcdonald/ofxCv)
* [ofxLibdc (optional)](https://github.com/kylemcdonald/ofxLibdc)
    * required for example-encode when using Libdc cameras
* [ofxPCL (optional)](https://github.com/satoruhiga/ofxPCL)
    * required for example-pcl; **currently requires modified ofxPCL**


Data Folder
--------

In ofxActiveScan examples, a data folder stores all information of
a projector-camera pair, including captured structured light images,
calibration parameters, a point cloud. Therefore, if you use
two projectors P1, P2 and one camera C1, you will need to create
two data folders for the P1-C1 pair and P2-C1 pair.

In order to pass the data folder path to an app, just drag the folder icon
to the openFrameworks window. For Linux users, since this feature is not supported,
specify the path as a command-line argument.

To begin with, only the file `config.yml` must be created in advance.
An example data folder is saved as `myData` in the repository.

First, use example-encode to capture structured-light images,
then, example-decode to decode the images,
next, example-calibrate to solve camera parameters,
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

* nsamples
    * default: 1000


example-encode
--------

This app projects and captures structured light patterns.
[f] to toggle fullscreen and [space] to start capturing.
The app window must be set fullscreen on the projector desktop.

Comment or uncomment `#define USE_LIBDC` to choose ofVideoGrabber or ofxLibdc.

Captured images are saved to `img`. The last image is taken for
colored point cloud and saved to `camPerspective.jpg`.


example-decode
--------

This app decodes the results of example-encode and outputs
`h.map`, `v.map`, `mask.bmp` and `reliable.bmp`.


example-calibrate
--------

This app solves camera/projector intrinsic parameters and extrinsic paramters from
the example-decode outputs. Calibration results are saved to `calibration.yml`.


example-triangulate
--------

Finally, this app reconstructs a 3D point cloud. The point cloud is saved to `out.ply`.
Press \[1] to view the point cloud (not working), \[2] to view the projector perspective
and \[3] to camera perspective.


example-recalibrate
--------

This app is for manually adjusting the point cloud and the calibration parameters.
First, press \[2] and [f] to project the point cloud on to the actual scene.
Then, pick a point, press [space], and move the point that is selected and changed blue
to the corresponding actual point by pressing arrow keys. When finished, press [enter].
Iterate these steps for at least six points, and press [c] to adjust the point cloud.


example-pcl
--------

This app is a PCL segmentation demo for projection mapping. The loaded 3D point cloud
is segmented by PCL segmentation feature, and subsequently displayed.
Press \[2] and [f] to start projection mapping.


example-stitch
--------

This app stitches two point clouds and outputs better camera parameters as `calibration2.yml`.
Requires two input data folders.


License
--------

This software is provided under the terms of the GNU General Public License.


[1]: http://staff.aist.go.jp/shun-yamazaki/research/calibration/ "Self-Calibration of Projector Camera Systems"
[2]: https://github.com/YCAMInterlab/ProCamToolkit/ "ProCamToolkit"
