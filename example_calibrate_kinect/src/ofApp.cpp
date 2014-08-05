/*
    This file is part of ofxActiveScan.

    ofxActiveScan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ofxActiveScan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ofxActiveScan.  If not, see <http://www.gnu.org/licenses/>.

    Naoto Hieda <micuat@gmail.com> 2013
 */

#include "ofApp.h"

using namespace ofxActiveScan;

void ofApp::setup() {
	if( rootDir.size() > 0 ) {
		init();
		pathLoaded = true;
	} else {
		pathLoaded = false;
	}
}

void ofApp::init() {
	ofSetLogLevel(OF_LOG_WARNING);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	
	kinect.open();		// opens first available kinect

	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	// load correspondences estimated by decode program 
	horizontal = Map2f(ofToDataPath(rootDir[0] + "/h.map", true));
	vertical = Map2f(ofToDataPath(rootDir[0] + "/v.map", true));
	ofImage mask;
	ofLoadImage(mask, ofToDataPath(rootDir[0] + "/mask.png"));
	maskMap = toAs(mask);
	
	ofImage depthImage;
	ofLoadImage(depthImage, ofToDataPath(rootDir[0] + "/camPerspectiveDepth.png", true));
	depthImage.setImageType(OF_IMAGE_GRAYSCALE);
	depth = depthImage.getPixelsRef();
}

void ofApp::update() {
	if( pathLoaded ) {
		kinect.update();
		if( kinect.isFrameNewDepth() ) {
			float f = 1500;
			cv::Mat proIntrinsic = (cv::Mat1d(3, 3) <<
								 f, 0, options.projector_width * 0.5 - 0.5,
								 0, f, options.projector_height * options.projector_horizontal_center - 0.5,
								 0, 0, 1);
			cv::Mat camIntrinsic;
			double camDistortion, proDistortion;
			cv::Mat proExtrinsic;
			
			cv::Mat distCoeffs;
			vector<cv::Mat> rvecs, tvecs;
			int flags =
			CV_CALIB_USE_INTRINSIC_GUESS |
			CV_CALIB_FIX_PRINCIPAL_POINT |
			CV_CALIB_FIX_ASPECT_RATIO |
			//CV_CALIB_FIX_K1 |
			//CV_CALIB_FIX_K2 |
			//CV_CALIB_FIX_K3 |
			CV_CALIB_ZERO_TANGENT_DIST;
			vector<vector<cv::Point3f> > referenceObjectPoints(1);
			vector<vector<cv::Point2f> > referenceImagePoints(1);
			int w = 640;
			int h = 480;
			int step = 2;
			for(int y = 0; y < h; y += step) {
				for(int x = 0; x < w; x += step) {
					float dist = depth.getPixels()[y * w + h];
					if( maskMap.cell(x, y) <= 0 ) continue;
					if(dist > 0) {
						referenceObjectPoints[0].push_back(ofxCv::toCv(kinect.getWorldCoordinateAt(x, y, dist)));
						referenceImagePoints[0].push_back(cv::Point2f(horizontal.cell(x, y), vertical.cell(x, y)));
//						referenceImagePoints[0].push_back(cv::Point2f(x, y));
					}
				}
			}
			
			cv::Size imageSize(options.projector_width, options.projector_height * (options.projector_horizontal_center + 0.1));
			cv::calibrateCamera(referenceObjectPoints, referenceImagePoints, imageSize, proIntrinsic, distCoeffs, rvecs, tvecs, flags);
			
//			ofxCv::Intrinsics intrinsics;
//			intrinsics.setup(proIntrinsic, imageSize);
			cv::Mat rot3x3;
			cv::Rodrigues(rvecs[0], rot3x3);
			double* rm = rot3x3.ptr<double>(0);
			double* tm = tvecs[0].ptr<double>(0);
			proExtrinsic = (cv::Mat1d(4,3) << rm[0], rm[3], rm[6],
							   rm[1], rm[4], rm[7],
							   rm[2], rm[5], rm[8],
							   tm[0], tm[1], tm[2]);
			proExtrinsic = proExtrinsic.t();
			
			cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::WRITE);
			cfs << "camIntrinsic"  << camIntrinsic;
			cfs << "camDistortion" << camDistortion;
			cfs << "proIntrinsic"  << proIntrinsic;
			cfs << "proDistortion" << proDistortion;
			cfs << "proExtrinsic"  << proExtrinsic;
			
			ofLogWarning() << proIntrinsic;
			ofLogWarning() << proExtrinsic;
			
			pathLoaded = false;
		}
	}
}

void ofApp::draw() {
	ofBackground(0);
}

void ofApp::keyPressed(int key) {
}


void ofApp::dragEvent(ofDragInfo dragInfo){
	if( !pathLoaded ) {
		for( int i = 0 ; i < dragInfo.files.size() ; i++ ) {
			rootDir.push_back(dragInfo.files[i]);
		}
		init();
		pathLoaded = true;
	}
}
