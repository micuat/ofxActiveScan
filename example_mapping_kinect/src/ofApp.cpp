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

// entry point
void ofApp::setup() {
	if( rootDir.size() > 0 ) {
		init();
		pathLoaded = true;
	} else {
		pathLoaded = false;
	}
}

void ofApp::init() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	
	kinect.open();		// opens first available kinect
	
	cameraMode = EASYCAM_MODE;
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::READ);
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	proSize.width = options.projector_width;
	proSize.height = options.projector_height;
	cout << proIntrinsic << endl;
	cout << proExtrinsic << endl;
	
	// set parameters for projection
	proCalibration.setup(proIntrinsic, proSize);
	
	ofEnableDepthTest();
}

void ofApp::update() {
	kinect.update();
}

void ofApp::draw() {
	if( pathLoaded ) {
		
		ofBackground(0);
		
		if(cameraMode == EASYCAM_MODE) {
			cam.begin();
			ofScale(1, -1, -1);
			ofScale(1000, 1000, 1000);
			ofTranslate(0, 0, -2);
		} else if(cameraMode == PRO_MODE) {
			ofSetupScreenPerspective(options.projector_width, options.projector_height);
			proCalibration.loadProjectionMatrix(0.0001, 100000000.0);
			cv::Mat m = proExtrinsic;
			cv::Mat extrinsics = (cv::Mat1d(4,4) <<
								  m.at<double>(0,0), m.at<double>(0,1), m.at<double>(0,2), m.at<double>(0,3),
								  m.at<double>(1,0), m.at<double>(1,1), m.at<double>(1,2), m.at<double>(1,3),
								  m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2), m.at<double>(2,3),
								  0, 0, 0, 1);
			extrinsics = extrinsics.t();
			glMultMatrixd((GLdouble*) extrinsics.ptr(0, 0));
		}
		
		drawPointCloud();
		
		if(cameraMode == EASYCAM_MODE) {
			cam.end();
		}
		
	}
}


void ofApp::drawPointCloud() {
	
	int w = 640;
	int h = 480;
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_POINTS);
	glPointSize(2);
	int step = 2;
	
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0 && kinect.getDistanceAt(x, y) < ofMap(mouseX, 0, ofGetWidth(), 0, 3000) ) {
				mesh.addColor(kinect.getDepthPixelsRef().getColor(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	mesh.draw();
}

void ofApp::keyPressed(int key) {
	if( pathLoaded ) {
		
		switch(key) {
			case '1': cameraMode = EASYCAM_MODE; break;
			case '2': cameraMode = PRO_MODE; break;
		}
		
	}
	
	if( key == 'f' ) {
		ofToggleFullscreen();
	}
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
