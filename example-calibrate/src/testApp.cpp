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

#include "testApp.h"

using namespace ofxActiveScan;

void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	// load correspondences estimated by decode program 
	Map2f horizontal(ofToDataPath(rootDir[0] + "/h.map", true));
	Map2f vertical(ofToDataPath(rootDir[0] + "/v.map", true));  
	ofImage mask;
	ofLoadImage(mask, ofToDataPath(rootDir[0] + "/mask.bmp"));
	
	Matd camIntrinsic, proIntrinsic;
	double camDistortion, proDistortion;
	Matd proExtrinsic;
	
	bool success = false;
	while( !success ) {
		try {
			calibrate(options, horizontal, vertical, toAs(mask),
					  camIntrinsic, camDistortion,
					  proIntrinsic, proDistortion, proExtrinsic);
			success = true;
		} catch ( std::runtime_error &e ) {
			ofLogVerbose() << "Caught exception : " << e.what();
			ofLogVerbose() << "Retrying calibration";
		}
	}
	
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::WRITE);
	cfs << "camIntrinsic"  << toCv(camIntrinsic);
	cfs << "camDistortion" << camDistortion;
	cfs << "proIntrinsic"  << toCv(proIntrinsic);
	cfs << "proDistortion" << proDistortion;
	cfs << "proExtrinsic"  << toCv(proExtrinsic);
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
}

void testApp::keyPressed(int key) {
}
