#include "testApp.h"

void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	// load correspondences estimated by decode program 
	ofxActiveScan::Map2f horizontal(ofToDataPath(rootDir + "/h.map", true));
	ofxActiveScan::Map2f vertical(ofToDataPath(rootDir + "/v.map", true));  
	ofxActiveScan::Map2f mask;
	slib::image::Read(mask, ofToDataPath(rootDir + "/mask.bmp", true));
	
	CProCamCalibrate calib(options);
	calib.Calibrate(horizontal,vertical,mask);
	
	calib.WriteCamIntrinsic(ofToDataPath(rootDir + "cam-intrinsic.txt", true));
	calib.WriteCamDistortion(ofToDataPath(rootDir + "cam-distortion.txt", true));
	calib.WriteProIntrinsic(ofToDataPath(rootDir + "pro-intrinsic.txt", true));
	calib.WriteProDistortion(ofToDataPath(rootDir + "pro-distortion.txt", true));
	calib.WriteProExtrinsic(ofToDataPath(rootDir + "pro-extrinsic.txt", true));
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
}

void testApp::keyPressed(int key) {
}
