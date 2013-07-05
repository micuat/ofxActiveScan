#include "testApp.h"

using namespace ofxActiveScan;

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
	Map2f horizontal(ofToDataPath(rootDir + "/h.map", true));
	Map2f vertical(ofToDataPath(rootDir + "/v.map", true));  
	ofImage mask;
	ofLoadImage(mask, ofToDataPath(rootDir + "/mask.bmp"));
	
	Matd camIntrinsic, proIntrinsic;
	double camDistortion, proDistortion;
	Matd proExtrinsic;

	calibrate(options, horizontal, vertical, toAs(mask),
			  camIntrinsic, camDistortion,
			  proIntrinsic, proDistortion, proExtrinsic);
	
	cv::FileStorage cfs(ofToDataPath(rootDir + "/calibration.dummy.yml"), cv::FileStorage::WRITE);
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
