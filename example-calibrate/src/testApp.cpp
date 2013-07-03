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
	
	slib::CMatrix<3,3,double> camIntrinsic, proIntrinsic;
	double camDistortion, proDistortion;
	slib::CMatrix<3,4,double> proExtrinsic;

	ofxActiveScan::calibration(options, horizontal, vertical, mask,
							   camIntrinsic, camDistortion,
							   proIntrinsic, proDistortion, proExtrinsic);
	
	cv::FileStorage ofs(ofToDataPath(rootDir + "/calibration.yml"), cv::FileStorage::WRITE);
	ofs << "camIntrinsic"  << ofxActiveScan::toOf(camIntrinsic);
	ofs << "camDistortion" << camDistortion;
	ofs << "proIntrinsic"  << ofxActiveScan::toOf(proIntrinsic);
	ofs << "proDistortion" << proDistortion;
	ofs << "proExtrinsic"  << ofxActiveScan::toOf(proExtrinsic);
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
}

void testApp::keyPressed(int key) {
}
