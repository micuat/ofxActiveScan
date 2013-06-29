#include "testApp.h"

void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	
	// load correspondences estimated by decode program 
	slib::Field<2,float> horizontal(argv[2]);
	slib::Field<2,float> vertical(argv[3]);  
	slib::Field<2,float> mask;  
	slib::image::Read(mask, argv[4]);
	
	CProCamCalibrate calib(argv[1]);
	calib.Calibrate(horizontal,vertical,mask);
	
	calib.WriteCamIntrinsic("cam-intrinsic.txt");
	calib.WriteCamDistortion("cam-distortion.txt");
	calib.WriteProIntrinsic("pro-intrinsic.txt");
	calib.WriteProDistortion("pro-distortion.txt");
	calib.WriteProExtrinsic("pro-extrinsic.txt");
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
}

void testApp::keyPressed(int key) {
}
