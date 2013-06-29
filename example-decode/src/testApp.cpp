#include "testApp.h"

void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	int devID;
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	
	std::vector<std::string> files(argc-2);
	for (int i=0; i<argc-2; i++)
		files[i]=argv[i+2];
	
	CDecode decode(argv[1]);
	decode.Decode(files);
	decode.WriteMap(0,"h.map");
	decode.WriteMap(1,"v.map");
	decode.WriteMask("mask.bmp");
	decode.WriteReliable("reliable.bmp");	
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
}

void testApp::keyPressed(int key) {
}
