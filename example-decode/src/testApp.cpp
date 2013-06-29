#include "testApp.h"

void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	
	decode.init(options, ofToDataPath(rootDir + "img/", true));
	
	// Save resulting maps
	decode.getMapHorizontal().Write(ofToDataPath(rootDir + "/h.map", true));
	decode.getMapVertical().Write(ofToDataPath(rootDir + "/v.map", true));
	
	mapMask = ofxActiveScan::toOf(decode.getMask());
	mapReliable = ofxActiveScan::toOf(decode.getReliable());
	
	mapMask.saveImage(ofToDataPath(rootDir + "/mask.bmp"));
	mapReliable.saveImage(ofToDataPath(rootDir + "/reliable.bmp"));
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
	
	ofScale(0.5, 0.5);
	mapMask.draw(0, 0);
	mapReliable.draw(mapMask.getWidth(), 0);
}

void testApp::keyPressed(int key) {
}
