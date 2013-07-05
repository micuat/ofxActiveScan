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
	
	Map2f asMask, asReliable;
	decode(options, mapHorizontal, mapVertical, asMask, asReliable,
		   ofToDataPath(rootDir + "img/", true));
	
	// Save resulting maps
	mapHorizontal.Write(ofToDataPath(rootDir + "/h.map", true));
	mapVertical.Write(ofToDataPath(rootDir + "/v.map", true));
	
	mapMask = toOf(asMask);
	mapReliable = toOf(asReliable);
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
