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
	fs["grayLow"] >> grayLow;
	fs["grayHigh"] >> grayHigh;
	fs["devID"] >> devID;
	fs["bufferTime"] >> bufferTime;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	encode.init(options);
	
	camera.listDevices();
	camera.setDeviceID(devID);
	camera.initGrabber(cw, ch);
	
	curIndex = -1;
	captureTime = 0;
	
	ofDirectory::createDirectory(rootDir + "img/", true, true);
}

void testApp::update() {
	unsigned long curTime = ofGetSystemTime();
	bool needToCapture = (0 <= curIndex) && (curIndex < encode.getSize())
		&& ((curTime - captureTime) > bufferTime);
	
	camera.update();
	if(camera.isFrameNew() && needToCapture) {
		curFrame.setFromPixels(camera.getPixels(), cw, ch, OF_IMAGE_COLOR);
		curFrame.saveImage(rootDir + "img/" + ofToString(curIndex + 10) + ".bmp");
//		curFrame.saveImage(rootDir + "img/" + ofToString(curIndex, 2, '0') + ".bmp");
		captureTime = curTime;
		curIndex++;
		if( curIndex < encode.getSize() ) {
			curPattern = encode.getPatternAt(curIndex);
		} else {
			curIndex = -1;
		}
	}
}

void testApp::draw() {
	ofBackground(0);
	
	if( curIndex >= 0 ) {
		ofSetColor(grayHigh);
		curPattern.draw(0, 0);
	} else {
		camera.draw(0, 0);
	}
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		curIndex = 0;
		curPattern = encode.getPatternAt(curIndex);
		captureTime = ofGetSystemTime();
	}
	if( key == 'f' ) {
		ofToggleFullscreen();
	}
}
