//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4547 2011-05-24 14:04:52Z shun $
//

#include "testApp.h"

void print_usage(const char *argv0)
{
	const char *prog = strrchr(argv0, '\\');
	if (prog)
		prog++;
	else
		prog = argv0;
	printf("Usage: %s <options>\n", prog);
	exit(-1);
}

void testApp::setup() {
	if (argc != 2)
		print_usage(argv[0]);
	
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	cv::FileStorage fs(ofToDataPath("config.yml"), cv::FileStorage::READ);
	int devID, pw, ph, grayLow, grayHigh;
	fs["proWidth"] >> pw;
	fs["proHeight"] >> ph;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["grayLow"] >> grayLow;
	fs["grayHigh"] >> grayHigh;
	fs["devID"] >> devID;
	fs["bufferTime"] >> bufferTime;
	
	encode.init(argv[1]);
	
	camera.listDevices();
	camera.setDeviceID(devID);
	camera.initGrabber(cw, ch);
	
	curIndex = -1;
	captureTime = 0;
}

void testApp::update() {
	unsigned long curTime = ofGetSystemTime();
	bool needToCapture = (0 <= curIndex) && (curIndex < encode.getSize())
		&& ((curTime - captureTime) > bufferTime);
	
	camera.update();
	if(camera.isFrameNew() && needToCapture) {
		curFrame.setFromPixels(camera.getPixels(), cw, ch, OF_IMAGE_COLOR);
		curFrame.saveImage(ofToString(curIndex) + ".bmp");
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
