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
	
	patterns = ofxActiveScan::encode(options);
	
#ifdef USE_LIBDC
	camera.setup();
	camera.set1394b(true);
	camera.setSize(cw, ch);
	camera.setBrightness(0);
	camera.setGain(0);
	camera.setExposure(1);
	camera.setGammaAbs(1);
	camera.setShutter(1);
	camera.printFeatures();
#else
	camera.listDevices();
	camera.setDeviceID(devID);
	camera.initGrabber(cw, ch);
#endif
	
	curIndex = -1;
	captureTime = 0;
	
	ofDirectory::createDirectory(rootDir + "img/", true, true);
}

void testApp::update() {
	unsigned long curTime = ofGetSystemTime();
	bool needToCapture = (0 <= curIndex) && (curIndex < patterns.size())
		&& ((curTime - captureTime) > bufferTime);
	
#ifdef USE_LIBDC
	if(camera.grabVideo(curFrame) && needToCapture) {
#else
	curFrame.setFromPixels(camera.getPixels(), cw, ch, OF_IMAGE_COLOR);
	if(camera.isFrameNew() && needToCapture) {
#endif
		curFrame.saveImage(rootDir + "img/" + ofToString(curIndex + 10) + ".bmp");
		captureTime = curTime;
		curIndex++;
		if( curIndex < patterns.size() ) {
			curPattern = patterns[curIndex];
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
		curFrame.update();
		curFrame.draw(0, 0);
	}
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		curIndex = 0;
		curPattern = patterns[curIndex];
		captureTime = ofGetSystemTime();
	}
	if( key == 'f' ) {
		ofToggleFullscreen();
	}
}
