#pragma once

#include "ofMain.h"
#include "ofxCv.h"

#include "ofxActiveScan.h"

class testApp : public ofBaseApp {
public:
	testApp() {}
	testApp(int _argc, char** _argv) {
		argc = _argc;
		argv = _argv;
	}
	void setup();
	void update();
	void draw();
	void keyPressed(int);
	
private:
	int argc;
	char** argv;
	
	ofxActiveScan::Encode encode;
	ofImage curPattern;
	ofImage curFrame;
	ofVideoGrabber camera;
	
	int curIndex;
	int cw, ch;
	int grayLow, grayHigh;
	int bufferTime;
	unsigned long captureTime;
};
