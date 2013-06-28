#pragma once

#include "ofMain.h"
#include "ofxCv.h"

#include "ofxActiveScan.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int);
	
private:
	ofxActiveScan::Encode encode;
	ofxActiveScan::Options options;
	ofImage curPattern;
	ofImage curFrame;
	ofVideoGrabber camera;
	
	int curIndex;
	int cw, ch;
	int grayLow, grayHigh;
	int bufferTime;
	unsigned long captureTime;
	
	string rootDir;
};
