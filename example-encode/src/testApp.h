#pragma once

#define USE_LIBDC

#include "ofMain.h"
#include "ofxCv.h"

#include "ofxActiveScan.h"

#ifdef USE_LIBDC
#include "ofxLibdc.h"
#endif

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int);
	
private:
	vector<ofFloatImage> patterns;
	ofxActiveScan::Options options;
	ofFloatImage curPattern;
	ofImage curFrame;
	
#ifdef USE_LIBDC
	ofxLibdc::PointGrey camera;
#else
	ofVideoGrabber camera;
#endif
	
	int curIndex;
	int cw, ch;
	int grayLow, grayHigh;
	int bufferTime;
	unsigned long captureTime;
	
	string rootDir;
};
