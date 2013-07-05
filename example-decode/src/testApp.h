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
	ofxActiveScan::Options options;
	ofxActiveScan::Map2f mapHorizontal, mapVertical;
	ofImage mapMask, mapReliable;
	
	int cw, ch;
	
	string rootDir;
};
