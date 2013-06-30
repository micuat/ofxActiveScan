#pragma once

#include "ofMain.h"

#include "ofxActiveScan.h"
#include "ofxCv.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	
private:
	ofxActiveScan::Options options;
	ofMesh mesh;
	ofEasyCam cam;
	
	int cw, ch;
	
	string rootDir;
};
