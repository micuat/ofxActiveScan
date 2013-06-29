#pragma once

#include "ofMain.h"
#include "ofxCv.h"

#include "ofxActiveScan.h"

#include "stdafx.h"

#include "Field.h"
#include "ImageBmpIO.h"

#include "calibrate.h"

#undef min
#undef max

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int);
	
private:
	ofxActiveScan::Options options;
	
	int cw, ch;
	
	string rootDir;
	
	int argc;
	char** argv;
};
