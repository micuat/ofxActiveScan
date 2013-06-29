#pragma once

#include <stdlib.h>

#include "stdafx.h"

#include "decode.h"

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
	
	int cw, ch;
	
	string rootDir;
	
	int argc;
	char **argv;
};
