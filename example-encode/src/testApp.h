#pragma once

#include "ofMain.h"

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
	int curIndex;
};
