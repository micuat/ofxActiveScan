#pragma once

#include "ofMain.h"

#include "stdafx.h"

#include "MathBaseLapack.h"
#include "Field.h"
#include "MiscUtil.h"
#include "ImageBmpIO.h"
#include "Stereo.h"
#include "LeastSquare.h"

#include "FundamentalMatrix.h"
#include "Options.h"

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
	
private:
	int argc;
	char** argv;
};
