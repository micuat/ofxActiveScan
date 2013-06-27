#pragma once

#include "ofMain.h"

#include <stdlib.h>

// convert TRACE into ofLog
#define TRACE DEBUGofLog

#if __STDC_VERSION__ >= 199901L
	#define DEBUGofLog(__VA_ARGS__) ofLog(OF_LOG_NOTICE, __VA_ARGS__)
#else
	#define DEBUGofLog(args...) ofLog(OF_LOG_NOTICE, ##args)
#endif
//

#include "stdafx.h"
#include "MiscUtil.h"
#include "encode.h"

namespace ofxActiveScan {

ofImage toOf(slib::Field<2,float>);

class Encode {
	int size;
	int width;
	int height;
	vector<ofImage> patterns;

public:
	Encode() {}
	
	int init(char*);
	
	int getSize();
	int getWidth();
	int getHeight();
	
	ofImage& getPatternAt(int i);
};

}
