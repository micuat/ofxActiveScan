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

#include "ProCamTools/stdafx.h"
#include "ProCamTools/common/MiscUtil.h"
#include "ProCamTools/encode.h"
#include "ofxActiveScanTypes.h"
#include "ofxActiveScanUtils.h"

namespace ofxActiveScan {

class Encode {
	int size;
	int width;
	int height;
	vector<ofImage> patterns;

public:
	Encode() {}
	
	int init(Options);
	
	int getSize();
	int getWidth();
	int getHeight();
	
	ofImage& getPatternAt(int i);
};

}
