#pragma once

#include "ofMain.h"

#include <stdlib.h>

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
