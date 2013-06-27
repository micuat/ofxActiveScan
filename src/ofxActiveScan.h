//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4547 2011-05-24 14:04:52Z shun $
//

#pragma once

#include "ofMain.h"

#include <stdlib.h>

#include "stdafx.h"
#include "MiscUtil.h"
#include "encode.h"

namespace ofxActiveScan {

class Encode {
	int size;
	int width;
	int height;
	vector<ofImage> patterns;

public:
	Encode() {}
	
	int init(char* fileName) {
		CEncode encode(fileName);
		
		size = encode.GetNumImages();
		for (int i = 0; i < size; i++) {
			slib::Field<2,float> fieldPattern;
			encode.GetImage(i, fieldPattern);
			
			width = fieldPattern.size(0);
			height = fieldPattern.size(1);
			ofImage pattern;
			pattern.allocate(width, height, OF_IMAGE_COLOR);
			patterns.push_back(pattern);
		}
		return 0;
	}
	
	int getSize() {
		return size;
	}
	
	int getWidth() {
		return width;
	}
	
	int getHeight() {
		return height;
	}
	
	ofImage& getPatternAt(int i) {
		return patterns[i];
	}
};

}
