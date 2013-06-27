//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4547 2011-05-24 14:04:52Z shun $
//

#include "testApp.h"

void print_usage(const char *argv0)
{
	const char *prog = strrchr(argv0, '\\');
	if (prog)
		prog++;
	else
		prog = argv0;
	printf("Usage: %s <options>\n", prog);
	exit(-1);
}

void testApp::setup() {
	if (argc != 2)
		print_usage(argv[0]);
	
	encode.init(argv[1]);
	
	curIndex = 0;
}

void testApp::update() {
}

void testApp::draw() {
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		curPattern = encode.getPatternAt(curIndex);
		curIndex++;
	}
}
