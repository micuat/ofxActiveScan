/*
    This file is part of ofxActiveScan.

    ofxActiveScan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ofxActiveScan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ofxActiveScan.  If not, see <http://www.gnu.org/licenses/>.

    Naoto Hieda <micuat@gmail.com> 2013
 */

#include "testApp.h"

using namespace ofxActiveScan;

void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	Map2f asMask, asReliable;
	decode(options, mapHorizontal, mapVertical, asMask, asReliable,
		   ofToDataPath(rootDir[0] + "/img/", true));
	
	// Save resulting maps
	mapHorizontal.Write(ofToDataPath(rootDir[0] + "/h.map", true));
	mapVertical.Write(ofToDataPath(rootDir[0] + "/v.map", true));
	
	mapMask = toOf(asMask);
	mapReliable = toOf(asReliable);
	mapMask.saveImage(ofToDataPath(rootDir[0] + "/mask.bmp"));
	mapReliable.saveImage(ofToDataPath(rootDir[0] + "/reliable.bmp"));
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
	
	ofScale(0.5, 0.5);
	mapMask.draw(0, 0);
	mapReliable.draw(mapMask.getWidth(), 0);
}

void testApp::keyPressed(int key) {
}
