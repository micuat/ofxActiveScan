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

void testApp::setup() {
	if( rootDir.size() > 0 ) {
		init();
		pathLoaded = true;
	} else {
		pathLoaded = false;
	}
}

void testApp::init() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofHideCursor();
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	int devID;
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["grayLow"] >> grayLow;
	fs["grayHigh"] >> grayHigh;
	fs["devID"] >> devID;
	fs["bufferTime"] >> bufferTime;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	patterns = ofxActiveScan::encode(options);
	ofImage blank;
	blank.allocate(options.projector_width, options.projector_height, OF_IMAGE_GRAYSCALE);
	patterns.push_back(blank);
	
#ifdef USE_LIBDC
	camera.setup(devID);
	camera.set1394b(true);
	camera.setSize(cw, ch);
	camera.setBrightness(0);
	camera.setGain(0);
	camera.setExposure(1);
	camera.setGammaAbs(1);
	camera.setShutter(1);
	camera.printFeatures();
#else
	camera.listDevices();
	camera.setDeviceID(devID);
	camera.initGrabber(cw, ch);
#endif
	
	curIndex = -1;
	captureTime = 0;
	
	ofDirectory::createDirectory(rootDir[0] + "/img/", true, true);
}

void testApp::update() {
	if( pathLoaded ) {
		
		unsigned long curTime = ofGetSystemTime();
		bool needToCapture = (0 <= curIndex) && (curIndex < patterns.size())
			&& ((curTime - captureTime) > bufferTime);
		
#ifdef USE_LIBDC
		if(camera.grabVideo(curFrame) && needToCapture) {
#else
		camera.update();
		curFrame.setFromPixels(camera.getPixels(), cw, ch, OF_IMAGE_COLOR);
		if(camera.isFrameNew() && needToCapture) {
#endif
			if( curIndex < patterns.size() - 1 ) {
				curFrame.saveImage(rootDir[0] + "/img/" + ofToString(curIndex + 10) + ".bmp");
				captureTime = curTime;
				curIndex++;
				curPattern = patterns[curIndex];
			} else {
				// last frame is for color mapping
				curFrame.saveImage(rootDir[0] + "/camPerspective.jpg");
				curIndex = -1;
				captureTime = 0;
			}
		}
		
	}
}

void testApp::draw() {
	ofBackground(0);
	
	if( pathLoaded ) {
		
		if( curIndex >= 0 ) {
			ofSetColor(grayHigh);
			curPattern.draw(0, 0);
		} else {
			curFrame.update();
			curFrame.draw(0, 0);
		}
		
	}
}

void testApp::keyPressed(int key) {
	if( pathLoaded ) {
		
		if(key == ' ') {
			curIndex = 0;
			curPattern = patterns[curIndex];
			captureTime = ofGetSystemTime();
		}
		if( key == 'f' ) {
			ofToggleFullscreen();
		}
		
	}
}

void testApp::dragEvent(ofDragInfo dragInfo){
	if( !pathLoaded ) {
		for( int i = 0 ; i < dragInfo.files.size() ; i++ ) {
			rootDir.push_back(dragInfo.files[i]);
		}
		init();
		pathLoaded = true;
	}
}
