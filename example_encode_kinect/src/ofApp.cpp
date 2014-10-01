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

#include "ofApp.h"

using namespace ofxActiveScan;

void ofApp::setup() {
	if( rootDir.size() > 0 ) {
		init();
		pathLoaded = true;
	} else {
		pathLoaded = false;
	}
}

void ofApp::init() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
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
	
	bool bColor = true;
	if( bColor ) {
		camera.setRegistration(true);
		camera.init(false);
	} else {
		camera.setRegistration(false);
		camera.init(true);
	}
	camera.listDevices();
	camera.open(devID);
	
	captureTime = 0;
	started = false;
	
	encoder = new Encoder(options);
	decoder = new Decoder(options);
	
	imageUpdateTrigger = false;
}

void ofApp::update() {
	if( pathLoaded ) {
		
		unsigned long curTime = ofGetSystemTime();
		bool needToCapture = started && ((curTime - captureTime) > bufferTime);
		
		camera.update();
		curFrame.setFromPixels(camera.getPixelsRef());
		curFrame.update();
		
		if( camera.isFrameNew() && needToCapture ) {
			
			if( !curPattern.isAllocated() ) { // first image
				curFrame.saveImage(rootDir[0] + "/camPerspective.jpg");
				prevFrame = curFrame;
			} else {
				// see if 20% of image changed
				if( difference(camera.getPixelsRef(), prevFrame.getPixelsRef()) < 20 / 3 ) {
					return;
				}
				
				decoder->AddImage(toAs(curFrame));
				prevFrame = curFrame;
				imageUpdateTrigger = false;
			}
			
			if( !encoder->IsFinished() ) {
				curPattern = toOf(encoder->GetImage());
				encoder->Proceed();
			} else {
				while ( decoder->IsFinished() ); // wait while decoding
				
				horizontal = decoder->GetHorizontal();
				vertical = decoder->GetVertical();
				mask = toOf(decoder->GetMask());
				reliable = toOf(decoder->GetReliable());
				
				horizontal.Write(ofToDataPath(rootDir[0] + "/h.map", true));
				vertical.Write(ofToDataPath(rootDir[0] + "/v.map", true));
				mask.saveImage(ofToDataPath(rootDir[0] + "/mask.png"));
				reliable.saveImage(ofToDataPath(rootDir[0] + "/reliable.png"));
				
				savePointCloud();
				
				started = false;
			}
		}
		
	}
}

void ofApp::draw() {
	ofBackground(0);
	
	if( pathLoaded ) {
		
		if( started ) {
			ofSetColor(grayHigh);
			if( curPattern.isAllocated() ) {
				curPattern.draw(0, 0);
				if( imageUpdateTrigger == false ) {
					imageUpdateTrigger = true;
					captureTime = ofGetSystemTime();
				}
			}
		} else if( ofGetWindowMode() != OF_FULLSCREEN ) {
			curFrame.draw(0, 0);
		}
		
	}
}

void ofApp::savePointCloud() {
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	
	int w = 640;
	int h = 480;
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			float dist = camera.getDistanceAt(x, y);
			if( reliable.getColor(x, y).getBrightness() < 128 ) continue;
			if( dist > 0 ) {
				mesh.addVertex(camera.getWorldCoordinateAt(x, y, dist));
				mesh.addTexCoord(ofVec2f(horizontal.cell(x, y), vertical.cell(x, y)));
			}
		}
	}
	mesh.save(ofToDataPath(rootDir[0] + "/" + ofGetTimestampString() + ".ply"));
}

void ofApp::keyPressed(int key) {
	if( pathLoaded ) {
		
		if(key == ' ') {
			started = true;
			prevFrame.setFromPixels(camera.getPixelsRef());
		}
		if( key == 'f' ) {
			ofToggleFullscreen();
		}
		
	}
}

void ofApp::dragEvent(ofDragInfo dragInfo){
	if( !pathLoaded ) {
		for( int i = 0 ; i < dragInfo.files.size() ; i++ ) {
			rootDir.push_back(dragInfo.files[i]);
		}
		init();
		pathLoaded = true;
	}
}

int ofApp::difference(ofPixels &p0, ofPixels &p1){
	// subtract two images
	ofPixels pout = p0;
	int numChanged = 0;
	int threshold = 3;
	for (int i = 0; i < pout.size(); i++) {
		pout[i] = ofMap(p0[i] - p1[i], -255, 255, 0, 255, true);
		if (pout[i] < 128 - threshold || 128 + threshold < pout[i]) {
			numChanged++;
		}
	}
	
	return numChanged * 100 / pout.size();
}
