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
	
	rootDir = "../../../SharedData/";
	
	/*
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	// load correspondences estimated by decode program 
	Map2f horizontal(ofToDataPath(rootDir + "/h.map", true));
	Map2f vertical(ofToDataPath(rootDir + "/v.map", true));  
	ofImage mask;
	ofLoadImage(mask, ofToDataPath(rootDir + "/mask.bmp"));
	*/
	
	mesh.resize(2);
	mesh[0].load(ofToDataPath("out.ply"));
	mesh[1].load(ofToDataPath("out2.ply"));
	
	curMesh = mesh.begin();
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
	
	cam.begin();
	ofScale(1, -1, -1);
	ofScale(1000, 1000, 1000);
	ofTranslate(0, 0, -2);
	
	curMesh->drawVertices();
	
	cam.end();
}

void testApp::keyPressed(int key) {
	if( key == '0' || key == '1' ) {
		curMesh = mesh.begin() + (key - '0');
	}
	if( key == 'c' ) {
		cv::Mat Rt = findTransform(inputPoints, targetPoints);
		
		cout << Rt << endl;
	}
}
