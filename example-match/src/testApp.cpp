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
	
	/*
	// load correspondences estimated by decode program 
	Map2f horizontal(ofToDataPath(rootDir + "/h.map", true));
	Map2f vertical(ofToDataPath(rootDir + "/v.map", true));  
	ofImage mask;
	ofLoadImage(mask, ofToDataPath(rootDir + "/mask.bmp"));
	*/
	
	if( rootDir.size() != 2 ) {
		ofLogError() << "Wrong argument number";
		exit();
	}
	
	vector<ofImage> masks;
	ofImage mask;
	masks.resize(rootDir.size());
	ofLoadImage(masks[0], ofToDataPath(rootDir[0] + "/mask.bmp"));
	ofLoadImage(masks[1], ofToDataPath(rootDir[1] + "/mask.bmp"));
	
	mask = masks[0];
	
	// Find camera pixels shared by two projectors
	for (int y=0; y<mask.getHeight(); y++) {
		for (int x=0; x<mask.getWidth(); x++) {
			if( masks[0].getColor(x, y).getLightness() < 128
				|| masks[1].getColor(x, y).getLightness() < 128 ) {
				//cout << x << ' ' << y << endl;
				mask.setColor(x, y, 0);
			}
		}
	}
	
	for( int i = 0 ; i < rootDir.size() ; i++ ) {
		ofxActiveScan::Options options;
		cv::Size camSize, proSize;
		
		cv::FileStorage fs(ofToDataPath(rootDir[i] + "/config.yml"), cv::FileStorage::READ);
		fs["proWidth"] >> options.projector_width;
		fs["proHeight"] >> options.projector_height;
		fs["camWidth"] >> cw;
		fs["camHeight"] >> ch;
		fs["vertical_center"] >> options.projector_horizontal_center;
		fs["nsamples"] >> options.nsamples;
		
		Map2f horizontal(ofToDataPath(rootDir[i] + "/h.map", true));
		Map2f vertical(ofToDataPath(rootDir[i] + "/v.map", true));  
		
		ofImage orgMask;
		ofLoadImage(orgMask, ofToDataPath(rootDir[i] + "/mask.bmp"));
		
		cv::Mat camIntrinsic, proIntrinsic, proExtrinsic;
		double camDist, proDist;
		
		cv::FileStorage cfs(ofToDataPath(rootDir[i] + "/calibration.yml"), cv::FileStorage::READ);
		cfs["camIntrinsic"] >> camIntrinsic;
		cfs["camDistortion"] >> camDist;
		cfs["proIntrinsic"] >> proIntrinsic;
		cfs["proDistortion"] >> proDist;
		cfs["proExtrinsic"] >> proExtrinsic;
		
		overlapMesh.push_back(
			triangulate(options, horizontal, vertical, toAs(mask),
					toAs(camIntrinsic), camDist,
					toAs(proIntrinsic), proDist, toAs(proExtrinsic), mask)
		);
		
		mesh.push_back(
			triangulate(options, horizontal, vertical, toAs(orgMask),
					toAs(camIntrinsic), camDist,
					toAs(proIntrinsic), proDist, toAs(proExtrinsic), orgMask)
		);
	}
	
	
	for( int i = 0 ; i < mesh[0].getNumVertices() ; i++ ) {
		mesh[0].setColor(i, ofColor::red);
	}
	for( int i = 0 ; i < mesh[1].getNumVertices() ; i++ ) {
		mesh[1].setColor(i, ofColor::green);
	}
	
	ofMesh input, target;
	
	// Downsample to reduce complexity
	for( int i = 0 ; i < overlapMesh[0].getNumVertices() ; i+=100 ) {
		input.addVertex(overlapMesh[0].getVertex(i));
		target.addVertex(overlapMesh[1].getVertex(i));
	}
	
	ofLogNotice() << "Input points: " << input.getNumVertices();
	
	cv::Mat Rt = findTransform(input, target, 2000);
	
	ofLogNotice() << Rt << endl;
	
	mesh[0] = transformMesh(mesh[0], Rt);
	overlapMesh[0] = transformMesh(overlapMesh[0], Rt);
	
	// Average point cloud
	for( int i = 0 ; i < overlapMesh[0].getNumVertices() ; i++ ) {
		avg.addVertex((overlapMesh[0].getVertex(i) + overlapMesh[1].getVertex(i)) / 2);
		avg.addColor(ofColor::white);
	}
	
	curMesh = mesh.begin();
	transformed = false;
	
	cam.cacheMatrices(true);
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
	
	cam.begin();
	ofScale(1, -1, -1);
	ofScale(1000, 1000, 1000);
	ofTranslate(0, 0, -2);
	
	if( transformed ) {
		mesh[0].drawVertices();
		mesh[1].drawVertices();
		avg.drawVertices();
	} else {
		curMesh->drawVertices();
	//	avg.drawVertices();
	}
	
	cam.end();
}

void testApp::keyPressed(int key) {
	if( key == '1' || key == '2' ) {
		curMesh = mesh.begin() + (key - '1');
		transformed = false;
	}
	if( key == '0' ) {
		transformed = true;
	}
}
