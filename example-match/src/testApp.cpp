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
	transformed = false;
	
	cam.cacheMatrices(true);
}

void testApp::update() {
	int n = curMesh->getNumVertices();
	float nearestDistance = 0;
	ofVec2f mouse(mouseX, mouseY);
	for(int i = 0; i < n; i++) {
		ofVec3f p = curMesh->getVertex(i);
		ofVec3f cur = cam.worldToScreen(ofVec3f(p.x*1000.0, p.y*-1000.0, p.z*-1000.0 + 2000.0));
		float distance = cur.distance(mouse);
		if(i == 0 || distance < nearestDistance) {
			nearestDistance = distance;
			nearestVertex = cur;
			nearestIndex = i;
		}
	}

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
	} else {
		curMesh->drawVertices();
	}
	
	cam.end();
	
	ofNoFill();
	ofSetColor(ofColor::yellow);
	ofSetLineWidth(2);
	ofCircle(nearestVertex, 4);
	ofSetLineWidth(1);
}

void testApp::keyPressed(int key) {
	if( key == '0' || key == '1' ) {
		curMesh = mesh.begin() + (key - '0');
	}
	if( key == ' ' ) {
		cout << nearestVertex << endl;
		if( curMesh == mesh.begin() ) {
			inputPoints.push_back(ofxCv::toCv(curMesh->getVertex(nearestIndex)));
		}
		if( curMesh == mesh.begin() + 1) {
			targetPoints.push_back(ofxCv::toCv(curMesh->getVertex(nearestIndex)));
		}
	}
	if( key == 'c' ) {
		cv::Mat Rt = findTransform(inputPoints, targetPoints, 20000);
		
		cout << Rt << endl;
		
		mesh[0] = transformMesh(mesh[0], Rt);
		
		transformed = true;
	}
}
