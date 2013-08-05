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

#pragma once

#include "ofMain.h"

#include "ofxCv.h"

#undef Success
#include "ofxPCL.h"

class testApp : public ofBaseApp
{

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	
	ofEasyCam cam;
	vector<ofVboMesh> meshes;
	vector<ofVboMesh>::iterator mit;

	cv::Mat camIntrinsic, proIntrinsic, proExtrinsic;
	ofxCv::Intrinsics camCalibration, proCalibration;
	cv::Size camSize, proSize;
	
	enum CameraMode {EASYCAM_MODE, PRO_MODE, CAM_MODE};
	CameraMode cameraMode;
	
	int cw, ch;
	
	vector<string> rootDir;
	
	int count;
};
