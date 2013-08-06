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

#include <iostream>
#include "testApp.h"

//--------------------------------------------------------------
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
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> proSize.width;
	fs["proHeight"] >> proSize.height;
	fs["camWidth"] >> camSize.width;
	fs["camHeight"] >> camSize.height;
	
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::READ);
	cfs["camIntrinsic"] >> camIntrinsic;
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	ofxPCL::PointCloud cloud(new ofxPCL::PointCloud::value_type);
	vector<ofxPCL::PointCloud> clouds;
	
	ofMesh orgMesh;
	orgMesh.load(ofToDataPath(rootDir[0] + "/out.ply"));
	cloud = ofxPCL::toPCL<ofxPCL::PointCloud>(orgMesh);
	
	clouds = ofxPCL::segmentation(cloud, pcl::SACMODEL_PLANE, 0.005, 20, 30);
	
	ofColor hues[] = {ofColor::red, ofColor::green, ofColor::blue, ofColor::cyan, ofColor::magenta, ofColor::yellow};
	int colorIndex = 0;
	for( int i = 0; i < clouds.size(); i++ ) {
		ofVboMesh tmpmesh = ofxPCL::toOF(clouds[i]);
		for( int j = 0; j < tmpmesh.getNumVertices(); j++ ) {
			tmpmesh.addColor(hues[colorIndex]);
		}
		meshes.push_back(tmpmesh);
		
		colorIndex++;
		if( colorIndex == 6 ) {
			colorIndex = 0;
		}
	}
	mit = meshes.begin();
	
	proCalibration.setup(proIntrinsic, proSize);
	camCalibration.setup(camIntrinsic, camSize);
	
	count = 0;
}

//--------------------------------------------------------------
void testApp::update()
{
	
}

//--------------------------------------------------------------
void testApp::draw()
{
	ofBackground(0);
	
	if( pathLoaded ) {
		
		glPointSize(2.0);
		
		if(cameraMode == EASYCAM_MODE) {
			cam.begin();
			ofScale(1, -1, -1);
		} else if(cameraMode == PRO_MODE) {
			ofSetupScreenPerspective(proSize.width, proSize.height);
			proCalibration.loadProjectionMatrix(0.0001, 100000000.0);
			cv::Mat m = proExtrinsic;
			cv::Mat extrinsics = (cv::Mat1d(4,4) <<
								  m.at<double>(0,0), m.at<double>(0,1), m.at<double>(0,2), m.at<double>(0,3),
								  m.at<double>(1,0), m.at<double>(1,1), m.at<double>(1,2), m.at<double>(1,3),
								  m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2), m.at<double>(2,3),
								  0, 0, 0, 1);
			extrinsics = extrinsics.t();
			glMultMatrixd((GLdouble*) extrinsics.ptr(0, 0));
		} else if(cameraMode == CAM_MODE) {
			ofSetupScreenPerspective(camSize.width, camSize.height);
			camCalibration.loadProjectionMatrix(0.0001, 100000000.0);
		}
		
		mit->drawVertices();
		if( count > 2 ) {
			advance(mit, 1);
			if( mit == meshes.end() ) {
				mit = meshes.begin();
			}
			count = 0;
		}
		count++;
		
		if(cameraMode == EASYCAM_MODE) {
			cam.end();
		}
		
	}
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
	switch(key) {
		case '1': cameraMode = EASYCAM_MODE; break;
		case '2': cameraMode = PRO_MODE; break;
		case '3': cameraMode = CAM_MODE; break;
	}
	
	if( key == 'f' ) {
		ofToggleFullscreen();
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{
	
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{
	
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{
	
}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg)
{
	
}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo)
{
	if( !pathLoaded ) {
		for( int i = 0 ; i < dragInfo.files.size() ; i++ ) {
			rootDir.push_back(dragInfo.files[i]);
		}
		init();
		pathLoaded = true;
	}
}
