#include <iostream>
#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> proSize.width;
	fs["proHeight"] >> proSize.height;
	fs["camWidth"] >> camSize.width;
	fs["camHeight"] >> camSize.height;
	
	cv::FileStorage cfs(ofToDataPath(rootDir + "/calibration.yml"), cv::FileStorage::READ);
	cfs["camIntrinsic"] >> camIntrinsic;
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	ofxPCL::PointCloud cloud(new ofxPCL::PointCloud::value_type);
	vector<ofxPCL::PointCloud> clouds;
	
	ofMesh orgMesh;
	orgMesh.load(ofToDataPath(rootDir + "/out.ply"));
	cloud = ofxPCL::toPCL<ofxPCL::PointCloud>(orgMesh);
	
	clouds = ofxPCL::segmentation(cloud, pcl::SACMODEL_PLANE, 0.005, 30, 30);
	
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
}

//--------------------------------------------------------------
void testApp::update()
{
	
}

//--------------------------------------------------------------
void testApp::draw()
{
	ofBackground(0);
	
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
	
	if(cameraMode == EASYCAM_MODE) {
		cam.end();
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
	
	if(key == ' ') {
		advance(mit, 1);
		if( mit == meshes.end() ) {
			mit = meshes.begin();
		}
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
	
}
