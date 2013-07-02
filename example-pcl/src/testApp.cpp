// Example from http://pointclouds.org/documentation/tutorials/extract_indices.php

#include <iostream>
#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
//	fs["proWidth"] >> options.projector_width;
//	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	
	ofxPCL::PointCloud cloud(new ofxPCL::PointCloud::value_type);
	vector<ofxPCL::PointCloud> clouds;
	
	pcl::io::loadPLYFile(ofToDataPath(rootDir + "/out.ply"), *cloud);
		
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	<< " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;
	
	ofxPCL::downsample(cloud, ofVec3f(0.01f, 0.01f, 0.01f));
	
	std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height 
	<< " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;
	
//	ofxPCL::savePointCloud("table_scene_lms400_inliers.pcd", cloud);
	
	clouds = ofxPCL::segmentation(cloud, pcl::SACMODEL_PLANE, 0.005, 100, 30);
	
	meshes.push_back(ofxPCL::toOF(cloud));
	for( int i = 0; i < clouds.size(); i++ ) {
		meshes.push_back(ofxPCL::toOF(clouds[i]));
	}
	mit = meshes.begin();
}

//--------------------------------------------------------------
void testApp::update()
{
	
}

//--------------------------------------------------------------
void testApp::draw()
{
	ofBackground(0);
	
	cam.begin();
	ofScale(100, 100, 100);
	glEnable(GL_DEPTH_TEST);
	
	mit->drawVertices();
	
	cam.end();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
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