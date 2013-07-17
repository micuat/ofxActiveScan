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
	
	int ret;
	vector<double> p, x;
	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	vector<cv::Point3d> input, target;
/*	data.push_back(cv::Point2d(0.0, 0.0));
	data.push_back(cv::Point2d(2.0, -2.0));
	data.push_back(cv::Point2d(1.0, 0.0));
	data.push_back(cv::Point2d(2.0, -1.0));
	data.push_back(cv::Point2d(0.0, 1.0));
	data.push_back(cv::Point2d(1.0, -2.0));
	data.push_back(cv::Point2d(-1.0, -1.0));
	data.push_back(cv::Point2d(3.0, -3.0)); */
	
	input.push_back(cv::Point3d(0, 0, 0));
	target.push_back(cv::Point3d(1, 0, 1));
	input.push_back(cv::Point3d(1, 0, 0));
	target.push_back(cv::Point3d(1, 1, 1));
	input.push_back(cv::Point3d(1, 1, 0));
	target.push_back(cv::Point3d(1, 1, 0));
	input.push_back(cv::Point3d(0, 1, 0));
	target.push_back(cv::Point3d(1, 0, 0));
	input.push_back(cv::Point3d(0, 0, 1));
	target.push_back(cv::Point3d(0, 0, 1));
	input.push_back(cv::Point3d(-1, 0, 1));
	target.push_back(cv::Point3d(0, -1, 1));
	
	cv::Mat Rt = findTransform(input, target);
	
	cout << Rt << endl;
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
}

void testApp::keyPressed(int key) {
}
