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

void levmarFocalFitting(double *p, double *x, int m, int n, void *data) {
	ofApp *app;
	app = static_cast<ofApp *>(data);
	cv::Mat &intrinsic = app->proIntrinsic;
	cv::Mat &extrinsic = app->proExtrinsic;
	
	double f = p[0];
	ofLogWarning() << p[0];
	intrinsic = (cv::Mat1d(3, 3) <<
					f, 0, app->options.projector_width * 0.5 - 0.5,
					0, f, app->options.projector_height * app->options.projector_horizontal_center - 0.5,
					0, 0, 1);
	ofMatrix4x4 mat;
	mat.makeIdentityMatrix();
	mat.rotate(p[1], 1, 0, 0);
	mat.rotate(p[2], 0, 1, 0);
	mat.rotate(p[3], 0, 0, 1);
	extrinsic = (cv::Mat1d(4,3) << mat(0, 0), mat(0, 1), mat(0, 2),
					mat(1, 0), mat(1, 1), mat(1, 2),
					mat(2, 0), mat(2, 1), mat(2, 2),
					p[4], p[5], p[6]);
	extrinsic = extrinsic.t();
	
	x[0] = 0;
	app->pointsReprojection.clear();
	app->pointsReprojection.setMode(OF_PRIMITIVE_LINES);
	int count = 0;
	// return reprojection error
	for( int i = 0 ; i < n ; i++ ) {
		x[i] = 0;
		
		cv::Point2d pReproject;
		cv::Point3d &p = app->referenceObjectPoints.at(0).at(i);
		cv::Mat pMat = (cv::Mat1d(4, 1) << p.x, p.y, p.z, 1);
		cv::Mat pReprojectMat = intrinsic * extrinsic * pMat;
		pReproject.x = pReprojectMat.at<double>(0) / pReprojectMat.at<double>(2);
		pReproject.y = pReprojectMat.at<double>(1) / pReprojectMat.at<double>(2);
		
		app->pointsReprojection.addVertex(ofVec3f(pReproject.x, pReproject.y, 0));
		app->pointsReprojection.addColor(ofColor::red);
		app->pointsReprojection.addVertex(ofVec3f(app->referenceImagePoints.at(0).at(i).x, app->referenceImagePoints.at(0).at(i).y, 0));
		app->pointsReprojection.addColor(ofColor::white);
		
		pReproject -= app->referenceImagePoints.at(0).at(i);
		float xtmp = cv::norm(pReproject);
//		ofLogWarning() << xtmp;
		if( xtmp < 0 || isnan(xtmp) || isinf(xtmp) ) {
			xtmp = 1e30;
		} else if( xtmp > 100 ) {
			count++;
		}
		x[i] += xtmp;
	}
	
	if( count < 30 ) {
		for( int i = 0 ; i < n ; i++ ) {
			if( x[i] > 100 && x[i] < 1e30) {
				x[i] = 0;
			}
		}
	}
}

void ofApp::setup() {
	if( rootDir.size() > 0 ) {
		init();
		pathLoaded = true;
	} else {
		pathLoaded = false;
	}
}

void ofApp::init() {
	ofSetLogLevel(OF_LOG_WARNING);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	
	kinect.open();		// opens first available kinect

	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	// load correspondences estimated by decode program 
	horizontal = Map2f(ofToDataPath(rootDir[0] + "/h.map", true));
	vertical = Map2f(ofToDataPath(rootDir[0] + "/v.map", true));
	ofImage mask;
	ofLoadImage(mask, ofToDataPath(rootDir[0] + "/mask.png"));
	maskMap = toAs(mask);
	
	ofImage depthImage;
	ofLoadImage(depthImage, ofToDataPath(rootDir[0] + "/camPerspectiveDepth.png", true));
	depthImage.setImageType(OF_IMAGE_GRAYSCALE);
	depth = depthImage.getPixelsRef();
}

void ofApp::update() {
	if( pathLoaded ) {
		kinect.update();
		if( kinect.isFrameNew() ) {
			kinectCalibration();
			pathLoaded = false;
		}
	}
}

void ofApp::kinectCalibration() {
	int w = 640;
	int h = 480;
	int step = 2;
	referenceObjectPoints.resize(1);
	referenceImagePoints.resize(1);
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			float dist = kinect.getDistanceAt(x, y);
			if( maskMap.cell(x, y) <= 0 ) continue;
			if(dist > 0) {
				referenceObjectPoints[0].push_back(ofxCv::toCv(kinect.getWorldCoordinateAt(x, y, dist)));
				referenceImagePoints[0].push_back(cv::Point2d(horizontal.cell(x, y), vertical.cell(x, y)));
			}
		}
	}
	ofLogWarning() << referenceImagePoints[0].size();
	
	// levmar setup begin
	int ret;
	vector<double> p, x;
	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	
	opts[0] = LM_INIT_MU;
	opts[1] = 1E-15;
	opts[2] = 1E-15;
	opts[3] = 1E-20;
	opts[4] = LM_DIFF_DELTA;
	
	p.resize(8);
	x.resize(referenceObjectPoints[0].size());
	
	p[0] = 1500; // focal length guess
	p[1] = 0;
	p[2] = 0;
	p[3] = 0;
	p[4] = 0;
	p[5] = 0;
	p[6] = 0;
	for( int i = 0 ; i < x.size() ; i++ ) {
		// minimize norm
		x[i] = 0.0;
	}
	int nIteration = 1000;
	// levmar setup end
	
	ret = dlevmar_dif(levmarFocalFitting, &p[0], &x[0], p.size(), x.size(), nIteration, opts, info, NULL, NULL, this);
	
	ofLog(OF_LOG_WARNING, "Levenberg-Marquardt returned %d in %g iter, reason %g", ret, info[5], info[6]);
	ofLog(OF_LOG_WARNING, "Solution:");
	
	for( int i = 0 ; i < p.size() ; ++i )
		ofLog(OF_LOG_WARNING, "%.7g", p[i]);
	
	ofLog(OF_LOG_WARNING, "Minimization info:");
	
	for( int i = 0 ; i < LM_INFO_SZ ; ++i )
		ofLog(OF_LOG_WARNING, "%g", info[i]);
	
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::WRITE);
	cfs << "camIntrinsic"  << camIntrinsic;
	cfs << "camDistortion" << camDistortion;
	cfs << "proIntrinsic"  << proIntrinsic;
	cfs << "proDistortion" << proDistortion;
	cfs << "proExtrinsic"  << proExtrinsic;
	
	ofLogWarning() << proIntrinsic;
	ofLogWarning() << proExtrinsic;
}

void ofApp::draw() {
	ofBackground(0);
	pointsReprojection.draw();
}

void ofApp::keyPressed(int key) {
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
