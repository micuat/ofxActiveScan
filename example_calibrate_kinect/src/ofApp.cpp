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

double lensDistortionCoeff = 1e-4;

void levmarFocalFitting(double *p, double *x, int m, int n, void *data) {
	ofApp *app;
	app = static_cast<ofApp *>(data);
	cv::Mat &intrinsic = app->proIntrinsic;
	cv::Mat &extrinsic = app->proExtrinsic;
	
	double f = 1700.48402893266;
	ofLogWarning() << p[0] << " " << p[7];
	intrinsic = (cv::Mat1d(3, 3) <<
					f, 0, app->options.projector_width * 0.5 - 0.5,
					0, f, 924.16198645433894,
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
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_LINES);
	int count = 0;
	// return reprojection error
	for( int i = 0 ; i < n ; i++ ) {
		x[i] = 0;
		
		cv::Point2d pReproject;
		cv::Point3d &pt = app->referenceObjectPoints.at(0).at(i);
		cv::Mat pMat = (cv::Mat1d(4, 1) << pt.x, pt.y, pt.z, 1);
		cv::Mat pReprojectMat = intrinsic * extrinsic * pMat;
		pReproject.x = pReprojectMat.at<double>(0) / pReprojectMat.at<double>(2);
		pReproject.y = pReprojectMat.at<double>(1) / pReprojectMat.at<double>(2);
		
		double xRad = pReproject.x - (app->options.projector_width * 0.5 - 0.5);
		double yRad = pReproject.y - (app->options.projector_height * p[7] - 0.5);
		double sqLen = xRad * xRad + yRad * yRad;
		pReproject.x = xRad / (1.0 - p[8] * lensDistortionCoeff * sqLen) + (app->options.projector_width * 0.5 - 0.5);
		pReproject.y = yRad / (1.0 - p[8] * lensDistortionCoeff * sqLen) + (app->options.projector_height * p[7] - 0.5);
		
		mesh.addVertex(ofVec3f(pReproject.x, pReproject.y, 0));
		mesh.addColor(ofColor::red);
		mesh.addVertex(ofVec3f(app->referenceImagePoints.at(0).at(i).x, app->referenceImagePoints.at(0).at(i).y, 0));
		mesh.addColor(ofColor::white);
		
		pReproject -= app->referenceImagePoints.at(0).at(i);
		float xtmp = cv::norm(pReproject);
		if( xtmp < 0 || isnan(xtmp) || isinf(xtmp) ) {
			xtmp = 1e30;
		} else if( xtmp > 100 ) {
			count++;
		}
		x[i] += xtmp;
	}
	
//	if( count < 30 ) {
//		for( int i = 0 ; i < n ; i++ ) {
//			if( x[i] > 100 && x[i] < 1e30) {
//				x[i] = 0;
//			}
//		}
//	}
	app->pointsReprojection = mesh;
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
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	calibration.app = this;
	calibration.startThread();
	pathLoaded = false;
}

void ofApp::update() {
}

void KinectCalibration::threadedFunction() {
	app->referenceObjectPoints.resize(1);
	app->referenceImagePoints.resize(1);
	
	ofDirectory dir(ofToDataPath(app->rootDir[0]));
	dir.allowExt("ply");
	dir.listDir();
	
	ofMesh mesh;
	// append meshes
	for( int j = 0; j < dir.numFiles(); j++ ){
		ofMesh curMesh;
		curMesh.load(dir.getPath(j));
		for( int i = 0; i < curMesh.getNumVertices(); i+=2 ) {
			mesh.addVertex(curMesh.getVertex(i));
			mesh.addTexCoord(curMesh.getTexCoord(i));
		}
	}
	// downsample
	float threshold = 100; // mm
	float sqTh = threshold * threshold;
	for( int i = 0; i < mesh.getNumVertices(); i++ ) {
		if( mesh.getTexCoord(i).x < 0 ) continue;
		app->referenceObjectPoints[0].push_back(ofxCv::toCv(mesh.getVertex(i)));
		app->referenceImagePoints[0].push_back(ofxCv::toCv(mesh.getTexCoord(i)));
	}
	ofLogWarning() << app->referenceImagePoints[0].size();
	
	app->proIntrinsic = (cv::Mat1d(3, 3) <<
							1699.4161051180934, 0, 531.68731922681593,
							0, 1701.5519527472304, 924.16198645433894,
							0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat1d(4, 1) <<
						  -1.3976824758462122e-01, 1.6504530150224475e-01,
						  1.5693006340259787e-03, 2.1149550740275625e-04);
	cv::Mat r, t;
	cv::solvePnP(app->referenceObjectPoints[0], app->referenceImagePoints[0], app->proIntrinsic, distCoeffs, r, t);
	
	cv::Mat mat;
	cv::Rodrigues(r, mat);
	mat = mat.t();

	app->proExtrinsic = (cv::Mat1d(4,3) << mat.at<double>(0, 0), mat.at<double>(0, 1), mat.at<double>(0, 2),
					mat.at<double>(1, 0), mat.at<double>(1, 1), mat.at<double>(1, 2),
					mat.at<double>(2, 0), mat.at<double>(2, 1), mat.at<double>(2, 2),
					t.at<double>(0), t.at<double>(1), t.at<double>(2));
	app->proExtrinsic = app->proExtrinsic.t();
	
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_LINES);
	
	vector<cv::Point3d> objectPoints;
	vector<cv::Point2d> imagePoints;
	
	vector<cv::Point2d> repImagePoints;
	
	cv::projectPoints(app->referenceObjectPoints[0], r, t, app->proIntrinsic, distCoeffs, repImagePoints);

	for( int i = 0 ; i < app->referenceObjectPoints[0].size() ; i++ ) {
		cv::Point2d pReproject = repImagePoints.at(i);
		
		pReproject -= app->referenceImagePoints.at(0).at(i);
		float xtmp = cv::norm(pReproject);
		
		if( xtmp < 5 ) {
			objectPoints.push_back(app->referenceObjectPoints.at(0).at(i));
			imagePoints.push_back(app->referenceImagePoints.at(0).at(i));
		}
	}
	ofLogWarning() << imagePoints.size();

	cv::solvePnP(objectPoints, imagePoints, app->proIntrinsic, distCoeffs, r, t);
	
	cv::Rodrigues(r, mat);
	mat = mat.t();
	
	app->proExtrinsic = (cv::Mat1d(4,3) << mat.at<double>(0, 0), mat.at<double>(0, 1), mat.at<double>(0, 2),
						 mat.at<double>(1, 0), mat.at<double>(1, 1), mat.at<double>(1, 2),
						 mat.at<double>(2, 0), mat.at<double>(2, 1), mat.at<double>(2, 2),
						 t.at<double>(0), t.at<double>(1), t.at<double>(2));
	app->proExtrinsic = app->proExtrinsic.t();
	
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_LINES);
	float reperror = 0;
	
	cv::projectPoints(objectPoints, r, t, app->proIntrinsic, distCoeffs, repImagePoints);

	for( int i = 0 ; i < objectPoints.size() ; i++ ) {
		cv::Point2d pReproject = repImagePoints.at(i);
		
		mesh.addVertex(ofVec3f(pReproject.x, pReproject.y, 0));
		mesh.addColor(ofColor::red);
		mesh.addVertex(ofVec3f(imagePoints.at(i).x, imagePoints.at(i).y, 0));
		mesh.addColor(ofColor::white);
		
		pReproject -= imagePoints.at(i);
		reperror += cv::norm(pReproject);

	}
	ofLogWarning() << "reperror: " << reperror / objectPoints.size();
	cv::FileStorage cfs(ofToDataPath(app->rootDir[0] + "/calibration.yml"), cv::FileStorage::WRITE);
	cfs << "proIntrinsic"  << app->proIntrinsic;
	cfs << "proDistortion" << distCoeffs;
	cfs << "proExtrinsic"  << app->proExtrinsic;
	cfs << "rvec"  << r;
	cfs << "tvec"  << t;

	ofLogWarning() << app->proIntrinsic;
	ofLogWarning() << app->proExtrinsic;
	ofLogWarning() << mat;
	ofLogWarning() << t;
	
	app->pointsReprojection = mesh;
}

void ofApp::draw() {
	if( calibration.isThreadRunning() ) {
		ofBackground(0);
	} else {
		ofBackground(54);
	}

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
