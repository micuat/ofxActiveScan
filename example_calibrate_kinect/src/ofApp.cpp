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
	ofLogWarning() << p[0] << " " << p[7];
	intrinsic = (cv::Mat1d(3, 3) <<
					f, 0, app->options.projector_width * 0.5 - 0.5,
					0, f, app->options.projector_height * p[7] - 0.5,
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
		pReproject.x = xRad / (1.0 - p[8] / 1e4 * sqLen) + (app->options.projector_width * 0.5 - 0.5);
		pReproject.y = yRad / (1.0 - p[8] / 1e4 * sqLen) + (app->options.projector_height * p[7] - 0.5);
		
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
		for( int j = i + 1; j < mesh.getNumVertices(); j++ ) {
			if( mesh.getTexCoord(j).x < 0 ) continue;
			if( mesh.getVertex(j).squareDistance(mesh.getVertex(i)) < sqTh ) {
				mesh.getTexCoord(j).x = -1;
				mesh.getTexCoord(j).y = -1;
			}
		}
	}
	ofLogWarning() << app->referenceImagePoints[0].size();
	
	// levmar setup begin
	int ret;
	vector<double> p, x;
	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	
	opts[0] = LM_INIT_MU;
	opts[1] = 1E-15;
	opts[2] = 1E-15;
	opts[3] = 1E-20;
	opts[4] = LM_DIFF_DELTA;
	
	p.resize(9);
	x.resize(app->referenceObjectPoints[0].size());
	
	p[0] = 450; // 0: focal length
	p[1] = 0; // 1-3: rotation
	p[2] = 0;
	p[3] = 0;
	p[4] = 0; // 4-6: translation
	p[5] = 0;
	p[6] = 0;
	p[7] = 1.36; // 7: vertical lens shift
	p[8] = 0.001; // 8: lens distortion
	for( int i = 0 ; i < x.size() ; i++ ) {
		// minimize norm
		x[i] = 0.0;
	}
	int nIteration = 1000;
	// levmar setup end
	
	ret = dlevmar_dif(levmarFocalFitting, &p[0], &x[0], p.size(), x.size(), nIteration, opts, info, NULL, NULL, app);
	
	ofLog(OF_LOG_WARNING, "Levenberg-Marquardt returned %d in %g iter, reason %g", ret, info[5], info[6]);
	ofLog(OF_LOG_WARNING, "Solution:");
	
	for( int i = 0 ; i < p.size() ; ++i )
		ofLog(OF_LOG_WARNING, "%.7g", p[i]);
	
	ofLog(OF_LOG_WARNING, "Minimization info:");
	
	for( int i = 0 ; i < LM_INFO_SZ ; ++i )
		ofLog(OF_LOG_WARNING, "%g", info[i]);
	
	cv::FileStorage cfs(ofToDataPath(app->rootDir[0] + "/calibration.yml"), cv::FileStorage::WRITE);
	cfs << "camIntrinsic"  << app->camIntrinsic;
	cfs << "camDistortion" << app->camDistortion;
	cfs << "proIntrinsic"  << app->proIntrinsic;
	cfs << "proDistortion" << app->proDistortion;
	cfs << "proExtrinsic"  << app->proExtrinsic;
	
	ofLogWarning() << app->proIntrinsic;
	ofLogWarning() << app->proExtrinsic;
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
