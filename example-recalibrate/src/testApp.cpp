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

// entry point
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
	
	cameraMode = EASYCAM_MODE;
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> camSize.width;
	fs["camHeight"] >> camSize.height;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::READ);
	cfs["camIntrinsic"] >> camIntrinsic;
	cfs["camDistortion"] >> camDist;
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proDistortion"] >> proDist;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	proSize.width = options.projector_width;
	proSize.height = options.projector_height;
	
	// horizontal and vertical correspondences between projector and camera
	horizontal = Map2f(ofToDataPath(rootDir[0] + "/h.map", true));
	vertical = Map2f(ofToDataPath(rootDir[0] + "/v.map", true));
	
	ofLoadImage(mask, ofToDataPath(rootDir[0] + "/mask.bmp"));
	// use this when camPerspective is a color image
	//ofLoadImage(camPerspective, ofToDataPath(rootDir[0] + "/camPerspective.jpg"));
	ofLoadImage(camPerspective, ofToDataPath(rootDir[0] + "/mask.bmp"));
	
	ofLogNotice() << "Start triangulation with initial calibration parameters";
	mesh = triangulate(options, horizontal, vertical, toAs(mask),
			toAs(camIntrinsic), camDist,
			toAs(proIntrinsic), proDist, toAs(proExtrinsic), camPerspective);
	mesh.save(ofToDataPath(rootDir[0] + "/out.ply"));
	
	// set parameters for projection
	proCalibration.setup(proIntrinsic, proSize);
	camCalibration.setup(camIntrinsic, camSize);
	
	curImagePoint = imagePoints.end();
}

void testApp::update() {
	if( pathLoaded ) {

		// if not selected
		if( curImagePoint == imagePoints.end() ) {
			// Mouse Picking
			int n = mesh.getNumVertices();
			float nearestDistance = 0;
			ofVec2f mouse(mouseX, mouseY);
			for(int i = 0; i < n; i++) {
				ofVec3f p = mesh.getVertex(i);
				cv::Mat pCv = (cv::Mat1d(4, 1) << p.x, p.y, p.z, 1);
				cv::Mat pPlane = proIntrinsic * proExtrinsic * pCv;
				pPlane = pPlane / pPlane.at<double>(2, 0);
				ofVec2f cur = ofVec2f(pPlane.at<double>(0, 0), pPlane.at<double>(1, 0));
				float distance = cur.distance(mouse);
				if(i == 0 || distance < nearestDistance) {
					nearestDistance = distance;
					nearestVertex = cur;
					nearestIndex = i;
				}
			}
		}
		
	}
}

void testApp::draw() {
	ofBackground(0);
	
	if( pathLoaded ) {

		if(cameraMode == EASYCAM_MODE) {
			cam.begin();
			ofScale(1, -1, -1);
			ofScale(1000, 1000, 1000);
			ofTranslate(0, 0, -2);
		} else if(cameraMode == PRO_MODE) {
			ofSetupScreenPerspective(options.projector_width, options.projector_height);
			
			// draw circle
			ofPushStyle();
			ofSetLineWidth(2);
			if( curImagePoint == imagePoints.end() ) {
				ofNoFill();
				ofSetColor(ofColor::yellow);
				ofCircle(nearestVertex, 4);
			} else {
				ofFill();
				ofSetColor(ofColor::cyan);
				ofCircle(curImagePoint->x, curImagePoint->y, 4);
			}
			
			ofPopStyle();
			
			proCalibration.loadProjectionMatrix(0.001, 1000000000.0);
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
			camCalibration.loadProjectionMatrix(0.001, 1000000000.0);
		}
		
		mesh.drawVertices();
		
		if(cameraMode == EASYCAM_MODE) {
			cam.end();
		}
		
	}
}

void testApp::keyPressed(int key) {
	if( pathLoaded ) {

		switch(key) {
			case '1': cameraMode = EASYCAM_MODE; break;
			case '2': cameraMode = PRO_MODE; break;
			case '3': cameraMode = CAM_MODE; break;
		}
		if( key == 'f' ) {
			ofToggleFullscreen();
		}
		if( key == ' ' ) {
			objectPointsRef.push_back(nearestIndex);
			imagePoints.push_back(ofxCv::toCv(nearestVertex));
			
			curImagePoint = imagePoints.end() - 1;
		}
		if( key == OF_KEY_RETURN ) {
			curImagePoint = imagePoints.end();
		}
		if( curImagePoint != imagePoints.end() ) {
			if( key == OF_KEY_UP ) {
				--(curImagePoint->y);
			}
			if( key == OF_KEY_DOWN ) {
				++(curImagePoint->y);
			}
			if( key == OF_KEY_LEFT ) {
				--(curImagePoint->x);
			}
			if( key == OF_KEY_RIGHT ) {
				++(curImagePoint->x);
			}
		}
		if( key == 'c' ) {
			int minPointNum = 6;
			if( objectPointsRef.size() >= minPointNum ) {
				// Re-Calibration
				ofLogNotice() << "Re-calibrate extrinsics";
				
				vector<cv::Point3f> objectPoints;
				vector<cv::Point2f> undistImagePoints;
				Vec2d proPrincipal;
				proPrincipal[0] = (options.projector_width+1) / 2.0;
				proPrincipal[1] = options.projector_height * options.projector_horizontal_center;
				
				for( int k = 0 ; k < objectPointsRef.size() ; k++ ) {
					// Convert object point reference to actual points (this is required after retriangulation)
					objectPoints.push_back(ofxCv::toCv(mesh.getVertex(objectPointsRef[k])));
					
					// Undistort image points
					Vec2d undistP;
					slib::fmatrix::CancelRadialDistortion(proDist, proPrincipal, toAs(imagePoints[k]), undistP);
					undistImagePoints.push_back(toCv(undistP));
				}
				
				cv::Mat distCoeffs;
				cv::Mat m = proExtrinsic;
				cv::Mat r = (cv::Mat1d(3,3) <<
						m.at<double>(0,0), m.at<double>(0,1), m.at<double>(0,2),
						m.at<double>(1,0), m.at<double>(1,1), m.at<double>(1,2),
						m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2));
				cv::Mat rvec, tvec;
				cv::Rodrigues(r, rvec);
				tvec = (cv::Mat1d(3,1) <<
						m.at<double>(0,3), m.at<double>(1,3), m.at<double>(2,3));
				
				cv::solvePnP(objectPoints, undistImagePoints, proIntrinsic, distCoeffs, rvec, tvec, true);
				
				cv::Rodrigues(rvec, r);
				cv::Mat t = tvec;
				proExtrinsic = (cv::Mat1d(3,4) <<
						r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), t.at<double>(0,0),
						r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2), t.at<double>(1,0),
						r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2), t.at<double>(2,0));
				
				// save to yml; may need to preserve previous result
				cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::WRITE);
				cfs << "camIntrinsic" << camIntrinsic;
				cfs << "camDistortion" << camDist;
				cfs << "proIntrinsic" << proIntrinsic;
				cfs << "proDistortion" << proDist;
				cfs << "proExtrinsic" << proExtrinsic;
				
				// Re-Triangulation
				ofLogNotice() << "Re-triangulate mesh";
				
				mesh = triangulate(options, horizontal, vertical, toAs(mask),
						toAs(camIntrinsic), camDist,
						toAs(proIntrinsic), proDist, toAs(proExtrinsic), camPerspective);
				mesh.save(ofToDataPath(rootDir[0] + "/out.ply"));
			} else {
				ofLogError() << "Minimum " + ofToString(minPointNum) + " points required for re-calibration";
			}
		}
		
	}
}

void testApp::dragEvent(ofDragInfo dragInfo){
	if( !pathLoaded ) {
		for( int i = 0 ; i < dragInfo.files.size() ; i++ ) {
			rootDir.push_back(dragInfo.files[i]);
		}
		init();
		pathLoaded = true;
	}
}
