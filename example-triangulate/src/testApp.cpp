#include "testApp.h"

using namespace ofxActiveScan;

// entry point
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cameraMode = EASYCAM_MODE;
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> camSize.width;
	fs["camHeight"] >> camSize.height;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	cv::FileStorage cfs(ofToDataPath(rootDir + "/calibration.yml"), cv::FileStorage::READ);
	cfs["camIntrinsic"] >> camIntrinsic;
	cfs["camDistortion"] >> camDist;
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proDistortion"] >> proDist;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	proSize.width = options.projector_width;
	proSize.height = options.projector_height;
	
	// horizontal and vertical correspondences between projector and camera
	horizontal = Map2f(ofToDataPath(rootDir + "/h.map", true));
	vertical = Map2f(ofToDataPath(rootDir + "/v.map", true));
	
	ofLoadImage(mask, ofToDataPath(rootDir + "/mask.bmp"));
	
	
	ofLogNotice() << "Start triangulation with initial calibration parameters";
	mesh = triangulate(options, horizontal, vertical, toAs(mask),
					   toAs(camIntrinsic), camDist,
					   toAs(proIntrinsic), proDist, toAs(proExtrinsic));
	mesh.save(ofToDataPath(rootDir + "/out.ply"));
	
	// set parameters for projection
	proCalibration.setup(proIntrinsic, proSize);
	camCalibration.setup(camIntrinsic, camSize);
	
	curImagePoint = imagePoints.end();
}

void testApp::update() {
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

void testApp::draw() {
	ofBackground(0);
	
	if(cameraMode == EASYCAM_MODE) {
		cam.begin();
		ofScale(1, -1, -1);
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

void testApp::keyPressed(int key) {
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
			
			// Convert object point reference to actual points
			vector<cv::Point3f> objectPoints;
			for( int k = 0 ; k < objectPointsRef.size() ; k++ ) {
				objectPoints.push_back(ofxCv::toCv(mesh.getVertex(objectPointsRef[k])));
			}
			
			cv::Mat distCoeffs = (cv::Mat1d(4,1) << proDist, proDist, 0, 0);
			cv::Mat m = proExtrinsic;
			cv::Mat r = (cv::Mat1d(3,3) <<
					m.at<double>(0,0), m.at<double>(0,1), m.at<double>(0,2),
					m.at<double>(1,0), m.at<double>(1,1), m.at<double>(1,2),
					m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2));
			cv::Mat rvec, tvec;
			cv::Rodrigues(r, rvec);
			tvec = (cv::Mat1d(3,1) <<
					m.at<double>(0,3), m.at<double>(1,3), m.at<double>(2,3));
			
			cv::solvePnP(objectPoints, imagePoints, proIntrinsic, distCoeffs, rvec, tvec, true);
			
			cv::Rodrigues(rvec, r);
			cv::Mat t = tvec;
			proExtrinsic = (cv::Mat1d(3,4) <<
					r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), t.at<double>(0,0),
					r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2), t.at<double>(1,0),
					r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2), t.at<double>(2,0));
			
			// save to yml; may need to preserve previous result
			cv::FileStorage cfs(ofToDataPath(rootDir + "/calibration.yml"), cv::FileStorage::WRITE);
			cfs << "camIntrinsic" << camIntrinsic;
			cfs << "camDistortion" << camDist;
			cfs << "proIntrinsic" << proIntrinsic;
			cfs << "proDistortion" << proDist;
			cfs << "proExtrinsic" << proExtrinsic;
			
			// Re-Triangulation
			ofLogNotice() << "Re-triangulate mesh";
			
			mesh = triangulate(options, horizontal, vertical, toAs(mask),
					toAs(camIntrinsic), camDist,
					toAs(proIntrinsic), proDist, toAs(proExtrinsic));
			mesh.save(ofToDataPath(rootDir + "/out.ply"));
		} else {
			ofLogError() << "Minimum " + ofToString(minPointNum) + " points required for re-calibration";
		}
	}
}
