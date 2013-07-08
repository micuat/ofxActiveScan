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
	cout << camIntrinsic << endl;
	cout << proIntrinsic << endl;
	cout << proExtrinsic << endl;
	
	// horizontal and vertical correspondences between projector and camera
	Map2f horizontal(ofToDataPath(rootDir + "/h.map", true));
	Map2f vertical(ofToDataPath(rootDir + "/v.map", true));
	
	ofImage mask;
	ofLoadImage(mask, ofToDataPath(rootDir + "/mask.bmp"));

	mesh = triangulate(options, horizontal, vertical, toAs(mask),
					   toAs(camIntrinsic), camDist,
					   toAs(proIntrinsic), proDist, toAs(proExtrinsic));
	mesh.save(ofToDataPath(rootDir + "/out.ply"));
	
	// set parameters for projection
	proCalibration.setup(proIntrinsic, proSize);
	camCalibration.setup(camIntrinsic, camSize);
	
	objectPoints.resize(1);
	imagePoints.resize(1);
}

void testApp::update() {
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
	
	// Re-Calibration
	if( objectPoints[0].size() >= 6 ) {
		cv::Mat distCoeffs;
		vector<cv::Mat> rvecs, tvecs;
		
		cv::Mat m = proIntrinsic;
		proIntrinsic = (cv::Mat1d(3,3) <<
						m.at<double>(0,0), m.at<double>(0,1), proSize.width/2,
						m.at<double>(1,0), m.at<double>(1,1), proSize.height/2,
						m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2));
		
		cv::calibrateCamera(objectPoints, imagePoints, proSize, proIntrinsic,
							distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS);
		
		cv::Mat r;
		cv::Rodrigues(rvecs[0], r);
		cv::Mat t = tvecs[0];
		proExtrinsic = (cv::Mat1d(3,4) <<
						r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), t.at<double>(0,0),
						r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2), t.at<double>(1,0),
						r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2), t.at<double>(2,0));
	}
}

void testApp::draw() {
	ofBackground(0);
	
	if(cameraMode == EASYCAM_MODE) {
		cam.begin();
		ofScale(1, -1, -1);
	} else if(cameraMode == PRO_MODE) {
		ofSetupScreenPerspective(options.projector_width, options.projector_height);
		
		ofPushStyle();
		ofNoFill();
		ofSetColor(ofColor::yellow);
		ofSetLineWidth(2);
		ofCircle(nearestVertex, 4);
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
		objectPoints[0].push_back(ofxCv::toCv(mesh.getVertex(nearestIndex)));
		imagePoints[0].push_back(ofxCv::toCv(nearestVertex));
	}
	if( key == OF_KEY_UP ) {
	}
	if( key == OF_KEY_DOWN ) {
	}
	if( key == OF_KEY_LEFT ) {
	}
	if( key == OF_KEY_RIGHT ) {
	}
	
	ofVec3f p = mesh.getVertex(nearestIndex);
	cv::Mat pCv = (cv::Mat1d(4, 1) << p.x, p.y, p.z, 1);
	cv::Mat pPlane = proIntrinsic * proExtrinsic * pCv;
	pPlane = pPlane / pPlane.at<double>(2, 0);
	cout << nearestVertex << " " << pPlane << endl;
	ofVec2f mouse(mouseX, mouseY);
	cout << nearestVertex.distance(mouse) << " " << mouse << endl;
}
