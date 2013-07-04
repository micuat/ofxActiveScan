#include "testApp.h"

// entry point
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cameraMode = EASYCAM_MODE;
	
	tx = ty = 0;

	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> camSize.width;
	fs["camHeight"] >> camSize.height;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	cv::FileStorage cfs(ofToDataPath(rootDir + "/calibration.yml"), cv::FileStorage::READ);
	cfs["camIntrinsic"] >> camIntrinsic;
	cfs["camDistortion"] >> xi2;
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proDistortion"] >> xi1;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	proSize.width = options.projector_width;
	proSize.height = options.projector_height;
	cout << camIntrinsic << endl;
	cout << proIntrinsic << endl;
	cout << proExtrinsic << endl;
	
	string plyFilename = ofToDataPath(rootDir + "/out.ply", true);
	m_plyfilename = new char[plyFilename.length() + 1];
	strcpy(m_plyfilename, plyFilename.c_str());
	
	// horizontal and vertical correspondences between projector and camera
	ofxActiveScan::Map2f horizontal(ofToDataPath(rootDir + "/h.map", true));
	ofxActiveScan::Map2f vertical(ofToDataPath(rootDir + "/v.map", true));
	
	string vmapFilename = ofToDataPath(rootDir + "/v.map", true);
	m_vmapfilename = new char[vmapFilename.length() + 1];
	strcpy(m_vmapfilename, vmapFilename.c_str());
	
	ofxActiveScan::Map2f mask;
	slib::image::Read(mask, ofToDataPath(rootDir + "/mask.bmp", true));
	
	slib::CVector<2,double>
		cod1=make_vector<double>((options.projector_width+1)/2.0,options.projector_height*options.projector_horizontal_center),
		cod2=(make_vector(1.0,1.0)+mask.size())/2;

	// intrinsic matrices of projector and camera
	slib::CMatrix<3,3,double> matKpro, matKcam;
	matKcam = ofxActiveScan::toPC<3, 3, double>(camIntrinsic);
	matKpro = ofxActiveScan::toPC<3, 3, double>(proIntrinsic);
	
	// extrinsic matrices of projector and camera
	slib::CMatrix<3,4,double> proRt, camRt;
	proRt = ofxActiveScan::toPC<3, 4, double>(proExtrinsic);
	camRt = make_diagonal_matrix(1,1,1).AppendCols(make_vector(0,0,0));//CMatrix<3,4,double>::GetIdentity(); // reconstruction is in camera coordinate frame

	// compute projection matrices of projector and camera
	std::vector<slib::CMatrix<3,4,double> > matrices(2);
	matrices[0] = matKcam * camRt; // camera
	matrices[1] = matKpro * proRt; // projector

	// fundamental matrix
	CMatrix<3,3,double> matR;
	CVector<3,double> vecT;
	matR.Initialize(proRt.ptr());
	vecT.Initialize(proRt.ptr()+9);
	CMatrix<3,3,double> matF = transpose_of(inverse_of(matKpro)) * GetSkewSymmetric(vecT) * matR * inverse_of(matKcam);

	// triangulate 3d points
	std::vector<CVector<3,double> > result;
	for (int y=0; y<horizontal.size(1); y++)
	{
		if (y % (horizontal.size(1)/100) == 0)
			printf("\rtriangulation: %d%% done", 100*y/horizontal.size(1));

		int nbehind=0;
		for (int x=0; x<horizontal.size(0); x++)
		{
			if (mask.cell(x,y))
			{
				// 2D correspondence
				std::vector<CVector<2,double> > p2d(2);

				// camra coordinate
				slib::fmatrix::CancelRadialDistortion(xi2,cod2,make_vector<double>(x,y),p2d[0]);

				// projector coordinate
				double proj_y;
				if (m_vmapfilename)
				{
					proj_y = vertical.cell(x,y);
				}
				else
				{
					CVector<3,double> epiline = matF * GetHomogeneousVector(p2d[0]);
					proj_y = -(epiline[0] * horizontal.cell(x,y) + epiline[2]) / epiline[1];
				}
				slib::fmatrix::CancelRadialDistortion(xi1,cod1,make_vector<double>(horizontal.cell(x,y),proj_y),p2d[1]);

				// triangulate
				CVector<3,double> p3d;
				SolveStereo(p2d, matrices, p3d);
				
				p3d *= 100;
				
				// save
				result.push_back(p3d);
				if (p3d[2]<0)
					nbehind++;
				
				mesh.addVertex(ofVec3f(p3d[0], p3d[1], p3d[2]));
				cv::Mat pprojected, p3, nullmat;
				nullmat = (cv::Mat1d(3, 4) << 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0);
				p3 = (cv::Mat1d(4, 1) << p3d[0], p3d[1], p3d[2], 1);
				
				/*
				cout << p2d[0][0] << " " << p2d[0][1] << endl;
				cout << p2d[1][0] << " " << p2d[1][1] << endl;
				cout << "3d coordinate " << p3 << endl;
				pprojected = camIntrinsic * nullmat * p3;
				cout << "cam plane " << pprojected / pprojected.at<double>(2, 0) << endl;
				pprojected = proIntrinsic * proExtrinsic * p3;
				cout << "pro plane " << pprojected / pprojected.at<double>(2, 0) << endl;
				 */
			}
		}
		if (m_debug && nbehind)
			TRACE("\rfound %d points behind viewpoint.\n", nbehind);
	}
	printf("\n");
	// export triangular mesh in PLY format
	WritePly(result, mask, m_plyfilename);
	
	proCalibration.setup(proIntrinsic, proSize);
	camCalibration.setup(camIntrinsic, camSize);
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
	
	if(cameraMode == EASYCAM_MODE) {
		cam.begin();
		ofScale(1, -1, -1);
	} else if(cameraMode == PRO_MODE) {
		ofSetupScreenPerspective(options.projector_width, options.projector_height);
		proCalibration.loadProjectionMatrix();
		cv::Mat m = proExtrinsic;
		cv::Mat extrinsics = (cv::Mat1d(4,4) <<
						  m.at<double>(0,0), m.at<double>(0,1), m.at<double>(0,2), m.at<double>(0,3),
						  m.at<double>(1,0), m.at<double>(1,1), m.at<double>(1,2), m.at<double>(1,3),
						  m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2), m.at<double>(2,3),
						  0, 0, 0, 1);
		extrinsics = extrinsics.t();
		glMultMatrixd((GLdouble*) extrinsics.ptr(0, 0));
		ofTranslate(tx, ty, 0);
	} else if(cameraMode == CAM_MODE) {
		ofSetupScreenPerspective(camSize.width, camSize.height);
		camCalibration.loadProjectionMatrix();
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
		case OF_KEY_DOWN: ty += 10; break;
		case OF_KEY_UP: ty -= 10; break;
		case OF_KEY_RIGHT: tx += 10; break;
		case OF_KEY_LEFT: tx -= 10; break;
	}
	cout << tx << " " << ty << endl;
	if( key == 'f' ) {
		ofToggleFullscreen();
	}
}
