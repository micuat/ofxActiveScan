#include "testApp.h"

// entry point
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	cv::FileStorage cfs(ofToDataPath(rootDir + "/calibration.yml"), cv::FileStorage::READ);
	cv::Mat cami, proi, proe;
	double xi1, xi2;
	cfs["camIntrinsic"] >> cami;
	cfs["camDistortion"] >> xi2;
	cfs["proIntrinsic"] >> proi;
	cfs["proDistortion"] >> xi1;
	cfs["proExtrinsic"] >> proe;
	
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
	matKcam = ofxActiveScan::toPC<3, 3, double>(cami);
	matKpro = ofxActiveScan::toPC<3, 3, double>(proi);
	
	// extrinsic matrices of projector and camera
	slib::CMatrix<3,4,double> proRt, camRt;
	proRt = ofxActiveScan::toPC<3, 4, double>(proe);
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

				// save
				result.push_back(p3d);
				if (p3d[2]<0)
					nbehind++;
				
				mesh.addVertex(ofVec3f(p3d[0], p3d[1], p3d[2]));
			}
		}
		if (m_debug && nbehind)
			TRACE("\rfound %d points behind viewpoint.\n", nbehind);
	}
	printf("\n");
	// export triangular mesh in PLY format
	WritePly(result, mask, m_plyfilename);
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
	
	cam.begin();
	
	ofScale(100, -100, -100);
	mesh.drawVertices();
	
	cam.end();
}
