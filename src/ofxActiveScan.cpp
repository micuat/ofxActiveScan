#include "ofxActiveScan.h"

namespace ofxActiveScan {

int Encode::init(Options op) {
	CEncode encode(op);
	width = op.projector_width;
	height = op.projector_height;
	
	size = encode.GetNumImages();
	for (int i = 0; i < size; i++) {
		Map2f field;
		encode.GetImage(i, field);
		
		patterns.push_back(toOf(field));
	}
	return 0;
}

int Encode::getSize() {
	return size;
}

int Encode::getWidth() {
	return width;
}

int Encode::getHeight() {
	return height;
}

ofImage& Encode::getPatternAt(int i) {
	return patterns[i];
}

int Decode::init(Options op, string path) {
	CDecode decode(op);
	
	ofDirectory dir(ofToDataPath(path, true));
	dir.listDir();
	
	std::vector<std::string> files;
	for(int i = 0; i < dir.numFiles(); i++) {
		files.push_back(dir.getPath(i));
	}
	
	ofLogNotice() << "Decode::init() start decoding";
	decode.Decode(files);
	
	mapH = decode.GetMap(0);
	mapV = decode.GetMap(1);
	mapMask = decode.GetMask();
	mapReliable = decode.GetReliable();
}

Map2f& Decode::getMapHorizontal() {
	return mapH;
}

Map2f& Decode::getMapVertical() {
	return mapV;
}

Map2f& Decode::getMask() {
	return mapMask;
}

Map2f& Decode::getReliable() {
	return mapReliable;
}

void calibrate(Options options, Map2f hmap, Map2f vmap, Map2f mmap, 
			   Matd& camIntrinsic, double& camDist,
			   Matd& proIntrinsic, double& proDist,
			   Matd& proExtrinsic)
{
	CProCamCalibrate calib(options);
	calib.Calibrate(hmap, vmap, mmap);
	
	camIntrinsic = calib.GetCamIntrinsic();
	camDist      = calib.GetCamDistortion();
	proIntrinsic = calib.GetProIntrinsic();
	proDist      = calib.GetProDistortion();
	proExtrinsic = calib.GetProExtrinsic();
}

ofMesh triangulate(Options o, Map2f hmap, Map2f vmap, Map2f mmap,
				   Matd cKd, double cD,
				   Matd pKd, double pD, Matd Rtd)
{
	slib::CMatrix<3,3,double> cK(cKd.ptr());
	slib::CMatrix<3,3,double> pK(pKd.ptr());
	slib::CMatrix<3,4,double> Rt(Rtd.ptr());
	
	return triangulate(o, hmap, vmap, mmap, cK, cD, pK, pD, Rt);
}

ofMesh triangulate(Options options, Map2f hmap, Map2f vmap, Map2f mmap, 
				   slib::CMatrix<3,3,double> matKcam, double camDist,
				   slib::CMatrix<3,3,double> matKpro, double proDist,
				   slib::CMatrix<3,4,double> proRt)
{
	ofMesh mesh;
	
	slib::CVector<2,double>
		cod1=make_vector<double>((options.projector_width+1)/2.0,options.projector_height*options.projector_horizontal_center),
		cod2=(make_vector(1.0,1.0)+mmap.size())/2;
	
	// extrinsic matrices of projector and camera
	slib::CMatrix<3,4,double> camRt;
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
	for (int y=0; y<hmap.size(1); y++)
	{
		if (y % (hmap.size(1)/10) == 0)
			TRACE("triangulation: %d%% done", 100*y/hmap.size(1));
		
		int nbehind=0;
		for (int x=0; x<hmap.size(0); x++)
		{
			if (mmap.cell(x,y))
			{
				// 2D correspondence
				std::vector<CVector<2,double> > p2d(2);
				
				// camra coordinate
				slib::fmatrix::CancelRadialDistortion(camDist,cod2,make_vector<double>(x,y),p2d[0]);
				
				// projector coordinate
				double proj_y;
				
				proj_y = vmap.cell(x,y);
				
				//CVector<3,double> epiline = matF * GetHomogeneousVector(p2d[0]);
				//proj_y = -(epiline[0] * hmap.cell(x,y) + epiline[2]) / epiline[1];
				
				slib::fmatrix::CancelRadialDistortion(proDist,cod1,make_vector<double>(hmap.cell(x,y),proj_y),p2d[1]);
				
				// triangulate
				CVector<3,double> p3d;
				SolveStereo(p2d, matrices, p3d);
				
				// save
				//result.push_back(p3d);
				if (p3d[2]<0)
					nbehind++;
				
				mesh.addVertex(ofVec3f(p3d[0], p3d[1], p3d[2]));
			}
		}
		if (nbehind)
			TRACE("found %d points behind viewpoint.", nbehind);
	}
	
	return mesh;
}

};
