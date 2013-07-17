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

#include "ofxActiveScan.h"

namespace ofxActiveScan {

vector<ofImage> encode(Options op) {
	vector<ofImage> patterns;
	
	CEncode encode(op);
	
	for(int i = 0; i < encode.GetNumImages(); i++) {
		Map2u field;
		encode.GetImage(i, field);
		
		patterns.push_back(toOf(field));
	}
	
	return patterns;
}

void decode(Options op, Map2f& mapH, Map2f& mapV,
			Map2f& mapMask, Map2f& mapReliable, string path)
{
	CDecode decode(op);
	
	ofDirectory dir(ofToDataPath(path));
	dir.listDir();
	dir.sort();
	
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
				   Matd pKd, double pD, Matd Rtd, ofImage cp)
{
	slib::CMatrix<3,3,double> cK(cKd.ptr());
	slib::CMatrix<3,3,double> pK(pKd.ptr());
	slib::CMatrix<3,4,double> Rt(Rtd.ptr());
	
	return triangulate(o, hmap, vmap, mmap, cK, cD, pK, pD, Rt, cp);
}

ofMesh triangulate(Options options, Map2f hmap, Map2f vmap, Map2f mmap, 
				   slib::CMatrix<3,3,double> matKcam, double camDist,
				   slib::CMatrix<3,3,double> matKpro, double proDist,
				   slib::CMatrix<3,4,double> proRt, ofImage cp)
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
				if( cp.bAllocated() ) {
					mesh.addColor(cp.getColor(x, y));
				}
			}
		}
		if (nbehind)
			TRACE("found %d points behind viewpoint.", nbehind);
	}
	
	return mesh;
}

cv::Mat findTransform(vector<cv::Point3d>& input, vector<cv::Point3d>& target)
{
	// minimum 7 points required for rotation(3 params), translate(3 params), scale(1 param)
	assert(input.size() == target.size() && input.size() >= 7);
	
	int ret;
	vector<double> p, x;
	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	vector<cv::Point3d> data;
	
	for( int i = 0 ; i < input.size() ; i++ ) {
		data.push_back(input[i]);
		data.push_back(target[i]);
	}
	
	opts[0] = LM_INIT_MU;
	opts[1] = 1E-15;
	opts[2] = 1E-15;
	opts[3] = 1E-20;
	opts[4] = LM_DIFF_DELTA;
	
	p.resize(input.size());
	x.resize(target.size());
	
	for( int i = 0 ; i < p.size() ; i++ ) {
		p[i] = 0.0;
	}
	for( int i = 0 ; i < x.size() ; i++ ) {
		// minimize norm
		x[i] = 0.0;
	}
	
	ret = dlevmar_dif(levmar_3dNorm, &p[0], &x[0], p.size(), x.size(), 1000, opts, info, NULL, NULL, &data[0]);
	
	ofLog(OF_LOG_VERBOSE, "Levenberg-Marquardt returned %d in %g iter, reason %g", ret, info[5], info[6]);
	ofLog(OF_LOG_VERBOSE, "Solution:");
	
	for( int i = 0 ; i < p.size() ; ++i )
		ofLog(OF_LOG_VERBOSE, "%.7g", p[i]);
	
	ofLog(OF_LOG_VERBOSE, "Minimization info:");
	
	for( int i = 0 ; i < LM_INFO_SZ ; ++i )
		ofLog(OF_LOG_VERBOSE, "%g", info[i]);
	
	levmar_3dNorm(&p[0], &x[0], p.size(), x.size(), &data[0]);
	for( int i = 0 ; i < x.size() ; ++i )	
		ofLogVerbose() << x[i];
	
	cv::Mat rvec = (cv::Mat1d(3, 1) << p[0], p[1], p[2]);
	cv::Mat r;
	cv::Rodrigues(rvec, r);
	
	cv::Mat Rt = (cv::Mat1d(4, 4) << r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), p[3],
		      r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2), p[4],
		      r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2), p[5],
		      0, 0, 0, 1);
	
	return Rt;
}

/*
void levmar_2dNorm(double *p, double *x, int m, int n, void *data) {
	cv::Point2d *d;
	d = static_cast<cv::Point2d *>(data);
	
	cv::Mat Rt = (cv::Mat1d(3, 3) << cos(p[0]), -sin(p[0]), p[1],
		      sin(p[0]), cos(p[0]), p[2],
		      0, 0, 1);
	
	for( int i = 0 ; i < n ; i++ ) {
		cv::Mat orig = (cv::Mat1d(3, 1) << d[i*2].x, d[i*2].y, 1);
		cv::Mat transformed = Rt * orig;
		cv::Mat target = (cv::Mat1d(3, 1) << d[i*2 + 1].x, d[i*2 + 1].y, 1);
		x[i] = cv::norm(transformed, target);
	}
}
*/

void levmar_3dNorm(double *p, double *x, int m, int n, void *data) {
	cv::Point3d *d;
	d = static_cast<cv::Point3d *>(data);
	
	cv::Mat rvec = (cv::Mat1d(3, 1) << p[0], p[1], p[2]);
	cv::Mat r;
	cv::Rodrigues(rvec, r);
	
	cv::Mat Rt = (cv::Mat1d(4, 4) << r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), p[3],
		      r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2), p[4],
		      r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2), p[5],
		      0, 0, 0, 1);
	
	for( int i = 0 ; i < n ; i++ ) {
		cv::Mat orig = (cv::Mat1d(4, 1) << d[i*2].x, d[i*2].y, d[i*2].z, 1);
		cv::Mat transformed = Rt * orig;
		cv::Mat target = (cv::Mat1d(4, 1) << d[i*2 + 1].x, d[i*2 + 1].y, d[i*2 + 1].z, 1);
		x[i] = cv::norm(transformed, target);
	}
}

};
