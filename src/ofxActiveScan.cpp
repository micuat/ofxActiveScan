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

Map2f calibrate(Options& options, Map2f& hmap, Map2f& vmap, Map2f& mmap, Map2f& rmap,
			   Matd& camIntrinsic, double& camDist,
			   Matd& proIntrinsic, double& proDist,
			   Matd& proExtrinsic)
{
	CProCamCalibrate calib(options);
	calib.Calibrate(hmap, vmap, rmap);
	
	camIntrinsic = calib.GetCamIntrinsic();
	camDist      = calib.GetCamDistortion();
	proIntrinsic = calib.GetProIntrinsic();
	proDist      = calib.GetProDistortion();
	proExtrinsic = calib.GetProExtrinsic();
	
	Map2f mmapIn = mmap;
	
	Matd fundamental = calib.GetFundamental();
	
	for( int y = 0; y < mmap.size(1); y++ ) {
		for( int x = 0; x < mmap.size(0); x++ ) {
			if( mmap.cell(x, y) ) {
				Matd pl(3, 1);
				Matd pr(1, 3);
				pl(0, 0) = x;
				pl(1, 0) = y;
				pl(2, 0) = 1.0;
				pr(0, 0) = hmap.cell(x, y);
				pr(0, 1) = vmap.cell(x, y);
				pr(0, 2) = 1.0;
				Matd epline = fundamental * pl;
				epline = epline * (1.0 / epline(2, 0));
				Matd result = pr * epline;
				
				if( result(0, 0) > 3.0 )
					mmapIn.cell(x, y) = 0;
			}
		}
	}
	
	return mmapIn;
}

ofMesh triangulate(Options& o, Map2f& hmap, Map2f& vmap, Map2f& mmap,
				   Matd& cKd, double cD,
				   Matd& pKd, double pD, Matd& Rtd, ofImage& cp)
{
	slib::CMatrix<3,3,double> cK(cKd.ptr());
	slib::CMatrix<3,3,double> pK(pKd.ptr());
	slib::CMatrix<3,4,double> Rt(Rtd.ptr());
	
	return triangulate(o, hmap, vmap, mmap, cK, cD, pK, pD, Rt, cp);
}

ofMesh triangulate(Options& o, Map2f& hmap, Map2f& vmap, Map2f& mmap,
				   Matd& cKd, double cD,
				   Matd& pKd, double pD, Matd& Rtd, ofImage& cp, Map2i& indices)
{
	slib::CMatrix<3,3,double> cK(cKd.ptr());
	slib::CMatrix<3,3,double> pK(pKd.ptr());
	slib::CMatrix<3,4,double> Rt(Rtd.ptr());
	
	return triangulate(o, hmap, vmap, mmap, cK, cD, pK, pD, Rt, cp, indices);
}

ofMesh triangulate(Options& options, Map2f& hmap, Map2f& vmap, Map2f& mmap,
				   slib::CMatrix<3,3,double>& matKcam, double camDist,
				   slib::CMatrix<3,3,double>& matKpro, double proDist,
				   slib::CMatrix<3,4,double>& proRt, ofImage& cp)
{
	Map2i indices;
	return triangulate(options, hmap, vmap, mmap, matKcam, camDist, matKpro, proDist, proRt, cp, indices);
}

ofMesh triangulate(Options& options, Map2f& hmap, Map2f& vmap, Map2f& mmap,
				   slib::CMatrix<3,3,double>& matKcam, double camDist,
				   slib::CMatrix<3,3,double>& matKpro, double proDist,
				   slib::CMatrix<3,4,double>& proRt, ofImage& cp, Map2i& indices)
{
	ofMesh mesh;
	indices.Initialize(hmap.size());
	indices.Clear(-1);
	
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
			TRACE("triangulation: %d%% done\n", 100*y/hmap.size(1));
		
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
				if (p3d[2]<0) {
					nbehind++;
				} else {
					indices.cell(x, y) = mesh.getNumVertices();
					mesh.addVertex(ofVec3f(p3d[0], p3d[1], p3d[2]));
					if( cp.bAllocated() ) {
						mesh.addColor(cp.getColor(x, y));
					}
				}
			}
		}
		if (nbehind)
			TRACE("found %d points behind viewpoint.\n", nbehind);
	}
	
	return mesh;
}

};
