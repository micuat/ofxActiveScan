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

cv::Mat findTransform(vector<cv::Point3d>& input, vector<cv::Point3d>& target, int nIteration)
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
	
	p.resize(7);
	x.resize(target.size());
	
	for( int i = 0 ; i < p.size() ; i++ ) {
		p[i] = 0.0;
	}
	for( int i = 0 ; i < x.size() ; i++ ) {
		// minimize norm
		x[i] = 0.0;
	}
	
	ret = dlevmar_dif(levmar_3dNorm, &p[0], &x[0], p.size(), x.size(), nIteration, opts, info, NULL, NULL, &data[0]);
	
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
	
	r = r * p[6];
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
	
	r = r * p[6];
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

ofMesh transformMesh(ofMesh mesh, cv::Mat Rt) {
	ofMesh m;
	for( int i = 0 ; i < mesh.getNumVertices() ; i++ ) {
		ofVec3f orig = mesh.getVertex(i);
		cv::Mat o = (cv::Mat1d(4, 1) << orig.x, orig.y, orig.z, 1);
		cv::Mat t = Rt * o;
		ofVec3f transformed(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0));
		
		m.addVertex(transformed);
		m.addColor(mesh.getColor(i));
	}
	
	return m;
}

};
