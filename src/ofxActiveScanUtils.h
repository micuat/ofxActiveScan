#pragma once

#include "ofMain.h"
#include "ofxActiveScanTypes.h"
#include "ofxCv.h"

#include "ProCamTools/common/Field.h"
#include "ProCamTools/Options.h"

namespace ofxActiveScan {

template <int Tm, int Tn, typename T>
inline cv::Mat toCv(const slib::CMatrix<Tm, Tn, T> cmat) {
	// it would be better to dump array and construct cv::Mat with array
	cv::Mat m = cv::Mat_<T>(Tm, Tn);
	
	for( int i = 0 ; i < Tm ; i++ ) {
		for( int j = 0 ; j < Tn ; j++ ) {
			m.at<T>(i, j) = cmat(i, j);
		}
	}
	
	return m;
}

inline cv::Mat toCv(const Matd cmat) {
	int mr = cmat.GetNumRows();
	int mc = cmat.GetNumCols();
	
	cv::Mat m = cv::Mat_<double>(mr, mc);
	
	for( int i = 0 ; i < mr ; i++ ) {
		for( int j = 0 ; j < mc ; j++ ) {
			m.at<double>(i, j) = cmat(i, j);
		}
	}
	
	return m;
}

template <int Tm, int Tn, typename T>
inline slib::CMatrix<Tm, Tn, T> toAs(const cv::Mat m) {
	assert(m.rows == Tm && m.cols == Tn);
	slib::CMatrix<Tm, Tn, T> cmat;
	
	for( int i = 0 ; i < Tm ; i++ ) {
		for( int j = 0 ; j < Tn ; j++ ) {
			cmat(i, j) = m.at<T>(i, j);
		}
	}
	
	return cmat;
}

inline Matd toAs(const cv::Mat m) {
	int mr = m.rows;
	int mc = m.cols;
	
	Matd cmat(mr, mc);
	
	for( int i = 0 ; i < mr ; i++ ) {
		for( int j = 0 ; j < mc ; j++ ) {
			cmat(i, j) = m.at<double>(i, j);
		}
	}
	
	return cmat;
}

inline Vec2d toAs(const cv::Point2d p) {
	Vec2d v;
	v[0] = p.x;
	v[1] = p.y;
	return v;
}

inline Vec3d toAs(const cv::Point3d p) {
	Vec3d v;
	v[0] = p.x;
	v[1] = p.y;
	v[2] = p.z;
	return v;
}

inline cv::Point2d toCv(const Vec2d p) {
	return cv::Point2d(p[0], p[1]);
}

inline cv::Point3d toCv(const Vec3d p) {
	return cv::Point3d(p[0], p[1], p[2]);
}

ofFloatImage toOfF(Map2f);
ofImage toOf(Map2f);
Map2f toAs(ofImage);

};
