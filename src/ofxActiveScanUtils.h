#pragma once

#include "ofMain.h"
#include "ofxActiveScanTypes.h"
#include "ofxCv.h"

#include "ProCamTools/common/Field.h"
#include "ProCamTools/Options.h"

namespace ofxActiveScan {

template <int Tm, int Tn, typename T>
inline cv::Mat toOf(const slib::CMatrix<Tm, Tn, T> cmat) {
	// it would be better to dump array and construct cv::Mat with array
	cv::Mat m = cv::Mat_<T>(Tm, Tn);
	
	for( int i = 0 ; i < Tm ; i++ ) {
		for( int j = 0 ; j < Tn ; j++ ) {
			m.at<T>(i, j) = cmat(i, j);
		}
	}
	
	return m;
}

template <int Tm, int Tn, typename T>
inline slib::CMatrix<Tm, Tn, T> toAS(const cv::Mat m) {
	assert(m.rows == Tm && m.cols == Tn);
	slib::CMatrix<Tm, Tn, T> cmat;
	
	for( int i = 0 ; i < Tm ; i++ ) {
		for( int j = 0 ; j < Tn ; j++ ) {
			cmat(i, j) = m.at<T>(i, j);
		}
	}
	
	return cmat;
}

inline Matd toAS(const cv::Mat m) {
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

ofImage toOf(Map2f);

};
