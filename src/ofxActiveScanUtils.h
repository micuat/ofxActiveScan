#pragma once

#include "ofMain.h"
#include "ofxActiveScanTypes.h"
#include "ofxCv.h"

#include "ProCamTools/common/Field.h"
#include "ProCamTools/Options.h"

namespace ofxActiveScan {

//template <int Tm, int Tn, typename T>
//inline cv::Mat& toOf(slib::CMatrix<Tm,Tn,T>);

template <int Tm, int Tn, typename T>
inline cv::Mat toOf(slib::CMatrix<Tm,Tn,T> cmat) {
	// it would be better to dump array and construct cv::Mat with array
	cv::Mat m = cv::Mat_<T>(Tm, Tn);
	
	for( int i = 0 ; i < Tm ; i++ ) {
		for( int j = 0 ; j < Tn ; j++ ) {
			m.at<T>(i, j) = cmat(i, j);
		}
	}
	
	return m;
}

ofImage toOf(Map2f);

};
