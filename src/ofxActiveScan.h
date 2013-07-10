#pragma once

#include "ofMain.h"

#include <stdlib.h>

// convert TRACE into ofLog
#define TRACE DEBUGofLog

#if __STDC_VERSION__ >= 199901L
	#define DEBUGofLog(__VA_ARGS__) ofLog(OF_LOG_NOTICE, __VA_ARGS__)
#else
	#define DEBUGofLog(args...) ofLog(OF_LOG_NOTICE, ##args)
#endif
//

#include "ofxActiveScanTypes.h"
#include "ofxActiveScanUtils.h"

#include "ProCamTools/stdafx.h"
#include "ProCamTools/common/Field.h"
#include "ProCamTools/common/ImageBmpIO.h"
#include "ProCamTools/common/MiscUtil.h"
#include "ProCamTools/common/MathBaseLapack.h"
#include "ProCamTools/common/Stereo.h"
#include "ProCamTools/common/LeastSquare.h"
#include "ProCamTools/encode.h"
#include "ProCamTools/decode.h"
#include "ProCamTools/calibrate.h"
#include "ProCamTools/triangulate.h"
#include "ProCamTools/FundamentalMatrix.h"
#include "ProCamTools/Options.h"

namespace ofxActiveScan {

vector<ofFloatImage> encode(Options);

void decode(Options, Map2f&, Map2f&, Map2f&, Map2f&, string);

void calibrate(Options, Map2f, Map2f, Map2f, 
			   Matd&, double&,
			   Matd&, double&,
			   Matd&);

ofMesh triangulate(Options, Map2f, Map2f, Map2f, 
				   Matd, double,
				   Matd, double, Matd);

ofMesh triangulate(Options, Map2f, Map2f, Map2f, 
				   slib::CMatrix<3,3,double>, double,
				   slib::CMatrix<3,3,double>, double,
				   slib::CMatrix<3,4,double>);
	
}
