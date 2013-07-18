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

#include "levmar.h"

namespace ofxActiveScan {

vector<ofImage> encode(Options);

void decode(Options, Map2f&, Map2f&, Map2f&, Map2f&, string);

void calibrate(Options, Map2f, Map2f, Map2f, 
			   Matd&, double&,
			   Matd&, double&,
			   Matd&);

ofMesh triangulate(Options, Map2f, Map2f, Map2f, 
				   Matd, double,
				   Matd, double, Matd, ofImage);

ofMesh triangulate(Options, Map2f, Map2f, Map2f, 
				   slib::CMatrix<3,3,double>, double,
				   slib::CMatrix<3,3,double>, double,
				   slib::CMatrix<3,4,double>, ofImage);

cv::Mat findTransform(vector<cv::Point3d>&, vector<cv::Point3d>&, int nIteration = 1000);

// callback function for levmar
//void levmar_2dNorm(double *, double *, int, int, void *);
void levmar_3dNorm(double *, double *, int, int, void *);

ofMesh transformMesh(ofMesh, cv::Mat);

}
