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

#define TRACE printf

#include "ofxActiveScanTypes.h"
#include "ofxActiveScanUtils.h"
#include "ofxActiveScanTransform.h"

#include "Field.h"
#include "ImageBmpIO.h"
#include "MiscUtil.h"
#include "MathBaseLapack.h"
#include "Stereo.h"
#include "LeastSquare.h"
#include "encode.h"
#include "decode.h"
#include "calibrate.h"
#include "triangulate.h"
#include "FundamentalMatrix.h"
#include "Options.h"

namespace ofxActiveScan {

void calibrate(Options&, Map2f&, Map2f&, Map2f&,
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

}
