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

#include "ofxActiveScanTypes.h"
#include "ofxActiveScanUtils.h"

#include "levmar.h"

namespace ofxActiveScan {

cv::Mat findTransform(vector<cv::Point3d>&, vector<cv::Point3d>&, int nIteration = 1000);

// callback function for levmar
//void levmar_2dNorm(double *, double *, int, int, void *);
void levmar_3dNorm(double *, double *, int, int, void *);

ofMesh transformMesh(ofMesh, cv::Mat);

}
