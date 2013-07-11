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

#include "ProCamTools/common/Field.h"
#include "ProCamTools/Options.h"

namespace ofxActiveScan {
	
typedef options_t Options;
typedef slib::Field<2,unsigned char> Map2u;
typedef slib::Field<2,float> Map2f;
typedef slib::CDynamicMatrix<double> Matd;
typedef slib::CVector<2,double> Vec2d;
typedef slib::CVector<3,double> Vec3d;

};
