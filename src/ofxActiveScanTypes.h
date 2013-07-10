#pragma once

#include "ProCamTools/common/Field.h"
#include "ProCamTools/Options.h"

namespace ofxActiveScan {
	
typedef options_t Options;
typedef slib::Field<2,float> Map2f;
typedef slib::CDynamicMatrix<double> Matd;
typedef slib::CVector<2,double> Vec2d;
typedef slib::CVector<3,double> Vec3d;

};
