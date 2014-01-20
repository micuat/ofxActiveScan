//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: Stereo.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once

#include <vector>

namespace slib
{

//----------------------------------------------------------------------
// triangulate a 3D point from a set of projections
// by the homogeneous DLT method (see "multiple view geometry" p.312)
//----------------------------------------------------------------------

template<typename T> inline
void SolveStereo(const std::vector< CVector<2,T> >& p2d,
				   const std::vector< CMatrix<3,4,T> >& proj,
				   CVector<3,T>& pos3d)
{
	int num = p2d.size();
	CDynamicMatrix<T> matA(2 * num, 4);
	for (int i=0; i<num; i++)
	{
		// normalize a point using homogeneous representation
		CVector<3,T> p = GetHomogeneousVector(p2d[i]);
		p *= sqrt(T(2)) / GetNorm2(p); 

		matA(2*i,  0) = p[0] * proj[i](2,0) - p[2] * proj[i](0,0);
		matA(2*i,  1) = p[0] * proj[i](2,1) - p[2] * proj[i](0,1);
		matA(2*i,  2) = p[0] * proj[i](2,2) - p[2] * proj[i](0,2);
		matA(2*i,  3) = p[0] * proj[i](2,3) - p[2] * proj[i](0,3);

		matA(2*i+1,0) = p[1] * proj[i](2,0) - p[2] * proj[i](1,0);
		matA(2*i+1,1) = p[1] * proj[i](2,1) - p[2] * proj[i](1,1);
		matA(2*i+1,2) = p[1] * proj[i](2,2) - p[2] * proj[i](1,2);
		matA(2*i+1,3) = p[1] * proj[i](2,3) - p[2] * proj[i](1,3);
	}

	CDynamicVector<T> x;
	FindRightNullVector(matA, x);

	x[0] /= x[3];
	x[1] /= x[3];
	x[2] /= x[3];
	pos3d.Initialize( x.ptr() );
}

}
