//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: FundamentalMatrix.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once

#include <vector>

#include "MathBase.h"

namespace slib {
namespace fmatrix {

//////////////////////////////////////////////////////////////////////
// fundamental matrix 
//////////////////////////////////////////////////////////////////////

void EstimateFundamentalMatrixAlgebraic(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CMatrix<3,3,double>& fundamental);

void EstimateFundamentalMatrixGeometric(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CMatrix<3,3,double>& fundamental // [in/out] 
	);

// using EstimateFundamentalMatrixAlgebraic()
void EstimateFundamentalMatrixRansac(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CMatrix<3,3,double>& fundamental);

//////////////////////////////////////////////////////////////////////
// radial fundamental matrix
//////////////////////////////////////////////////////////////////////

void EstimateRadialFundamentalMatrixAlgebraic(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	const CVector<2,double>& cod1,
	const CVector<2,double>& cod2,
	double& xi1, // [in/out]
	double& xi2, // [in/out]
	CMatrix<3,3,double>& fundamental // [in/out]
	);

void EstimateRadialFundamentalMatrixGeometric(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	const CVector<2,double>& cod1,
	const CVector<2,double>& cod2,
	double& xi1, // [in/out] 
	double& xi2, // [in/out] 
	CMatrix<3,3,double>& fundamental // [in/out] 
	);

void EstimateRadialFundamentalMatrixApriori(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CVector<2,double>& cod1,// [in/out] 
	CVector<2,double>& cod2,// [in/out] 
	double& xi1, // [in/out] 
	double& xi2, // [in/out] 
	CMatrix<3,3,double>& fundamental // [in/out] 
	);

// using EstimateRadialFundamentalMatrixApriori()
void EstimateRadialFundamentalMatrixRansac(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CVector<2,double>& cod1, // [in/out]
	CVector<2,double>& cod2, // [in/out]
	double& xi1,  // [in/out]
	double& xi2,  // [in/out]
	CMatrix<3,3,double>& fundamental // [in/out]
	);

// using EstimateRadialFundamentalMatrixApriori()
void RefineRadialFundamentalMatrix(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CVector<2,double>& cod1,
	CVector<2,double>& cod2,
	double& xi1, 
	double& xi2, 
	CMatrix<3,3,double>& fundamental);

//////////////////////////////////////////////////////////////////////
// calibration
//////////////////////////////////////////////////////////////////////

void EstimateFocalLengthBougnoux(
	const CMatrix<3,3,double>& fundamental, 
	const CVector<3,double>& pp1, 
	const CVector<3,double>& pp2,
	double& f1squared, 
	double& f2squared);

template <typename T> inline
void CancelRadialDistortion(
	const T xi, 
	const CVector<2,T>& center,
	const CVector<2,T>& pos, 
	CVector<2,T>& undist) 
{
	CVector<2,T> r = pos - center;
	undist = r / (1 + xi * dot(r, r)) + center;
}

template <typename T> inline
void ApplyRadialDistortion(
	const T xi, 
	const CVector<2,T>& center,
	const CVector<2,T>& pos, 
	CVector<2,T>& dist) 
{
	CVector<2,T> diff = pos - center;
	T r2 = dot(diff, diff);
	if (xi * r2)
		dist = diff * (1 - sqrt(1 - 4 * xi * r2)) / (2 * xi * r2) + center;
	else
		dist = pos;
}

} // namespace fmatrix
} // namespace slib
