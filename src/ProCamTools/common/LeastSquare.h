//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: LeastSquare.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once
#include "MathBaseLapack.h"

namespace slib
{

//
// minimize |Ax| under the constraint |x|=1
//
template <typename T> inline
void FindRightNullVector(const CDynamicMatrix<T>& matA, CDynamicVector<T>& x)
{
	// singular value decomposition
	CDynamicMatrix<T>  matVt;
	CDynamicVector<T> vecW;
	SingularValueDecomposition(matA, vecW, matVt);

	x.Resize(matA.GetNumCols());

	int r = matVt.GetNumRows()-1;
	for (int c=0; c<matA.GetNumCols(); c++)
		x[c] = matVt(r,c);
}

template <int nRows, int nCols, typename T> inline
void FindRightNullVector(const CMatrix<nRows,nCols,T>& matA, CVector<nCols,T>& x)
{
	CDynamicMatrix<T> m(matA);
	CDynamicVector<T> v(nCols);
	FindRightNullVector(m, v);
	x.Initialize(v.ptr());
}

// minimize |Ax-b|
template <typename T> inline
void SolveLeastSquare(const CDynamicMatrix<T>& mat, const CDynamicVector<T>& vec, CDynamicVector<T>& solution)
{
	// singular value decomposition
	CDynamicMatrix<T> matU, matVt;
	CDynamicVector<T> vecW;
	SingularValueDecomposition(mat, matU, vecW, matVt);
	CDynamicMatrix<T> matW = CDynamicMatrix<T>::GetZero(mat.GetNumRows(), mat.GetNumCols());
	for (int d=0; d<vecW.GetNumRows(); d++)
		if (vecW[d] > 1e-6)
			matW(d,d) = 1.0/vecW[d];
		else
			matW(d,d) = 0;
	solution = transpose_of(matU * matW * matVt) * vec;
}

// minimize |Ax| under the constraint |Cx|=1
// ref) Multiview Geometry 2nd Ed. p.596
template <typename T> inline
void SolveLeastSquareLinearConstraint(const CDynamicMatrix<T>& matA, const CDynamicMatrix<T>& matC, const int rankC, CDynamicVector<T>& x)
{
	if (matA.GetNumCols() != matC.GetNumCols())
		throw std::runtime_error("number of columns mismatch");
	
	int nRows = matA.GetNumRows();
	int nCols = matA.GetNumCols();

	// (i)
	CDynamicMatrix<T>  matVt;
	CDynamicVector<T> vecW;
	SingularValueDecomposition(matC,  vecW, matVt);
	CDynamicMatrix<T> matAd = matA * transpose_of(matVt);

	// (ii)
	CDynamicMatrix<T> matA1d(nRows, rankC);
	CDynamicMatrix<T> matA2d(nRows, nCols-rankC);
	for (int c=0; c<rankC; c++)
		for (int r=0; r<nRows; r++)
			matA1d(r,c)=matAd(r,c);
	for (int c=rankC; c<nCols; c++)
		for (int r=0; r<nRows; r++)
			matA2d(r,c-rankC)=matAd(r,c);

	// (iii)
	CDynamicMatrix<T> matD1inv(rankC,rankC);
	for (int c=0; c<rankC; c++)
		for (int r=0; r<rankC; r++)
			matD1inv(r,c) = (r==c) ? 1.0/vecW[r] : 0;

	// (iv)
	CDynamicMatrix<T> identity = CDynamicMatrix<T>::GetIdentity(nRows,nRows);
	CDynamicMatrix<T> matA2dPInv = GetPseudoInverse(matA2d);
	CDynamicMatrix<T> matAdd = (matA2d * matA2dPInv - identity) * matA1d * matD1inv;

	// (v)
	CDynamicVector<T> xdd;
	FindRightNullVector(matAdd, xdd);

	// (vi)
	CDynamicVector<T> x1d = matD1inv * xdd;
	CDynamicVector<T> x2d = -matA2dPInv * matA1d * x1d;
	CDynamicVector<T> xd(nCols);
	for (int r=0; r<rankC; r++)
		xd[r] = x1d[r];
	for (int r=rankC; r<nCols; r++)
		xd[r] = x2d[r-rankC];

	// (vii)
	x = transpose_of(matVt) * xd;
}



};