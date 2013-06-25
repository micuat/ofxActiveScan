#pragma once

#include "mathbaseutil.h"

namespace slib
{

// Refer to: 
// http://research.microsoft.com/~zhang/Papers/TR98-71.pdf
// Appendix C
template <typename T>
inline
void FitRotationMatrix(const CMatrix<3,3,T>& mat, CMatrix<3,3,T>& ret)
{
	CMatrix<3,3,T> matU, matVt;
	CVector<3,T> vecW;
	SingularValueDecomposition(mat, true, true, matU, vecW, matVt);
	ret = matU * matVt;
}

template <typename T>
inline
void GetEulerAngle(const CMatrix<3,3,T>& mat, T& theta, T& phi, T& psi)
{
	double sin_theta = sqrt(mat(2,0)*mat(2,0)+mat(2,1)*mat(2,1));
	theta = atan2(sin_theta, mat(2,2)); 

	if (sin_theta != 0)
	{
		phi = atan2(mat(0,2)/sin_theta, -mat(1,2)/sin_theta);
		psi = atan2(mat(2,0)/sin_theta,  mat(2,1)/sin_theta);
	}
	else
	{
		phi = 0;
		psi = atan2(-mat(0,1),mat(0,0));
	}
}

template <typename T>
inline
void GetEulerMatrix(const T theta, const T phi, const T psi, CMatrix<3,3,T>& mat)
{
	double ct = cos(theta);
	double st = sin(theta);
	double cp = cos(phi);
	double sp = sin(phi);
	double cs = cos(psi);
	double ss = sin(psi);
	
	mat(0,0) = cp*cs - sp*ct*ss;
	mat(0,1) = -cp*ss-sp*ct*cs;
	mat(0,2) = sp*st;

	mat(1,0) = sp*cs + cp*ct*ss;
	mat(1,1) = -sp*ss + cp*ct*cs;
	mat(1,2) = -cp*st;

	mat(2,0) = st*ss;
	mat(2,1) = st*cs;
	mat(2,2) = ct;
}
};