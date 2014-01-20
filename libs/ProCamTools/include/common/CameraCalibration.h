//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: CameraCalibration.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once

#include "MathBase.h"
#include "MiscUtil.h"
#include "DLT.h"

namespace slib
{

//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

template <typename T>
inline
void DecomposeProjection(
	const CMatrix<3,4,T>& projection,
	CMatrix<3,4,T>& intrinsic,
	CMatrix<4,4,T>& extrinsic,
	T& scale)
{
	CVector<3,T> p1 = make_vector(projection(0,0),projection(0,1),projection(0,2));
	CVector<3,T> p2 = make_vector(projection(1,0),projection(1,1),projection(1,2));
	CVector<3,T> p3 = make_vector(projection(2,0),projection(2,1),projection(2,2));
	T p03 = projection(0,3);
	T p13 = projection(1,3);
	T p23 = projection(2,3);

	scale = GetNorm2(p3);
	T sign  = (p23 > 0) ? 1 : -1;
	p1 /= scale * sign;
	p2 /= scale * sign;
	p3 /= scale * sign;
	p03 *= sign;
	p13 *= sign;
	p23 *= sign;

	// intrinsics
	T theta = acos( - dot(cross(p1,p3),cross(p2,p3)) / (GetNorm2(cross(p1,p3))*GetNorm2(cross(p2,p3))));
	T au = GetNorm2(cross(p1, p3)) * sin(theta);
	T av = GetNorm2(cross(p2, p3)) * sin(theta);
	T u0 = dot(p1, p3);
	T v0 = dot(p2, p3);

	// extrinsics
	CVector<3,T> r1 = (p1 + (p2-p3*v0) * au/av*cos(theta) - p3*u0) / au;
	CVector<3,T> r2 = (p2 - p3*v0) * sin(theta)/au;
	CVector<3,T> r3 = p3;
	CVector<3,T> t = make_vector(
		p03/au + (p13-v0*p23)/av*cos(theta) - u0*p23/au,
		sin(theta)/av*(p13 - v0*p23),
		p23);

	intrinsic(0,0) = au; intrinsic(0,1) = -au/tan(theta); intrinsic(0,2) = u0; intrinsic(0,3) = 0;
	intrinsic(1,0) =  0; intrinsic(1,1) =  av/sin(theta); intrinsic(1,2) = v0; intrinsic(1,3) = 0;
	intrinsic(2,0) =  0; intrinsic(2,1) =              0; intrinsic(2,2) =  1; intrinsic(2,3) = 0;

	extrinsic(0,0) = r1[0]; extrinsic(0,1) = r1[1]; extrinsic(0,2) = r1[2]; extrinsic(0,3) = t[0];
	extrinsic(1,0) = r2[0]; extrinsic(1,1) = r2[1]; extrinsic(1,2) = r2[2]; extrinsic(1,3) = t[1];
	extrinsic(2,0) = r3[0]; extrinsic(2,1) = r3[1]; extrinsic(2,2) = r3[2]; extrinsic(2,3) = t[2];
	extrinsic(3,0) =     0; extrinsic(3,1) =     0; extrinsic(3,2) =     0; extrinsic(3,3) =    1;

#ifdef _DEBUG
	CMatrix<3,4,T> debug = intrinsic * extrinsic * GetScalingMatrix(scale) * sign;
	DUMP_VECMAT(intrinsic);
	DUMP_VECMAT(extrinsic);
	TRACE("scale = %f\n", scale);
#endif

}

template <typename T>
inline
CVector<4,T> GetProjectionCenter(const CMatrix<3,4,T>& projection)
{
	CMatrix<3,3,T> mat = make_matrix(
		projection(0,0), projection(0,1), projection(0,2),
		projection(1,0), projection(1,1), projection(1,2),
		projection(2,0), projection(2,1), projection(2,2));
	CVector<3,T> p4 = make_vector(projection(0,3), projection(1,3), projection(2,3));

	return GetHomogeneousVector(-inverse_of(mat) * p4);
}

//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

/*
template <typename T>
inline
bool EstimateProjectionMatrix(
	const std::vector<CVector<2,T> >& p2d,
	const std::vector<CVector<3,T> >& pos3d,
	CMatrix<3,4,T>& matP,
	bool bNormalize = true)
{
	_ASSERTE(p2d.size() == pos3d.size());
	if (p2d.size()<6)
		throw std::runtime_error(format("error: insufficient number of points %d\n", p2d.size()));

	std::vector<CVector<2,T> > tp2d;
	std::vector<CVector<3,T> > tpos3d;
	CVector<2,T> center2d = make_vector(0,0);
	CVector<3,T> center3d = make_vector(0,0,0);
	T scale2d=1, scale3d=1;
	if (bNormalize)
	{
		dlt::normalize(p2d, tp2d, scale2d, center2d);
		dlt::normalize(pos3d, tpos3d, scale3d, center3d);
	}
	else
	{
		tp2d = p2d;
		tpos3d = pos3d;
	}

	CDynamicMatrix<double> mat(2 * p2d.size(), 12);
	for (int r=0; r<p2d.size(); r++)
	{
		mat(2*r  ,0) =tpos3d[r][0];
		mat(2*r  ,1) =tpos3d[r][1];
		mat(2*r  ,2) =tpos3d[r][2];
		mat(2*r  ,3) =1;
		mat(2*r  ,4) =0;
		mat(2*r  ,5) =0;
		mat(2*r  ,6) =0;
		mat(2*r  ,7) =0;
		mat(2*r  ,8) =-tp2d[r][0] * tpos3d[r][0];
		mat(2*r  ,9) =-tp2d[r][0] * tpos3d[r][1];
		mat(2*r  ,10)=-tp2d[r][0] * tpos3d[r][2];
		mat(2*r  ,11)=-tp2d[r][0];

		mat(2*r+1,0) =0;
		mat(2*r+1,1) =0;
		mat(2*r+1,2) =0;
		mat(2*r+1,3) =0;
		mat(2*r+1,4) =tpos3d[r][0];
		mat(2*r+1,5) =tpos3d[r][1];
		mat(2*r+1,6) =tpos3d[r][2];
		mat(2*r+1,7) =1;
		mat(2*r+1,8) =-tp2d[r][1] * tpos3d[r][0];
		mat(2*r+1,9) =-tp2d[r][1] * tpos3d[r][1];
		mat(2*r+1,10)=-tp2d[r][1] * tpos3d[r][2];
		mat(2*r+1,11)=-tp2d[r][1];
	}

	CDynamicMatrix<double> mat2;
	transpose_of(mat, mat2);
	mat2 *= mat;

	CDynamicMatrix<double> matEigenVectors(12,12);
	CDynamicVector<double> vecEigenValues(12);
	CalcJacobiTransformation(mat2, matEigenVectors, vecEigenValues);

	double minval=FLT_MAX/2;
	for (int i=0; i<12; i++)
	{
		if (vecEigenValues[i] < minval)
		{
			minval = std::min(minval, vecEigenValues[i]);
			for (int r=0, j=0; r<3; r++)
				for (int c=0; c<4; c++, j++)
					matP(r,c) = matEigenVectors(j,i);
		}
	}

	if (bNormalize)
	{
		matP = make_matrix<T>(
				1/scale2d, 0, center2d[0],
				0, 1/scale2d, center2d[1],
				0, 0, 1
			) *	matP * make_matrix<T>(
				scale3d, 0, 0, -scale3d*center3d[0],
				0, scale3d, 0, -scale3d*center3d[1],
				0, 0, scale3d, -scale3d*center3d[2],
				0, 0, 0, 1
			);
	}

	DUMP_VECMAT(matP);
	return true;
}
*/

template <typename T> inline
bool AlmostEqual(const T& v1, const T& v2)
{
	return abs(v1-v2) < 1e-5;
}

//
// solve focal lengths of two cameras, a1 and a2, 
// given the columns and non-zero singular values of F, 
// u1, u2, v1, v2, s1, s2, such that
//
//    u2.w'.u2         u1.w'.u2         u1.w'.u1
// ------------- = - ------------- = -------------
// s1.s1.v1.w.v1     s1.s2.v1.w.v2   s2.s2.v2.w.v2
//
// w  = diag(a1.a1, a1.a1, 1)
// w' = diag(a2.a2, a2.a2, 1)
//
// Refer to equations on p.472 of Multi-view Geometry rev.2 
//
template <typename T> inline
bool EstimateFocalLengthByKruppa(const CVector<3,T>& u1, const CVector<3,T>& u2, 
								 const CVector<3,T>& v1, const CVector<3,T>& v2,
								 const T& s1, const T& s2, 
								 T& a1, T& a2)
{
#ifdef _DEBUG
	DUMP_VECMAT(u1);
	DUMP_VECMAT(u2);
	DUMP_VECMAT(v1);
	DUMP_VECMAT(v2);
	TRACE("s1=%f, s2=%f\n",s1,s2);
#endif

	T u1a = u1[0]*u1[0] + u1[1]*u1[1];
	T u1c = u1[2]*u1[2];
	T v1a = (v1[0]*v1[0] + v1[1]*v1[1]) * s1 * s1;
	T v1c = v1[2]*v1[2] * s1 * s1;

	T u2a = u2[0]*u2[0] + u2[1]*u2[1];
	T u2c = u2[2]*u2[2];
	T v2a = (v2[0]*v2[0] + v2[1]*v2[1]) * s2 * s2;
	T v2c = v2[2]*v2[2] * s2 * s2;

	T u12a = u1[0]*u2[0] + u1[1]*u2[1];
	T u12c = u1[2]*u2[2];
	T v12a = (v1[0]*v2[0] + v1[1]*v2[1]) * s1 * s2;
	T v12c = v1[2]*v2[2] * s1 * s2;

	T p1 = u2a*v12a + v1a*u12a;
	T p2 = u2c*v12a + v1a*u12c;
	T p3 = u2a*v12c + v1c*u12a;
	T p4 = u2c*v12c + v1c*u12c;

	T q1 = u1a*v12a + v2a*u12a;
	T q2 = u1c*v12a + v2a*u12c;
	T q3 = u1a*v12c + v2c*u12a;
	T q4 = u1c*v12c + v2c*u12c;
	
	T a = p2*q1 - p1*q2;
	T b = (p2*q3+p4*q1) - (p1*q4+p3*q2);
	T c = p4*q3 - p3*q4;
	T d = b*b-4*a*c; // descriminant

#ifdef _DEBUG
	TRACE("quadratic equation: %e * x2 + %e * x + %e = 0\n",a,b,c);
#endif

	TRACE("determinant = %e\n", d);

	if (a == 0) 
	{
		TRACE("linear equation: a = %f\n", (float)a);
		return false;
	}

	T scale = std::max(abs(b),abs(c));
	if (AlmostEqual(a/scale, (T)0)) 
	{
		TRACE("warning: possibly ill-conditioned\n");
	}

	if (d < 0) 
	{
		TRACE("error: d = %f \n", (float)d);
		return false;
	}

	T A1 = (-b+sqrt(d))/(2*a);
	T A2 = (-b-sqrt(d))/(2*a);

	if (A1 > 0 && !AlmostEqual(A1,(T)1))
	{
		T A3 = -(p2*A1+p4)/(p1*A1+p3);
		if (A3 > 0 && !AlmostEqual(A3, (T)1))
		{
			a1 = sqrt(A1);
			a2 = sqrt(A3);

			// check ambiguity
			if (A2 > 0 && !AlmostEqual(A2, (T)1))
			{
				T A4 = -(p2*A2+p4)/(p1*A2+p3);
				if (A4 > 0 && !AlmostEqual(A4, (T)1))
					TRACE("warning: solution ambiguous\n");
			}
		}

		else if (A2 > 0 && !AlmostEqual(A2, (T)1))
		{
			T A4 = -(p2*A2+p4)/(p1*A2+p3);
			if (A4 > 0 && !AlmostEqual(A4, (T)1))
			{
				a1 = sqrt(A2);
				a2 = sqrt(A4);
			}
			else
			{
				TRACE("error: no possible solution\n");
				return false;
			}
		}
		else
		{
			TRACE("error: no possible solution\n");
			return false;
		}
	}
	else // A1 is not the solution
	{
		if (A2 > 0 && !AlmostEqual(A2, (T)1))
		{
			T A4 = -(p2*A2+p4)/(p1*A2+p3);
			if (A4 > 0 && !AlmostEqual(A4, (T)1))
			{
				a1 = sqrt(A2);
				a2 = sqrt(A4);
			}
			else
			{
				TRACE("error: no possible solution\n");
				return false;
			}
		}
		else
		{
			TRACE("error: no possible solution\n");
				return false;
		}
	}

	return true;
}

// generate essential matrix from intrinsics and fundamental matrix
// such that: E = K1t * F * K2
template <typename T> inline
void EstimateEssentialMatrix(const CMatrix<3,3,T>& K1, const CMatrix<3,3,T>& matF, const CMatrix<3,3,T>& K2, CMatrix<3,3,T>& matE)
{
	// compute essential matrix
	CDynamicMatrix<T> matE2 ( transpose_of(K1) * matF * K2);

	// enforcing identical singular value constraint 
	CDynamicMatrix<T> matU, matVt;
	CDynamicVector<T> vecW;
	SingularValueDecomposition(matE2, matU, vecW, matVt);

	// apply rank constraint
	CDynamicMatrix<T> matW = make_diagonal_matrix(1,1,1);//CDynamicMatrix<T>::GetIdentity(3,3);
	matW(2,2) = 0;
	matE.Initialize( (matU * matW * matVt).ptr() );
}

// decompose essential matrix into relative pose by singular value decomposition
template <typename T> inline
void EstimateRelativePoseByEssentialMatrix(const CMatrix<3,3,T>& matE, CMatrix<3,3,T>& matR, CVector<3,T>& vecT)
{
	// singular value decomposition
	CMatrix<3,3,T> matU, matVt;
	CVector<3,T> vecW;
	SingularValueDecomposition(matE, true, true, matU, vecW, matVt);

	// (candidate) translation
	CVector<3,T> u3 = make_vector(matU(0,2),matU(1,2),matU(2,2));
	CVector<3,T> translation[] = { u3, -u3 };

	// (candidate) rotation
	double el[] = {
		0,-1,0,
		1, 0,0,
		0, 0,1
	};
	CMatrix<3,3,T> matW(el);
	CMatrix<3,3,T> rotation[] = 
	{
		matU * matW * matVt,
		matU * transpose_of(matW) * matVt
	};

	// check that the matrix is rotation, not flip
	CVector<3,T> r1 = make_vector(rotation[0](0,0),rotation[0](1,0),rotation[0](2,0));
	CVector<3,T> r2 = make_vector(rotation[0](0,1),rotation[0](1,1),rotation[0](2,1));
	CVector<3,T> r3 = make_vector(rotation[0](0,2),rotation[0](1,2),rotation[0](2,2));
	if (dot(cross(r1,r2),r3)<0)
	{
		rotation[0] *= -1;
		rotation[1] *= -1;
	}

	// initialize stereo pair
	CVector<3,T> ref = make_vector(0,0,1); // a 3D point in front of camera
	CVector<3,T> epi = matE * ref; // corresponding epipolar line in projector view
	std::vector<CVector<2,T> > pos2d(2);
	pos2d[0]=make_vector<T>(0,-epi[2]/epi[1]); // the point on the line and at image center
	pos2d[1]=make_vector<T>(0,0);
	std::vector<CMatrix<3,4,T> > proj(2);
	proj[1].Fill(0);// = CMatrix<3,4,T>::GetZero();
	proj[1](0,0)=proj[1](1,1)=proj[1](2,2)=1;//

	// choose a valid solition out of 4 candidates
	bool found = false;
	for (int rot=0; rot<2; rot++)
	{
		for (int tra=0; tra<2; tra++)
		{
			// set projection of a projector
			for (int r=0; r<3; r++)
			{
				for (int c=0; c<3; c++)
					proj[0](r,c) = rotation[rot](r,c);
				proj[0](r,3) = translation[tra][r];
			}

			CVector<3,T> p3d;
			SolveStereo(pos2d,proj,p3d);

			if (p3d[2]>0) // in front of camera
			{
				p3d = proj[0] * GetHomogeneousVector(p3d);
				if (p3d[2]>0) // in front of projector
				{
					if (found)
						TRACE("error: multiple solutions found.\n");
					matR=rotation[rot]; 
					vecT=translation[tra];
					found = true;
				}
			}
		}
	}
	DUMP_VECMAT(matR);
	DUMP_VECMAT(vecT);
}

}
