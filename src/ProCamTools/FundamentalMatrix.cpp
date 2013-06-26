//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: FundamentalMatrix.h 4414 2011-03-10 15:06:35Z shun $
//

#pragma once

#include <cmath>
#include <stdexcept>
#include <levmar.h>

#include "stdafx.h"
#include "levmar_util.h"

#define ENABLE_SAMPSON_APPROXIMATION

#include "MathBaseUtil.h"
#include "DLT.h"
#include "LeastSquare.h"
#include "ransac.h"
#ifndef ENABLE_SAMPSON_APPROXIMATION
#include "sba.h"
#include "sba_util.h"
#endif

#include "FundamentalMatrix.h"


namespace slib {
namespace fmatrix {

	std::string format_str(const char *fmt, ...)
	{
		va_list param;
		va_start(param, fmt);
		char message[1024];
		vsnprintf(message, 1023, fmt, param);
		message[1023] = 0;
		return std::string(message);
	}


//////////////////////////////////////////////////////////////////////
// fundamental matrix 
//////////////////////////////////////////////////////////////////////

//
// algebraic minimization
//

namespace {

// generate the coefficient matrix for linear solution 
void generate_kronecker_matrix(
	const std::vector<CVector<2,double> >& tp1, // normalized
	const std::vector<CVector<2,double> >& tp2, // normalized
	CDynamicMatrix<double>& matA // coefficient matrix for further nonlinear optimization
	) 
{
	matA.Resize(tp1.size(), 9);
	for (int i=0; i<tp1.size(); i++)
	{
		matA(i, 0) = tp1[i][0] * tp2[i][0];
		matA(i, 1) = tp1[i][0] * tp2[i][1];
		matA(i, 2) = tp1[i][0];
		matA(i, 3) = tp1[i][1] * tp2[i][0];
		matA(i, 4) = tp1[i][1] * tp2[i][1];
		matA(i, 5) = tp1[i][1];
		matA(i, 6) =             tp2[i][0];
		matA(i, 7) =             tp2[i][1];
		matA(i, 8) = 1;
	}
}

// minimize the algebraic error |x'Fx| in least squares manner 
void solve_algebraic_closed_form(
	const CDynamicMatrix<double>& matA, // coefficient matrix for further nonlinear optimization
	CMatrix<3,3,double>& fundamental // out
	)
{
	// algebraic optimization
	CDynamicVector<double> f;
	FindRightNullVector(matA, f);
	fundamental.Initialize(f.ptr());
	fundamental = transpose_of(fundamental);

	// enforce rank-2 constraint
	// by replacing the least singular value with 0
	CMatrix<3,3,double> matU, matVt;
	CVector<3,double> vecW;
	SingularValueDecomposition(fundamental, true, true, matU, vecW, matVt);
	vecW[2] = 0;
	fundamental = matU * make_diagonal_matrix(vecW) * matVt;
}

} // nameless namespace

// estimate the fundamental matrix F such that argmin|p1.F.p2| 
void EstimateFundamentalMatrixAlgebraic(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CMatrix<3,3,double>& fundamental)
{
	int npoints = p1.size();
	if (p1.size() != p2.size() || npoints < 8)
		throw std::runtime_error(format_str("input errror in %s", __FUNCTION__));

	// normalize
	std::vector<CVector<2,double> > tp1, tp2;
	double scale1, scale2;
	CVector<2,double> center1, center2;
	dlt::normalize(p1, tp1, scale1, center1);
	dlt::normalize(p2, tp2, scale2, center2);

	// algebraic minimization 
	CDynamicMatrix<double> matA;
	generate_kronecker_matrix(tp1, tp2, matA);
	solve_algebraic_closed_form(matA, fundamental);

	// cancel normalization
	CMatrix<3,3,double> matS1 = make_matrix<double>(
		scale1, 0, 0,
		0, scale1, 0,
		-scale1*center1[0], -scale1*center1[1], 1);
	CMatrix<3,3,double> matS2 = make_matrix<double>(
		scale2, 0, -scale2*center2[0],
		0, scale2, -scale2*center2[1],
		0, 0, 1);
	fundamental = matS1 * fundamental * matS2;
}

//
// geometric minimization
//

namespace {

#ifdef ENABLE_SAMPSON_APPROXIMATION

struct fundamental_geometric_data_t
{
	const std::vector<CVector<2,double> >& p1, &p2;
};

// sampson approximation
void fundamental_sampson_error(double *p, double *hx, int m, int n, void *adata)
{
	CMatrix<3,3,double> fundamental = GetSkewSymmetric(CMatrix<3,1,double>(p)) * CMatrix<3,3,double>(p+3);

	fundamental_geometric_data_t *data = (fundamental_geometric_data_t *)adata;
	for (int i=0; i<n; i++)
	{
		CVector<3,double> p1F = transpose_of(fundamental) * GetHomogeneousVector(data->p1[i]);
		CVector<3,double> Fp2 = fundamental * GetHomogeneousVector(data->p2[i]);
		double d = p1F[0]*p1F[0] + p1F[1]*p1F[1] + Fp2[0]*Fp2[0] + Fp2[1]*Fp2[1];
		hx[i] = dot(GetHomogeneousVector(data->p1[i]), Fp2) / sqrt(d);
	}
}

// TODO: to implement abgebraic derivative

#else
// bundle adjustment

void pairwise_stereo(
	const CVector<2,double>& x1,
	const CVector<2,double>& x2,
	const CMatrix<3,4,double>& matP1, 
	const CMatrix<3,4,double>& matP2, 
	CVector<3,double>& X)
{
	std::vector<CVector<2,double> > p2d(2);
	p2d[0] = x1;
	p2d[1] = x2;

	std::vector<CMatrix<3,4,double> > proj(2);
	proj[0] = matP1;
	proj[1] = matP2;

	SolveStereo(p2d, proj, X);
}

void fundamental_ba_error(int j, int i, double *aj, double *bi, double *xij, void *adata)
{
	CMatrix<3,4,double> mat(aj);
	CVector<4,double> X = make_vector<double>(bi[0],bi[1],bi[2],1);
	CVector<3,double> x = mat * X;
	xij[0] = x[0] / x[2];
	xij[1] = x[1] / x[2];
}

void fundamental_ba_jacobian(int j, int i, double *aj, double *bi, double *Aij, double *Bij, void *adata)
{
	CMatrix<3,4,double> mat(aj);
	CVector<4,double> X = make_vector(bi[0],bi[1],bi[2],1.0);
	CVector<3,double> x = mat * X;

	// x coordinate (row1)
	Aij[0]  = X[0]/x[2];
	Aij[1]  = 0;
	Aij[2]  =-x[0]*X[0]/(x[2]*x[2]);

	Aij[3]  = X[1]/x[2];
	Aij[4]  = 0;
	Aij[5]  =-x[0]*X[1]/(x[2]*x[2]);

	Aij[6]  = X[2]/x[2];
	Aij[7]  = 0;
	Aij[8]  =-x[0]*X[2]/(x[2]*x[2]);

	Aij[9]  = X[3]/x[2];
	Aij[10] = 0;
	Aij[11] =-x[0]*X[3]/(x[2]*x[2]);

	// y coordinate (row2)
	Aij[12] = 0;
	Aij[13] = X[0]/x[2];
	Aij[14] =-x[1]*X[0]/(x[2]*x[2]);

	Aij[15] = 0;
	Aij[16] = X[1]/x[2];
	Aij[17] =-x[1]*X[1]/(x[2]*x[2]);

	Aij[18] = 0;
	Aij[19] = X[2]/x[2];
	Aij[20] =-x[1]*X[2]/(x[2]*x[2]);

	Aij[21] = 0;
	Aij[22] = X[3]/x[2];
	Aij[23] =-x[1]*X[3]/(x[2]*x[2]);

	CVector<4,double> c1 = make_vector(mat(0,0),mat(0,1),mat(0,2),mat(0,3)); 
	CVector<4,double> c2 = make_vector(mat(1,0),mat(1,1),mat(1,2),mat(1,3)); 
	CVector<4,double> c3 = make_vector(mat(2,0),mat(2,1),mat(2,2),mat(2,3)); 
	CVector<4,double> dxdX = (x[2]*c1 - x[0]*c3) / (x[2]*x[2]);
	CVector<4,double> dydX = (x[2]*c2 - x[1]*c3) / (x[2]*x[2]);

	// x coordinate
	Bij[0] = dxdX[0];
	Bij[1] = dxdX[1];
	Bij[2] = dxdX[2];

	// y coordinate
	Bij[3] = dydX[0];
	Bij[4] = dydX[1];
	Bij[5] = dydX[2];
}
#endif
} // nameless namespace

// minimize the geometric error 
void EstimateFundamentalMatrixGeometric(const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CMatrix<3,3,double>& fundamental // in & out
	)
{
	int npoints = p1.size();

#ifdef ENABLE_SAMPSON_APPROXIMATION
	// minimize the sampson error 
	if (p1.size() != p2.size() || npoints < 12)
		throw std::runtime_error(format_str("input errror in [%s]", __FUNCTION__));

	int m = 12; 
	int n = npoints; 

	// F = [e1]x.M
	// M = [e1]x.F
	double *p = new double [m];
	CVector<3,double> e1;
	FindRightNullVector(transpose_of(fundamental), e1);
	memcpy(p, e1.ptr(), sizeof(e1));
	CMatrix<3,3,double> M = GetSkewSymmetric(e1) * fundamental;
	memcpy(p+3, M.ptr(), sizeof(M));

	// initial mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2
	double opts[LM_OPTS_SZ] = { LM_INIT_MU, 0, 0, 0, -LM_DIFF_DELTA};
//	double opts[LM_OPTS_SZ] = { LM_INIT_MU, LM_STOP_THRESH, LM_STOP_THRESH, LM_STOP_THRESH, LM_DIFF_DELTA};
	double info[LM_INFO_SZ];
	int itmax = 1000;

	const fundamental_geometric_data_t adata = { p1, p2, };
	dlevmar_dif(fundamental_sampson_error, 
		p, 0, m, n,
		itmax, opts, info, 
		0, 0, (void *)&adata);
	print_levmar_info(info);

	fundamental = GetSkewSymmetric(CMatrix<3,1,double>(p)) * CMatrix<3,3,double>(p+3);
	delete [] p;
#else
	// minimize the geometric error |x-PX|+|x'-P'X| by bundle adjustment
	// See: Multiple View Geometry 2nd Edition p.285
	if (p1.size() != p2.size() || npoints < 24) // TODO: should be 12
		throw std::runtime_error(format_str("input errror in %s", __FUNCTION__));

	// P1 = [ I | 0 ]
	CMatrix<3,4,double> matP1 = CMatrix<3,4,double>::GetIdentity();

	// P2 = [ [e2]x.Ft | e2 ]
	CVector<3,double> e2;
	FindRightNullVector(fundamental, e2);
	CMatrix<3,3,double> e2xFt = GetSkewSymmetric(e2) * transpose_of(fundamental);
	CMatrix<3,4,double> matP2 = e2xFt.AppendCols(e2);

	// number of parameters
	int ncameras = 2; // projector,camera 
	int nparams_camera = 12; 
	int nparams_point = 3; // X,Y,Z

	// visibility
	char *vmask = new char [ncameras * npoints];
	memset(vmask, 1, ncameras * npoints);

	// parameters
	double *p = new double [ncameras*nparams_camera + npoints*nparams_point];
	memcpy(p, matP1.ptr(), sizeof(matP1)); // P1
	memcpy(p+nparams_camera, matP2.ptr(), sizeof(matP2)); // P2

	// observation
	int nparams_measure = 2;
	double *measurement = new double [npoints*ncameras*nparams_measure];

	for (int i=0; i<npoints; i++)
	{
		// 2D coordinates
		measurement[4*i+0] = p1[i][0]; // p1
		measurement[4*i+1] = p1[i][1];
		measurement[4*i+2] = p2[i][0]; // p2
		measurement[4*i+3] = p2[i][1];

		// 3D coordinates
		CVector<3,double> p3d;
		pairwise_stereo(p1[i], p2[i], matP1, matP2, p3d);
		memcpy(p + ncameras*nparams_camera + nparams_point*i, p3d.ptr(), sizeof (p3d));
	}

	// the scale factor for initial \mu,
	// stopping thresholds for 
	// ||J^T e||_inf, ||dp||_2, ||e||_2 and (||e||_2-||e_new||_2)/||e||_2
	//	const double opts[SBA_OPTSSZ] = { SBA_INIT_MU, SBA_STOP_THRESH, SBA_STOP_THRESH, SBA_STOP_THRESH, SBA_STOP_THRESH};
	double opts[SBA_OPTSSZ] = { SBA_INIT_MU, 0, 0, 0, 0};
	double info[SBA_INFOSZ];
	int verbose = 1;
	int itmax = 1000;

#if 0
	// test gradient
	// CHKDER does not perform reliably if cancellation or rounding
	// errors cause a severe loss of significance in the evaluation of
	// a function.  Therefore, none of the components of X should be
	// unusually small (in particular, zero) or any other value which
	// may cause loss of significance.	
	itmax=0;
	for (int i=0; i<npoints*nparams_point; i++)
		p[ncameras*nparams_camera+i]+=1-(float)rand()/RAND_MAX;
#endif
	sba_motstr_levmar(npoints, 0, ncameras, 1, vmask, p, 
		nparams_camera, nparams_point, measurement, 0, nparams_measure, 
		fundamental_ba_error, fundamental_ba_jacobian, 0, 
		itmax, verbose, opts, info);
	print_sba_info(info);

	// recover P2
	CMatrix<3,3,double> matM;
	matM.Initialize(p+12);
	CVector<3,double> vecT;
	vecT.Initialize(p+21);
	fundamental = transpose_of(matM) * GetSkewSymmetric(vecT);

	delete [] measurement;
	delete [] vmask;
	delete [] p;
#endif
}

//////////////////////////////////////////////////////////////////////
// radial fundamental matrix 
//////////////////////////////////////////////////////////////////////

//
// algebraic minimization
//

namespace {

void generate_kronecker_matrix_lift(
	const std::vector<CVector<3,double> >& lifted1,
	const std::vector<CVector<3,double> >& lifted2,
	CDynamicMatrix<double>& matA)
{
	int npoints = lifted1.size();
	matA.Resize(npoints, 16);
	for (int r=0; r<npoints; r++)
	{
		for (int c1=0; c1<4; c1++)
		{
			double l1 = (c1<3) ? lifted1[r][c1] : 1;
			for (int c2=0; c2<3; c2++)
				matA(r,4*c1+c2) = l1*lifted2[r][c2];
			matA(r,4*c1+3) = l1;
		}
	}
}

void lift_coorinates(
	const std::vector<CVector<2,double> >& pos, 
	const CVector<2,double>& cod,
	std::vector<CVector<3,double> >& lifted)
{
	const int npoints = pos.size();
	lifted.resize(npoints);
	for (int i=0; i<npoints; i++) 
	{
		const CVector<2,double> p = pos[i] - cod;
		lifted[i] = make_vector(p[0]*p[0]+p[1]*p[1], p[0], p[1]);
	}
}

} // nameless namespace

void EstimateRadialFundamentalMatrixAlgebraic(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	const CVector<2,double>& cod1,
	const CVector<2,double>& cod2,
	double& xi1,
	double& xi2,
	CMatrix<3,3,double>& fundamental) 
{
	int npoints = p1.size();
	if (p1.size() != p2.size() || npoints < 15)
		throw std::runtime_error(format_str("input errror in %s", __FUNCTION__));

	// lifted coordinates
	std::vector<CVector<3,double> > lifted1, lifted2;
	lift_coorinates(p1, cod1, lifted1);
	lift_coorinates(p2, cod2, lifted2);

	// normalize for DLT
	std::vector<CVector<3,double> > normalized_lifted1, normalized_lifted2;
	CMatrix<4,4,double> denormalize_trans1,denormalize_trans2;
	dlt::normalize_anisotropic(lifted1, normalized_lifted1, denormalize_trans1);
	dlt::normalize_anisotropic(lifted2, normalized_lifted2, denormalize_trans2);

	// generate coefficient matrix
	CDynamicMatrix<double> matLiftedKronecker;
	generate_kronecker_matrix_lift(normalized_lifted1, normalized_lifted2, matLiftedKronecker);

	// solve least square
	CDynamicVector<double> fdot;
	FindRightNullVector(matLiftedKronecker, fdot);
	CMatrix<4,4,double> radial(fdot.ptr());
	radial = transpose_of(radial);

	// enforce rank constraint
	CMatrix<4,4,double> matU, matVt;
	CVector<4,double> singular_values;
	SingularValueDecomposition(radial, true, true, matU, singular_values, matVt);

	// left distortion
	CMatrix<4,4,double> invt1 = inverse_of(denormalize_trans1);
	CVector<4,double> u1 = invt1 * matU.GetSubMatrix<4,1>(0,2);
	CVector<4,double> u2 = invt1 * matU.GetSubMatrix<4,1>(0,3);
	CVector<4,double> dir1 = u1 / u1[3] - u2 / u2[3];
	xi1 = u1[3] / ((dir1[0]*dir1[1]*u1[1]+dir1[0]*dir1[2]*u1[2])/(dir1[1]*dir1[1]+dir1[2]*dir1[2])-u1[0]);

	// right distortion
	CMatrix<4,4,double> invt2 = inverse_of(denormalize_trans1);
	CVector<4,double> v1 = invt2 * transpose_of(matVt.GetSubMatrix<1,4>(2,0)  );
	CVector<4,double> v2 = invt2 * transpose_of(matVt.GetSubMatrix<1,4>(3,0)  );
	CVector<4,double> dir2 = v1 / v1[3] - v2 / v2[3];
	xi2 = v1[3] / ((dir2[0]*dir2[1]*v1[1]+dir2[0]*dir2[2]*v1[2])/(dir2[1]*dir2[1]+dir2[2]*dir2[2])-v1[0]);

	// undistortion
	std::vector<CVector<2,double> > undist1(npoints), undist2(npoints);
	for (int i=0; i<npoints; i++) 
	{
		CancelRadialDistortion(xi1,cod1,p1[i],undist1[i]);
		CancelRadialDistortion(xi2,cod2,p2[i],undist2[i]);
	}

	// estimate fundamental matrix 
	EstimateFundamentalMatrixAlgebraic(undist1, undist2, fundamental);
}

//
// geometric minimization
//

namespace {

#ifdef ENABLE_SAMPSON_APPROXIMATION
struct radial_geometric_data_t
{
	const std::vector<CVector<2,double> > &p1,&p2;
	const CVector<2,double> &cod1, &cod2;
};

// sampson approximation
void radial_sampson_error(double *parameter, double *hx, int m, int n, void *adata)
{
	CMatrix<3,3,double> fundamental(parameter);
	double xi1 = parameter[9];
	double xi2 = parameter[10];
	radial_geometric_data_t *data = (radial_geometric_data_t *)adata;
	for (int i=0; i<n; i++)
	{
		CVector<2,double> u1,u2;
		CancelRadialDistortion(xi1,data->cod1,data->p1[i],u1);
		CancelRadialDistortion(xi2,data->cod2,data->p2[i],u2);
		CVector<3,double> p1F = transpose_of(fundamental) * GetHomogeneousVector(u1);
		CVector<3,double> Fp2 = fundamental * GetHomogeneousVector(u2);
		double d = p1F[0]*p1F[0] + p1F[1]*p1F[1] + Fp2[0]*Fp2[0] + Fp2[1]*Fp2[1];
		hx[i] = dot(GetHomogeneousVector(u1), Fp2) / sqrt(d);
	}
}

// TODO: to implement abgebraic derivative

#else
// bundle adjustment

void radial_ba_error(int j, int i, double *aj, double *bi, double *xij, void *adata)
{
	fundamental_ba_error(j, i, aj, bi, xij, 0);

	// apply distortion
	const double& xi = aj[12];
	const CVector<2,double>& c = ((const CVector<2,double> *)adata)[j];

	CVector<2,double> dist;
	ApplyRadialDistortion(xi, c, make_vector(xij[0],xij[1]), dist);
	xij[0] = dist[0];
	xij[1] = dist[1];
}

void radial_ba_jacobian(int j, int i, double *aj, double *bi, double *Aij, double *Bij, void *adata)
{
	fundamental_ba_jacobian(j, i, aj, bi, Aij, Bij, 0);

	CMatrix<3,4,double> mat(aj);
	const double& xi = aj[12];
	const CVector<2,double>& c = ((const CVector<2,double> *)adata)[j];
	CVector<4,double> X = make_vector(bi[0],bi[1],bi[2],1.0); // 3D
	CVector<2,double> x = GetEuclideanVector(mat * X); // non-distorted 2D

	double r = dot(x-c,x-c);
	double R = (1 - sqrt(1 - 4*xi*r)) / (2*xi*r);

	double dRdr  = 1/(r*sqrt(1-4*xi*r)) - (1-sqrt(1-4*xi*r))/(2*xi*r*r);
	double dRdu  = dRdr*2*(x[0]-c[0]);
	double dRdv  = dRdr*2*(x[1]-c[1]);
	double dRdxi = 1/(xi*sqrt(1-4*xi*r)) - (1-sqrt(1-4*xi*r))/(2*xi*xi*r);

	double jac[] = {
		dRdu *(x[0]-c[0])+R, dRdu *(x[1]-c[1]), 
		dRdv *(x[0]-c[0]),   dRdv *(x[1]-c[1])+R, 
		dRdxi*(x[0]-c[0]),   dRdxi*(x[1]-c[1]),
	};
	CMatrix<2,3,double> jacobian(jac);
	CMatrix<3,13,double> ajac;
	for (int r=0;r<2;r++) {
		for (int c=0;c<12;c++) 
			ajac(r,c)=Aij[2*r+c];
		ajac(r,12)=0;
	}
	for (int c=0;c<12;c++) 
		ajac(2,c)=0;
	ajac(2,12)=1;
	CMatrix<2,13,double> aij=jacobian*ajac;
	memcpy(Aij, transpose_of(aij).ptr(), sizeof(aij));

	CMatrix<3,3,double> bjac;
	for (int r=0;r<2;r++)
		for (int c=0;c<3;c++)
			bjac(r,c)=Bij[2*r+c];
	for (int c=0;c<3;c++)
		bjac(2,c)=0;
	CMatrix<2,3,double> bij=jacobian*bjac;
	memcpy(Bij, transpose_of(bij).ptr(), sizeof(bij));
}
#endif
} // nameless namespace

void EstimateRadialFundamentalMatrixGeometric(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	const CVector<2,double>& cod1,
	const CVector<2,double>& cod2,
	double& xi1, // [in/out] 
	double& xi2, // [in/out] 
	CMatrix<3,3,double>& fundamental // [in/out] 
	)
{
	int npoints = p1.size();

#ifdef ENABLE_SAMPSON_APPROXIMATION
	// sampson
	if (p1.size() != p2.size() || npoints < 11) // TODO: correct?
		throw std::runtime_error(format_str("input errror in %s", __FUNCTION__));

	int m = 9+2; // fmatrix(9), xi(2)

	double *p = new double [m];
	memcpy(p, fundamental.ptr(), sizeof(fundamental));
	p[9]=xi1;
	p[10]=xi2;

	int n = npoints; // correspondence

	// initial mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2
	double opts[LM_OPTS_SZ] = { LM_INIT_MU, 0, 0, 0, -LM_DIFF_DELTA};
//	double opts[LM_OPTS_SZ] = { LM_INIT_MU, LM_STOP_THRESH, LM_STOP_THRESH, LM_STOP_THRESH, LM_DIFF_DELTA};
	double info[LM_INFO_SZ];
	int itmax = 1000;

	const radial_geometric_data_t adata = { p1, p2, cod1, cod2 };
	dlevmar_dif(radial_sampson_error, 
		p, 0, m, n,
		itmax, opts, info, 
		0, 0, (void *)&adata);
	print_levmar_info(info);

	fundamental.Initialize(p);
	xi1=p[9];
	xi2=p[10];
	delete [] p;
#else
	// reprojection
	if (p1.size() != p2.size() || npoints < 26) // TODO: should be 13?
		throw std::runtime_error(format_str("input errror in %s", __FUNCTION__));

	// P1 = [ I | 0 ]
	CMatrix<3,4,double> matP1 = CMatrix<3,4,double>::GetIdentity();

	// P2 = [ [e2]x.Ft | e2 ]
	CVector<3,double> e2;
	FindRightNullVector(fundamental, e2);
	CMatrix<3,3,double> e2xFt = GetSkewSymmetric(e2) * transpose_of(fundamental);
	CMatrix<3,4,double> matP2 = e2xFt.AppendCols(e2);

	// number of parameters
	int ncameras = 2; // projector,camera
	int nparams_camera = 13; // 12+distortion
	int nparams_point = 3; // X,Y,Z

	// visibility
	char *vmask = new char [ncameras * npoints];
	memset(vmask, 1, ncameras * npoints);

	// parameters
	double *p = new double [ncameras*nparams_camera + npoints*nparams_point];
	memcpy(p, matP1.ptr(), sizeof(matP1)); // P1
	p[12] = xi1;
	memcpy(p+nparams_camera, matP2.ptr(), sizeof(matP2)); // P2
	p[nparams_camera+12] = xi2;

	// observation
	int nparams_measure = 2; // x,y
	double *measurement = new double [npoints*ncameras*nparams_measure];

	for (int i=0; i<npoints; i++)
	{
		// 2D coordinates
		measurement[4*i+0] = p1[i][0]; // p1
		measurement[4*i+1] = p1[i][1];
		measurement[4*i+2] = p2[i][0]; // p2
		measurement[4*i+3] = p2[i][1];

		// 3D coordinates
		CVector<3,double> p3d;
		CVector<2,double> undist1, undist2;
		CancelRadialDistortion(xi1, cod1, p1[i], undist1);
		CancelRadialDistortion(xi2, cod2, p2[i], undist2);
		pairwise_stereo(undist1, undist2, matP1, matP2, p3d);
		memcpy(p + ncameras*nparams_camera + nparams_point*i, p3d.ptr(), sizeof (p3d));
	}

	// additional data
	const CVector<2,double> center[] = { cod1, cod2 };

	// the scale factor for initial \mu,
	// stopping thresholds for 
	// ||J^T e||_inf, ||dp||_2, ||e||_2 and (||e||_2-||e_new||_2)/||e||_2
	//	const double opts[SBA_OPTSSZ] = { SBA_INIT_MU, SBA_STOP_THRESH, SBA_STOP_THRESH, SBA_STOP_THRESH, SBA_STOP_THRESH};
	double opts[SBA_OPTSSZ] = { SBA_INIT_MU, 0, 0, 0, 0};
	double info[SBA_INFOSZ];
	int verbose = 1;
	int itmax = 1000;

#if 0
	// test gradient
	// CHKDER does not perform reliably if cancellation or rounding
	// errors cause a severe loss of significance in the evaluation of
	// a function.  Therefore, none of the components of X should be
	// unusually small (in particular, zero) or any other value which
	// may cause loss of significance.	
	itmax=0;
	for (int i=0; i<npoints*nparams_point; i++)
		p[ncameras*nparams_camera+i]+=1-(float)rand()/RAND_MAX;
#endif
	sba_motstr_levmar(npoints, 0, ncameras, 0, vmask, p, 
		nparams_camera, nparams_point, measurement, 0, nparams_measure,
		radial_ba_error, radial_ba_jacobian, (void *)center, 
		itmax, verbose, opts, info);
	print_sba_info(info);

	// recover P2
	matP1.Initialize(p);
	double row4e[] = { 0,0,0,1 };
	CMatrix<4,4,double> invP1 = inverse_of(matP1.AppendRows(CMatrix<1,4,double>(row4e)));
	matP2 = CMatrix<3,4,double>(p+nparams_camera) * invP1;

	// recover fundamental matrix
	CMatrix<3,3,double> matM(matP2.ptr());
	CVector<3,double> vecT(matP2.ptr()+9);
	fundamental = transpose_of(matM) * GetSkewSymmetric(vecT);
	xi1 = p[12];
	xi2 = p[nparams_camera+12];

	delete [] measurement;
	delete [] vmask;
	delete [] p;
#endif
}

//
// a-priori minimization
// See: "Reconstruction from two views using approximate calibration"
//

// estimate intrinsic parameters of projector and camera using fundamental matrix
// assuming: p1.fundamental.p2 = 0 
void EstimateFocalLengthBougnoux(
	const CMatrix<3,3,double>& fundamental, 
	const CVector<3,double>& pp1, const CVector<3,double>& pp2,
	double& f1squared, double& f2squared)
{
	// epipoles
	CVector<3,double> e1, e2;
	FindRightNullVector(transpose_of(fundamental), e1);
	FindRightNullVector(fundamental, e2);

	// diag(1,1,0);
	CMatrix<3,3,double> matI = make_diagonal_matrix(make_vector<double>(1,1,0));

	f1squared = 
		-dot(pp2, GetSkewSymmetric(e2) * matI * transpose_of(fundamental) * pp1 * transpose_of(pp1) * fundamental * pp2) / 
		dot(pp2, GetSkewSymmetric(e2) * matI * transpose_of(fundamental) * matI * fundamental * pp2);

	f2squared = 
		-dot(pp1, GetSkewSymmetric(e1) * matI * fundamental * pp2 * transpose_of(pp2) * transpose_of(fundamental) * pp1) / 
		dot(pp1, GetSkewSymmetric(e1) * matI * fundamental * matI * transpose_of(fundamental) * pp1);
}

namespace {

// cod
double wp1=0.01;
double wp2=0.01;

// f_min
double wz1=0.01;
double wz2=0.01;
double focus_min=100;

// sampson approximation
void apriori_sampson_error(double *parameter, double *hx, int m, int n, void *adata)
{
	CMatrix<3,3,double> fundamental(parameter);
	CVector<3,double> pp1 = make_vector(parameter[9],parameter[10],1.0);
	CVector<3,double> pp2 = make_vector(parameter[11],parameter[12],1.0);
	double xi1 = parameter[13];
	double xi2 = parameter[14];

	const struct radial_geometric_data_t* data = (const struct radial_geometric_data_t *)adata;

	// C_R
	int npoints = n-8;
	for (int i=0; i<npoints; i++, hx++) 
	{
		CVector<2,double> u1,u2;
		CancelRadialDistortion(xi1,make_vector(parameter[9],parameter[10]),data->p1[i],u1);
		CancelRadialDistortion(xi2,make_vector(parameter[11],parameter[12]),data->p2[i],u2);

		CVector<3,double> p1 = GetHomogeneousVector(u1);
		CVector<3,double> p2 = GetHomogeneousVector(u2);
		CVector<3,double> Fp1 = transpose_of(fundamental) * p1;
		CVector<3,double> Fp2 = fundamental * p2;

		double product = dot(p1, Fp2);
		double normal = Fp1[0]*Fp1[0] + Fp1[1]*Fp1[1] + Fp2[0]*Fp2[0] + Fp2[1]*Fp2[1];
		double sq_normal = sqrtf(normal);

		*hx = product/sq_normal;
	}

	// C_f
	double f1squared,f2squared;
	EstimateFocalLengthBougnoux(fundamental,pp1,pp2,f1squared,f2squared);
	if (focus_min*focus_min-f1squared>0)
		hx[0]=wz1*(focus_min*focus_min-f1squared);
	else
		hx[0]=0;
	if (focus_min*focus_min-f2squared>0)
		hx[1]=wz2*(focus_min*focus_min-f2squared);
	else
		hx[1]=0;

	// C_p
	hx[2]=wp1*(pp1[0]-data->cod1[0]);
	hx[3]=wp1*(pp1[1]-data->cod1[1]);
	hx[4]=wp1*(pp2[0]-data->cod2[0]);
	hx[5]=wp1*(pp2[1]-data->cod2[1]);

	// C_xi
	hx[6]=xi1;
	hx[7]=xi2;
}

// TODO: to implement abgebraic derivative

} // nameless namespace 

void EstimateRadialFundamentalMatrixApriori(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	 CVector<2,double>& cod1,// [in/out] 
	 CVector<2,double>& cod2,// [in/out] 
	double& xi1, // [in/out] 
	double& xi2, // [in/out] 
	CMatrix<3,3,double>& fundamental // [in/out] 
	)
{
	int npoints = p1.size();
	if (p1.size() != p2.size() || npoints < 15) // TODO: correct?
		throw std::runtime_error(format_str("input errror in %s", __FUNCTION__));

	// parameters
	int m = 9 + 6;// fmatrix, cod(4), xi(2)
	double *parameter = new double [m];
	memcpy(parameter, fundamental.ptr(), sizeof(fundamental)); 
	parameter[9]=cod1[0];
	parameter[10]=cod1[1];
	parameter[11]=cod2[0];
	parameter[12]=cod2[1];
	parameter[13]=xi1;
	parameter[14]=xi2;

	// observation
	int n = npoints + 8; // correspondence, f2(2), cod(4), xi(2)

	// initial mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2
	double opts[LM_OPTS_SZ] = { LM_INIT_MU, 0, 0, 0, -LM_DIFF_DELTA};
//	double opts[LM_OPTS_SZ] = { LM_INIT_MU, LM_STOP_THRESH, LM_STOP_THRESH, LM_STOP_THRESH, LM_DIFF_DELTA};
	double info[LM_INFO_SZ];
	int itmax = 1000;

	const radial_geometric_data_t adata = { p1, p2, cod1, cod2 };

	dlevmar_dif(apriori_sampson_error, 
		parameter, 0, m, n,
		itmax, opts, info, 
		0, 0, (void *)&adata);
	print_levmar_info(info);

	// recover fundamental matrix
	fundamental.Initialize(parameter);

	cod1 = make_vector(parameter[9],parameter[10]);
	cod2 = make_vector(parameter[11],parameter[12]);
	xi1 = parameter[13];
	xi2 = parameter[14];
	
	delete [] parameter;
}

//////////////////////////////////////////////////////////////////////
// RANSAC 
//////////////////////////////////////////////////////////////////////

namespace ransac {

// estimator_t: paramtype (*)(const std::vector<datatype>&, const odatatype&);
// evaluator_t: double (*)(const datatype&, const odatatype&, const paramtype&);
CMatrix<3,3,double> fundamental_estimator(const std::vector<std::pair<CVector<2,double>,CVector<2,double> > >& data)
{
	int ndata = data.size();
	std::vector<CVector<2,double> > p1(ndata),p2(ndata);
	for (int i=0; i<ndata; i++)
	{
		p1[i]=data[i].first;
		p2[i]=data[i].second;
	}
	CMatrix<3,3,double> fundamental;
	EstimateFundamentalMatrixAlgebraic(p1,p2,fundamental);
	return fundamental;
}

double fundamental_evaluator(const std::pair<CVector<2,double>,CVector<2,double> >& data, const CMatrix<3,3,double>& fundamental)
{
	CMatrix<3,3,double> D3 = make_diagonal_matrix(1,1,0);
	CVector<3,double> p1 = GetHomogeneousVector(data.first);
	CVector<3,double> p2 = GetHomogeneousVector(data.second);
	double d1 = std::abs(dot(p2, transpose_of(fundamental) * p1)) / GetNorm2(D3 * transpose_of(fundamental) * p1);
	double d2 = std::abs(dot(p1, fundamental * p2)) / GetNorm2(D3 * fundamental * p2);
	return (d1 + d2) / 2;
}

struct radial_param_t
{
	CMatrix<3,3,double> fundamental;
	double xi1,xi2;
	CVector<2,double> cod1,cod2;
};

struct radial_estimator
{
	radial_estimator(const CVector<2,double>& c1, const CVector<2,double>& c2) : cod1(c1), cod2(c2)
	{
	}
	
	radial_param_t operator()(const std::vector<std::pair<CVector<2,double>,CVector<2,double> > >& data) const
	{
		int ndata = data.size();
		std::vector<CVector<2,double> > p1(ndata),p2(ndata);
		for (int i=0; i<ndata; i++)
		{
			p1[i]=data[i].first;
			p2[i]=data[i].second;
		}
		radial_param_t param;

		// initial solution
		EstimateRadialFundamentalMatrixAlgebraic(p1,p2,cod1,cod2,param.xi1,param.xi2,param.fundamental);

		// final solution
		param.cod1=cod1;
		param.cod2=cod2;
		EstimateRadialFundamentalMatrixApriori(p1,p2,param.cod1,param.cod2,param.xi1,param.xi2,param.fundamental);
		return param;
	}

	const CVector<2,double> &cod1, &cod2;
};

struct radial_evaluator
{
	double operator ()(const std::pair<CVector<2,double>,CVector<2,double> >& data, const radial_param_t& param) const
	{
		CVector<2,double> u1,u2;
		CancelRadialDistortion(param.xi1,param.cod1,data.first, u1);
		CancelRadialDistortion(param.xi2,param.cod2,data.second,u2);

		CMatrix<3,3,double> D3 = make_diagonal_matrix(1,1,0);
		CVector<3,double> p1 = GetHomogeneousVector(u1);
		CVector<3,double> p2 = GetHomogeneousVector(u2);
		double d1 = std::abs(dot(p2, transpose_of(param.fundamental) * p1)) / GetNorm2(D3 * transpose_of(param.fundamental) * p1);
		double d2 = std::abs(dot(p1, param.fundamental * p2)) / GetNorm2(D3 * param.fundamental * p2);
		return (d1 + d2) / 2;
	}

};

} // namespace ransac

// using EstimateFundamentalMatrixAlgebraic()
void EstimateFundamentalMatrixRansac(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CMatrix<3,3,double>& fundamental)
{
	int npoints = p1.size();

	// ransac
	std::vector<std::pair<CVector<2,double>,CVector<2,double> > > data(npoints);
	for (int i=0; i<npoints; i++)
		data[i] = std::make_pair(p1[i],p2[i]);
	double maxerror = 1;
	Ransac(data, ransac::fundamental_estimator, ransac::fundamental_evaluator, 
		8, maxerror, 0.95, fundamental);

	// refine using only inliers
	std::vector<std::pair<CVector<2,double>,CVector<2,double> > > inliers;
	FindInliers(data, ransac::fundamental_evaluator, maxerror, fundamental, inliers);
	int nin = inliers.size();
	std::vector<CVector<2,double> > in1(nin), in2(nin);
	for (int i=0; i<nin; i++)
	{
		in1[i]=inliers[i].first;
		in2[i]=inliers[i].second;
	}
	EstimateFundamentalMatrixGeometric(in1,in2,fundamental);
}

// using EstimateRadialFundamentalMatrixApriori()
void EstimateRadialFundamentalMatrixRansac(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CVector<2,double>& cod1,
	CVector<2,double>& cod2,
	double& xi1, 
	double& xi2, 
	CMatrix<3,3,double>& fundamental)
{
	int npoints = p1.size();

	std::vector<std::pair<CVector<2,double>,CVector<2,double> > > data(npoints);
	for (int i=0; i<npoints; i++)
		data[i] = std::make_pair(p1[i],p2[i]);
	ransac::radial_param_t param;
	double maxerror = 1;
	Ransac(data, ransac::radial_estimator(cod1,cod2), ransac::radial_evaluator(), 
		15, maxerror, .95, param);

	cod1=param.cod1;
	cod2=param.cod2;
	xi1=param.xi1;
	xi2=param.xi2;
	fundamental=param.fundamental;
}

// using EstimateRadialFundamentalMatrixApriori()
void RefineRadialFundamentalMatrix(
	const std::vector<CVector<2,double> >& p1,
	const std::vector<CVector<2,double> >& p2,
	CVector<2,double>& cod1,
	CVector<2,double>& cod2,
	double& xi1, 
	double& xi2, 
	CMatrix<3,3,double>& fundamental)
{
	int npoints = p1.size();

	std::vector<std::pair<CVector<2,double>,CVector<2,double> > > data(npoints);
	for (int i=0; i<npoints; i++)
		data[i] = std::make_pair(p1[i],p2[i]);

	// refine using only inliers
	double maxerror = 1;
	const ransac::radial_param_t param = { fundamental, xi1, xi2, cod1, cod2 };
	std::vector<std::pair<CVector<2,double>,CVector<2,double> > > inliers;
	FindInliers(data, ransac::radial_evaluator(), maxerror, param, inliers);
	int nin = inliers.size();
	TRACE("refining with %d inliers.\n", nin);

	std::vector<CVector<2,double> > in1(nin), in2(nin);
	for (int i=0; i<nin; i++)
	{
		in1[i]=inliers[i].first;
		in2[i]=inliers[i].second;
	}
	EstimateRadialFundamentalMatrixApriori(in1,in2,cod1,cod2,xi1,xi2,fundamental);
}

} // namespace fmatrix
} // namespace slib

