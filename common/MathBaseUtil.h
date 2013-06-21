//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: MathBaseUtil.h 4304 2010-12-07 09:02:51Z shun $
//

#pragma once

#include "MathBase.h"

namespace slib
{

//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

template <int nNumRows, int nNumCols, typename T> inline
const CMatrix<nNumCols, nNumRows, T> GetPseudoInverse(const CMatrix<nNumRows, nNumCols, T>& mat)
{
	CMatrix<nNumCols,nNumRows,T> transposed = transpose_of(mat);
	if (nNumCols < nNumRows)
		return inverse_of(transposed * mat) * transposed; // over-constrained
	else
		return transposed * inverse_of(mat * transposed); // under-constrained
}

/*
//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

template <int nDimension, typename T>
inline
void CalcJacobiTransformation(
							  const CMatrix<nDimension,nDimension, T>& mat,
							  CMatrix<nDimension,nDimension, T>& matEigenVectors,
							  CVector<nDimension, T>& vecEigenValues)
{
	// assert the symmetry of a matrix
	for (int i=0; i<nDimension; i++)
		for (int j=i; j<nDimension; j++)
			_ASSERTE(mat(i,j)==mat(j,i));

	// set pointers for NR routine
	CMatrix<nDimension,nDimension, double> matTmpMatrix;
	CMatrix<nDimension,nDimension, double> matTmpEigenVector;
	for (int i=0; i<nDimension; i++)
		for (int j=0; j<nDimension; j++)
			matTmpMatrix( j,i)=static_cast<double>(mat(i,j));// transposed (CMatrix format => NR format)
	double *elementTmpMatrix[nDimension];
	double *elementTmpEigenVector[nDimension];
	for (int i=0; i<nDimension; i++)
	{
		elementTmpMatrix[i] = (double*)matTmpMatrix.ptr() + i*nDimension;
		elementTmpMatrix[i]--;
		elementTmpEigenVector[i] = (double*)matTmpEigenVector.ptr() + i*nDimension;
		elementTmpEigenVector[i]--;
	}

	CVector<nDimension, double> vecTmpEigenValue;
	double *elementTmpEigenValue = (double*)vecTmpEigenValue.ptr();
	int nrot = 5*nDimension*nDimension;
	jacobi(elementTmpMatrix-1, nDimension, elementTmpEigenValue-1, elementTmpEigenVector-1, &nrot);

	// copy the result
	for (int r=0; r<nDimension; r++)
		vecEigenValues[r] = T(elementTmpEigenValue[r]);
	for (int c=0; c<nDimension; c++)
		for (int r=0; r<nDimension; r++)
			matEigenVectors(c, r)= T(matTmpEigenVector(r, c)); // transposed (NR format => CMatrix format)
}
*/

/*
//--------------------------------------------------------------------
// Applies singular value decomposition (SVD)
//--------------------------------------------------------------------

template <int dim1, int dim2, typename T>
inline
void SingularValueDecomposition(
						const CMatrix<dim1,dim2,T>& mat,
						CMatrix<dim1,dim2,T>& matU,
						CVector<dim2,T>& vecW,
						CMatrix<dim2,dim2,T>& matVt)
{
	STATIC_ASSERT(dim1>=dim2);

	// A = U.W.V^T
	double **A = dmatrix(1,dim1,1,dim2);
	double  *W = dvector(1,dim2);
	double **V = dmatrix(1,dim2,1,dim2);

	for (int c=0; c<dim2; c++)
		for (int r=0; r<dim1; r++)
			A[r+1][c+1]=mat(r,c);

	// Singular value decomposition
	dsvdcmp(A, dim1, dim2, W, V);

	for (int c=0; c<dim2; c++)
		for (int r=0; r<dim1; r++)
			matU(r,c)=A[r+1][c+1];

	for (int r=0; r<dim2; r++)
		vecW[r]=W[r+1];
		for (int c=0; c<dim2; c++)
	for (int r=0; r<dim2; r++)
			matVt(r,c)=V[c+1][r+1];//transpose

	free_dmatrix(V,1,dim2,1,dim2);
	free_dvector(W,1,dim2);
	free_dmatrix(A,1,dim1,1,dim2);
}

template <typename T>
inline
void SingularValueDecomposition(
						const CDynamicMatrix<T>& mat,
						CDynamicMatrix<T>& matU,
						CDynamicVector<T>& vecW,
						CDynamicMatrix<T>& matVt)
{
	int rows = mat.GetNumRows();
	int cols = mat.GetNumCols();

	// A(r,c) = U(r,c).W(c,c).V^T(c,c)
	CDynamicMatrix<double> matfUT(cols,rows); // transposed for NR format
	CDynamicVector<double> vecfW(cols);
	CDynamicMatrix<double> matfVt(cols,cols);

		for (int j=0; j<cols; j++)
	for (int i=0; i<rows; i++)
			matfUT(j,i)=static_cast<double>(mat(i,j));// transposed (CMatrix format => NR format)

	double **elementU = new double * [rows];
	for (int i=0; i<rows; i++)
	{
		elementU[i] = (double *)matfUT.ptr() + i*cols;
		elementU[i]--;
	}
	double **elementV = new double * [cols];
	for (int i=0; i<cols; i++)
	{
		elementV[i] = (double *)matfVt.ptr() + i*cols;
		elementV[i]--;
	}
	double *elementW = (double *)vecfW.ptr();

	// Singular value decomposition
	dsvdcmp(elementU-1, rows, cols, elementW-1, elementV-1);

	matU.Resize(rows,cols);
	vecW.Resize(cols);
	matVt.Resize(cols,cols);
	for (int i=0; i<cols; i++)
	{
		vecW[i] = T(vecfW[i]);
		for (int j=0; j<rows; j++)
			matU ( j,i)=T(matfUT(i,j)); // transposed (NR format => CMatrix format)
		for (int j=0; j<cols; j++)
			matVt( i,j)=T(matfVt(i,j));
	}

	delete [] elementU;
	delete [] elementV;
}
*/

/*
//--------------------------------------------------------------------
// Applies Choleski Decomposition 
//--------------------------------------------------------------------

template <int nDimension, typename T>
inline
void CalcCholeskiDecomposition(
						const CMatrix<nDimension,nDimension, T>& mat,
						CMatrix<nDimension,nDimension, T>& matU)
{
	double **choleski = dmatrix(1,nDimension,1,nDimension);
		for (int c=0; c<nDimension; c++)
	for (int r=0; r<nDimension; r++)
			choleski[r+1][c+1]=mat(r,c);
	double *diag = dvector(1,nDimension);

	choldc(choleski,nDimension,diag);

	matU = CMatrix<nDimension,nDimension,double>::GetZero();
		for (int c=r; c<nDimension; c++)
	for (int r=0; r<nDimension; r++)
			if (r==c)
				matU(r,c)=diag[r+1];
			else
				matU(r,c)=choleski[c+1][r+1];

	free_dmatrix(choleski,1,nDimension,1,nDimension);
	free_dvector(diag,1,nDimension);
}

template <typename T>
inline
void CalcCholeskiDecomposition(
						const CDynamicMatrix<T>& mat,
						CDynamicMatrix<T>& matU)
{
	int dim = mat.GetNumRows();
	int dim2 = mat.GetNumCols();
	if (dim!=dim2)
		throw std::runtime_error("not a square matrix");

	double **choleski = dmatrix(1,dim,1,dim);
		for (int c=0; c<dim; c++)
	for (int r=0; r<dim; r++)
			choleski[r+1][c+1]=mat(r,c);
	double *diag = dvector(1,dim);

	choldc(choleski,dim,diag);

	matU.Resize(dim,dim);
	GetZero(matU);
		for (int c=0; c<nDimension; c++)
	for (int r=0; r<nDimension; r++)
			if (r==c)
				matU(r,c)=diag[r+1];
			else
				matU(r,c)=choleski[c+1][r+1];

	free_dmatrix(choleski,1,nDimension,1,nDimension);
	free_dvector(diag,1,nDimension);
}
*/

//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

template <int nDimension, typename T>
inline
T trace_of(const CMatrix<nDimension,nDimension, T>& mat)
{
	T iret = T(0);
	for (int i=0; i<nDimension; i++)
		iret += mat(i,i);
	return iret;
}

/*
//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

template <int nDimension, typename T>
inline
CMatrix<nDimension,nDimension, T>& GenerateSymmetricMatrix(
	const CVector<nDimension, T>& vec1,
	const CVector<nDimension, T>& vec2,
	CMatrix<nDimension,nDimension, T>& iret
	)
{
		for (int j=0; j<nDimension; j++)
	for (int i=0; i<nDimension; i++)
			iret(i,j) = vec1[i] * vec2[j];
	return iret;
}
*/

/*
//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

template <int nDimension, typename T>
inline
CMatrix<nDimension,nDimension, T>& GenerateCovarianceMatrix(
	const CVector<nDimension, T>& vec,
	CMatrix<nDimension,nDimension, T>& iret
	)
{
	for (int j=0; j<nDimension; j++)
		for (int i=0; i<nDimension; i++)
			iret(i,j) = vec[i] * vec[j];
	return iret;
}
*/

template <typename T> inline
const CDynamicMatrix<T> GetPseudoInverse(const CDynamicMatrix<T>& mat)
{
	CDynamicMatrix<T> transposed = transpose_of(mat);
	if (mat.GetNumCols() < mat.GetNumRows())
		return inverse_of(transposed * mat) * transposed;
	else
		return transposed * inverse_of(mat * transposed);
}
/*
//----------------------------------------------------------------------
//
//----------------------------------------------------------------------

template <typename T>
inline
void CalcJacobiTransformation(
	const CDynamicMatrix<T>& mat,
	CDynamicMatrix<T>& matEigenVectors,
	CDynamicVector<T>& vecEigenValues)
{
	_ASSERTE(mat.GetNumCols()==mat.GetNumRows());
	int nDimension = mat.GetNumCols();

	//// assert the symmetry of a matrix
	//for (int i=0; i<nDimension; i++)
	//	for (int j=i; j<nDimension; j++)
	//		_ASSERTE(mat(i,j)==mat(j,i));

	// set pointers for NR routine
	CDynamicMatrix<double> matTmpMatrix(nDimension, nDimension);
	CDynamicMatrix<double> matTmpEigenVector(nDimension, nDimension);
	for (int i=0; i<nDimension; i++)
		for (int j=0; j<nDimension; j++)
			matTmpMatrix( j,i)=static_cast<double>(mat(i,j));// transposed (CMatrix format => NR format)

	double **elementTmpMatrix=new double*[nDimension];
	double **elementTmpEigenVector=new double*[nDimension];
	for (int i=0; i<nDimension; i++)
	{
		elementTmpMatrix[i] = (double*)matTmpMatrix.ptr() + i*nDimension;
		elementTmpMatrix[i]--;
		elementTmpEigenVector[i] = (double*)matTmpEigenVector.ptr() + i*nDimension;
		elementTmpEigenVector[i]--;
	}

	CDynamicVector<double> vecTmpEigenValue(nDimension);
	double *elementTmpEigenValue = (double*)vecTmpEigenValue.ptr();
	int nrot = 5*nDimension*nDimension;
	jacobi(elementTmpMatrix-1, nDimension, elementTmpEigenValue-1, elementTmpEigenVector-1, &nrot);

	// copy the result
	for (int r=0; r<nDimension; r++)
		vecEigenValues[r] = T(elementTmpEigenValue[r]);
		for (int c=0; c<nDimension; c++)
	for (int r=0; r<nDimension; r++)
			matEigenVectors(c, r)= T(matTmpEigenVector(r, c)); // transposed (NR format => CMatrix format)

	delete [] elementTmpMatrix;
	delete [] elementTmpEigenVector;
}
*/

} // namespace slib
