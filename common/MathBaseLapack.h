//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: MathBaseLapack.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once

// http://software.intel.com/en-us/articles/intel-mkl-link-line-advisor/
#pragma comment(lib,"mkl_c_dll.lib")

#undef small // conflict with "rpcndr.h" used in MFC
#include <mkl_lapack.h>

#include "MathBaseUtil.h"

#undef min
#undef max

namespace slib
{

//------------------------------------------------------------
// SVD
//------------------------------------------------------------

namespace {

template <typename CMatrixType, typename CVectorType> inline
void SingularValueDecomposition(const CMatrixType& mat, // in
								bool bCalcU, // in
								bool bCalcVt, // in
								CMatrixType& matU, // out
								CVectorType& vecW, // out
								CMatrixType& matVt // out
						)
{
	int rows = mat.GetNumRows();
	int cols = mat.GetNumCols();
	int dim = std::min(rows,cols);
	
	char jobu = bCalcU ? 'A' : 'N';
	char jobvt = bCalcVt ? 'A' : 'N';
	int m = rows;
	int n = cols;
	double *a = new double [rows*cols]; // will be destroyed
	for (int i=0; i<rows*cols; i++) a[i] = mat.ptr()[i];
	int lda = m;
	double *s = new double [dim];
	double *u = 0;
	if (bCalcU) u = new double [rows*rows];
	int ldu = m;
	double *vt = 0;
	if (bCalcVt) vt = new double [cols*cols];
	int ldvt = n;
	int lwork = std::max(3*std::min(m, n)+std::max(m, n), 5*std::min(m,n)); 
	double *work = new double [lwork];
	int info;
	DGESVD(&jobu,&jobvt,&m,&n,a,&lda,s,u,&ldu,vt,&ldvt,work,&lwork,&info);

	if (info < 0)
		TRACE("Error: %d-th parameter of DGESVD had an illegal value.\n", -info);
	else if (info > 0)
		TRACE("Error: DGESVD did not converge.", info);

	delete [] a;
	delete [] work;

	_ASSERTE(vecW.GetNumRows() == dim);
	for (int i=0; i<dim; i++)
		vecW[i] = s[i];
	delete [] s;

	if (bCalcU) {
		_ASSERTE(matU.GetNumRows()==rows && matU.GetNumCols()==rows);
		matU.Initialize(u);
		delete [] u;
	}

	if (bCalcVt) {
		_ASSERTE(matVt.GetNumRows()==cols && matVt.GetNumCols()==cols);
		matVt.Initialize(vt);
		delete [] vt;
	}
}

} // unnaned namespace

template <typename T> inline
void SingularValueDecomposition(const CDynamicMatrix<T>& mat,
						CDynamicMatrix<T>& matU,
						CDynamicVector<T>& vecW,
						CDynamicMatrix<T>& matVt)
{
	matU.Resize(mat.GetNumRows(),mat.GetNumRows());
	vecW.Resize(std::min(mat.GetNumRows(),mat.GetNumCols()));
	matVt.Resize(mat.GetNumCols(),mat.GetNumCols());
	SingularValueDecomposition(mat, true, true, matU, vecW, matVt);
}

template <typename T> inline
void SingularValueDecomposition(const CDynamicMatrix<T>& mat,
						CDynamicVector<T>& vecW,
						CDynamicMatrix<T>& matVt)
{
	CDynamicMatrix<T> matU;
	vecW.Resize(std::min(mat.GetNumRows(),mat.GetNumCols()));
	matVt.Resize(mat.GetNumCols(),mat.GetNumCols());
	SingularValueDecomposition(mat, false, true, matU, vecW, matVt);
}

template <typename T> inline
void SingularValueDecomposition(const CDynamicMatrix<T>& mat,
						CDynamicMatrix<T>& matU,
						CDynamicVector<T>& vecW)
{
	matU.Resize(mat.GetNumRows(),mat.GetNumRows());
	vecW.Resize(std::min(mat.GetNumRows(),mat.GetNumCols()));
	CDynamicMatrix<T> matVt;
	SingularValueDecomposition(mat, true, false, matU, vecW, matVt);
}

template <typename T> inline
void SingularValueDecomposition(const CDynamicMatrix<T>& mat,
						CDynamicVector<T>& vecW)
{
	CDynamicMatrix<T> matU, matVt;
	vecW.Resize(std::min(mat.GetNumRows(),mat.GetNumCols()));
	SingularValueDecomposition(mat, false, false, matU, vecW, matVt);
}

//------------------------------------------------------------
// Inverse
//------------------------------------------------------------

template <typename T> inline
const CDynamicMatrix<T> inverse_of(const CDynamicMatrix<T>& mat)
{
	if (mat.GetNumRows()==2&&mat.GetNumCols()==2) 
	{
		T det = mat(0,0) * mat(1,1) - mat(0,1) * mat(1,0);
		CDynamicMatrix<T> iret(2,2);
		iret(0,0)= mat(1,1)/det;
		iret(0,1)=-mat(0,1)/det;
		iret(1,0)=-mat(1,0)/det;
		iret(1,1)= mat(0,0)/det;
		return iret;
	}
	else if (mat.GetNumRows()==3&&mat.GetNumCols()==3) 
	{
		T det = mat(0,0)*mat(1,1)*mat(2,2) + mat(0,1)*mat(1,2)*mat(2,0) + mat(0,2)*mat(1,0)*mat(2,1) - mat(0,0)*mat(1,2)*mat(2,1) - mat(0,1)*mat(1,0)*mat(2,2) - mat(0,2)*mat(1,1)*mat(2,0);
		CDynamicMatrix<T> iret(3,3);
		iret(0,0)=(mat(1,1)*mat(2,2)-mat(1,2)*mat(2,1))/det;
		iret(0,1)=(mat(0,2)*mat(2,1)-mat(0,1)*mat(2,2))/det;
		iret(0,2)=(mat(0,1)*mat(1,2)-mat(0,2)*mat(1,1))/det;
		iret(1,0)=(mat(1,2)*mat(2,0)-mat(1,0)*mat(2,2))/det;
		iret(1,1)=(mat(0,0)*mat(2,2)-mat(0,2)*mat(2,0))/det;
		iret(1,2)=(mat(0,2)*mat(1,0)-mat(0,0)*mat(1,2))/det;
		iret(2,0)=(mat(1,0)*mat(2,1)-mat(1,1)*mat(2,0))/det;
		iret(2,1)=(mat(0,1)*mat(2,0)-mat(0,0)*mat(2,1))/det;
		iret(2,2)=(mat(0,0)*mat(1,1)-mat(0,1)*mat(1,0))/det;
		return iret;
	}

	int m = mat.GetNumRows();
	int n = mat.GetNumCols();
	CDynamicMatrix<double> result(m, n);
	for (int c=0; c<n; c++)
		for (int r=0; r<m; r++)
			result(r,c) = mat(r,c);

	double *a = const_cast<double *>(result.ptr());
	int lda = m;
	int *ipiv = new int [std::min(m,n)];
	int info;
	DGETRF(&m, &n, a, &lda, ipiv, &info); 

	if (info>0)
	{
		delete [] ipiv;
		throw std::runtime_error("singular matrix");
	}

	int lwork = m;
	double *work = new double [lwork];
	DGETRI(&n, a, &lda, ipiv, work, &lwork, &info);

	delete [] ipiv;
	delete [] work;

	CDynamicMatrix<T> iret(m, n);
	for (int c=0; c<n; c++)
		for (int r=0; r<m; r++)
			iret(r,c) = result(r,c);
	return iret;
}

template <typename T> inline
T determinant_of(const CDynamicMatrix<T>& mat)
{
	int m = mat.GetNumRows();
	int n = mat.GetNumCols();
	CDynamicMatrix<double> result(m, n);
	for (int c=0; c<n; c++)
		for (int r=0; r<m; r++)
			result(r,c) = mat(r,c);

	int minmn = std::min(m,n);
	double *a = const_cast<double *>(result.ptr());
	int lda = m;
	int *ipiv = new int [minmn];
	int info;
	DGETRF(&m, &n, a, &lda, ipiv, &info);

	if (info > 0) {
		delete [] ipiv;
		return 0;
	}

	T det = 1;
	for(int i=0; i < minmn; i++) {
		det *= result(i,i);
		if(ipiv[i] != i+1)
			det = -det;
	}

	delete [] ipiv;

	return det;
}

}