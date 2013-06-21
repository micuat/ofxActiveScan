//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp)
//  and the National Institute of Advanced Industrial Science and Technology
//
// $Id: MathBase.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once

#ifndef TRACE
#define TRACE printf
#endif

#ifndef _ASSERTE
#define _ASSERTE assert
#endif

#undef min
#undef max

#include <string.h>

namespace slib
{

#ifndef BOOST_STATIC_ASSERT
// static assertion (taken from <boost/static_assert.hpp>)
#define STATIC_ASSERT(B) \
	typedef mathbase_static_assert_test< \
		sizeof(MATHBASE_STATIC_ASSERTION_FAILURE< bool(B) >) \
	> mathbase_static_assert_typedef_
template <int x> struct mathbase_static_assert_test{};
template <bool x> struct MATHBASE_STATIC_ASSERTION_FAILURE;
template <> struct MATHBASE_STATIC_ASSERTION_FAILURE<true> { enum { value = 1 }; };
#else
#define STATIC_ASSERT BOOST_STATIC_ASSERT
#endif

//---------- static matrix/vector

template <int nDimension, typename T>
class CVector;

template <int nNumRows, int nNumCols, typename T>
class CMatrix
{
protected:
	T m[nNumRows*nNumCols]; // row major order

public:
	// constructors
	CMatrix(void)
	{
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] = T();
	}

	explicit CMatrix(T const * const e) {
		Initialize(e);
	}

	// copy
	CMatrix(const CMatrix& mat)
	{
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] = mat.m[i];
	}

	const CMatrix& operator =(const CMatrix& mat)
	{
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] = mat.m[i];
		return *this;
	}

	// coercion
	template <typename T2>
	CMatrix(const CMatrix<nNumRows, nNumCols, T2>& mat)
	{
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] = mat.ptr()[i];
	}

	template <typename T2>
	const CMatrix& operator =(const CMatrix<nNumRows, nNumCols, T2>& mat)
	{
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] = mat.ptr()[i];
		return *this;
	}

	// helpers
	template <typename T2>
	const CMatrix& Initialize(T2 const * const element)
	{
		_ASSERTE(element);
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] = element[i];
		return *this;
	}

	template <typename T2>
	void Fill(T2 const e) 
	{
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] = e;
	}

	//---------- accessors
	T& operator ()(const int r, const int c)
	{
		_ASSERTE(0<=r && r<nNumRows && 0<=c && c<nNumCols);
		return m[r + nNumRows * c];
	}

	T const& operator ()(const int r, const int c) const
	{
		_ASSERTE(0<=r && r<nNumRows && 0<=c && c<nNumCols);
		return m[r + nNumRows * c];
	}

	T const *ptr(void) const { return m; }

	//---------- arithmetic operators
	template <typename T2>
	const CMatrix& operator +=(const CMatrix<nNumRows,nNumCols,T2>& mat)
	{
		for (int i=0; i<nNumRows*nNumCols; i++) m[i] += mat.m[i];
		return *this;
	}

	template <typename T2>
	const CMatrix& operator -=(const CMatrix<nNumRows,nNumCols,T2>& mat)
	{
		for (int i=0; i<nNumRows*nNumCols; i++) m[i] -= mat.m[i];
		return *this;
	}

	template <typename T2>
	const CMatrix& operator *=(const CMatrix<nNumCols,nNumCols,T2>& mat) {
		return *this = *this * mat;
	}

	// "s" may be self-referencing
	const CMatrix& operator *=(T const s) 
	{
		for (int i=0; i<nNumRows*nNumCols; i++)
			m[i] *= s;
		return *this;
	}

	// "s" may be self-referencing
	template <typename T2>
	const CMatrix& operator /=(T2 const s)
	{
		_ASSERTE(s != 0);
		for (int i=0; i<nNumRows*nNumCols; i++) m[i] /= s;
		return *this;
	}

	// unary operator
	CMatrix operator -(void) const {
		return *this * (-1);
	}

	const CMatrix& operator +(void) const {
		return *this;
	}

	// binary operator
	CMatrix operator +(const CMatrix& v2) const {
		return CMatrix(*this) += v2;
	}

	CMatrix operator -(const CMatrix& v2) const {
		return CMatrix(*this) -= v2;
	}

	template <int nNumCols2>
	CMatrix<nNumRows,nNumCols2,T> operator *(const CMatrix<nNumCols,nNumCols2,T>& mat2) const
	{
		CMatrix<nNumRows,nNumCols2,T> iret;
		for (int c=0; c<nNumCols2; c++)
			for (int r=0; r<nNumRows; r++)
			{
				iret(r, c) = T(0);
				for (int i=0; i<nNumCols; i++)
					iret(r,c) += (*this)(r,i) * mat2(i,c);
			}
		return iret;
	}

	CMatrix operator *(T const s) const {
		return CMatrix(*this) *= s;
	}

	template <typename T2>
	CMatrix operator /(T2 const s) const {
		return CMatrix(*this) /= s;
	}

	friend const CMatrix operator *(const double s, const CMatrix mat) {
		return mat * s;
	}

	//---------- relationship operators
	bool operator == (const CMatrix& mat2) const {
		return !memcmp(m, mat2.m, sizeof (T) * nNumRows * nNumCols);
	}

	bool operator != (const CMatrix& mat2) const {
		return !(*this == mat2);
	}

	//---------- helper methods
	CVector<nNumRows,T> GetColumn(int c) const 
	{
		CVector<nNumRows,T> iret;
		for (int r=0; r<nNumRows; r++)
			iret[r]=(*this)(r,c);
		return iret;
	}

	CVector<nNumRows,T> GetRow(int r) const
	{
		CVector<nNumCols,T> iret;
		for (int c=0; c<nNumCols; c++)
			iret[c]=(*this)(r,c);
		return iret;
	}

	template <int nSubRows,int nSubCols>
	CMatrix<nSubRows,nSubCols,T> GetSubMatrix(int sr, int sc) const
	{
		STATIC_ASSERT(nSubRows<=nNumRows);
		STATIC_ASSERT(nSubCols<=nNumCols);
		CMatrix<nSubRows,nSubCols,T> iret;
		for (int c=0; c<nSubCols; c++)
			for (int r=0; r<nSubRows; r++)
				iret(r,c)=(*this)(sr+r,sc+c);
		return iret;
	}

	template <int nNewCols, typename T2>
	CMatrix<nNumRows,nNumCols+nNewCols,T> AppendCols(const CMatrix<nNumRows,nNewCols,T2>& mat) const 
	{
		CMatrix<nNumRows,nNumCols+nNewCols,T> iret;
		T* p = (T*)iret.ptr();

		// left half
		int nelem1 = nNumRows*nNumCols;
		for (int i=0; i<nelem1; i++)
			p[i] = m[i];

		// right half
		int nelem2 = nNumRows * nNewCols;
		for (int i=0; i<nelem2; i++)
			p[i+nelem1] = mat.ptr()[i];
		return iret;
	}

	template <int nNewRows>
	CMatrix<nNumRows+nNewRows,nNumCols,T> AppendRows(const CMatrix<nNewRows,nNumCols,T>& mat) const
	{
		CMatrix<nNumRows+nNewRows,nNumCols,T> iret;
		for (int c=0; c<nNumCols; c++) {
			int r=0;
			for (; r<nNumRows; r++)
				iret(r,c) = (*this)(r,c);
			for (int r2=0; r<nNumRows+nNewRows; r++, r2++)
				iret(r,c) = mat(r2,c);
		}
		return iret;
	}

	T min(void) const 
	{
		T iret = m[0];
		for (int i=1; i<nNumRows*nNumCols; i++)
			iret = std::min(iret, m[i]);
		return iret;
	}

	T max(void) const
	{
		T iret = m[0];
		for (int i=1; i<nNumRows*nNumCols; i++)
			iret = std::max(iret, m[i]);
		return iret;
	}

	CMatrix abs(void) const
	{
		CMatrix result;
		for (int i=0; i<nNumRows*nNumCols; i++)
			result.m[i] = std::abs(m[i]);
		return result;
	}

	// Note:
	// - static const variable cannot be used in the dynamic version.
	// - non-static method cannot be used in template variables.
	static int GetNumRows(void) { return nNumRows; }
	static int GetNumCols(void) { return nNumCols; }

	void Read(const std::string& filename) 
	{
		TRACE("matrix <= %s\n", filename.c_str());
		FILE *fr = fopen(filename.c_str(), "rb");
		if (!fr)
			throw std::runtime_error("failed to open");

		for (int r=0; r<nNumRows; r++) {
			for (int c=0; c<nNumCols; c++) {
				double el;
				if (fscanf(fr, "%lf", &el) != 1) {
					fclose(fr);
					throw std::runtime_error("failed to parse");
				}
				(*this)(r,c) = el;
			}
		}

		fclose(fr);
	}

	void Write(const std::string& filename) const 
	{
		TRACE("matrix => %s\n", filename.c_str());
		FILE *fw = fopen(filename.c_str(), "wb");
		if (!fw)
			throw std::runtime_error("failed to open");

		for (int r=0; r<nNumRows; r++) {
			for (int c=0; c<nNumCols; c++) 
				fprintf(fw, "% le ", double((*this)(r,c)));
			fprintf(fw, "\n");
		}

		fclose(fw);
	}
};

// construction
template <typename T>
inline
CMatrix<2, 2, T> make_matrix(
		T const& a00, T const& a01,
		T const& a10, T const& a11)
{
	T element[] =
	{
		a00, a10,
		a01, a11,
	};
	return CMatrix<2, 2, T>(element);
}

template <typename T>
inline
CMatrix<3, 3, T> make_matrix(
	T const& a00, T const& a01, T const& a02,
	T const& a10, T const& a11, T const& a12,
	T const& a20, T const& a21, T const& a22)
{
	T element[] =
	{
		a00, a10, a20,
		a01, a11, a21,
		a02, a12, a22,
	};
	return CMatrix<3, 3, T>(element);
}

template <typename T>
inline
CMatrix<4, 4, T> make_matrix(
	T const& a00, T const& a01, T const& a02, T const& a03,
	T const& a10, T const& a11, T const& a12, T const& a13,
	T const& a20, T const& a21, T const& a22, T const& a23,
	T const& a30, T const& a31, T const& a32, T const& a33)
{
	T element[] =
	{
		a00, a10, a20, a30,
		a01, a11, a21, a31,
		a02, a12, a22, a32,
		a03, a13, a23, a33,
	};
	return CMatrix<4, 4, T>(element);
}

template <int nDim, typename T>
inline
CMatrix<nDim,nDim,T> make_diagonal_matrix(const CVector<nDim,T>& v)
{
	T element[nDim*nDim];
	for (int i=0; i<nDim*nDim; i++)
	for (int c=0; c<nDim; c++)
		for (int r=0; r<nDim; r++)
			element[r+nDim*c] = (r==c)?v[r]:0;
	return CMatrix<nDim,nDim,T>(element);
}

template <typename T>
inline
CMatrix<2,2,T> make_diagonal_matrix(const T& e1, const T& e2) {
	return make_diagonal_matrix(make_vector(e1,e2));
}

template <typename T>
inline
CMatrix<3,3,T> make_diagonal_matrix(const T& e1, const T& e2, const T& e3) {
	return make_diagonal_matrix(make_vector(e1,e2,e3));
}

template <typename T>
inline
CMatrix<4,4,T> make_diagonal_matrix(const T& e1, const T& e2, const T& e3, const T& e4) {
	return make_diagonal_matrix(make_vector(e1,e2,e3,e4));
}

// helper functions
template <int nNumRows, int nNumCols, typename T>
inline
CMatrix<nNumCols, nNumRows, T> transpose_of(const CMatrix<nNumRows,nNumCols,T>& mat)
{
	CMatrix<nNumCols, nNumRows, T> iret;
	for (int c=0; c<nNumCols; c++)
		for (int r=0; r<nNumRows; r++)
			iret(c,r) = mat(r,c);
	return iret;
}

template <typename T>
inline
T determinant_of(const CMatrix<2, 2, T>& a) {
	return a(0,0) * a(1,1) - a(0,1) * a(1,0);
}

template <typename T>
inline
T determinant_of(const CMatrix<3, 3, T>& a)
{
	return a(0,0)*a(1,1)*a(2,2) + a(0,1)*a(1,2)*a(2,0) + a(0,2)*a(1,0)*a(2,1)
		 - a(0,0)*a(1,2)*a(2,1) - a(0,1)*a(1,0)*a(2,2) - a(0,2)*a(1,1)*a(2,0);
}

template <typename T>
inline
T determinant_of(const CMatrix<4,4,T>& a)
{
	return ((a(0,0)*a(1,1)-a(0,1)*a(1,0))*a(2,2)+(a(0,2)*a(1,0)-
		a(0,0)*a(1,2))*a(2,1)+(a(0,1)*a(1,2)-a(0,2)*a(1,1))*a(2,0))*a(3,3)+
		((a(0,1)*a(1,0)-a(0,0)*a(1,1))*a(2,3)+(a(0,0)*a(1,3)-a(0,3)*a(1,0))*
		a(2,1)+(a(0,3)*a(1,1)-a(0,1)*a(1,3))*a(2,0))*a(3,2)+((a(0,0)*a(1,2)-
		a(0,2)*a(1,0))*a(2,3)+(a(0,3)*a(1,0)-a(0,0)*a(1,3))*a(2,2)+(a(0,2)*
		a(1,3)-a(0,3)*a(1,2))*a(2,0))*a(3,1)+((a(0,2)*a(1,1)-a(0,1)*a(1,2))*
		a(2,3)+(a(0,1)*a(1,3)-a(0,3)*a(1,1))*a(2,2)+(a(0,3)*a(1,2)-a(0,2)*
		a(1,3))*a(2,1))*a(3,0);
}

template <typename T>
inline
CMatrix<2,2,T> inverse_of(const CMatrix<2,2,T>& a)
{
	T det = determinant_of(a);
	return make_matrix(
		 a(1,1), -a(0,1),
		-a(1,0),  a(0,0)
		) / det;
}

template <typename T>
inline
CMatrix<3,3,T> inverse_of(const CMatrix<3,3,T>& a)
{
	T det = determinant_of(a);
	return make_matrix(
		(a(1,1)*a(2,2)-a(1,2)*a(2,1)), (a(0,2)*a(2,1)-a(0,1)*a(2,2)), (a(0,1)*a(1,2)-a(0,2)*a(1,1)),
		(a(1,2)*a(2,0)-a(1,0)*a(2,2)), (a(0,0)*a(2,2)-a(0,2)*a(2,0)), (a(0,2)*a(1,0)-a(0,0)*a(1,2)),
		(a(1,0)*a(2,1)-a(1,1)*a(2,0)), (a(0,1)*a(2,0)-a(0,0)*a(2,1)), (a(0,0)*a(1,1)-a(0,1)*a(1,0))
		) / det;
}

template <typename T>
inline
CMatrix<4,4,T> inverse_of(const CMatrix<4,4,T>& a)
{
	T det = determinant_of(a);
	return make_matrix(
		 ((a(1,1)*a(2,2)-a(1,2)*a(2,1))*a(3,3)+(a(1,3)*a(2,1)-a(1,1)*a(2,3))*a(3,2)+(a(1,2)*a(2,3)-a(1,3)*a(2,2))*a(3,1)),
		-((a(0,1)*a(2,2)-a(0,2)*a(2,1))*a(3,3)+(a(0,3)*a(2,1)-a(0,1)*a(2,3))*a(3,2)+(a(0,2)*a(2,3)-a(0,3)*a(2,2))*a(3,1)),
		 ((a(0,1)*a(1,2)-a(0,2)*a(1,1))*a(3,3)+(a(0,3)*a(1,1)-a(0,1)*a(1,3))*a(3,2)+(a(0,2)*a(1,3)-a(0,3)*a(1,2))*a(3,1)),
		-((a(0,1)*a(1,2)-a(0,2)*a(1,1))*a(2,3)+(a(0,3)*a(1,1)-a(0,1)*a(1,3))*a(2,2)+(a(0,2)*a(1,3)-a(0,3)*a(1,2))*a(2,1)),
		-((a(1,0)*a(2,2)-a(1,2)*a(2,0))*a(3,3)+(a(1,3)*a(2,0)-a(1,0)*a(2,3))*a(3,2)+(a(1,2)*a(2,3)-a(1,3)*a(2,2))*a(3,0)),
		 ((a(0,0)*a(2,2)-a(0,2)*a(2,0))*a(3,3)+(a(0,3)*a(2,0)-a(0,0)*a(2,3))*a(3,2)+(a(0,2)*a(2,3)-a(0,3)*a(2,2))*a(3,0)),
		-((a(0,0)*a(1,2)-a(0,2)*a(1,0))*a(3,3)+(a(0,3)*a(1,0)-a(0,0)*a(1,3))*a(3,2)+(a(0,2)*a(1,3)-a(0,3)*a(1,2))*a(3,0)),
		 ((a(0,0)*a(1,2)-a(0,2)*a(1,0))*a(2,3)+(a(0,3)*a(1,0)-a(0,0)*a(1,3))*a(2,2)+(a(0,2)*a(1,3)-a(0,3)*a(1,2))*a(2,0)),
		 ((a(1,0)*a(2,1)-a(1,1)*a(2,0))*a(3,3)+(a(1,3)*a(2,0)-a(1,0)*a(2,3))*a(3,1)+(a(1,1)*a(2,3)-a(1,3)*a(2,1))*a(3,0)),
		-((a(0,0)*a(2,1)-a(0,1)*a(2,0))*a(3,3)+(a(0,3)*a(2,0)-a(0,0)*a(2,3))*a(3,1)+(a(0,1)*a(2,3)-a(0,3)*a(2,1))*a(3,0)),
		 ((a(0,0)*a(1,1)-a(0,1)*a(1,0))*a(3,3)+(a(0,3)*a(1,0)-a(0,0)*a(1,3))*a(3,1)+(a(0,1)*a(1,3)-a(0,3)*a(1,1))*a(3,0)),
		-((a(0,0)*a(1,1)-a(0,1)*a(1,0))*a(2,3)+(a(0,3)*a(1,0)-a(0,0)*a(1,3))*a(2,1)+(a(0,1)*a(1,3)-a(0,3)*a(1,1))*a(2,0)),
		-((a(1,0)*a(2,1)-a(1,1)*a(2,0))*a(3,2)+(a(1,2)*a(2,0)-a(1,0)*a(2,2))*a(3,1)+(a(1,1)*a(2,2)-a(1,2)*a(2,1))*a(3,0)),
		 ((a(0,0)*a(2,1)-a(0,1)*a(2,0))*a(3,2)+(a(0,2)*a(2,0)-a(0,0)*a(2,2))*a(3,1)+(a(0,1)*a(2,2)-a(0,2)*a(2,1))*a(3,0)),
		-((a(0,0)*a(1,1)-a(0,1)*a(1,0))*a(3,2)+(a(0,2)*a(1,0)-a(0,0)*a(1,2))*a(3,1)+(a(0,1)*a(1,2)-a(0,2)*a(1,1))*a(3,0)),
		 ((a(0,0)*a(1,1)-a(0,1)*a(1,0))*a(2,2)+(a(0,2)*a(1,0)-a(0,0)*a(1,2))*a(2,1)+(a(0,1)*a(1,2)-a(0,2)*a(1,1))*a(2,0))
		) / det;
}

#define DUMP_VECMAT(mat) \
{\
	TRACE("["__FUNCTION__"] " #mat " =:\n"); \
	for (int i__=0; i__<(mat).GetNumRows(); i__++) \
	{ \
		TRACE("\t["); \
		for (int j__=0; j__<(mat).GetNumCols(); j__++) \
			TRACE(" % g", double((mat)(i__,j__))); \
		TRACE(" ]\n"); \
	} \
}

//----------------------------------------------------------------------
// vector
//----------------------------------------------------------------------

template <int nDimension, typename T>
class CVector : public CMatrix<nDimension,1,T>
{
public:

	//---------- initializers
	CVector(void) : CMatrix<nDimension,1,T>() {}

	explicit CVector(T const * const element) 
		: CMatrix<nDimension,1,T>(element) {}

	// copy
	CVector(const CVector& vec) 
		: CMatrix<nDimension,1,T>(vec) {}

	const CVector& operator =(const CVector& vec)
	{
		CMatrix<nDimension,1,T>::operator=(vec);
		return *this;
	}

	// coersion
	template <typename T2>
	CVector(const CMatrix<nDimension,1,T2>& vec)
		: CMatrix<nDimension,1,T>(vec) {}

	template <typename T2>
	const CVector& operator =(const CMatrix<nDimension,1,T2>& vec)
	{
		CMatrix<nDimension,1,T>::operator=(vec);
		return *this;
	}

	//---------- operators
	T& operator [](const int n)
	{
		_ASSERTE(n<nDimension);
		return CMatrix<nDimension,1,T>::m[n];
	}

	T const& operator [](const int n) const
	{
		_ASSERTE(n<nDimension);
		return CMatrix<nDimension,1,T>::m[n];
	}
};

// construction
template <typename T>
inline
CVector<2, T> make_vector(T const& e1, T const& e2)
{
	T element[] = { e1, e2 };
	return CVector<2, T>(element);
}

template <typename T>
inline
CVector<3, T> make_vector(T const& e1, T const& e2, T const& e3)
{
	T element[] = { e1, e2, e3 };
	return CVector<3, T>(element);
}

template <typename T>
inline
CVector<4, T> make_vector(T const& e1, T const& e2, T const& e3, T const& e4)
{
	T element[] = { e1, e2, e3, e4 };
	return CVector<4, T>(element);
}

// vector utils
template <int nDimension, typename T>
inline
CVector<nDimension,T> GetNormalized(const CVector<nDimension,T>& vec)
{
	T len = GetNorm2(vec);
	_ASSERTE(len != 0);
	return vec / len;
}

//template <int nDimension, typename T>
//inline
//T GetNorm2Squared(const CMatrix<nDimension,1,T>& vec)
//{
//	return dot(vec, vec);
//}

template <int nDimension, typename T>
inline
T GetNorm2(const CMatrix<nDimension,1,T>& vec)
{
	return T(sqrt((double)dot(vec, vec)));
}

template <int nDimension, typename T>
inline
T dot(const CMatrix<nDimension,1,T>& vec0, const CMatrix<nDimension,1,T>& vec1)
{
	T iret = T(0);
	for (int i=0; i<nDimension; i++)
		iret += vec0(i,0) * vec1(i,0);
	return iret;
}

template <typename T>
inline
CVector<3, T> cross(const CMatrix<3,1,T>& vec0, const CMatrix<3,1,T>& vec1)
{
	return make_vector(
		vec0(1,0) * vec1(2,0) - vec0(2,0) * vec1(1,0),
		vec0(2,0) * vec1(0,0) - vec0(0,0) * vec1(2,0),
		vec0(0,0) * vec1(1,0) - vec0(1,0) * vec1(0,0));
}

template <typename T>
inline
T crossZ(const CMatrix<2,1,T>& vec0, const CMatrix<2,1,T>& vec1)
{
	return vec0(0,0) * vec1(1,0) - vec0(1,0) * vec1(0,0);
}

template <typename T>
inline
CMatrix<3,3,T> GetSkewSymmetric(CMatrix<3,1,T>& vec)
{
	return make_matrix<T>(
		0, -vec(2,0), vec(1,0),
		vec(2,0), 0, -vec(0,0),
		-vec(1,0), vec(0,0), 0
		);
}

template <int nDimension, typename T>
inline
CVector<nDimension+1,T> GetHomogeneousVector(const CMatrix<nDimension,1,T>& vec)
{
	CVector<nDimension+1,T> iret;
	for (int i=0; i<nDimension; i++)
		iret[i] = vec(i, 0);
	iret[nDimension] = 1;
	return iret;
}

template <int nDimension, typename T>
inline
CVector<nDimension-1,T> GetEuclideanVector(const CMatrix<nDimension,1,T>& vec)
{
	CVector<nDimension-1,T> iret;
	for (int i=0; i<nDimension-1; i++)
		iret[i] = vec(i, 0) / vec(nDimension-1, 0);
	return iret;
}

// rigid transformation

// --- scaling

template <typename T>
inline
CMatrix<4,4,T> GetScalingMatrix(T const& s)
{
	return GetScalingMatrix<T>(s,s,s);
}

template <typename T>
inline
CMatrix<4,4,T> GetScalingMatrix(T const& sx, T const& sy, T const& sz)
{
	CMatrix<4,4,T> mat = make_diagonal_matrix<T>(1,1,1,1);
	mat(0,0)=sx;
	mat(1,1)=sy;
	mat(2,2)=sz;
	return mat;
}

// --- translation

template <typename T>
inline
CVector<3,T> GetTranslationVector(const CMatrix<4,4,T>& mat)
{
	return make_vector(mat(0,3),mat(1,3),mat(2,3));
}

template <typename T>
inline
CMatrix<4,4,T> GetTranslationMatrix(const CMatrix<4,4,T>& mat)
{
	return GetTranslationMatrix(mat(0,3),mat(1,3),mat(2,3));
}

template <typename T>
inline
CMatrix<4,4,T> GetTranslationMatrix(const CMatrix<3,1,T>& t)
{
	return GetTranslationMatrix(t(0,0), t(1,0), t(2,0));
}

template <typename T>
inline
CMatrix<4,4,T> GetTranslationMatrix(T const& tx, T const& ty, T const& tz)
{
	CMatrix<4,4,T> mat = make_diagonal_matrix<T>(1,1,1,1);
	mat(0,3)=tx;
	mat(1,3)=ty;
	mat(2,3)=tz;
	return mat;
}

// --- rotation

template <int nDimension, typename T>
inline
void GetEulerAngles(const CMatrix<nDimension,nDimension,T>& mat, T& a, T& b, T& c)
{
	STATIC_ASSERT(nDimension==3||nDimension==4);

	// assume b >= 0
	if (abs(mat(2,2)) != 1)
	{
		a = atan2(mat(1,2),  mat(0,2));
		b = atan2((T)sqrt(mat(0,2)*mat(0,2) + mat(1,2)*mat(1,2)), mat(2,2));
		c = atan2(mat(2,1), -mat(2,0));
	}
	else
	{
		// assume c = 0
		a = atan2(mat(1,0), mat(1,1));
		b = 0;
		c = 0;
	}
}

template <int nDimension, typename T>
inline
void GetRollPitchYawAngles(const CMatrix<nDimension,nDimension,T>& mat, T& rollRadian, T& pitchRadian, T& yawRadian)
{
	STATIC_ASSERT(nDimension==3||nDimension==4);

	// assume pitchRadian >= 0
	T cosPitch = sqrt(mat(0,0)*mat(0,0) + mat(1,0)*mat(1,0));
	if (cosPitch)
	{
		rollRadian  = atan2( mat(1,0), mat(0,0));
		pitchRadian = atan2(-mat(2,0), cosPitch);
		yawRadian   = atan2( mat(2,1), mat(2,2));
	}
	else
	{
		rollRadian  = -atan2(mat(0,1), mat(1,1));
		pitchRadian = -mat(2,0) * M_PI/2;
		yawRadian   = 0;
	}
}

template <typename T>
inline
CMatrix<4,4,T> GetRotationMatrix(const CMatrix<4,4,T>& mat)
{
	CMatrix<4,4,T> mat2 = mat;
	mat2(0,3)=0;
	mat2(1,3)=0;
	mat2(2,3)=0;
	return mat2;
}

template <typename T>
inline
CMatrix<4,4,T> GetRotationMatrixByRollPitchYaw(T rollRadian, T pitchRadian, T yawRadian)
{
	double cr = cos(rollRadian);
	double sr = sin(rollRadian);
	double cp = cos(pitchRadian);
	double sp = sin(pitchRadian);
	double cy = cos(yawRadian);
	double sy = sin(yawRadian);
	T element[] =
	{
		cr*cp,          sr*cp,         -sp,    0,
		cr*sp*sy-sr*cy, sr*sp*sy+cr*cy, cp*sy, 0,
		cr*sp*cy+sr*sy, sr*sp*cy-cr*sy, cp*cy, 0,
		0,0,0,1,
	};
	CMatrix<4,4,T> mat;
	return mat.Initialize(element);
}

template <typename T>
inline
CMatrix<4,4,T> GetRotationMatrixByEuler(T a, T b, T c)
{
	double ca = cos(a);
	double sa = sin(a);
	double cb = cos(b);
	double sb = sin(b);
	double cc = cos(c);
	double sc = sin(c);
	T element[] =
	{
		 ca*cb*cc-sa*sc,  sa*cb*cc+ca*sc, -sb*cc, 0,
		-ca*cb*sc-sa*cc, -sa*cb*sc+ca*cc,  sb*sc, 0,
		 ca*sb,           sa*sb,           cb, 0,
        0,0,0,1,
	};
	return CMatrix<4,4,T>(element);
}

template <typename T>
inline
CMatrix<4,4,T> GetRotationMatrixByAxis(const CVector<3,T>& axis, float cosangle, float sinangle)
{
	T len = GetNorm2(axis);
	if (!len)
		return make_diagonal_matrix<T>(1,1,1,1);

	CVector<3, T> unitv = GetNormalized(axis);
	T element[] =
	{
		unitv[0] * unitv[0] + (1 - unitv[0] * unitv[0]) * cosangle,
		unitv[0] * unitv[1] * (1 - cosangle) + unitv[2] * sinangle,
		unitv[0] * unitv[2] * (1 - cosangle) - unitv[1] * sinangle,
		0,
		unitv[0] * unitv[1] * (1 - cosangle) - unitv[2] * sinangle,
		unitv[1] * unitv[1] + (1 - unitv[1] * unitv[1]) * cosangle,
		unitv[1] * unitv[2] * (1 - cosangle) + unitv[0] * sinangle,
		0,
		unitv[0] * unitv[2] * (1 - cosangle) + unitv[1] * sinangle,
		unitv[1] * unitv[2] * (1 - cosangle) - unitv[0] * sinangle,
		unitv[2] * unitv[2] + (1 - unitv[2] * unitv[2]) * cosangle,
		0,
		0,0,0,1,
	};
	return CMatrix<4,4,T>(element);
}

template <typename T>
inline
CMatrix<4,4,T> GetRotationMatrixByAxis(const CVector<3,T>& axis, float angle)
{
	return GetRotationMatrixByAxis(axis, cos(angle), sin(angle));
}

template <typename T>
inline
CMatrix<4,4,T> GetRotationMatrixByMouseDrag(T sx, T sy, T dx, T dy)
{
	CVector<3,T> v1 = GetNormalized(make_vector<T>(sx,sy,1));
	CVector<3,T> v2 = GetNormalized(make_vector<T>(sx+dx,sy+dy,1));
	CVector<3,T> cr = cross(v1,v2);
	return GetRotationMatrixByAxis(cr, dot(v1,v2), GetNorm2(cr));
}

#if defined(MK_LBUTTON)
template <typename T>
inline
CMatrix<4,4,T> GetTransformationMatrixByMouseDrag(T sx, T sy, T dx, T dy, int nFlags)
{
	// left button
	if ( (nFlags & MK_LBUTTON) &&
		!(nFlags & MK_MBUTTON) &&
		!(nFlags & MK_RBUTTON))
	{
		return TranslationMatrix<T>(dx, -dy, 0);
	}
	// middle button
	else if ((!(nFlags & MK_LBUTTON) &&
			   (nFlags & MK_MBUTTON) &&
			  !(nFlags & MK_RBUTTON)) ||
			 ( (nFlags & MK_LBUTTON) &&
			  !(nFlags & MK_MBUTTON) &&
			   (nFlags & MK_RBUTTON)))
	{
		return TranslationMatrix<T>(0, 0, dy);
	}
	// right button
	else if (!(nFlags & MK_LBUTTON) &&
			 !(nFlags & MK_MBUTTON) &&
			  (nFlags & MK_RBUTTON))
	{
		return RotationMatrixByMouseDrag(sx, -sy, dx, -dy);
	}
	_ASSERTE(!"warning: supposed not to reach here.\n");
	return CMatrix<4,4,T>::GetIdentity();
}
#endif

//---------- dynamic matrix/vector

template <typename T>
class CDynamicVector;

template <typename T>
class CDynamicMatrix
{
protected:
	int m_nNumRows;
	int m_nNumCols;
	T *m; // row major order

public:
	//---------- constructors
	CDynamicMatrix(void)
		: m_nNumRows(0), m_nNumCols(0), m(0)
	{
	}

	CDynamicMatrix(int nRows, int nCols)
		: m(0)
	{
		Resize(nRows, nCols);
	}

	CDynamicMatrix(int nRows, int nCols, T const * const element)
		: m(0)
	{
		Resize(nRows, nCols);
		Initialize(element);
	}

	// copy
	CDynamicMatrix(const CDynamicMatrix& mat)
		: m(0)
	{
		*this = mat;
	}

	const CDynamicMatrix& operator =(const CDynamicMatrix& mat)
	{
		Resize(mat.GetNumRows(), mat.GetNumCols());
		return Initialize(mat.m);
	}

	// coercing
	template <typename T2>
	CDynamicMatrix(const CDynamicMatrix<T2>& mat)
		: m(0)
	{
		*this = mat;
	}

	template <typename T2>
	const CDynamicMatrix& operator =(const CDynamicMatrix<T2>& mat)
	{
		Resize(mat.GetNumRows(), mat.GetNumCols());
		for (int i=0; i<mat.GetNumRows()*mat.GetNumCols(); i++)
			m[i]=mat.ptr()[i];
		return *this;
	}

	// conversion from static version
	template <int nRows, int nCols, typename T2>
	CDynamicMatrix(const CMatrix<nRows,nCols,T2>& mat)
		: m(0)
	{
		*this = mat;
	}

	template <int nRows, int nCols, typename T2>
	const CDynamicMatrix& operator =(const CMatrix<nRows,nCols,T2>& mat)
	{
		Resize(nRows,nCols);
		for (int i=0; i<nRows*nCols; i++)
			m[i]=mat.ptr()[i];
		return *this;
	}

	// destructor
	~CDynamicMatrix(void)
	{
		delete [] m;
	}

	// helpers
	void Resize(int nRows, int nCols)
	{
		_ASSERTE(nRows>=0 && nCols>=0);
		if (m) delete [] m;
		m_nNumRows = nRows;
		m_nNumCols = nCols;
		m = new T [nRows * nCols];
	}

	template <typename T2>
	const CDynamicMatrix& Initialize(T2 const * const element)
	{
		_ASSERTE(m);
//		_ASSERTE(m_nNumRows*m_nNumCols);
		_ASSERTE(element);
		for (int i=0; i<m_nNumRows*m_nNumCols; i++)
			m[i] = element[i];
		return *this;
	}

	template <typename T2>
	void Fill(T2 const e) {
		for (int i=0; i<m_nNumRows*m_nNumCols; i++)
			m[i] = e;
	}

	//---------- accessors
	T& operator()(const int nRow, const int nCol)
	{
		_ASSERTE(nRow<m_nNumRows && nCol<m_nNumCols);
		return m[nRow + m_nNumRows * nCol];
	}

	T const& operator()(const int nRow, const int nCol) const
	{
		_ASSERTE(nRow<m_nNumRows && nCol<m_nNumCols);
		return m[nRow + m_nNumRows * nCol];
	}

	//---------- arithmetic operators
	const CDynamicMatrix& operator +=(const CDynamicMatrix& mat)
	{
		_ASSERTE(m_nNumRows==mat.GetNumRows() && m_nNumCols==mat.GetNumCols());
		for (int i=0; i<m_nNumRows*m_nNumCols; i++)
			m[i] += mat.m[i];
		return *this;
	}

	const CDynamicMatrix& operator -=(const CDynamicMatrix& mat)
	{
		_ASSERTE(m_nNumRows==mat.GetNumRows() && m_nNumCols==mat.GetNumCols());
		for (int i=0; i<m_nNumRows*m_nNumCols; i++)
			m[i] -= mat.m[i];
		return *this;
	}

	const CDynamicMatrix& operator *=(const CDynamicMatrix& mat)
	{
		return *this = *this * mat;
	}

	// "s" may be a self-reference
	const CDynamicMatrix& operator *=(T const s)
	{
		for (int i=0; i<m_nNumRows*m_nNumCols; i++)
			m[i] *= s;
		return *this;
	}

	// "s" may be a self-reference
	template <typename T2>
	const CDynamicMatrix& operator /=(T2 const s)
	{
		_ASSERTE(s != 0);
		for (int i=0; i<m_nNumRows*m_nNumCols; i++) m[i] /= s;
		return *this;
	}

	// unary operator
	CDynamicMatrix operator -(void) const
	{
		return *this * (-1);
	}

	const CDynamicMatrix& operator +(void) const
	{
		return *this;
	}

	// binary operator
	CDynamicMatrix operator +(const CDynamicMatrix& v2) const
	{
		return CDynamicMatrix(*this) += v2;
	}

	CDynamicMatrix operator -(const CDynamicMatrix& v2) const
	{
		return CDynamicMatrix(*this) -= v2;
	}

	CDynamicMatrix operator *(const CDynamicMatrix& mat) const
	{
		_ASSERTE(m_nNumCols==mat.GetNumRows());
		CDynamicMatrix iret(m_nNumRows,mat.GetNumCols());
		for (int nc=0; nc<mat.GetNumCols(); nc++) {
			for (int nr=0; nr<m_nNumRows; nr++) {
				iret(nr,nc)=0;
				for (int ncc=0; ncc<mat.GetNumRows(); ncc++)
					iret(nr,nc)+=(*this)(nr,ncc)*mat(ncc,nc);
			}
		}
		return iret;
	}

	CDynamicMatrix operator *(T const s) const
	{
		return CDynamicMatrix(*this) *= s;
	}

	template <typename T2>
	CDynamicMatrix operator /(T2 const s) const
	{
		return CDynamicMatrix(*this) /= s;
	}

	friend CDynamicMatrix operator *(T const s, const CDynamicMatrix& mat)
	{
		return mat * s;
	}

	//---------- relationship operators
	bool operator == (const CDynamicMatrix& mat2) const
	{
		return
			m_nNumRows == mat2.GetNumRows() &&
			m_nNumCols == mat2.GetNumCols() &&
			!memcmp(m, mat2.m, sizeof (T) * m_nNumRows * m_nNumCols);
	}

	bool operator != (const CDynamicMatrix& mat2) const
	{
		return !(*this == mat2);
	}

	////---------- static methods
	//static CDynamicMatrix GetZero(const int r, const int c)
	//{
	//	CDynamicMatrix iret(r,c);
	//	for (int c=0; c<iret.GetNumCols(); c++)
	//		for (int r=0; r<iret.GetNumRows(); r++)
	//			iret(r,c) = 0;
	//	return iret;
	//}

	//static void GetOne(CDynamicMatrix& iret)
	//{
	//	_ASSERTE(iret.GetNumRows() * iret.GetNumCols());
	//	for (int c=0; c<iret.GetNumCols(); c++)
	//		for (int r=0; r<iret.GetNumRows(); r++)
	//			iret(r,c) = 1;
	//}

	//static CDynamicMatrix GetIdentity(const int row, const int col)
	//{
	//	CDynamicMatrix iret(row, col);
	//	for (int c=0; c<col; c++)
	//		for (int r=0; r<row; r++)
	//				iret(r,c) = (r==c) ? 1 : 0;
	//	return iret;
	//}

	//static CDynamicMatrix GetDiagonal(const CDynamicVector<T>& vec)
	//{
	//	int row = vec.GetNumRows();
	//	CDynamicMatrix iret(row, row);
	//	for (int c=0; c<row; c++)
	//		for (int r=0; r<row; r++)
	//				iret(r,c) = (r==c) ? vec[r] : 0;
	//	return iret;
	//}

	//---------- utilities

	T min(void) const {
		T iret = m[0];
		for (int i=1; i<m_nNumRows*m_nNumCols; i++)
			iret = std::min(iret, m[i]);
		return iret;
	}

	T max(void) const {
		T iret = m[0];
		for (int i=1; i<m_nNumRows*m_nNumCols; i++)
			iret = std::max(iret, m[i]);
		return iret;
	}

	CDynamicMatrix abs(void) const {
		CDynamicMatrix result(m_nNumRows,m_nNumCols);
		for (int i=0; i<m_nNumRows*m_nNumCols; i++)
			result.m[i] = std::abs(m[i]);
		return result;
	}

	T const *ptr(void) const { return m; }

	int GetNumRows(void) const { return m_nNumRows; }

	int GetNumCols(void) const { return m_nNumCols; }

	CDynamicVector<T> GetColumn(int c) const {
		_ASSERTE(!"not implemented");
	}

	CDynamicVector<T> GetRow(int r) const {
		_ASSERTE(!"not implemented");
	}

	CDynamicMatrix GetSubMatrix(int nr, int nc, int sr, int sc) const 
	{
		_ASSERTE(nr<=GetNumRows());
		_ASSERTE(nc<=GetNumCols());
		CDynamicMatrix iret(nr,nc);
		for (int c=0; c<nc; c++)
			for (int r=0; r<nr; r++)
				iret(r,c)=(*this)(sr+r,sc+c);
		return iret;
	}

	CDynamicMatrix AppendCols(const CDynamicMatrix& mat) const 
	{
		_ASSERTE(m_nNumRows == mat.GetNumRows());
		CDynamicMatrix result(m_nNumRows, m_nNumCols + mat.GetNumCols());
		// left half
		int nelem1 = m_nNumRows * m_nNumCols;
		memcpy(result.m, m, sizeof (T) * nelem1);
		// right half
		int nelem2 = mat.GetNumRows() * mat.GetNumCols();
		memcpy(result.m + nelem1, mat.m, sizeof (T) * nelem2);
		return result;
	}

	CDynamicMatrix AppendRows(const CDynamicMatrix& mat) const 
	{
		_ASSERTE(m_nNumCols == mat.GetNumCols());
		CDynamicMatrix result(m_nNumRows + mat.GetNumRows(), m_nNumCols);
		for (int c=0; c<m_nNumCols; c++) {
			for (int r=0; r<m_nNumRows; r++)
				result(r,c) = (*this)(r,c);
			for (int r=0; r<mat.GetNumRows(); r++)
				result(r+m_nNumRows,c) = mat(r,c);
		}
		return result;
	}
};

template <typename T>
class CDynamicVector : public CDynamicMatrix<T>
{
public:
	//---------- constructors
	CDynamicVector(void)
	{
	}

	explicit CDynamicVector(int nDim)
		: CDynamicMatrix<T>(nDim,1)
	{
	}

	CDynamicVector(int nDim, T const * const element)
		: CDynamicMatrix<T>(nDim, 1, element)
	{
	}

	// copy
	CDynamicVector(const CDynamicVector& vec)
		: CDynamicMatrix<T>(vec)
	{
	}

	const CDynamicVector& operator =(const CDynamicVector& vec)
	{
		return CDynamicMatrix<T>::operator =(vec);
	}

	// coercion
	template <typename T2>
	CDynamicVector(const CDynamicMatrix<T2>& vec)
		: CDynamicMatrix<T>(vec)
	{
	}

	template <typename T2>
	const CDynamicVector& operator =(const CDynamicMatrix<T2>& vec)
	{
		return CDynamicMatrix<T>::operator =(vec);
	}

	// helpers
	void Resize(int nRows)
	{
		CDynamicMatrix<T>::Resize(nRows, 1);
	}

	//---------- operators
	T& operator [](const int n)
	{
		_ASSERTE(0<=n && n<CDynamicMatrix<T>::m_nNumRows);
		return CDynamicMatrix<T>::m[n];
	}

	T const& operator [](const int n) const
	{
		_ASSERTE(0<=n && n<CDynamicMatrix<T>::m_nNumRows);
		return CDynamicMatrix<T>::m[n];
	}
};

//---------- helpers for CDynamicMatrix

template <typename T>
inline
CDynamicMatrix<T> GetNormalized(const CDynamicMatrix<T>& vec)
{
	T len = GetNorm2(vec);
	if (len == 0)
	{
		TRACE("warning: attempted to normalize a zero vector.\n");
		return vec;
	}
	else
		return vec / len;
}

template <typename T>
inline
T GetNorm2Squared(const CDynamicMatrix<T>& vec)
{
	return dot(vec, vec);
}

template <typename T>
inline
CDynamicMatrix<T> transpose_of(const CDynamicMatrix<T>& mat)
{
	CDynamicMatrix<T> iret(mat.GetNumCols(), mat.GetNumRows());
	for (int c=0; c<mat.GetNumCols(); c++)
		for (int r=0; r<mat.GetNumRows(); r++)
			iret(c,r) = mat(r,c);
	return iret;
}

//---------- helpers for CDynamicVector

template <typename T>
inline
T GetNorm2(const CDynamicMatrix<T>& vec)
{
	return T(sqrt((double)dot(vec, vec)));
}

template <typename T>
inline
T dot(const CDynamicMatrix<T>& v1, const CDynamicMatrix<T>& v2)
{
	_ASSERTE(v1.GetNumCols()==1 && v2.GetNumCols()==1);
	_ASSERTE(v1.GetNumRows()==v2.GetNumRows());
	T iret=0;
	for (int d=0; d<v1.GetNumRows(); d++)
		iret += v1(d,0) * v2(d,0);
	return iret;
}

template <typename T>
inline
CDynamicMatrix<T> GetSkewSymmetric(CDynamicMatrix<T>& vec)
{
	if (vec.GetNumRows() != 3)
		throw std::runtime_error("not a 3-vector");

	CDynamicMatrix<T> iret(3,3);
	T element[] = {
		0, vec(2,0), -vec(1,0),
		-vec(2,0), 0, vec(0,0),
		vec(1,0), -vec(0,0), 0
	};
	return iret.Initialize(element);
}

} // namespace slib
