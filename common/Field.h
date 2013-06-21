//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: Field.h 4567 2011-05-26 16:02:30Z shun $
//

#pragma once

#include "MathBase.h"

#undef min
#undef max

namespace slib
{

// N-dimensional array
template <int nDimension, typename T>
class Field
{
public:
	// constructor
	Field(void)
	: m_cell(0)
	{
		m_size.Fill(0);
	}

	explicit Field(CVector<nDimension,int> const& size)
	: m_cell(0)
	{
		Initialize(size);
	}

	Field(int x, int y) 
	: m_cell(0) 
	{
		Initialize(make_vector(x,y)); 
	}

	Field(int x, int y, int z) 
	: m_cell(0) 
	{
		Initialize(make_vector(x,y,z));
	}

	Field(CVector<nDimension,int> const& size, T const& val)
	: m_cell(0)
	{
		Initialize(size);
		Clear(val);
	}

	Field(Field const& fld)
	: m_cell(0)
	{
		*this = fld;
	}

	template <typename T2>
	explicit Field(const Field<nDimension,T2>& fld)
	: m_cell(0)
	{
		*this = fld;
	}

	Field(const std::string& filename)
	: m_cell(0)
	{
		Read(filename);
	}

	// initializer
	void Initialize(const CVector<nDimension,int>& size)
	{
		if (m_size==size)
			return;

		Invalidate();

		m_size = size;
		m_cell = new T [GetSizeOfArray()];
	}

	void Initialize(int x, int y)
	{
		Initialize(make_vector(x,y)); 
	}

	void Initialize(int x, int y, int z)
	{
		Initialize(make_vector(x,y,z)); 
	}

	void Clear(T const& val)
	{
		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			m_cell[i] = val;
	}

	// destructor
	~Field()
	{
		Invalidate();
	}

	void Invalidate()
	{
		delete [] m_cell;
		m_cell = 0;
		m_size.Fill(0);// = CVector<nDimension,int>::GetZero();
	}

	// assignment 
	const Field & operator =(const Field& fld)
	{
		if ((void *)&fld != (void *)this)
			Copy(fld);
		return *this;
	}

	template <typename T2>
	const Field & operator =(const Field<nDimension,T2> & fld)
	{
		Copy(fld);
		return *this;
	}

	template <typename T2>
	const Field& operator +=(const Field<nDimension,T2>& fld)
	{
		if (m_size!=fld.m_size)
			throw std::runtime_error("[" __FUNCTION__ "] operation not allowed");

		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			m_cell[i] += fld.m_cell[i];

		return *this;
	}

	template <typename T2>
	const Field& operator -=(const Field<nDimension,T2>& fld)
	{
		if (m_size!=fld.size())
			throw std::runtime_error("size mismatched in " __FUNCTION__);

		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			m_cell[i] -= fld.ptr(i);

		return *this;
	}

	const Field& operator *=(double const s)
	{
		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			m_cell[i] *= s;
		return *this;
	}

	const Field& operator *=(const Field& fld)
	{
		if (m_size!=fld.size())
			throw std::runtime_error("size mismatched in " __FUNCTION__);
		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			m_cell[i] *= fld.ptr(i);

		return *this;
	}

	const Field& operator /=(double const s)
	{
		_ASSERTE(s!=0);
		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			m_cell[i] /= s;

		return *this;
	}

	// operator 
	const Field<nDimension,T> operator +(const Field<nDimension,T>& f2) const
	{
		return Field<nDimension,T>(*this)+=f2;
	}

	const Field<nDimension,T> operator -(const Field<nDimension,T>& f2) const
	{
		return Field<nDimension,T>(*this)-=f2;
	}

	const Field<nDimension,T> operator *(const double s) const
	{
		return Field<nDimension,T>(*this)*=s;
	}

	friend const Field<nDimension,T> operator *(const double s, const Field<nDimension,T>& f1)
	{
		return Field<nDimension,T>(f1)*=s;
	}

	const Field<nDimension,T> operator /(const double s) const
	{
		return Field<nDimension,T>(*this)/=s;
	}

	// accessor
	T const *ptr() const { return m_cell; }

	T const& ptr(int i) const { return m_cell[i]; }

	const CVector<nDimension,int>& size(void) const
	{
		return m_size;
	}

	int size(int n) const
	{
		_ASSERTE(0<=n && n<nDimension);
		return m_size[n];
	}

	T& cell(const CVector<nDimension,int>& pos)
	{
		_ASSERTE(IsInside(pos));
		int nArrayPos = pos[nDimension-1];
		for (int d=nDimension-2; d>=0; d--)
			nArrayPos = pos[d] + m_size[d] * nArrayPos;
		return m_cell[nArrayPos];
	}

	T const& cell(const CVector<nDimension,int>& pos) const
	{
		_ASSERTE(IsInside(pos));
		int nArrayPos = pos[nDimension-1];
		for (int d=nDimension-2; d>=0; d--)
			nArrayPos = pos[d] + m_size[d] * nArrayPos;
		return m_cell[nArrayPos];
	}

	T& cell(int x, int y)     
	{ 
		return cell(make_vector(x,y));
	}

	T const& cell(int x, int y) const 
	{
		return cell(make_vector(x,y));
	}

	T& cell(int x, int y, int z)       
	{
		return cell(make_vector(x,y,z));
	}

	T const& cell(int x, int y, int z) const 
	{
		return cell(make_vector(x,y,z));
	}

	// helper 
	bool IsInside(const CVector<nDimension,float>& pos)const
	{
		for (int i = 0; i < nDimension; i++)
			if (pos[i] < 0 || pos[i] > m_size[i] - 1)
				return false;
		return true;
	}

	bool IsInside(const float x, const float y) const 
	{
		return IsInside(make_vector(x,y)); 
	}

	bool IsInside(const float x, const float y, const float z) const 
	{ 
		return IsInside(make_vector(x,y,z)); 
	}

	bool IsRegular(void) const
	{
#define is_power_of_two(x) (!((x) & ((x)-1)));
		for (int i = 0; i < nDimension; i++)
			if (!is_powers_of_two(m_size[i]))
				return false;
		return true;
#undef is_power_of_two
	}

	T min(void) const {
		T iret = m_cell[0];
		int nSizeArray = GetSizeOfArray();
		for (int i=1; i<nSizeArray; i++)
			iret = std::min(iret, m_cell[i]);
		return iret;
	}

	T max(void) const {
		T iret = m_cell[0];
		int nSizeArray = GetSizeOfArray();
		for (int i=1; i<nSizeArray; i++)
			iret = std::max(iret, m_cell[i]);
		return iret;
	}

	T sum(void) const {
		T iret = m_cell[0];
		int nSizeArray = GetSizeOfArray();
		for (int i=1; i<nSizeArray; i++)
			iret += m_cell[i];
		return iret;
	}

	T average(void) const {
		return sum() / GetSizeOfArray();
	}

	// file
	void Read(const std::string& filename)
	{
		TRACE("fld <= %s\n", filename.c_str());
		FILE *fr = fopen(filename.c_str(), "rb");
		if (!fr)
			throw std::runtime_error(format("failed to open '%s'", filename.c_str()));

		CVector<nDimension,int>size;
		for (int i = 0; i < nDimension; i++)
			fscanf(fr, "%d", &size[i]);
		char buf[BUFSIZ];
		fgets(buf, BUFSIZ, fr);
		Initialize(size);

		int nSizeArray = m_size[0];
		for (int d=1; d<nDimension; d++)
			nSizeArray *= m_size[d];
		if (fread((void *)m_cell, sizeof(T), nSizeArray, fr) != nSizeArray)
		{
			fclose(fr);
			throw std::runtime_error(format("failed to read all data in '%s'", filename.c_str()));
		}

		fclose(fr);
	}

	void Write(const std::string& filename) const
	{
		TRACE("fld => %s\n", filename.c_str());
		FILE *fw = fopen(filename.c_str(), "wb");
		if (!fw)
			throw std::runtime_error(format("failed to open '%s'", filename.c_str()));

		for (int i = 0; i < nDimension; i++)
			fprintf(fw, "%d ", m_size[i]);
		fprintf(fw, "\n");

		int nSizeArray = m_size[0];
		for (int d=1; d<nDimension; d++)
			nSizeArray *= m_size[d];
		if (fwrite((void *)m_cell, sizeof(T), nSizeArray, fw) != nSizeArray)
		{
			fclose(fw);
			throw std::runtime_error(format("failed to write data to '%s'", filename.c_str()));
		}

		fclose(fw);
	}

	// return the total number of elements
	int GetSizeOfArray() const
	{
		int iret = m_size[0];
		for (int d=1; d<nDimension; d++)
			iret *= m_size[d];
		return iret;
	}

	// iteration
	template <typename function_t>
	void ForEach(function_t callback)
	{
		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			callback(m_cell[i]);
	}

	// iteration
	template <typename function_t>
	void ForEach(function_t callback) const
	{
		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			callback(m_cell[i]);
	}

private:
	// copy internal objects
	template <typename T2>
	void Copy(const Field<nDimension,T2> & fld)
	{
		Initialize(fld.size());

		int nSizeArray = GetSizeOfArray();
		for (int i = 0; i < nSizeArray; i++)
			m_cell[i] = fld.ptr(i);
	}

protected:
	T *m_cell;				// array of data in ascending-dimension order
	CVector<nDimension,int> m_size;		// size of data
};

// interpolate values using adjacent elements
template <int nDimension,typename T,typename float_t>
inline
T GetInterpolatedCell(const Field<nDimension,T>& fld, const CVector<nDimension,float_t>& pos) 
{
	if (!fld.IsInside(pos))
		throw std::runtime_error("[" __FUNCTION__ "] out of bound");

	T val = T();
	CVector<nDimension,int> gridpos;
	CollectAdjacentCells(fld, nDimension - 1, 1, gridpos, pos, val);
	return val;
}

// collect adjacent elements
template <int nDimension,typename T,typename float_t>
inline
void CollectAdjacentCells(const Field<nDimension,T>& fld, int dim, float ratio, CVector<nDimension,int>& gridpos, const CVector<nDimension,float_t>& pos, T& val) 
{
	if (dim >= 0)
	{
		int lower = pos[dim];
		float r = pos[dim]-lower;

		gridpos[dim] = lower;
		if (ratio*(1-r) > 0)
			CollectAdjacentCells(fld, dim-1, ratio*(1-r), gridpos, pos, val);

		gridpos[dim] = lower + 1;
		if (ratio*r > 0)
			CollectAdjacentCells(fld, dim-1, ratio*r,     gridpos, pos, val);
	}
	else
	{
		val += fld.cell(gridpos) * ratio;
	}
}

}								// slib
