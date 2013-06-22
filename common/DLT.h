//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: DLT.h 4577 2011-05-27 14:35:11Z shun $
//

#pragma once

#include <cmath>

namespace slib
{
namespace dlt
{

// ----------------------------------------------------------------------
// multiple view geometry
// ----------------------------------------------------------------------

template <int nDimension, typename T>
inline void normalize(
	const std::vector<CVector<nDimension,T> >& src,
	std::vector<CVector<nDimension,T> >& dst,
	T& scale,
	CVector<nDimension,T>& center)
{
	int num_data = src.size();
	dst.resize(num_data);

	center.Fill(0);
	for (int i=0; i<num_data; i++)
		center += src[i];
	center /= src.size();

	T sum = 0;
	for (int i=0; i<num_data; i++)
	{
		dst[i] = src[i] - center;
		sum += GetNorm2(dst[i]);
	}
	scale = (float)num_data / sum * sqrt(double(nDimension));

	for (int i=0; i<num_data; i++)
		dst[i] *= scale;
}

template <int nDimension, typename T>
inline void normalize(
	const std::vector<CVector<nDimension,T> >& src,
	std::vector<CVector<nDimension,T> >& dst,
	CMatrix<nDimension+1,nDimension+1,T>& trans)
{
	T scale;
	CVector<nDimension,T> center;
	normalize(src,dst,scale,center);
	trans.Fill(0); 
	for (int d=0; d<nDimension; d++) 
	{
		trans(d,d)=scale;
		trans(d,nDimension)=-scale*center[d];
	}
	trans(nDimension,nDimension)=1;
}

template <int nDimension, typename T>
inline void normalize_anisotropic(
	const std::vector<CVector<nDimension,T> >& src,
	std::vector<CVector<nDimension,T> >& dst,
	CVector<nDimension,T>& scale,
	CVector<nDimension,T>& center)
{
	int num_data = src.size();
	dst.resize(num_data);

	center.Fill(0);
	for (int i=0; i<num_data; i++)
		center += src[i];
	center /= src.size();

	CVector<nDimension,T> sum;
	sum.Fill(0);
	for (int i=0; i<num_data; i++)
	{
		dst[i] = src[i] - center;
		for (int d=0; d<nDimension; d++)
			sum[d] += std::abs(dst[i][d]);
	}
	for (int d=0; d<nDimension; d++)
		scale[d] = (float)num_data / sum[d];

	for (int i=0; i<num_data; i++)
		for (int d=0; d<nDimension; d++)
			dst[i][d] *= scale[d];
}

template <int nDimension, typename T>
inline void normalize_anisotropic(
	const std::vector<CVector<nDimension,T> >& src,
	std::vector<CVector<nDimension,T> >& dst,
	CMatrix<nDimension+1,nDimension+1,T>& trans)
{
	CVector<nDimension,T> scale;
	CVector<nDimension,T> center;
	normalize_anisotropic(src,dst,scale,center);
	trans.Fill(0);
	trans(nDimension,nDimension) = 1;
	for (int d=0; d<nDimension; d++) 
	{
		trans(d,d)=scale[d];
		trans(d,nDimension)=-scale[d]*center[d];
	}
}

} // namespace dlt
} // namespace slib
