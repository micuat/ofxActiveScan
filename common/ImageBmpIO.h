//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageMagickIO.h 4048 2010-06-03 07:51:52Z shun $
//

// This header conflicts: 'ImageMagickIO.h', 'ImageAtlIO.h'
//

#pragma once

#include "BmpUtil.h"
#include "Field.h"

namespace slib
{
namespace image
{

// read a gray-scale image 
template <typename T>
inline
void Read(Field<2,T>& fld, const std::string& filename, float scale)
{
	CBmpImage img;
	if (!img.LoadBmp(filename))
		ThrowRuntimeError("failed to open %s",filename.c_str());

	fld.Initialize(img.GetWidth(), img.GetHeight());

	switch (img.GetNumChannels())
	{
	case 1:
		for (int y=0; y<img.GetHeight(); y++)
			for (int x=0; x<img.GetWidth(); x++)
				fld.cell(x,y) = img.pixel(x,y,0)/255.0f*scale;
		break;
	case 3: // BGR
	case 4: // BGRX
		for (int y=0; y<img.GetHeight(); y++)
			for (int x=0; x<img.GetWidth(); x++)
				fld.cell(x,y) = rgb_to_intensity(img.pixel(x,y,0),img.pixel(x,y,1),img.pixel(x,y,2))/255.0f*scale;
		break;
	default:
		break;
	}
}

#undef rgb_to_intensity

// read an RGB image 
template <typename T>
inline
void Read(Field<2,CVector<3,T> >& fld, const std::string& filename, float scale)
{
	CBmpImage img;
	if (!img.LoadBmp(filename))
		ThrowRuntimeError("failed to open %s",filename.c_str());

	fld.Initialize(img.GetWidth(), img.GetHeight());

	switch (img.GetNumChannels())
	{
	case 1:
		for (int y=0; y<img.GetHeight(); y++)
			for (int x=0; x<img.GetWidth(); x++)
				for (int c=0; c<3; c++)
					fld.cell(x,y)[c] = img.pixel(x,y,0)/255.0f*scale;
		break;
	case 3: // BGR
	case 4: // BGRX
		for (int y=0; y<img.GetHeight(); y++)
			for (int x=0; x<img.GetWidth(); x++)
				for (int c=0; c<3; c++)
					fld.cell(x,y)[c] = img.pixel(x,y,c)/255.0f*scale;
		break;
	default:
		ThrowRuntimeError("unsupported format: %s",filename.c_str());
		break;
	}
}

// write a gray-scale image 
template <typename T>
inline
void Write(const Field<2,T>& fld, const std::string& filename, float maxval)
{
	CBmpImage img;
	img.Initialize(fld.size(0), fld.size(1), 1);

#define clamp(a) std::min<int>(255,std::max<int>(0,(a)))
	for (int y=0; y<img.GetHeight(); y++)
		for (int x=0; x<img.GetWidth(); x++)
			img.pixel(x,y,0)=clamp(fld.cell(x,y) * 255 / maxval);
#undef clamp

	if (!img.WriteBmp(filename))
		ThrowRuntimeError("failed to open %s",filename.c_str());
}

// write an RGB image 
template <typename T>
inline
void Write(const Field<2,CVector<3,T> >& fld, const std::string& filename, float scale)
{
	CBmpImage img;
	img.Initialize(fld.size(0), fld.size(1), 3);

#define clamp(a) std::min<int>(255,std::max<int>(0,(a)))
	for (int y=0; y<img.GetHeight(); y++)
		for (int x=0; x<img.GetWidth(); x++)
		{
			img.pixel(x,y,0)=clamp(fld.cell(x,y)[0] * 255 / scale);
			img.pixel(x,y,1)=clamp(fld.cell(x,y)[1] * 255 / scale);
			img.pixel(x,y,2)=clamp(fld.cell(x,y)[2] * 255 / scale);
		}
#undef clamp

	if (!img.WriteBmp(filename))
		ThrowRuntimeError("failed to open %s",filename.c_str());
}

}
}

#include "ImageIOBase.h"
