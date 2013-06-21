//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageBase.h 4546 2011-05-24 12:20:43Z shun $
//

#pragma once

#include "Field.h"
#include "ColorConv.h"

#undef min
#undef max

namespace slib
{
namespace image
{

inline
float GetIntensity(const float r, const float g, const float b) 
{
	return 0.299*r+0.587*g+0.114*b;
}

inline
float GetIntensity(const CVector<3,float>& p)
{
	return GetIntensity(p[0],p[1],p[2]);
}

inline
void ConvertToRGB(const Field<2,float>& gray, Field<2,CVector<3,float> >& rgb)
{
	rgb.Initialize(gray.size());
	for (int y=0; y<rgb.size(1); y++)
		for (int x=0; x<rgb.size(0); x++)
			rgb.cell(x,y) = make_vector(gray.cell(x,y),gray.cell(x,y),gray.cell(x,y));
}

inline
void ConvertToHueMap(const Field<2,float>& gray, Field<2,CVector<3,float> >& hue)
{
	hue.Initialize(gray.size());
	for (int y=0; y<hue.size(1); y++)
		for (int x=0; x<hue.size(0); x++)
		{
			float r,g,b;
			HSV2RGB<float>(2*M_PI*gray.cell(x,y),1,1,r,g,b);
			hue.cell(x,y)=make_vector(r,g,b);
		}
}

inline
void ConvertToJetMap(const Field<2,float>& gray, Field<2,CVector<3,float> >& hue)
{
	hue.Initialize(gray.size());
	for (int y=0; y<hue.size(1); y++)
		for (int x=0; x<hue.size(0); x++)
		{
			float v = std::min(1.0f, std::max(0.0f, gray.cell(x,y)));
			float r, g, b;
			Scalar2Jet(v, r, g, b);
			hue.cell(x,y)=make_vector(r,g,b);
		}
}

inline
void ConvertToRGBMap(const Field<2,float>& gray, Field<2,CVector<3,float> >& rgb)
{
	rgb.Initialize(gray.size());
	for (int y=0; y<rgb.size(1); y++)
		for (int x=0; x<rgb.size(0); x++)
		{
			float v = gray.cell(x,y);
			float b = std::min(1.0f, std::max(0.0f, -3*v+2));
			float g = std::min(1.0f, std::max(0.0f, std::min(-3*v+3, 3*v-1)));
			float r = std::min(1.0f, std::max(0.0f, 3*v-2));
			rgb.cell(x,y)=make_vector(r,g,b);
		}
}

inline
void ConvertToSignedColor(const Field<2,float>& src, Field<2,CVector<3,float> >& rgb)
{
	rgb.Initialize(src.size());
	rgb.Clear(make_vector(0,0,0));
	for (int y=0; y<rgb.size(1); y++)
		for (int x=0; x<rgb.size(0); x++)
			if (src.cell(x,y)<0)
				rgb.cell(x,y)[0] = -src.cell(x,y);
			else
				rgb.cell(x,y)[1] = src.cell(x,y);
}

inline
void ConvertToGray(const Field<2,CVector<3,float> >& rgb, Field<2,float>& gray)
{
	gray.Initialize(rgb.size());
	for (int y=0; y<gray.size(1); y++)
		for (int x=0; x<gray.size(0); x++)
			gray.cell(x,y) = GetIntensity(rgb.cell(x,y));
}

inline
void ConvertToChannel(const Field<2,CVector<3,float> >& rgb, int ch, Field<2,float>& gray)
{
	gray.Initialize(rgb.size());
	for (int y=0; y<gray.size(1); y++)
		for (int x=0; x<gray.size(0); x++)
			gray.cell(x,y) = rgb.cell(x,y)[ch];
}

inline
void ConvertToMaxChannel(const Field<2,CVector<3,float> >& rgb, Field<2,float>& gray)
{
	gray.Initialize(rgb.size());
	for (int y=0; y<gray.size(1); y++)
		for (int x=0; x<gray.size(0); x++)
			gray.cell(x,y) = std::max(std::max(rgb.cell(x,y)[0],rgb.cell(x,y)[1]),rgb.cell(x,y)[2]);
}

inline
void NormalizeImage(const Field<2,float>& src, Field<2,float>& dst)
{
	float maxval = src.max();
	if (maxval > 0)
		dst = src / maxval;
	else
	{
		char s[256];
		sprintf(s, "[%s] warning: attempting to normalize a non-positive image.\n", __FUNCTION__);
		TRACE(s);
		dst = src;
	}
}

// 'dst' may be alias to 'src'
inline
void NormalizeImage(const Field<2,CVector<3,float> >& src, Field<2,CVector<3,float> >& dst)
{
	float maxpixel = 0;
	for (int y=0; y<src.size(1); y++)
		for (int x=0; x<src.size(0); x++)
			for (int c=0; c<3; c++)
				maxpixel = std::max(maxpixel, std::abs(src.cell(x,y)[c]));

	if (maxpixel==0) {
		char s[256];
		sprintf(s, "[%s] warning: black image.\n", __FUNCTION__);
		TRACE(s);
	}

	dst = src / maxpixel;
}

// legend

inline void AddJetMapLegend(Field<2,CVector<3,float> >& img, float height=0.2, float width=0.02) {
	int size = img.size(1);
	int barwidth  = size*width;
	int barheight = size*height;
	for (int y=0; y<barheight; y++) {
		CVector<3,float> c;
		Scalar2Jet((float)y/barheight, c[0], c[1], c[2]);
		for (int x=0; x<barwidth; x++) {
			if (x==0 || x==barwidth-1 || y==0 || y==barheight-1)
				img.cell(x+barwidth,y+barwidth) = make_vector(1,1,1);
			else
				img.cell(x+barwidth,y+barwidth) = c;
		}
	}
}

inline void AddGrayMapLegend(Field<2,float>& img, float height=0.2, float width=0.02) {
	int size = img.size(1);
	int barwidth  = size*width;
	int barheight = size*height;
	for (int y=0; y<barheight; y++) {
		float c = (float)y/barheight;
		for (int x=0; x<barwidth; x++) {
			if (x==0 || x==barwidth-1 || y==0 || y==barheight-1)
				img.cell(x+barwidth,y+barwidth) = 1;
			else
				img.cell(x+barwidth,y+barwidth) = c;
		}
	}
}

} // image
} // slib
