//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ColorConv.h 4316 2010-12-08 23:48:35Z shun $
//

#pragma once

#ifndef M_PI
#define _USE_MATH_DEFINES
#include <cmath>
#endif

#ifndef _ASSERTE
#include <cassert>
#define _ASSERTE assert
#endif

#define min2(a,b) (((a)<(b))?(a):(b))
#define min3(a,b,c) min2((a),min2((b),(c)))

#define max2(a,b) (((a)>(b))?(a):(b))
#define max3(a,b,c) max2((a),max2((b),(c)))

namespace slib
{
namespace image
{

// Refer to: 
// http://en.wikipedia.org/wiki/Color_space 
// http://ja.wikipedia.org/wiki/%E8%89%B2%E7%A9%BA%E9%96%93
// http://www.equasys.de/colorconversion.html

// Y, Luminance, Intensity
template<typename T>
inline
T RGB2Luminance(T r, T g, T b)
{
	_ASSERTE(0<=r && r<=1);
	_ASSERTE(0<=g && g<=1);
	_ASSERTE(0<=b && b<=1);

	return 0.299 * r + 0.587 * g + 0.114 * b;
}

// YUV
// for analog PAL/NTSC signal, not for any digital video
template<typename T>
inline
void RGB2YUV(T r, T g, T b, T& y, T& u, T& v)
{
	_ASSERTE(0<=r && r<=1);
	_ASSERTE(0<=g && g<=1);
	_ASSERTE(0<=b && b<=1);

	y =  0.299 * r + 0.587 * g + 0.114 * b;
	u = -0.147 * r - 0.289 * g + 0.436 * b;
	v =  0.615 * r - 0.515 * g - 0.100 * b;
}

template<typename T>
inline
void YUV2RGB(T y, T u, T v, T& r, T& g, T& b)
{
	_ASSERTE(0<=y && y<=1);
	_ASSERTE(-0.436<=u && u<=0.436);
	_ASSERTE(-0.615<=v && v<=0.615);

	r = y + 1.140 * v;
	g = y - 0.395 * u - 0.581 * v;
	b = y + 2.032 * u;
}

// HSV, HSB
template <typename T>
inline
void RGB2HSV(T r, T g, T b, T &h, T &s, T &v)
{
	_ASSERTE(0<=r && r<=1);
	_ASSERTE(0<=g && g<=1);
	_ASSERTE(0<=b && b<=1);

	v = max3(r, g, b);
	T diff = v - min3(r, g, b);

	if (v != 0)
		s = diff / v;
	else
		s = 0;

	if (diff == 0)
		h = 0;
	else
	{
		if (r == v) // r is max
			h = M_PI / 3 * (g-b)/diff;
		else if (g == v) // g is max
			h = M_PI / 3 * ((b-r)/diff+2);
		else // b is max
			h = M_PI / 3 * ((r-g)/diff+4);

		while (h < 0)
			h += 2 * M_PI;
	}
}

template <typename T>
inline
void HSV2RGB(T h, T s, T v, T &r, T &g, T &b)
{
	h = fmod(h, T(2*M_PI));
	while (h < 0)
		h += 2 * M_PI;
	_ASSERTE(0<=s && s<= 1);
	_ASSERTE(0<=v && v<= 1);

	if (s == 0)
	{
		r = v;
		g = v;
		b = v;
		return;
	}

	int hi = (int)(h/(M_PI/3)) % 6;
	float f = h/(M_PI/3) - hi;

	float p = v * (1 - s);
	float q = v * (1 - f*s);
	float t = v * (1 - (1-f)*s);
	switch (hi)
	{
	case 0:
		r = v, g = t, b = p;
		break;
	case 1:
		r = q, g = v, b = p;
		break;
	case 2:
		r = p, g = v, b = t;
		break;
	case 3:
		r = p, g = q, b = v;
		break;
	case 4:
		r = t, g = p, b = v;
		break;
	case 5:
	default:
		r = v, g = p, b = q;
		break;
	}
}

template <typename T>
inline
void GetHueColor(T const v, T& r, T& g, T& b)
{
	_ASSERTE(0<=v && v<=1);

	HSV2RGB<T>(2*M_PI*v,1,1,r,g,b);
}

template <typename T>
inline
void Scalar2Jet(T const v, T& r, T& g, T& b)
{
	_ASSERTE(0<=v && v<=1);

	if (v >= 0.8333)
	{
		r = 1.0-4*(v-0.8333);
		g=b=0;
	}
	else if (v < 0.1666)
	{
		b = 1.0+4*(v-0.1666);
		r=g=0;
	}
	else
	{
		//     [ blue,  red  ]
		// v = [0.1666,0.8333] =>
		// h = [4/3*pi,0.0000] 
		float h = (1.0-(v-0.1666)/0.6666)*4/3*M_PI;
		HSV2RGB<float>(h,1,1,r,g,b);
	}
}

#undef min2
#undef min3
#undef max2
#undef max3

} // image
} // slib

