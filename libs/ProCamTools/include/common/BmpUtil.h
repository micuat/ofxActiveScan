//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: BmpUtil.h 4551 2011-05-25 17:11:53Z shun $
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

#include <stdexcept>
#include <string.h>

namespace slib
{
namespace image
{

namespace {
inline 
float rgb_to_intensity(float r,float g,float b) { return 0.299*r+0.587*g+0.114*b; }
}

//--------------------------------------------------------------------
// color space conversion for an image
//--------------------------------------------------------------------

// src can be identical to dst. 
// pixels must be de-aligned.
inline 
void Gray2Rgb(
    const int wh,
    const unsigned char *const src,
    unsigned char *const dst)
{
    for (int i = wh - 1; i > -1; i--)
    {
        dst[3 * i + 0] = src[i];
        dst[3 * i + 1] = src[i];
        dst[3 * i + 2] = src[i];
    }
}

// src can be identical to dst. 
// pixels must be de-aligned.
inline void Gray2Rgba(
    const int wh,
    const unsigned char *const src,
    unsigned char *const dst)
{
    for (int i = wh - 1; i > -1; i--)
    {
        dst[4 * i + 0] = src[i];
        dst[4 * i + 1] = src[i];
        dst[4 * i + 2] = src[i];
        dst[4 * i + 3] = 255;
    }
}

// src can be identical to dst. 
// pixels must be de-aligned.
inline void Rgb2Gray(
    const int wh,
    const unsigned char *const src,
    unsigned char *const dst)
{
    for (int i = 0; i < wh; i++)
        dst[i] = rgb_to_intensity(src[3 * i + 0], src[3 * i + 1], src[3 * i + 2]);
}

// src can be identical to dst. 
// pixels must be de-aligned.
inline void Rgb2Rgba(
    const int wh,
    const unsigned char *const src,
    unsigned char *const dst)
{
    for (int i = wh - 1; i > -1; i--)
    {
        dst[4 * i + 0] = src[3 * i + 0];
        dst[4 * i + 1] = src[3 * i + 1];
        dst[4 * i + 2] = src[3 * i + 2];
        dst[4 * i + 3] = 255;
    }
}

// src can be identical to dst. 
// pixels must be de-aligned.
inline void Rgba2Gray(
    const int wh,
    const unsigned char *const src,
    unsigned char *const dst)
{
    for (int i = 0; i < wh; i++)
        dst[i] = rgb_to_intensity(src[4 * i + 0], src[4 * i + 1], src[4 * i + 2]);
}

// src can be identical to dst. 
// pixels must be de-aligned.
inline void Rgba2Rgb(
    const int wh,
    const unsigned char *const src,
    unsigned char *const dst)
{
    for (int i = 0; i < wh; i++)
    {
        dst[3 * i + 0] = src[4 * i + 0];
        dst[3 * i + 1] = src[4 * i + 1];
        dst[3 * i + 2] = src[4 * i + 2];
    }
}

//--------------------------------------------------------------------
// data alignment for windows bitmap
//--------------------------------------------------------------------

// src can be identical to dst.
inline void AlignDword(
    const int w,
    const int h,
    const int ch,
    const unsigned char *const src,
    unsigned char *const dst)
{
    int pad = (4 - ((w * ch) % 4)) % 4;

    for (int j = h-1; j >= 0; j--)
		for (int i = w*ch-1; i >= 0; i--)
			*(dst + (w*ch + pad) * j + i) = *(src + w*ch * j + i);
}

// src can be identical to dst.
inline void DealignDword(
    const int w,
    const int h,
    const int ch,
    const unsigned char *const src,
    unsigned char *const dst)
{
    int pad = (4 - ((w * ch) % 4)) % 4;

    for (int j = 0; j < h; j++)
		for (int i = 0; i < w*ch; i++)
			*(dst + w*ch*j + i) = *(src + (pad+w*ch)*j + i);
}

// src can be identical to dst.
// pixels must be de-aligned.
inline void SwapRB(
    const int w,
    const int h,
    const int ch,
    const unsigned char *const src,
    unsigned char *const dst)
{
    if (ch == 1)
        return;
	_ASSERTE(ch == 3 || ch == 4);

	for (int y=0; y<h; y++)
		for (int x=0; x<w; x++)
		{
			int idx = ch * (x + w * y);
			unsigned char r = src[idx];
			dst[0 + idx] = src[2 + idx];
			dst[1 + idx] = src[1 + idx];
			dst[2 + idx] = r;
			if (ch > 3)
				dst[3 + idx] = src[3 + idx];
		}
}

// src can be identical to dst.
inline void FlipVertically(
    const int line,
    const int h,
    const unsigned char *const src,
    unsigned char *const dst)
{
    unsigned char *tmp = new unsigned char[line];

    for (int i = 0; i < h / 2; i++)
    {
        memcpy(tmp, src + line * i, line);
        memcpy(dst + line * i, src + line * (h - i - 1), line);
        memcpy(dst + line * (h - i - 1), tmp, line);
    }

    delete [] tmp;
}

// src can be identical to dst.
inline void GetBmpPixelsFromPpmPixels(
    const unsigned int nWidth,
    const unsigned int nHeight,
    const unsigned int nChannel,
    const unsigned char *pucPpmPixel,
    unsigned char *pucBmpPixel)
{
    _ASSERTE(nChannel == 1 || nChannel == 3);

    int pad = (4 - ((nWidth * nChannel) % 4)) % 4;
    unsigned int nNumBmpPixels = (nWidth * nChannel + pad) * nHeight;
    memcpy(pucBmpPixel, pucPpmPixel, nWidth*nHeight*nChannel);

    SwapRB(nWidth, nHeight, nChannel, pucBmpPixel, pucBmpPixel);
    FlipVertically(nWidth*nChannel, nHeight, pucBmpPixel, pucBmpPixel);
    AlignDword(nWidth, nHeight, nChannel, pucBmpPixel, pucBmpPixel);
}

// a new memory is allocated for possible larger data
inline unsigned char *AllocateBmpPixelsFromPpmPixels(
    const int nWidth,
    const int nHeight,
    const int nChannel,
    const unsigned char *pucPpmPixel)
{
    _ASSERTE(nChannel == 1 || nChannel == 3);
    int pad = (4 - ((nWidth * nChannel) % 4)) % 4;
    unsigned long nNumBmpPixels = (nWidth * nChannel + pad) * nHeight;
    unsigned char *iret = new unsigned char [nNumBmpPixels];
    GetBmpPixelsFromPpmPixels(nWidth, nHeight, nChannel, pucPpmPixel, iret);
    return iret;
}

//--------------------------------------------------------------------
// utilities for windows DIB
//--------------------------------------------------------------------

// initialize a BITMAPINFO object
inline void GetBitmapInfo(
    const int nWidth,
    const int nHeight,
    const int nChannel,
	unsigned char * const pucBitmapInfo)
{
    _ASSERTE(nChannel == 1 || nChannel == 3 || nChannel == 4);
    _ASSERTE(pucBitmapInfo);

    unsigned long colors = 0;
    if (nChannel == 1) colors = 256;

    unsigned long offset = 40 + 4 * colors;
    int pad = (4 - ((nWidth * nChannel) % 4)) % 4;
    unsigned long size = (nWidth * nChannel + pad) * nHeight + offset;

    unsigned char *s = (unsigned char *) & size;
    unsigned char *w = (unsigned char *) & nWidth;
    unsigned char *h = (unsigned char *) & nHeight;
    unsigned char *o = (unsigned char *) & offset;
    unsigned char *c = (unsigned char *) & colors;

    unsigned char head[] =
        {
            40, 0, 0, 0,
            w[0], w[1], w[2], w[3],
            h[0], h[1], h[2], h[3],
            1, 0,
            nChannel * 8, 0,
            0, 0, 0, 0, // compression
            0, 0, 0, 0, // size
            1, 0, 0, 0, // x / meter
            1, 0, 0, 0, // y / meter
            c[0], c[1], c[2], c[3], // color used
            c[0], c[1], c[2], c[3], // color important
        };

	memcpy(pucBitmapInfo, head, 40);
    if (nChannel == 1)
        for (int i = 0; i < 256; i++)
        {
            unsigned char uc[] = { i, i, i, i, };
            memcpy(pucBitmapInfo+40+4*i, uc, 4);
        }
}

/*
// allocate a BITMAPINFO object
inline unsigned char *AllocateBitmapInfo(
    const int nWidth,
    const int nHeight,
    const int nChannel
	)
{
    _ASSERTE(nChannel == 1 || nChannel == 3 || nChannel == 4);
    unsigned long colors = 0;
    if (nChannel == 1) colors = 256;
    unsigned long offset = 40 + 4 * colors;
	unsigned char *iret = new unsigned char [offset];
	GetBitmapInfo(nWidth, nHeight, nChannel, iret);

	return iret;
}
*/

//--------------------------------------------------------------------
// conversion between .BMP and .PPM format
//--------------------------------------------------------------------

// src can be identical to dst.
inline void GetPpmPixelsFromBmpPixels(
    const int nWidth,
    const int nHeight,
    const int nChannel,
    const unsigned char *pucBmpPixel,
    unsigned char *pucPpmPixel)
{
    _ASSERTE(nChannel == 1 || nChannel == 3);

    DealignDword(nWidth, nHeight, nChannel, pucBmpPixel, pucPpmPixel);
    FlipVertically(nWidth*nChannel, nHeight, pucPpmPixel, pucPpmPixel);
    SwapRB(nWidth, nHeight, nChannel, pucPpmPixel, pucPpmPixel);
}

// allocated memory must be relased in the calling function.
inline unsigned char *AllocatePpmPixelsFromBmpPixels(
    const int nWidth,
    const int nHeight,
    const int nChannel,
    const unsigned char *pucBmpPixel)
{
    _ASSERTE(nChannel == 1 || nChannel == 3);
    unsigned char *iret = new unsigned char [nWidth * nHeight * nChannel];
    GetPpmPixelsFromBmpPixels(nWidth, nHeight, nChannel, pucBmpPixel, iret);
    return iret;
}

//--------------------------------------------------------------------
// file IO
//--------------------------------------------------------------------

// write ppm
template <typename pixel_t>
inline int WritePpm(
    const char *const file,
    const unsigned int width,
    const unsigned int height,
    const unsigned int ch,
    const pixel_t *const p,
	float scale = 1)
// pixels must be:
// (1) packed (i.e. not DWORD aligned)
// (2) pixel format is Gray or RGB
// (3) stored in top-to-bottom order
{
    _ASSERTE(ch == 1 || ch == 3);

    if (ch != 1 && ch != 3)
        return -1;

    FILE *fw = fopen(file, "wb");
    if (!fw)
	{
        TRACE("error: couldn't open %s\n", file);
        return -1;
	}

    if (ch == 1)
        fprintf(fw, "P5\n");
    else
        fprintf(fw, "P6\n");
    fprintf(fw, "%d %d\n255\n", width, height);

	pixel_t maxpixel=0;
	if (scale < 0)
	{
		for (unsigned int y=0; y<height; y++)
			for (unsigned int x=0; x<width; x++)
				for (unsigned int c=0; c<ch; c++)
					maxpixel = std::max(maxpixel, p[c + ch * (x + width * y)]);
		scale = 255.0f / maxpixel;
	}

	for (unsigned int y=0; y<height; y++)
		for (unsigned int x=0; x<width; x++)
			for (unsigned int c=0; c<ch; c++)
				fputc((int)(scale * p[c + ch * (x + width * y)]), fw);

    fclose(fw);

    return 0;
}

// load ppm
template <typename pixel_t>
inline int ReadPpm(
    const char *const file,
    int *const w,
    int *const h,
    short *const ch,
    pixel_t **const p,
	pixel_t scale = 1
	)
{
    FILE *fr = fopen(file, "rb");
    if (!fr)
	{
        TRACE("error: couldn't open %s\n", file);
		return -1;
	}

    char buf[BUFSIZ];
    fgets(buf, BUFSIZ, fr);
    if (!strncmp(buf, "P6", 2))
        *ch = 3;
    else if (!strncmp(buf, "P5", 2))
        *ch = 1;
    else
    {
        fclose(fr);
        return -1;
    }

    while (fgets(buf, BUFSIZ, fr) && buf[0] == '#');
    sscanf(buf, "%d%d", w, h);
    while (fgets(buf, BUFSIZ, fr) && buf[0] == '#');
    int nNumColors;
    sscanf(buf, "%d", &nNumColors);

    int nSize = (*w) * (*h) * (*ch);
    *p = new pixel_t[nSize];

	for (int y=0; y<*h; y++)
		for (int x=0; x<*w; x++)
			for (int c=0; c<*ch; c++)
			{
				(*p)[c + *ch * (x + *w * y)] = scale * fgetc(fr);
				if ((*p)[c + *ch * (x + *w * y)] < 0)
					return -1;
			}

    fclose(fr);
	return 0;
}

// write bmp
inline int WriteBmp(
    const char *const file,
    const unsigned long width,
    const unsigned long height,
    const int ch,
    const unsigned char *const p)
// pixels must be:
// (1) DWORD aligned,
// (2) pixel format is Gray, BGR or BGRA,
// (3) stored in bottom-to-top order.
{
    _ASSERTE(ch == 1 || ch == 3 || ch == 4);
    unsigned long colors = 0;
    if (ch == 1) colors = 256;

    unsigned long offset = 54 + 4 * colors;
    int pad = (4 - ((width * ch) % 4)) % 4;
    unsigned long size = (width * ch + pad) * height + offset;
	unsigned long scale = 725669/256;
    unsigned char *s = (unsigned char *) & size;
    unsigned char *x = (unsigned char *) & scale;
    unsigned char *w = (unsigned char *) & width;
    unsigned char *h = (unsigned char *) & height;
    unsigned char *o = (unsigned char *) & offset;
    unsigned char *c = (unsigned char *) & colors;

    unsigned char head[] =
	{
		'B', 'M',
		s[0], s[1], s[2], s[3],
		0, 0,
		0, 0,
		o[0], o[1], o[2], o[3],
		40, 0, 0, 0,
		w[0], w[1], w[2], w[3],
		h[0], h[1], h[2], h[3],
		1, 0,
		ch * 8, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		x[0], x[1], x[2], x[3],
		x[0], x[1], x[2], x[3],
		c[0], c[1], c[2], c[3],
		c[0], c[1], c[2], c[3],
	};

    FILE *fw = fopen(file, "wb");
    if (!fw)
    {
        TRACE("error: couldn't open %s\n", file);
        return -1;
    }

    fwrite(head, sizeof(head), 1, fw);

    if (ch == 1)
        for (int i = 0; i < 256; i++)
        {
            unsigned char uc[] = { i, i, i, i, };
            fwrite(uc, 1, 4, fw);
        }

    fwrite(p, 1, (width * ch + pad) * height, fw);
    fclose(fw);
    return 0;
}

// write bmp to memory (allocate memory mapped dib)
inline int WriteBmpToMemory(
    const unsigned int width,
    const unsigned int height,
    const int ch,
    const unsigned char *const pucBmpPixels,
    unsigned char ** const ppucDib,
    unsigned long * const pnDibSize)
// pixels must be:
// (1) DWORD aligned,
// (2) pixel format is Gray, BGR or BGRA,
// (3) stored in bottom-to-top order.
{
    _ASSERTE(pucBmpPixels);
    _ASSERTE(ch == 1 || ch == 3 || ch == 4);
    _ASSERTE(ppucDib);

    unsigned long colors = 0;
    if (ch == 1) colors = 256;

    unsigned long offset = 54 + 4 * colors;
    int pad = (4 - ((width * ch) % 4)) % 4;
    int nDibSize = (width * ch + pad) * height + offset;

    unsigned char *s = (unsigned char *)nDibSize;
    unsigned char *w = (unsigned char *) & width;
    unsigned char *h = (unsigned char *) & height;
    unsigned char *o = (unsigned char *) & offset;
    unsigned char *c = (unsigned char *) & colors;

	// BITMAP
    unsigned char head[] =
        {
            'B', 'M',
            s[0], s[1], s[2], s[3],
            0, 0,
            0, 0,
            o[0], o[1], o[2], o[3],
            40, 0, 0, 0,
            w[0], w[1], w[2], w[3],
            h[0], h[1], h[2], h[3],
            1, 0,
            ch * 8, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            1, 0, 0, 0,
            1, 0, 0, 0,
            c[0], c[1], c[2], c[3],
            c[0], c[1], c[2], c[3],
        };

    (*ppucDib) = new unsigned char [nDibSize];
    unsigned char *ptr = (*ppucDib);
    memcpy(ptr, head, sizeof(head));
    ptr += sizeof(head);
    if (ch == 1)
        for (int i = 0; i < 256; i++)
        {
            unsigned char uc[] = { i, i, i, i, };
            memcpy(ptr, uc, 4);
            ptr += 4;
        }
    memcpy(ptr, pucBmpPixels, (width * ch + pad) * height);

	if (pnDibSize)
		*pnDibSize = nDibSize;
    return 0;
}

// load bmp
inline int ReadBmp(
    const char *const file,
    int *const w,
    int *const h,
    short *const ch,
    unsigned char **const p)
{
    FILE *fr = fopen(file, "rb");
    if (!fr)
	{
        TRACE("error: couldn't open %s\n", file);
		return -1;
	}

    fseek(fr, 10, SEEK_CUR);
    unsigned long offset;
    fread(&offset, 4, 1, fr);
    fseek(fr, 4, SEEK_CUR);
    fread(w, 4, 1, fr);
    fread(h, 4, 1, fr);
    fseek(fr, 2, SEEK_CUR);
    fread(ch, 2, 1, fr);

    if (*ch != 8 && *ch != 24 && *ch != 32)
    {
        TRACE("error: unsupported number of channels.\n");
        fclose(fr);
        return -1;
    }

    *ch /= 8;
    unsigned long comp;
    fread(&comp, 4, 1, fr);

    if (!(comp==0 || comp==3))
    {
        TRACE("error: unsupported compressed file (%lu).\n", comp);
        fclose(fr);
        return -1;
    }

    if (*ch == 1)
    {
        TRACE("warning: 8bit indexed color\n");
        TRACE("warning: color table ignored assuming gray-scale.\n");
    }

    fseek(fr, offset - 34, SEEK_CUR);
    int pad = (4 - (((*w) * (*ch)) % 4)) % 4;
    *p = new unsigned char[(*w * (*ch) + pad) * (*h)];
    fread(*p, 1, (*w * (*ch) + pad) * (*h), fr);
    fclose(fr);

    return 0;
}

//--------------------------------------------------------------------
// CBmpImage class
//--------------------------------------------------------------------
class CBmpImage
{
    unsigned char *m_pixel;
    int m_width;
    int m_height;
    short m_num_channels;

public:
	// property
	int GetWidth(void) const { return m_width; }
	int GetHeight(void) const { return m_height; }
	int GetNumChannels(void) const { return m_num_channels; }

public:
	CBmpImage()
	: m_pixel(0)
	, m_width(0)
	, m_height(0)
	, m_num_channels(0)
    {
	}

    CBmpImage(const CBmpImage& bmp)
	: m_pixel(0)
	{
		*this = bmp;
	}

	CBmpImage(const std::string& filename)
	: m_pixel(0)
	{
		LoadBmp(filename);
	}

	CBmpImage(const int w, const int h, const int c)
	: m_pixel(0)
	{
		Initialize(w,h,c);
	}

    ~CBmpImage()
    {
        if (m_pixel)
            delete [] m_pixel;
    }

 	const CBmpImage& operator =(const CBmpImage& bmp)
	{
		if (this == &bmp)
			return *this;

		Initialize(bmp.m_width, bmp.m_height, bmp.m_num_channels);

		int nPad = (4 - ((m_width * m_num_channels) % 4)) % 4;
		int nSize = (m_width * m_num_channels + nPad) * m_height;
		memcpy(m_pixel, bmp.m_pixel, nSize);

		return *this;
	}

	bool IsInside(const int& x, const int& y) const
	{
		return (x>=0) && (x<m_width) && (y>=0) && (y<m_height);
	}

    unsigned char *pixel_ptr()
    {
        return m_pixel;
    }

    const unsigned char *pixel_ptr() const
    {
        return m_pixel;
    }

	const unsigned char & pixel(const int& x, const int& y, const int& ch) const
	{
		if (!(IsInside(x,y) && ch>=0 && ch<m_num_channels))
			throw std::runtime_error("out of range");

		int nPad = (4 - ((m_width * m_num_channels) % 4)) % 4;
		int nPosY = m_height-1-y;
		int nCh = ch;
		if (m_num_channels>1)
		{
			if (nCh==0) nCh=2;
			else if (nCh==2) nCh=0;
		}
		return m_pixel[m_num_channels * x + (m_width * m_num_channels + nPad) * nPosY + nCh];
	}

	unsigned char & pixel(const int& x, const int& y, const int& ch)
	{
		if (!(IsInside(x,y) && ch>=0 && ch<m_num_channels))
			throw std::runtime_error("out of range");

		int nPad = (4 - ((m_width * m_num_channels) % 4)) % 4;
		int nPosY = m_height-1-y;
		int nCh = ch;
		if (m_num_channels > 1)
		{
			if (nCh == 0) nCh = 2;
				else if (nCh == 2) nCh = 0;
		}
		return m_pixel[m_num_channels * x + (m_width * m_num_channels + nPad) * nPosY + nCh];
	}

	void Initialize(const int x, const int y, const int ch)
	{
		m_width = x;
		m_height = y;
		m_num_channels = ch;

		int nPad = (4 - ((m_width * m_num_channels) % 4)) % 4;
		int nSize = (m_width * m_num_channels + nPad) * m_height;
        if (m_pixel)
            delete [] m_pixel;
		m_pixel = new unsigned char [nSize];
		Clear(0);
	}

	void Clear(const unsigned char& val = 0)
	{
		for (int x=0; x<m_width; x++)
			for (int y=0; y<m_height; y++)
				for (int c=0; c<m_num_channels; c++)
					pixel(x,y,c) = val;
	}

	bool LoadBmp(const std::string& filename)
    {
		TRACE("bmp <= %s\n", filename.c_str());
		if (m_pixel)
            delete [] m_pixel;
        if (ReadBmp(filename.c_str(), &m_width, &m_height, &m_num_channels, &m_pixel))
			throw std::runtime_error("failed to load a bitmap");
		return true;
    }

    bool LoadPpm(const std::string& filename)
    {
		TRACE("ppm <= %s\n", filename.c_str());
        if (m_pixel)
            delete [] m_pixel;
        if (ReadPpm(filename.c_str(), &m_width, &m_height, &m_num_channels, &m_pixel))
			throw std::runtime_error("failed to load a bitmap");
		SwapRB(m_width, m_height, m_num_channels, m_pixel, m_pixel);
		FlipVertically(m_width * m_num_channels, m_height, m_pixel, m_pixel);

		int nPad = (4 - ((m_width * m_num_channels) % 4)) % 4;
		if (nPad)
		{
			int nSize = (m_width * m_num_channels + nPad) * m_height;
			unsigned char *pix = new unsigned char [nSize];
			AlignDword(m_width, m_height, m_num_channels, m_pixel, pix);
			delete [] m_pixel;
			m_pixel = pix;
		}

		return true;
    }

    bool WriteBmp(const std::string& filename) const
	{
		TRACE("bmp => %s\n", filename.c_str());
		if (slib::image::WriteBmp(filename.c_str(), m_width, m_height, m_num_channels, m_pixel))
			throw std::runtime_error("failed to write a bitmap");
		return true;
    }

    bool WritePpm(const std::string& filename) const
    {
		TRACE("ppm => %s\n", filename.c_str());
		unsigned char *pucPPM = AllocatePpmPixelsFromBmpPixels(m_width, m_height, m_num_channels, m_pixel);
		if (slib::image::WritePpm(filename.c_str(), m_width, m_height, m_num_channels, pucPPM))
		{
			delete [] pucPPM;
			throw std::runtime_error("failed to write a bitmap");
		}
		delete [] pucPPM;
        return true;
    }
};

#undef rgb_to_intensity

} // image
} // slib
