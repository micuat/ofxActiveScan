//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: decode.cpp 4556 2011-05-25 23:00:28Z shun $
//

#include "stdafx.h"

#include "../BuildFlags.h"
#if defined(USE_IMAGEMAGICK)
#include "ImageMagickIO.h"
#elif defined(USE_ATLIMAGE)
#include "ImageAtlIO.h"
#else
#include "ImageBmpIO.h"
#endif

#include "ImageBase.h"
#include "../GrayCode.h"
#include "../Options.h"

using namespace slib;

void decode_gray(const options_t& options, const char * const *filenames, int nbits, Field<2,float>& gray, Field<2,int>& error)
{
	std::vector<Field<2,float>> diff(nbits);
	for (int bit = nbits - 1; bit >= 0; bit--)
	{
		Field<2,float> complementary;
		image::Read(diff[bit], *filenames++);
		image::Read(complementary, *filenames++);

		float maxval = std::max(diff[bit].max(), complementary.max());
		diff[bit] -= complementary;

		if (error.size() != diff[bit].size()) {
			error.Initialize(diff[bit].size());
			error.Clear(0);
		}

		// count error
		float threshold = options.intensity_threshold * maxval;
		CountGraycodeUncertainty(diff[bit], threshold, error);
	}

	// decode graycode
	DecodeGrayCodeImages(diff, gray);
}

void decode_phase(const options_t& options, const char *const *filenames,const Field<2,float>& gray,Field<2,float>& phase, Field<2,float>&error)
{
	std::vector<Field<2,float>> phaseimages(options.num_fringes);
	for (int i = 0; i < options.num_fringes; i++) 
		image::Read(phaseimages[i], *filenames++);

	DecodePhaseCodeImages(phaseimages, phase);

	UnwrapPhase(phase, options.fringe_interval*options.num_fringes, gray, phase, error);
}

void generate_mask(const Field<2,int>& error, int nbits, Field<2,float>& mask)
{
	mask.Initialize(error.size());
	for (int y=0; y<error.size(1); y++)
		for (int x=0; x<error.size(0); x++)
			if (error.cell(x,y) < nbits-1)
				mask.cell(x,y)=1;
			else
				mask.cell(x,y)=0;
}

void write_images(const char *suffix, const Field<2,float>& gray, const Field<2,float>& phase, const Field<2,float>& mask)
{
	float s = 1.0/gray.size(0);
	DumpCode(gray, mask, format("gray-%s.bmp", suffix), s);
	DumpCode(phase, mask, format("phase-%s.bmp", suffix), s);
}

void decode(const options_t& options, const std::vector<Field<2,float>>&gray, const std::vector<Field<2,float>>& phase, Field<2,float>&mask)
{
		Field<2,float>gray_code;
		Field<2,int> gray_error;
		decode_gray(options,gray,nbits,gray_code,gray_error);
		generate_mask(gray_error, nbits, mask);

		Field<2,float>phase_code,phase_error;
		decode_phase(options,phase,gray_code,phase_code,phase_error);

		if (options.debug)
			write_images("h", gray_code,phase_code,mask);
	}

	// write mask
	if (options.horizontal) 
		image::Write(mask[0], "mask.bmp");
	else
		image::Write(mask[1], "mask.bmp");
}
