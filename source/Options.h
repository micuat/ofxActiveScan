//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: Options.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once
#include "MiscUtil.h"
#include "IniFile.h"

struct options_t
{
// projector dimension
	int projector_width;					
	int projector_height;					
// vertical position of projector's principal point 
// 0:top, 1:bottom
// 0.83;	// for EMP765
// 0.92;	// for EMP1735W 16:9
// 0.87;	// for EMP1735W 16:10
// 0.86;	// for EMP1735W 4:3
	float projector_horizontal_center;
	int num_fringes;				// number of sinusoidal pattern
	int fringe_interval;
	bool horizontal;			// coding in horizontal direction
	bool vertical;				// coding in vertical direction
	bool complementary;			// OBSOLETE: binarize images using complementary patterns; otherwise thresholding is used
	bool debug;					// debug
	float intensity_threshold;
	int nsamples;

	options_t() : 
		projector_width(1024), projector_height(768), projector_horizontal_center(0.5),	// projector 
		num_fringes(8), fringe_interval(1),		// fringe patterns
		horizontal(true), vertical(true), // directions
		complementary(true), // binary code
		debug(false), // debug flag
		intensity_threshold(0.1), // mask threshold
		nsamples(0) // 0=no subsampling
	{
	}

	void load(const std::string& filename)
	{
		slib::CIniFile ini;
		ini.Load(filename);

		projector_width = ini.GetInt("projector","width");
		projector_height = ini.GetInt("projector","height");
		projector_horizontal_center = ini.GetFloat("projector","vertical_center");

		num_fringes = ini.GetInt("phase","nfringes");
		fringe_interval = ini.GetInt("phase","interval");

		horizontal = ini.GetBool("pattern","horizontal");
		vertical = ini.GetBool("pattern","vertical");
		complementary = ini.GetBool("pattern","complementary");
		debug = ini.GetBool("pattern","debug");

		intensity_threshold = ini.GetFloat("reconstruction","threshold");
		nsamples = ini.GetInt("reconstruction","nsamples");

		ini.Dump();
	}

	int get_num_bits(int direction) const {
		if (direction)
			return  ceilf(logf(projector_height) / logf(2));
		else 
			return  ceilf(logf(projector_width) / logf(2));
	}
};
