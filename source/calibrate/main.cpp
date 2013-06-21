//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: main.cpp 4590 2011-05-30 22:13:45Z shun $
//

#include "stdafx.h"

#include "Field.h"
#include "ImageBmpIO.h"

#include "calibrate.h"

#undef min
#undef max

void print_usage(const char *argv0)
{
	const char *prog = strrchr(argv0, '\\');
	if (prog)
		prog++;
	else
		prog = argv0;
	printf("Usage: %s <options> <h.map> <v.map> <mask.bmp>\n",prog);
	exit(-1);
}

int main(int argc, char *argv[]) 
{
	try 
	{
		// parse commandline options
		if (argc!=5) print_usage(argv[0]);

		// load correspondences estimated by decode program 
		slib::Field<2,float> horizontal(argv[2]);
		slib::Field<2,float> vertical(argv[3]);  
		slib::Field<2,float> mask;  
		slib::image::Read(mask, argv[4]);

		CProCamCalibrate calib(argv[1]);
		calib.Calibrate(horizontal,vertical,mask);

		calib.WriteCamIntrinsic("cam-intrinsic.txt");
		calib.WriteCamDistortion("cam-distortion.txt");
		calib.WriteProIntrinsic("pro-intrinsic.txt");
		calib.WriteProDistortion("pro-distortion.txt");
		calib.WriteProExtrinsic("pro-extrinsic.txt");
	} 
	catch (const std::exception& e) 
	{
		fprintf(stderr,"error: %s\n", e.what());
		return -1;
	}

	return 0;
}

