//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: decode.cpp 4546 2011-05-24 12:20:43Z shun $
//

#include <stdlib.h>

#include "stdafx.h"

#include "decode.h"

void print_usage(const char *basename)
{
	const char *p = strrchr(basename, '\\');
	if (p)
		p++;
	else
		p = basename;
	fprintf(stderr, "Usage: %s <options> <images....>\n", p);
	exit(-1);
}

int main(int argc, char *argv[])
{
	try
	{
		// parse commandline options
		if (argc < 2) print_usage(argv[0]);

		std::vector<std::string> files(argc-2);
		for (int i=0; i<argc-2; i++)
			files[i]=argv[i+2];

		CDecode decode(argv[1]);
		decode.Decode(files);
		decode.WriteMap(0,"h.map");
		decode.WriteMap(1,"v.map");
		decode.WriteMask("mask.bmp");
		decode.WriteReliable("reliable.bmp");
	}
	catch(const std::exception & e)
	{
		fprintf(stderr, "exception: %s\n", e.what());
		return -1;
	}

	return 0;
}
