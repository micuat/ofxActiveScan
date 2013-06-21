//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4547 2011-05-24 14:04:52Z shun $
//

#include "stdafx.h"

#include "MiscUtil.h"

#include "encode.h"

void print_usage(const char *argv0)
{
	const char *prog = strrchr(argv0, '\\');
	if (prog)
		prog++;
	else
		prog = argv0;
	printf("Usage: %s <options>\n", prog);
	exit(-1);
}

int main(int argc, char *argv[])
{
	try
	{
		if (argc != 2)
			print_usage(argv[0]);

		CEncode encode(argv[1]);
		int nimages = encode.GetNumImages();

		for (int i=0; i<nimages; i++) 
			encode.WriteImage(i, slib::format("pattern-%03d.bmp", i));
	}
	catch (const std::exception& e)
	{
		fprintf(stderr, "exception: %s\n", e.what());
		return -1;
	}

	return 0;
}
