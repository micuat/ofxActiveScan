//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageMagickIO.h 4304 2010-12-07 09:02:51Z shun $
//

#pragma once

#define TRACE printf

namespace slib
{

template <typename T> inline 
void print_levmar_info(const T info[LM_INFO_SZ])
{
	switch ((int)info[6])
	{
	case 1:
		TRACE("levmar: stopped by small gradient J^T e\n");
		break;
	case 2:
		TRACE("levmar: stopped by small Dp\n");
		break;
	case 3:
		TRACE("levmar: stopped by itmax\n");
		break;
	case 4:
		TRACE("levmar: singular matrix. Restart from current p with increased mu \n");
		break;
	case 5:
		TRACE("levmar: no further error reduction is possible. Restart with increased mu\n");
		break;
	case 6:
		TRACE("levmar: stopped by small ||e||_2\n");
		break;
	case 7:
	default:
		TRACE("levmar: stopped by invalid (i.e. NaN or Inf) func values\n");
	}
	TRACE("levmar: iter=%d, sum2=%g -> %g (%g%% improved)\n", (int)info[5], info[0], info[1], 100.0*(1-info[1]/info[0]));
}

} // namespace slib
