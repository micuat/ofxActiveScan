//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4054 2010-06-03 15:03:40Z shun $
//

#pragma once

#include "stdafx.h"

#include "MathBaseLapack.h"
#include "Field.h"
#include "MiscUtil.h"
#include "ImageBmpIO.h"
#include "Stereo.h"
#include "LeastSquare.h"

#include "FundamentalMatrix.h"
#include "Options.h"

//struct options_t options;

using namespace slib;

// output filename
static char *m_plyfilename = "mesh.ply";

// filename of optional vertical correspondence 
static char *m_vmapfilename = 0;

static float m_max_edge_length = 0.1;
static float m_distortion_angle = 1;
static bool m_debug = false;

bool distorted(const int v1,const int v2,const int v3,const std::vector<CVector<3,double> >& result);

void WritePly(const std::vector<CVector<3,double> >& result, const Field<2,float>& mask, std::string filename);

