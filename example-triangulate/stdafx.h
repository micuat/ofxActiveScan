//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4054 2010-06-03 15:03:40Z shun $
//

#pragma once

#define _USE_MATH_DEFINES

#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif

#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cassert>

#include <vector>
#include <map>
#include <string>

#define TRACE printf
#define _ASSERTE assert
