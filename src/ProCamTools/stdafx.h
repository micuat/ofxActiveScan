//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: stdafx.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once

#define _USE_MATH_DEFINES

#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <cstdarg>

#include <vector>
#include <string>
#include <map>
#include <string>

#define _ASSERTE assert

#ifndef TRACE
#define TRACE printf
#endif
