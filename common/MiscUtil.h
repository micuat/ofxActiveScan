//
// Copyright (c) 2009-2010  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: MiscUtil.h 4567 2011-05-26 16:02:30Z shun $
//

#pragma once

#ifndef TRACE
#define TRACE printf
#endif

#undef min
#undef max

namespace slib
{

inline
void ThrowRuntimeError(const char *fmt, ...)
{
	va_list param;
	va_start(param, fmt);
	char message[1024];
	_vsnprintf(message, 1023, fmt, param);
	message[1023] = 0;
	TRACE(message);
	TRACE("\n");
	throw std::runtime_error(message);
}

inline
void ThrowLogicError(const char *fmt, ...)
{
	va_list param;
	va_start(param, fmt);
	char message[1024];
	_vsnprintf(message, 1023, fmt, param);
	message[1023] = 0;
	TRACE(message);
	throw std::logic_error(message);
}

inline
std::string format(const char *fmt, ...)
{
	std::string buffer;
	va_list     arguments;

	assert(fmt);
	va_start(arguments, fmt);
	try
	{
		int length = _vscprintf(fmt, arguments);
		if (length < 0)
			throw std::runtime_error("_vscprintf() failed.");
		buffer.assign(length, 0);
		int result = vsprintf(&buffer[0], fmt, arguments);
		if (result < 0)
			throw std::runtime_error("vsprintf() failed.");
	}
	catch (...)
	{
		va_end(arguments);
		throw;
	}
	va_end(arguments);
	return buffer;
}

inline
bool IsPowerOfTwo(const int n)
{
	return !(n & (n-1));
}

inline
int GetLeastPowerOfTwo(const int n)
{
	int num = n;
	while (!IsPowerOfTwo(num)) num++;
	return num;
}

} // slib
