#pragma once

namespace slib
{
namespace image
{

inline
void Read(Field<2,float>& fld, const std::string& filename)
{
	Read(fld, filename, 1);
}
inline
void Read(Field<2,unsigned char>& fld, const std::string& filename)
{
	Read(fld, filename, 255);
}

inline
void Read(Field<2,CVector<3,float> >& fld, const std::string& filename)
{
	Read(fld, filename, 1);
}
inline
void Read(Field<2,CVector<3,unsigned char> >& fld, const std::string& filename)
{
	Read(fld, filename, 255);
}

inline
void Write(const Field<2,float>& fld, const std::string& filename)
{
	Write(fld, filename, 1);
}
inline
void Write(const Field<2,unsigned char>& fld, const std::string& filename)
{
	Write(fld, filename, 255);
}

inline
void Write(const Field<2,CVector<3,float> >& fld, const std::string& filename)
{
	Write(fld, filename, 1);
}
inline
void Write(const Field<2,CVector<3,unsigned char> >& fld, const std::string& filename)
{
	Write(fld, filename, 255);
}
}
}
