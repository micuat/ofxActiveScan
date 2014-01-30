//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageMagickIO.h 4304 2010-12-07 09:02:51Z shun $
//

#pragma once

#include "Field.h"
#include "ImageBmpIO.h"

#include "Options.h"
#include "GrayCode.h"

class CEncode
{
public:
	CEncode(const options_t& o) : m_options(o), index(0) {}
	CEncode(const std::string& filename) : index(0) { m_options.load(filename); }

	int GetNumImages(void) const { 
		int n = 0;
		if ( m_options.horizontal) 
			n += 2 * m_options.get_num_bits(0) + m_options.num_fringes;
		if ( m_options.vertical)
			n += 2 * m_options.get_num_bits(1) + m_options.num_fringes;
		return n;
	}

/*	void WriteImage(int id, const std::string& filename) const {
		slib::Field<2,float> image;
		GetImage(id,image);
		slib::image::Write(image,filename);
	}
*/
	void GetImage(int id, slib::Field<2,unsigned char>& image) const {
		image.Initialize(m_options.projector_width,m_options.projector_height);
		if ( m_options.horizontal) {
			if (id < m_options.get_num_bits(0)*2) {
				get_gray(0, id, image);
				return;
			}
			id-=m_options.get_num_bits(0)*2;
			if (id < m_options.num_fringes) {
				get_phase(0, id, image);
				return;
			}
			id-=m_options.num_fringes;
		}
		if (m_options.vertical) {
			if (id < m_options.get_num_bits(1)*2) {
				get_gray(1, id, image);
				return;
			}
			id-=m_options.get_num_bits(1)*2;
			if (id < m_options.num_fringes) {
				get_phase(1, id, image);
				return;
			}
			id-=m_options.num_fringes;
		}
		throw std::runtime_error("invalid image id");
	}
	
	slib::Field<2,unsigned char> GetImage(int id) const {
		slib::Field<2,unsigned char> image;
		GetImage(id, image);
		return image;
	}
	
	slib::Field<2,unsigned char> GetImage() const {
		slib::Field<2,unsigned char> image;
		GetImage(index, image);
		return image;
	}
	
	bool IsFinished() const {
		return index >= GetNumImages();
	}
	
	void Proceed() {
		index++;
	}
	
	int GetIndex() const {
		return index;
	}

private:
	void get_gray(int direction, int id, slib::Field<2,unsigned char>& image) const {
		int level = m_options.get_num_bits(direction)  - 1 - id / 2;
		// id%2 == 1 if complementary
		GenerateGrayCodeImage(direction, level, image, id%2);
	}

	void get_phase(int direction, int id, slib::Field<2,unsigned char>& image) const {
			GeneratePhaseCodeImage(direction, m_options.num_fringes, id, image);
	}

private:
	options_t m_options;
	int index;
};
