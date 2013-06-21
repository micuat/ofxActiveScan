//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageMagickIO.h 4304 2010-12-07 09:02:51Z shun $
//

#pragma once

#include "Field.h"
#include "ImageBmpIO.h"
#include "ImageBase.h"

#include "../Options.h"
#include "../GrayCode.h"

class CDecode
{
public:
	CDecode(const options_t& o) : m_options(o) {}
	CDecode(const std::string& filename) { m_options.load(filename); }

	void Decode(const std::vector<std::string>& files) 
	{
		std::vector<std::string>::const_iterator it = files.begin();
		std::vector<slib::Field<2,float>> images;
		if (m_options.horizontal) 
		{
			int nbits = m_options.get_num_bits(0);
			images.resize(2*nbits);
			for (int i=0; i<2*nbits; i++)
				slib::image::Read(images[i], *it++);

			decode_gray(images,0);
			generate_mask(0);

			images.resize(m_options.num_fringes);
			for (int i=0; i<m_options.num_fringes; i++)
				slib::image::Read(images[i],*it++);

			decode_phase(images,0);

			if (m_options.debug)
				dump_images(0);

			convert_reliable_map(0);
		}

		if (m_options.vertical) 
		{
			int nbits = m_options.get_num_bits(1);
			images.resize(2*nbits);
			for (int i=0; i<2*nbits; i++)
				slib::image::Read(images[i], *it++);

			decode_gray(images,1);
			generate_mask(1);

			images.resize(m_options.num_fringes);
			for (int i=0; i<m_options.num_fringes; i++)
				slib::image::Read(images[i],*it++);

			decode_phase(images,1);

			if (m_options.debug)
				dump_images(1);

			convert_reliable_map(1);
		}

		// merge masks and reliable maps
		if (m_options.horizontal && m_options.vertical) 
		{
			for (int y = 0; y < m_mask[1].size(1); y++) 
			{
				for (int x = 0; x < m_mask[1].size(0); x++) 
				{
					if (!m_mask[1].cell(x, y))
						m_mask[0].cell(x, y) = 0;
					m_phase_error[0].cell(x,y) = std::min(m_phase_error[0].cell(x,y),m_phase_error[1].cell(x,y));
				}
			}
		}
	}

	const slib::Field<2,float>& GetMap(int direction) const { 
		return m_phase_map[direction];
	}

	void WriteMap(int direction, const std::string& filename) const {
		m_phase_map[direction].Write(filename);
	}

	const slib::Field<2,float>& GetMask(void) const { 
		if (m_options.horizontal) 
			return m_mask[0];
		else
			return m_mask[1];
	}

	void WriteMask(const std::string& filename) const {
		slib::image::Write(GetMask(),filename);
	}

	const slib::Field<2,float>& GetReliable(void) const { 
		if (m_options.horizontal) 
			return m_phase_error[0];
		else
			return m_phase_error[1];
	}

	void WriteReliable(const std::string& filename) const {
		slib::image::Write(GetReliable(),filename);
	}

private:
	void convert_reliable_map(int direction)
	{
		float maxerror = 2.0/m_options.num_fringes;
		slib::Field<2,float>& reliable = m_phase_error[direction];
		for (int y=0; y<reliable.size(1); y++) {
			for (int x=0; x<reliable.size(0); x++) {
				if (reliable.cell(x,y) < maxerror && 
					m_gray_error[direction].cell(x,y) < 2) {
					reliable.cell(x,y) = 1;
				} else {
					reliable.cell(x,y) = 0;
				}
			}
		}
	}

	void decode_gray(const std::vector<slib::Field<2,float>>& images, int direction)
	{
		int nbits = m_options.get_num_bits(direction);
		std::vector<slib::Field<2,float>> diff(nbits);
		for (int bit = 0; bit<nbits; bit++)
		{
			float maxval = std::max(images[2*bit].max(), images[2*bit+1].max());
			diff[nbits-1-bit] = images[2*bit]-images[2*bit+1];

			if (m_gray_error[direction].size() != images[0].size()) { 
				m_gray_error[direction].Initialize(images[0].size());
				m_gray_error[direction].Clear(0);
			}

			// count error
			float threshold = m_options.intensity_threshold * maxval;
			CountGraycodeUncertainty(diff[nbits-1-bit], threshold, m_gray_error[direction]);
		}

		// decode graycode
		DecodeGrayCodeImages(diff, m_gray_map[direction]);
	}

	void decode_phase(const std::vector<slib::Field<2,float>>& images, int direction)
	{
		DecodePhaseCodeImages(images, m_phase_map[direction]);

		UnwrapPhase(m_phase_map[direction], m_options.fringe_interval*m_options.num_fringes, m_gray_map[direction], m_phase_map[direction], m_phase_error[direction]);
	}

	void dump_images(int direction) const
	{
		char *suffix = direction ? "v" : "h";
		float s = 1.0/m_gray_map[direction].size(0);
		WriteCorrespondenceMap(m_gray_map[direction], m_mask[direction], slib::format("gray-%s.bmp", suffix), s);
		WriteCorrespondenceMap(m_phase_map[direction], m_mask[direction], slib::format("phase-%s.bmp", suffix), s);
		ExportCorrespondencePlot(m_gray_map[direction], m_mask[direction], slib::format("gray-%s.dat", suffix));
		ExportCorrespondencePlot(m_phase_map[direction], m_mask[direction], slib::format("phase-%s.dat", suffix));

		slib::Field<2,float> err;
		slib::Field<2,slib::CVector<3,float>> rgb;

		err=m_gray_error[direction];
		err /= m_options.get_num_bits(direction);
		slib::image::ConvertToJetMap(err,rgb);
		apply_mask(m_mask[direction],rgb);
		slib::image::Write(rgb,slib::format("gray-error-%s.bmp", suffix));

		err=m_phase_error[direction];
		err /= 0.5;
		slib::image::ConvertToJetMap(err,rgb);
		apply_mask(m_mask[direction],rgb);
		slib::image::Write(rgb,slib::format("phase-error-%s.bmp", suffix));
	}

	void apply_mask(const slib::Field<2,float>& mask, slib::Field<2,slib::CVector<3,float>>& img) const 
	{
		for (int y=0; y<mask.size(1); y++) 
			for (int x=0; x<mask.size(0); x++) 
				if (mask.cell(x,y) < 1)
					img.cell(x,y) = slib::make_vector(0,0,0);		
	}

	void generate_mask(int direction)
	{
		int nbits = m_options.get_num_bits(direction);
		m_mask[direction].Initialize(m_gray_error[direction].size());
		for (int y=0; y<m_gray_error[direction].size(1); y++) {
			for (int x=0; x<m_gray_error[direction].size(0); x++)
				if (m_gray_error[direction].cell(x,y) < nbits-1)
					m_mask[direction].cell(x,y) = 1;
				else
					m_mask[direction].cell(x,y) = 0;
		}
	}

private:
	options_t m_options;
	slib::Field<2,float> m_gray_map[2];
	slib::Field<2,float> m_phase_map[2];
	slib::Field<2,int> m_gray_error[2];
	slib::Field<2,float> m_phase_error[2]; // also used as reliable mask
	slib::Field<2,float> m_mask[2];
};
