//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: GrayCode.h 4590 2011-05-30 22:13:45Z shun $
//

#pragma once

#include "Field.h" // for GetPseudoInverse()
#include "MathBaseLapack.h" // for GetPseudoInverse()

namespace slib
{

//------------------------------------------------------------
// gray code
//------------------------------------------------------------

namespace {
inline
int ConvertGrayToBinary(const unsigned long graycode)
{
	int bincode = 0;
	int mask = 1;
	while (mask < graycode)
		mask *= 2;
	for (; mask; mask /= 2)
		bincode += (mask & graycode) ^ (mask & (bincode / 2));
	return bincode;
}
} // unnamed namespace

// generate a single bit plane of gray-code pattern
inline 
void GenerateGrayCodeImage(const int direction, const int level, Field<2,float> &bmp)
{
	for (int y = 0; y < bmp.size(1); y++)
	{
		for (int x = 0; x < bmp.size(0); x++)
		{
			int binary = direction ? y : x;
			int gray = binary ^ (binary >> 1);
			bmp.cell(x, y) = (gray & (1 << level)) ? 1 : 0;
		}
	}
}

inline 
void DecodeGrayCodeImages(const std::vector<Field<2,float> >& bmps, Field<2,float>& result)
{
	int nlevels = bmps.size();
	const CVector<2,int>& size = bmps[0].size();

	// load binary codes
	Field<2, unsigned long> binary(size, 0);
	for (int l=0; l<nlevels; l++)
		for (int y = 0; y < size[1]; y++)
			for (int x = 0; x < size[0]; x++)
				if (bmps[l].cell(x, y) > 0)
					binary.cell(x, y) += (1 << l);

	// decode
	result.Initialize(size);
	for (int y = 0; y < size[1]; y++)
		for (int x = 0; x < size[0]; x++)
			result.cell(x, y) = ConvertGrayToBinary(binary.cell(x, y));
}

// update valid region by thresholding
inline 
void CountGraycodeUncertainty(const Field<2,float> &diff, const float threshold, Field<2,int> &uncertainty)
{
	for (int y = 0; y < diff.size(1); y++)
		for (int x = 0; x < diff.size(0); x++)
			if (abs(diff.cell(x, y)) < threshold)
				uncertainty.cell(x, y)++;
}

//------------------------------------------------------------
// phase-shifting code
//------------------------------------------------------------

// generate moire pattern images.
// 'period' is the phase period of sinusoidal curve in pixel
inline 
void GeneratePhaseCodeImage(const int direction, const int period, const int phase, Field<2,float> &bmp)
{
	std::vector<float> table(period);
	for (int i = 0; i < period; i++)
		table[i] = sin(2.0 * M_PI * (i + phase) / period) / 2.0 + 0.5;

	for (int y = 0; y < bmp.size(1); y++)
		for (int x = 0; x < bmp.size(0); x++)
			bmp.cell(x, y) = table[(direction ? y : x) % period];
}

// generate phase image from moire pattern images.
inline 
void DecodePhaseCodeImages(const std::vector<Field<2,float> > &images, Field<2,float>& result)
{
	const CVector<2,int>& size = images[0].size();
	const int nphases = images.size();

	CDynamicMatrix<float> mat(nphases, 3);
	for (int r = 0; r < nphases; r++)
	{
		mat(r, 0) = cos(2 * M_PI * r / nphases);
		mat(r, 1) = sin(2 * M_PI * r / nphases);
		mat(r, 2) = 1;
	}
	mat = GetPseudoInverse(mat);

	result.Initialize(size);
	for (int y = 0; y < size[1]; y++)
	{
		for (int x = 0; x < size[0]; x++)
		{
			CDynamicVector<float> vec(nphases);
			for (int r = 0; r < nphases; r++)
				vec[r] = images[r].cell(x, y);
			vec = mat * vec;
			float A = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
			float phi = atan2(vec[0] / A, vec[1] / A);
			float m = vec[2];
			while (phi < 0)
				phi += 2 * M_PI;
			result.cell(x, y) = phi / (2 * M_PI);
		}
	}
}

//------------------------------------------------------------
// phase unwrapping
//------------------------------------------------------------

// unwrap phase 
// 'period' is phase period of sinusoidal curve in pixel
// 'reference' is reference integer code
// 'tolerance' is max correctable error in reference global code (must be less than half of period)
inline 
void UnwrapPhase(const Field<2,float> &phase, const int period, const Field<2,float> &reference, Field<2,float>& result, Field<2,float>& unwrap_error)
{
	// max correctable phase error
	float window = 2.0/period;

	result.Initialize(phase.size());
	unwrap_error.Initialize(phase.size());

	for (int y = 0; y < phase.size(1); y++) {
		for (int x = 0; x < phase.size(0); x++) {
			int graycode = reference.cell(x, y);	// in [0,width)
			float moire_phase = phase.cell(x, y);	// in [0,1)
			float gray_phase = (float)(graycode % period) / period;	// in [0,1)

			if (_isnan(moire_phase)) {
				result.cell(x, y) = graycode;
				unwrap_error.cell(x,y) = 0.5;
				continue;
			}

			// normalized:  moire in [gray-0.5, gray+0.5)
			//  0      -w        +w 1
			// -|-------o====g====o-|----
			// ----x=========m=========o-
			if (moire_phase >= gray_phase + 0.5)
				moire_phase -= 1;
			else if (moire_phase < gray_phase - 0.5)
				moire_phase += 1;

			float diff = abs(gray_phase - moire_phase);
			if (diff < window) {
				result.cell(x, y) = graycode - (graycode % period) + period * moire_phase;
			} else {
				result.cell(x, y) = graycode;
			}
			unwrap_error.cell(x,y) = diff;
		}
	}
}

//------------------------------------------------------------
// for debug
//------------------------------------------------------------

// dump a spatial pattern in color code
template <typename T> inline 
void WriteCorrespondenceMap(const Field<2, T>&code, const Field<2,float> &mask, const std::string & filename, float scale)
{
	// export a color images
#ifdef USE_8BIT_DUMP
	Field<2,float> img(code.size());
	for (int y = 0; y < code.size(1); y++) {
		for (int x = 0; x < code.size(0); x++) {
			if (mask.cell(x, y))
				img.cell(x, y) = scale * code.cell(x, y);
			else
				img.cell(x, y) = 0;
		}
	}
#else
	Field<2,CVector<3,float> > img(code.size());
	img.Clear(make_vector(0, 0, 0));
	for (int y = 0; y < code.size(1); y++) {
		for (int x = 0; x < code.size(0); x++) {
			if (mask.cell(x, y)) {
				float r, g, b;
				image::GetHueColor(scale * code.cell(x, y), r, g, b);
				img.cell(x, y) = make_vector(r, g, b);
			}
		}
	}
#endif
	image::Write(img, filename);
}
template <typename T> inline 
void ExportCorrespondencePlot(const Field<2, T>&code, const Field<2,float> &mask, const std::string & filename)
{
	// export data for gnuplot
	FILE *fw = fopen(filename.c_str(), "wb");
	for (int i = 0; i < code.size(0) && i < code.size(1); i++) {
		if (mask.cell(i,i))
			fprintf(fw, "%d %f\n", i, (float)code.cell(i,i));
	}
	fclose(fw);
}

} // namespace slib
