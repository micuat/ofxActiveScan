//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageMagickIO.h 4304 2010-12-07 09:02:51Z shun $
//

#pragma once

#include "MathBase.h"
#include "Field.h"
#include "ImageBmpIO.h"
#include "CameraCalibration.h"
#include "CalcStatistics.h"
#include "ImageBase.h"
#include "LeastSquare.h"
#include "Stereo.h"

#include "Options.h"
#include "FundamentalMatrix.h"

#include <stdlib.h>
#include <time.h>

class CProCamCalibrate
{
public:
	CProCamCalibrate(const options_t& o) 
		: m_options(o)  {}

	CProCamCalibrate(const std::string& filename)  { 
			m_options.load(filename);
	}

	void Calibrate(const slib::Field<2,float>& horizontal, 
		const slib::Field<2,float>& vertical, 
		const slib::Field<2,float>& mask)
	{
		srand(time(NULL));
		m_mask = mask;
		build_correspondence(horizontal,vertical);
		estimate_fundamental(fundamental);
		estimate_intrinsics(fundamental) ;
		estimate_extrinsics(fundamental);
	}

	void WriteCamIntrinsic(const std::string& filename) const { 
		m_cam_int.Write(filename);
	}

	void WriteCamDistortion(const std::string& filename) const {
		write_float(m_cam_dist, filename);
	}

	void WriteProIntrinsic(const std::string& filename) const { 
		m_pro_int.Write(filename);
	}

	void WriteProDistortion(const std::string& filename) const {
		write_float(m_pro_dist,filename);
	}

	void WriteProExtrinsic(const std::string& filename) const {
		m_pro_ext.Write(filename); 
	}
	
	slib::CMatrix<3,3,double> GetCamIntrinsic() const { 
		return m_cam_int;
	}
	
	double GetCamDistortion() const {
		return m_cam_dist;
	}
	
	slib::CMatrix<3,3,double> GetProIntrinsic() const { 
		return m_pro_int;
	}
	
	double GetProDistortion() const {
		return m_pro_dist;
	}
	
	slib::CMatrix<3,4,double> GetProExtrinsic() const {
		return m_pro_ext; 
	}
	
	slib::CMatrix<3,3,double> GetFundamental() const {
		return fundamental;
	}
	
private:
	void estimate_fundamental(slib::CMatrix<3,3,double>& fundamental)
	{
		// init
		m_pro_dist = 0;
		m_cam_dist = 0;
		m_pro_cod = slib::make_vector<double>((m_options.projector_width - 1) *0.5, (m_options.projector_height - 1) * m_options.projector_horizontal_center);
		m_cam_cod = slib::make_vector<double>(m_match.size(0) - 1,m_match.size(1) - 1) * 0.5;

		// estimate fundamental matrix
		reconstruct_epipolar_geometry(fundamental);
	}

	void estimate_intrinsics(const slib::CMatrix<3,3,double>& fundamental) 
	{
		double f1, f2;
		slib::fmatrix::EstimateFocalLengthBougnoux(fundamental, GetHomogeneousVector(m_pro_cod), GetHomogeneousVector(m_cam_cod), f1, f2);
		if (f1 < 0 || f2 < 0)
			throw std::runtime_error(slib::format("imaginary focal length: %f, %f", f1, f2));
		f1 = sqrt(f1);
		f2 = sqrt(f2);

		m_pro_int = slib::make_matrix<double>(
			f1,0,m_pro_cod[0],
			0,f1,m_pro_cod[1],
			0,0,1);
		m_cam_int=slib::make_matrix<double>(
			f2,0,m_cam_cod[0],
			0,f2,m_cam_cod[1],
			0,0,1);
	}

	void estimate_extrinsics(const slib::CMatrix<3,3,double>& fundamental)
	{
		// estimate essential matrix
		slib::CMatrix<3,3,double> essential;
		EstimateEssentialMatrix(m_pro_int, fundamental, m_cam_int, essential);

		// estimate extrinsics
		slib::CMatrix<3,3,double> rotation;
		slib::CVector<3,double> vecT;
		EstimateRelativePoseByEssentialMatrix(essential, rotation, vecT);

		// generate projection matrices
		for (int r=0; r<3; r++) 
		{
			for (int c=0; c<3; c++) 
				m_pro_ext(r,c)=rotation(r,c);
			m_pro_ext(r,3)=vecT[r];
		}
	}

	void write_float(double f, const std::string& filename) const
	{ 
		FILE *fw = fopen(filename.c_str(), "wb");
		if (!fw)
			throw std::runtime_error(slib::format("failed to open %s", filename.c_str()));
		fprintf(fw,"%e",f);
		fclose(fw);
	}

	void build_correspondence(const slib::Field<2,float>& horizontal, const slib::Field<2,float>& vertical)
	{
		m_match.Initialize(horizontal.size());
		for (int y=0; y<horizontal.size(1); y++) 
		{
			for (int x=0; x<horizontal.size(0); x++) 
			{
				slib::CVector<2,int> pos = slib::make_vector(x,y);
				m_match.cell(x,y) = slib::make_vector(horizontal.cell(pos), vertical.cell(pos));
			}
		}
	}

	// estimate fundamental matrix (F-matrix) using 2D pointwise correspondence
	// projector.F.camera = 0 
	void reconstruct_epipolar_geometry(slib::CMatrix<3,3,double>& fundamental)
	{
		// correspondences
		std::vector<slib::CVector<2,double> > p1; // projector coordinate
		std::vector<slib::CVector<2,double> > p2; // camera coordinates
		sample_points(p1, p2);

		if (m_options.debug)
		{
			TRACE("---------- fundamental / algebraic ----------\n");
			slib::fmatrix::EstimateFundamentalMatrixAlgebraic(p1, p2, fundamental);
			dump_epipolar_constraint(fundamental,   "reprojection-F-algebraic.bmp");

			TRACE("---------- fundamental / geometric ----------\n");
			slib::fmatrix::EstimateFundamentalMatrixGeometric(p1, p2, fundamental);
			dump_epipolar_constraint(fundamental,   "reprojection-F-geometric.bmp");

			TRACE("---------- fundamental / ransac ----------\n");
			slib::fmatrix::EstimateFundamentalMatrixRansac(p1, p2, fundamental);
			dump_epipolar_constraint(fundamental,   "reprojection-F-ransac.bmp");

			TRACE("---------- radial fundamental / algebraic ----------\n");
			slib::fmatrix::EstimateRadialFundamentalMatrixAlgebraic(p1, p2, m_pro_cod, m_cam_cod, m_pro_dist, m_cam_dist, fundamental);
			dump_epipolar_constraint(fundamental,"reprojection-R-algebraic.bmp");
			TRACE("distortion = %g, %g\n", m_pro_dist, m_cam_dist);

			TRACE("---------- radial fundamental / geometric ----------\n");
			m_pro_dist=m_cam_dist=0;
			slib::fmatrix::EstimateRadialFundamentalMatrixGeometric(p1, p2, m_pro_cod, m_cam_cod, m_pro_dist, m_cam_dist, fundamental);
			dump_epipolar_constraint(fundamental,"reprojection-R-geometric.bmp");
			TRACE("distortion = %g, %g\n", m_pro_dist, m_cam_dist);

			TRACE("---------- radial fundamental / apriori ----------\n");
			slib::CVector<2,double> backup_pro_cod(m_pro_cod), backup_cam_cod(m_cam_cod);
			m_pro_dist=m_cam_dist=0;
			slib::fmatrix::EstimateFundamentalMatrixAlgebraic(p1, p2, fundamental);
			slib::fmatrix::EstimateRadialFundamentalMatrixApriori(p1, p2, m_pro_cod, m_cam_cod, m_pro_dist, m_cam_dist, fundamental);
			dump_epipolar_constraint(fundamental,"reprojection-R-apriori.bmp");
			TRACE("distortion = %g, %g\n", m_pro_dist, m_cam_dist);
			TRACE("COD: projector = (%.2f, %.2f), camera = (%.2f, %.2f)\n",m_pro_cod[0],m_pro_cod[1],m_cam_cod[0],m_cam_cod[1]);
			m_pro_cod=backup_pro_cod;
			m_cam_cod=backup_cam_cod;

			TRACE("---------- radial fundamental / ransac ----------\n");
			m_pro_dist=m_cam_dist=0;
			slib::fmatrix::EstimateRadialFundamentalMatrixRansac(p1, p2, m_pro_cod, m_cam_cod, m_pro_dist, m_cam_dist, fundamental);
			dump_epipolar_constraint(fundamental,  "reprojection-R-ransac.bmp");
			TRACE("distortion = %g, %g\n", m_pro_dist, m_cam_dist);
			TRACE("COD: projector = (%.2f, %.2f), camera = (%.2f, %.2f)\n",m_pro_cod[0],m_pro_cod[1],m_cam_cod[0],m_cam_cod[1]);
			m_pro_cod=backup_pro_cod;
			m_cam_cod=backup_cam_cod;
		} 
		else 
		{
			slib::fmatrix::EstimateRadialFundamentalMatrixAlgebraic(p1, p2, m_pro_cod, m_cam_cod, m_pro_dist, m_cam_dist, fundamental);
			slib::fmatrix::EstimateRadialFundamentalMatrixApriori(p1, p2, m_pro_cod, m_cam_cod, m_pro_dist, m_cam_dist, fundamental);
		}

		p1.clear();
		p2.clear();
		for (int y=0; y<m_match.size(1); y++) 
		{
			for (int x=0; x<m_match.size(0); x++) 
			{
				if (m_mask.IsInside(x,y) && m_mask.cell(x,y)) 
				{
					p1.push_back(m_match.cell(x,y)); // projector
					p2.push_back(slib::make_vector(x, y) ); // camera
				}
			}
		}
		slib::fmatrix::RefineRadialFundamentalMatrix(p1, p2, m_pro_cod, m_cam_cod, m_pro_dist, m_cam_dist, fundamental);

		if (m_options.debug)
		{
			dump_epipolar_constraint(fundamental,"reprojection-F-final.bmp");
			dump_distortion_correction();
		}
	}

	void dump_distortion_correction(void) const
	{
		int linewidth = 2;
		slib::Field<2,slib::CVector<3,float> > src,dst;

		int pgrid = (m_options.projector_width - linewidth) / 40;
		src.Initialize(m_options.projector_width,m_options.projector_height);
		for (int y=0; y<m_options.projector_height; y++)
			for (int x=0; x<m_options.projector_width; x++)
				if (x % pgrid >= linewidth && y % pgrid >= linewidth) 
					src.cell(x,y)=slib::make_vector(1,1,1);
				else
					src.cell(x,y)=slib::make_vector(0,0,0);
		distort_projector_image(src,dst);
		slib::image::Write(dst,"distorted-projector.bmp");

		int cgrid = (m_mask.size(0) - linewidth) / 40;
		src.Initialize(m_mask.size());
		for (int y=0; y<m_mask.size(1); y++)
			for (int x=0; x<m_mask.size(0); x++)
				if (x % cgrid >= linewidth && y % cgrid >= linewidth)
					src.cell(x,y)=slib::make_vector(1,1,1);
				else
					src.cell(x,y)=slib::make_vector(0,0,0);
		undistort_camera_image(src,dst);
		slib::image::Write(dst,"undistorted-camera.bmp");
	}

	void sample_points( 
		std::vector<slib::CVector<2,double> >& p1, // projector coordinate
		std::vector<slib::CVector<2,double> >& p2) const // camera coordinates
	{
		if (m_options.nsamples) {
			// subsample correspondences for efficient computation
			slib::Field<2,float> sampled(m_match.size(),0);
			while (p1.size() < m_options.nsamples) 
			{
	#if 1
				int x = ((float)rand())/((float)RAND_MAX+1.0)*(float)m_match.size(0); 
				int y = ((float)rand())/((float)RAND_MAX+1.0)*(float)m_match.size(1);
	#else
				int x = slib::GaussianRandom(m_match.size(0)/2.0, m_match.size(0)/4.0);
				int y = slib::GaussianRandom(m_match.size(1)/2.0, m_match.size(1)/4.0);
	#endif
				if (m_mask.IsInside(x,y) && m_mask.cell(x,y) && !sampled.cell(x,y)) 
				{
					sampled.cell(x,y)=1;
					p1.push_back(m_match.cell(x,y)); // projector
					p2.push_back(slib::make_vector(x, y) ); // camera
				}
			}
	
			// dump the samples
			if (m_options.debug)
				slib::image::Write(sampled,"sample.bmp");
		} else {
			for (int y=0; y<m_mask.size(1); y++) {
				for (int x=0; x<m_mask.size(0); x++) {
					if (m_mask.cell(x,y)) {
						p1.push_back(m_match.cell(x,y)); // projector
						p2.push_back(slib::make_vector(x, y) ); // camera
					}
				}
			}
		}
	}

	// dump error in fundamental matrix (for debug purpose)
	void dump_epipolar_constraint(
		const slib::CMatrix<3,3,double>& fundamental, 
		const std::string& filename) const
	{
		slib::Field<2,float> error(m_match.size());
		float sum_error = 0;
		for (int y=0; y<m_match.size(1); y++)
		{
			for (int x=0; x<m_match.size(0); x++)
			{
				// camera 
				slib::CVector<2,double> pos = slib::make_vector(x,y);

				// projector 
				slib::CVector<2,double> e1;
				slib::fmatrix::CancelRadialDistortion(m_pro_dist,m_pro_cod,m_match.cell(pos),e1);
				slib::CVector<3,double> p1 = GetHomogeneousVector(e1);

				// camera 
				slib::CVector<2,double> e2;
				slib::fmatrix::CancelRadialDistortion(m_cam_dist,m_cam_cod,pos,e2);
				slib::CVector<3,double> p2 = GetHomogeneousVector(e2);

				// distance to epipolar line
				slib::CVector<3,double> l1 = transpose_of(fundamental) * p1;
				double d1 = abs(dot(l1,p2)) / sqrt(l1[0]*l1[0] + l1[1]*l1[1]);

				slib::CVector<3,double> l2 = fundamental * p2;
				double d2 = abs(dot(l2,p1)) / sqrt(l2[0]*l2[0] + l2[1]*l2[1]);

				// average
				float e = (d1 + d2) / 2;
				error.cell(pos) = e;
				sum_error += e;
			}
		}
		TRACE("sum_error = %e\n", sum_error);

		slib::Field<2,slib::CVector<3,float> > img(m_match.size());
		slib::image::ConvertToJetMap(error, img);
		for (int y=0; y<m_match.size(1); y++)
		{
			for (int x=0; x<m_match.size(0); x++)
			{
				slib::CVector<2,double> pos = slib::make_vector(x,y);
				if (!m_mask.cell(pos))
					img.cell(pos)=slib::make_vector(0,0,0);
			}
		}
		slib::image::AddJetMapLegend(img);
		slib::image::Write(img,filename.c_str());
	}

	void undistort_camera_image(
		const slib::Field<2,slib::CVector<3,float> >& src, slib::Field<2,slib::CVector<3,float> >& dst) const
	{
		dst.Initialize(src.size());
		for (int y=0; y<src.size(1); y++) 
		{
			for (int x=0; x<src.size(0); x++) 
			{
				slib::CVector<2,double> pos = slib::make_vector(x,y); // coordinate without distortion
				slib::fmatrix::ApplyRadialDistortion(m_cam_dist,m_cam_cod,pos,pos); // coordinate with distortion
				if (src.IsInside(pos))
					dst.cell(x,y) = GetInterpolatedCell(src,pos);
			}
		}
	}

	void distort_projector_image(
		const slib::Field<2,slib::CVector<3,float> >& src, slib::Field<2,slib::CVector<3,float> >& dst) const
	{
		dst.Initialize(src.size());
		for (int y=0; y<src.size(1); y++) 
		{
			for (int x=0; x<src.size(0); x++) 
			{
				slib::CVector<2,double> pos = slib::make_vector(x,y); // coordinate without distortion
				slib::fmatrix::CancelRadialDistortion(m_pro_dist,m_pro_cod,pos,pos); // coordinate with distortion
				if (src.IsInside(pos))
					dst.cell(x,y) = GetInterpolatedCell(src,pos);
			}
		}
	}

private:
	// input
	options_t m_options;
	slib::Field<2,slib::CVector<2,double> > m_match;
	slib::Field<2,float> m_mask;
	
	// output
	slib::CMatrix<3,3,double> fundamental;
	slib::CMatrix<3,3,double> m_cam_int;
	slib::CMatrix<3,3,double> m_pro_int;
	slib::CMatrix<3,4,double> m_pro_ext;
	slib::CVector<2,double> m_pro_cod;
	slib::CVector<2,double> m_cam_cod;
	double m_cam_dist;
	double m_pro_dist;
};
