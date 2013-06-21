//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4054 2010-06-03 15:03:40Z shun $
//

#include "stdafx.h"

#include "MathBaseLapack.h"
#include "Field.h"
#include "MiscUtil.h"
#include "ImageBmpIO.h"
#include "Stereo.h"
#include "LeastSquare.h"

#include "../FundamentalMatrix.h"
#include "../Options.h"

struct options_t options;

using namespace slib;

// output filename
static char *m_plyfilename = "mesh.ply";

// filename of optional vertical correspondence 
static char *m_vmapfilename = 0;

static float m_max_edge_length = 0.1;
static float m_distortion_angle = 1;
static bool m_debug = false;

bool distorted(const int v1,const int v2,const int v3,const std::vector<CVector<3,double> >& result) 
{
	if (result[v1][2]<0 || result[v2][2]<0 || result[v3][2]<0)
		return true;

	CVector<3,double> d12 = result[v2]-result[v1];
	CVector<3,double> d23 = result[v3]-result[v2];
	CVector<3,double> d31 = result[v1]-result[v3];
	double n12 = GetNorm2(d12);
	double n23 = GetNorm2(d23);
	double n31 = GetNorm2(d31);

	if (n12>m_max_edge_length || n23>m_max_edge_length || n31>m_max_edge_length)
		return true;

	double cos1=dot(d12,-d31)/(n12*n31);
	double cos2=dot(-d12,d23)/(n12*n23);
	double cos3=dot(-d23,d31)/(n23*n31);
	double maxcos = std::max(std::max(cos1,cos2),cos3);

	double t = cos(m_distortion_angle*M_PI/180);
	return maxcos > t;
}

void WritePly(const std::vector<CVector<3,double> >& result, const Field<2,float>& mask, std::string filename)
{
	// generate face indices
	Field<2,int> index(mask.size());
	for (int y=0, idx=0; y<mask.size(1); y++)
		for (int x=0; x<mask.size(0); x++)
			if (mask.cell(x,y))
				index.cell(x,y)=idx++;
			else
				index.cell(x,y)=-1;

	std::vector<CVector<3,int> > face;
	for (int y=0; y<mask.size(1)-1; y++)
	{
		for (int x=0; x<mask.size(0)-1; x++)
		{
			// sore in CCW order
			if (mask.cell(x,y) && mask.cell(x+1,y) && mask.cell(x+1,y+1) && 
				!distorted(index.cell(x,y),index.cell(x+1,y),index.cell(x+1,y+1),result))
				face.push_back(make_vector(index.cell(x,y),index.cell(x+1,y+1),index.cell(x+1,y)));
			if (mask.cell(x,y) && mask.cell(x+1,y+1) && mask.cell(x,y+1) && 
				!distorted(index.cell(x,y),index.cell(x+1,y+1),index.cell(x,y+1),result))
				face.push_back(make_vector(index.cell(x,y),index.cell(x,y+1),index.cell(x+1,y+1)));
		}
	}

	TRACE("ply => %s\n",filename.c_str());
	FILE *fw = fopen(filename.c_str(), "wb");
	fprintf(fw,
			"ply\n"
			"format binary_little_endian 1.0\n"
			"element vertex %d\n"
			"property float x\n"
			"property float y\n"
			"property float z\n"
			"element face %d\n"
			"property list uchar int vertex_indices\n"
			"end_header\n", result.size(), face.size());
	for (int i=0; i<result.size(); i++)
	{
		CVector<3,float> p = make_vector(result[i][0],result[i][1],result[i][2]);
		fwrite(&p, 12, 1, fw);
	} 
	for (int i=0; i<face.size(); i++)
	{
		fputc(3,fw);
		fwrite(&face[i], 12, 1, fw);
	}

	fclose(fw);
}

// print usage and exit
void print_usage(const char *argv0)
{
	const char *prog = strrchr(argv0, '\\');
	if (prog)
		prog++;
	else
		prog = argv0;
	printf("Usage: %s [-o <file.ply>] [-t <val>] [-v <v.map>] <h.map> <mask.bmp> <options> <cam-intrinsic> <cam-distortion> <pro-intrinsic> <pro-distortion> <pro-extrinsic>\n", prog);
	printf("-o <file.ply>: Export triangular mesh to <file.ply> in PLY format. (default: mesh.ply)\n");
	printf("-t <val>: Min angle of triangles. (default: %f)\n", m_distortion_angle);
	printf("-v <v.map>: Triangulate using vertical correspondence as well as horizontal. (default: empty)\n");
	printf("<h.map>: Specify horizontal correspondence between projector and camera.\n");
	printf("<mask.bmp>: Specify mask image of valid pixels.\n");
	printf("<cam-intrinsic>: Specify 3x3 camera intrinsic matrix.\n");
	printf("<cam-distortion>: Specify 3x3 camera intrinsic matrix.\n");
	printf("<pro-distortion>: Specify 3x3 projector intrinsic matrix.\n");
	printf("<pro-intrinsic>: Specify 3x3 projector intrinsic matrix.\n");
	printf("<pro-extrinsic>: Specify 3x4 projector extrinsic matrix\n");
	exit(-1);
}

// parse commandline options
int set_options(int argc, char *argv[])
{
	int argi = 1;
	while (argi < argc && argv[argi][0] == '-')
	{
		switch (argv[argi][1])
		{
		case 'o':
			m_plyfilename = argv[argi+1];
			argi+=2;
			break;
		case 't':
			m_distortion_angle = atof(argv[argi+1]);
			argi+=2;
			break;
		case 'v':
			m_vmapfilename = argv[argi+1];
			argi+=2;
			break;
		case 'd':
			m_debug = true;
			argi++;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}

	if (argi != argc - 8)
		print_usage(argv[0]);

	return argi;
}

// entry point
int main(int argc, char* argv[])
{
	try
	{
		// parse commandline options
		int argi = set_options(argc, argv);

		// horizontal and vertical correspondences between projector and camera
		Field<2,float> horizontal, vertical, mask;
		horizontal.Read(argv[argi++]);
		if (m_vmapfilename)
			vertical.Read(m_vmapfilename);
		image::Read(mask, argv[argi++]);
		options.load(argv[argi++]);
		CVector<2,double>
			cod1=make_vector<double>((options.projector_width+1)/2.0,options.projector_height*options.projector_horizontal_center),
			cod2=(make_vector(1.0,1.0)+mask.size())/2;

		// intrinsic matrices of projector and camera
		CMatrix<3,3,double> matKpro, matKcam;
		double xi1,xi2;
		FILE*fr;
		matKcam.Read(argv[argi++]);
		fr=fopen(argv[argi++],"rb");
		if (!fr) throw std::runtime_error("failed to open camera distortion");
		fscanf(fr,"%lf",&xi2);
		fclose(fr);
		matKpro.Read(argv[argi++]);
		fr=fopen(argv[argi++],"rb");
		if (!fr) throw std::runtime_error("failed to open projector distortion");
		fscanf(fr,"%lf",&xi1);
		fclose(fr);

		// extrinsic matrices of projector and camera
		CMatrix<3,4,double> proRt, camRt;
		proRt.Read(argv[argi++]);
		camRt = make_diagonal_matrix(1,1,1).AppendCols(make_vector(0,0,0));//CMatrix<3,4,double>::GetIdentity(); // reconstruction is in camera coordinate frame

		// compute projection matrices of projector and camera
		std::vector<CMatrix<3,4,double> > matrices(2);
		matrices[0] = matKcam * camRt; // camera
		matrices[1] = matKpro * proRt; // projector

		// fundamental matrix
		CMatrix<3,3,double> matR;
		CVector<3,double> vecT;
		matR.Initialize(proRt.ptr());
		vecT.Initialize(proRt.ptr()+9);
		CMatrix<3,3,double> matF = transpose_of(inverse_of(matKpro)) * GetSkewSymmetric(vecT) * matR * inverse_of(matKcam);

		// triangulate 3d points
		std::vector<CVector<3,double> > result;
		for (int y=0; y<horizontal.size(1); y++)
		{
			if (y % (horizontal.size(1)/100) == 0)
				printf("\rtriangulation: %d%% done", 100*y/horizontal.size(1));

			int nbehind=0;
			for (int x=0; x<horizontal.size(0); x++)
			{
				if (mask.cell(x,y))
				{
					// 2D correspondence
					std::vector<CVector<2,double> > p2d(2);

					// camra coordinate
					slib::fmatrix::CancelRadialDistortion(xi2,cod2,make_vector<double>(x,y),p2d[0]);

					// projector coordinate
					double proj_y;
					if (m_vmapfilename)
					{
						proj_y = vertical.cell(x,y);
					}
					else
					{
						CVector<3,double> epiline = matF * GetHomogeneousVector(p2d[0]);
						proj_y = -(epiline[0] * horizontal.cell(x,y) + epiline[2]) / epiline[1];
					}
					slib::fmatrix::CancelRadialDistortion(xi1,cod1,make_vector<double>(horizontal.cell(x,y),proj_y),p2d[1]);

					// triangulate
					CVector<3,double> p3d;
					SolveStereo(p2d, matrices, p3d);

					// save
					result.push_back(p3d);
					if (p3d[2]<0)
						nbehind++;
				}
			}
			if (m_debug && nbehind)
				TRACE("\rfound %d points behind viewpoint.\n", nbehind);
		}
		printf("\n");
		// export triangular mesh in PLY format
		WritePly(result, mask, m_plyfilename);
	}
	catch (const std::exception& e)
	{
		TRACE("error: %s\n", e.what());
		return -1;
	}
	return 0;
}

