//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: encode.cpp 4054 2010-06-03 15:03:40Z shun $
//

#include "triangulate.h"

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
