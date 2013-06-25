//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageMagickIO.h 4304 2010-12-07 09:02:51Z shun $
//

#pragma once

#include <stdlib.h>

namespace slib
{

// estimator_t: paramtype (*)(const std::vector<datatype>&;
// evaluator_t: double (*)(const datatype&, const paramtype&);
template <typename datatype, typename estimator_t, typename evaluator_t, typename paramtype> inline
void Ransac(
	const std::vector<datatype>& data, 
	const estimator_t estimator, 
	const evaluator_t evaluator,
	const int nsamples, 
	const double maxerror, 
	const double maxinlier_rate, // 0.95
	paramtype& param)
{
	int niteration = 1000;
	int ndata = data.size();
	int best_inliers = 0;
	std::vector<datatype> subset(nsamples);
	int i=0;
	for (; i<niteration; i++)
	{
		// choose samples
		std::vector<bool> sampled(ndata,false);
		for (int s=0; s<nsamples; s++)
		{
			int r;
			do {
				r = (float)rand()/(RAND_MAX+1)*ndata; 
			} while (sampled[r]);
			sampled[r] = true;
			subset[s] = data[r];
		}

		// estimate using a current set
		paramtype current_estimate = estimator(subset);

		// count number of inliers
		int ninliers=0;
		for (int s=0; s<ndata; s++)
		{
			double error = evaluator(data[s], current_estimate);
			if (error < maxerror)
				ninliers++;
		}

		// pick the best estimate
		if (best_inliers<ninliers)
		{
			best_inliers=ninliers;
			param=current_estimate;

			if (best_inliers==ndata)
			{
				TRACE("warning: all inliers. perhaps too small large threshold?\n");
				break;
			}
			if (maxinlier_rate*data.size()<=best_inliers)
			{
				break;
			}

			if (best_inliers > ndata/2)
			{
				// rate of inliers
				double w = (float)best_inliers / ndata;
				// the probability of selecting only inliers  
				double wn = pow(w,nsamples);
				// the probability that the RANSAC algorithm selects only inliers from the input data set
				double p = 0.99;
				int required_iteration = log(1-p)/log(1-wn);
				niteration = i + required_iteration;
			}
		}
	}
	if (best_inliers < ndata/2)
		TRACE("warning: too few inliers (%f%%)\n", 100.0*best_inliers/ndata);
	TRACE("found %d inliers out of %d after %d iterations.\n", best_inliers, data.size(), i);
}

template <typename datatype, typename evaluator_t, typename paramtype> inline
int FindInliers(
	const std::vector<datatype>& data, 
	const evaluator_t evaluator,
	const double maxerror, 
	const paramtype& param,
	std::vector<datatype>& inliers)
{
	inliers.clear();
	int ndata = data.size();
	for (int i=0; i<ndata; i++)
		if (evaluator(data[i], param) < maxerror)
			inliers.push_back(data[i]);
	return inliers.size();
}

} // namespace slib
