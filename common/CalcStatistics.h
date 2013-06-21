#pragma once

namespace slib
{

template <int nDimension, typename T> 
inline
void SafeCovarInverse(T & fDetInvCovar, CMatrix < nDimension,nDimension, T > &matCovar, T fDefaultVar = -1)
{
	CMatrix < nDimension,nDimension, T > matU, matV, matModifiedW;
	CVector < nDimension, T > vecW;

	CalcJacobiTransformation(matCovar, matU, vecW);

	matV = GetTransposed(matU);

	matModifiedW=CMatrix < nDimension,nDimension, T > ::GetZero();
	for (int i = 0; i < nDimension; i++)
	{
		matModifiedW(i, i) = vecW[i] + fDefaultVar;
		_ASSERTE(matModifiedW(i, i) > 0);
	}

	matCovar = matU * matModifiedW * matV;
	fDetInvCovar = CalcDeterminant(matCovar);
	matCovar = GetInverse(matCovar);
}

//--------------------------------------------------------------------
// Gaussian function
//--------------------------------------------------------------------

// 1-dimensional
template <typename T> 
inline
T Gaussian(const T x, const T mean = 0, const T var = 1)
{
	_ASSERTE(var > 0);
	return exp(-(x - mean) * (x - mean) / (2 * var)) / (sqrt(2 * M_PI) * sqrt(var));
}

// n-dimensional
template <int nDimension, typename T> 
inline
T Gaussian(const CVector<nDimension,T> &x, const CVector<nDimension,T> &mean, const CMatrix<nDimension,nDimension,T> &covar)
{
	return GaussianExt(x, mean, CalcDeterminant(invCovar), GetInverse(covar));
}

template <int nDimension, typename T> 
inline
T GaussianExt(const CVector<nDimension,T> &x, const CVector<nDimension,T> &mean, const T& detInvCovar, const CMatrix<nDimension,nDimension,T> &invCovar)
{
	_ASSERTE(detInvCovar > 0);
	CVector<nDimension,T> d = x - mean;
	return T(exp(-0.5 * dot(d, invCovar * d)) * sqrt(detInvCovar) / pow(2 * M_PI, nDimension / 2.0));
}

//--------------------------------------------------------------------
// pick a random value from this Gaussian distribution
//
// implemented using the Box-Muller transformation
// refer to:
// http://mathworld.wolfram.com/Box-MullerTransformation.html
// http://mathworld.wolfram.com/GaussianDistribution.html
//--------------------------------------------------------------------
template <typename T> 
inline
T GaussianRandom(const T mean = 0,	//< mean of distribution
					   const T stddev = 1,	//< standard deviation of distribution
					   double x1 = (double)rand() / (RAND_MAX + 1), //< random number ranging 0 <= x1 < 1
					   double x2 = (double)rand() / (RAND_MAX + 1)	//< random number ranging 0 <= x2 < 1
					   )
{
	_ASSERTE(0 <= x1 && x1 < 1);
	_ASSERTE(0 <= x2 && x2 < 1);
	_ASSERTE(stddev > 0);
	return T(stddev * sqrt(-2 * log(1 - x1)) * cos(2 * M_PI * x2) + mean);
	//  return T( stddev * sqrt(-2 * log(1-x1)) * sin (2 * M_PI * x2) + mean);
}

//--------------------------------------------------------------------
// statistics
//--------------------------------------------------------------------
template <typename T>
inline
bool CalcCorrelationCoefficient(const std::vector<T> &vecData1, const std::vector<T> &vecData2, T & dCorrelation)
{
	int sz = std::min(vecData1.size(), vecData2.size());

	T o_mean = 0, s_mean = 0;
	for (int i = 0; i < sz; i++)
	{
		o_mean += vecData1[i];
		s_mean += vecData2[i];
	}

	o_mean /= sz;
	s_mean /= sz;

	T os = 0, oo = 0, ss = 0;
	for (int i = 0; i < sz; i++)
	{
		oo += (vecData1[i] - o_mean) * (vecData1[i] - o_mean);
		ss += (vecData2[i] - s_mean) * (vecData2[i] - s_mean);
		os += (vecData1[i] - o_mean) * (vecData2[i] - s_mean);
	}

	if (oo * ss < 1e-100)
	{
		TRACE("data variance is too small.\n");
		return false;
	}

	dCorrelation = std::max(-1.0, std::min(1.0, os / sqrt(oo * ss)));	// correlation coefficient

	return true;
}

// see section.14.5 in numerical recipes
template <typename T>
inline
T CalcFalsePositiveProbability(const T dCorrelationCoefficient, const int nNumUsedData)
{
	if (fabs(dCorrelationCoefficient) == 1)
		return 0;

	T dFisherZ      = 0.5 * log((1 + dCorrelationCoefficient) / (1 - dCorrelationCoefficient));
	T dSignificance = erfcc(fabs(dFisherZ) * sqrt(nNumUsedData - 3.0) / sqrt(2.0));

	return dSignificance;
}

// see section.14.2 in numerical recipes
template <typename T>
inline
T CalcSignificance(const T dCorrelationCoefficient, const int nNumUsedData)
{
	if (fabs(dCorrelationCoefficient) == 1)
		return 0;

	T df   = nNumUsedData-2;
	T dStudentT = dCorrelationCoefficient * sqrt(df/((1.0-dCorrelationCoefficient)*(1.0+dCorrelationCoefficient)));
	return betai(0.5*df, 0.5, df/(df+dStudentT*dStudentT));
}

inline
double factorial(int d)
{
	if (d < 2)	
		return 1;
	else		
		return factorial(d-1) * d;
}

}
