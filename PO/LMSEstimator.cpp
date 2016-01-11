// LMSEstimator.cpp: implementation of the LMSEstimator class.
//
//////////////////////////////////////////////////////////////////////

#include "LMSEstimator.h"
//declare the start
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

bool LMSEstimator::ReadFDist(const std::string &fname)
{
	// 0.975, 0.95, 0.90 for n = 0, 1, 2 respectively.
	fDist.SetDim(10000,3);

	FILE *fp;
	fopen_s(&fp, fname.c_str(), "r");
	char buf[200];
	double a,b,c;
	if(fp == NULL) return false;
	for(int n=0; n<10000 && !feof(fp); )
	{
		fgets(buf, 199, fp);
		if(sscanf_s(buf, "%lf%lf%lf", &a, &b, &c)==3)
		{
			fDist.Set(n,0,a);
			fDist.Set(n,1,b);
			fDist.Set(n,2,c);
			n++;
		}
	}
	fclose(fp);
	return true;
}

double LMSEstimator::PickFDist(const int degFr, const int alpId) const
{
	if ( degFr < fDist.NumRow () )
	{
		return fDist.Get ( degFr, alpId );
	}
	else
	{
		return fDist.Get ( fDist.NumRow () - 1, alpId );
	}
}

bool LMSEstimator::Estimate(const Matrix &y, const Matrix &A, Matrix &x) const
{
	Matrix N = A.Trans() * A;

	double det = N.Det();
	if(det == 0.0 || det <= 1e-10) return false;
	x = N.Inv() * A.Trans() * y;
	return true;
}

bool LMSEstimator::Estimate(const Matrix &y, const Matrix &A, Matrix &x, Matrix &Sx, Matrix &N, double &varComp) const
{
	int i,j,k,m,n;
	double sum;
	A.Dim(m,n);
	x.SetDim(n,1);
	Sx.SetDim(n,n);
	N.SetDim(n,n);
	Matrix c(n,1);
	Matrix et(m,1);

	// Compute Normal Matrix N
	for (i=0; i<n; i++)
	{
		for (j=0; j<n; j++)
		{
			sum = 0;
			for(k=0; k<m; k++) sum += (A.Get(k,i) * A.Get(k,j));
			N.Set(i, j, sum);
		}
	}

	// Compute c
	for (i=0; i<n; i++)
	{
		sum = 0;
		for(k=0; k<m; k++) sum += (A.Get(k,i) * y.Get(k));
		c.Set(i, sum);
	}

	// Compute Sx
//	N.Inv(Sx);
	Sx = N.Inv();

	// Compute estimate of unknowns
	for (i=0; i<n; i++)
	{
		sum = 0;
		for(k=0; k<n; k++) sum += (Sx.Get(i,k) * c.Get(k));
		x.Set(i,0,sum);
	}

	// Compute estimate of erros
	for (i=0; i<m; i++)
	{
		sum = 0;
		for (j=0; j<n; j++) sum += (A.Get(i,j) * x.Get(j));
		et.Set(i, y.Get(i) - sum);
	}

	// Compute estimate of variance component
	varComp = 0;
	for (i=0; i<m; i++) varComp += (et.Get(i) * et.Get(i));
	varComp /= (m-n);
	return true;
}

bool LMSEstimator::SequentialEstimate(const double y1, const Matrix &A1, Matrix &x, Matrix &Sx, Matrix &N, double &varComp, const int nY) const
{
	unsigned int m,n;
	double diff, numer = 1;

	for (m=0; m<x.size(); m++)
		for (n=0; n<x.size(); n++)
			numer += N.Get(m,n) * x.Get(m) * x.Get(n);

	diff = y1 - A1.Get(0) * x.Get(0) + A1.Get(1) * x.Get(1) + A1.Get(2) * x.Get(2);

	Matrix AN(3,1);
	for (n=0; n<x.size(); n++)
		AN.Set(n, N.Get(n,0) * A1.Get(0) + N.Get(n,1) * A1.Get(1) + N.Get(n,2) * A1.Get(2));

	for (n=0; n<x.size(); n++)
		x.Set(n, 0, x.Get(n) + AN.Get(n) * diff / numer);

	for (m=0; m<x.size(); m++)
		for (n=0; n<x.size(); n++)
			Sx.Set(m,n, Sx.Get(m,n) - AN.Get(m) * AN.Get(n) / numer);

	varComp = (varComp * (nY-3) + diff * diff / numer) / (nY-2);
	return true;
}

bool LMSEstimator::HypothesisTest(const double y1, const Matrix &A1, const Matrix &x, const Matrix &N, const double varComp, const int nY, const int nAlpha) const
{
	unsigned int m,n;
	double T, R, denom, numer = 1;
	for (m=0; m<x.size(); m++)
		for (n=0; n<x.size(); n++)
			numer += N.Get(m,n) * x.Get(m) * x.Get(n);
	denom = A1.Get(0) * x.Get(0) + A1.Get(1) * x.Get(1) + A1.Get(2) * x.Get(2) - y1;
	denom *= denom;
	R = denom / numer;
	T = R / varComp;	// Test Statistic ~ F(1, n-3)
	if(T >= PickFDist(nY-x.size()-1, nAlpha)) return false;
	else return true;
}

bool LMSEstimator::TestUpdate(const double y1, const Matrix &A1, Matrix &x, Matrix &Sx, double &varComp, const int nY, const int nAlpha) const
{
	unsigned int m,n;
	double T, diff, numer = 1;
	for (m=0; m<x.size(); m++)
		for (n=0; n<x.size(); n++)
			numer += Sx.Get(m,n) * A1.Get(m) * A1.Get(n);
	diff = y1 - A1.Get(0) * x.Get(0) - A1.Get(1) * x.Get(1) - A1.Get(2) * x.Get(2);

//	FILE *fp = fopen("test1.txt", "a");

	// Hypothesis Test
	T = (diff * diff) / numer / varComp;	// Test Statistic ~ F(1, n-3)
//	fprintf(fp, "numer: %lf diff: %lf T: %lf\n", numer, diff, T);
	if(T >= PickFDist(nY-x.size()-1, nAlpha)) return false;

	Matrix AN(3,1);
	for (n=0; n<x.size(); n++)
		AN.Set(n, Sx.Get(n,0) * A1.Get(0) + Sx.Get(n,1) * A1.Get(1) + Sx.Get(n,2) * A1.Get(2));
//	fprintf(fp, "%s\n", AN.Str("A * N"));

	// Update
	for (n=0; n<x.size(); n++)
		x.Set(n, 0, x.Get(n) + AN.Get(n) * diff / numer);
	for (m=0; m<x.size(); m++)
		for (n=0; n<x.size(); n++)
			Sx.Set(m,n, Sx.Get(m,n) - AN.Get(m) * AN.Get(n) / numer);
	varComp = (varComp * (nY-3) + diff * diff / numer) / (nY-2);
/*
	fprintf(fp, "%s\n", x.Str("Updated Parameters"));
	fprintf(fp, "%s\n", Sx.Str("Updated Variance of Parameters"));
	fprintf(fp, "Variance Component: %lf\n", varComp);
	fclose(fp);
*/
	return true;
}
