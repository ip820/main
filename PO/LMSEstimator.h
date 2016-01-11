// LMSEstimator.h: interface for the LMSEstimator class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LMSESTIMATOR_H__20976C82_7591_11D5_AB38_00207818AC0F__INCLUDED_)
#define AFX_LMSESTIMATOR_H__20976C82_7591_11D5_AB38_00207818AC0F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Array.h"
#include "Matrix.h"

class LMSEstimator  
{
	Matrix fDist;
public:
	LMSEstimator() { }
	virtual ~LMSEstimator() { }

	// Read F-Distribution

	bool ReadFDist(const std::string &fname);
	double PickFDist(const int degFr, const int alpId) const;

	// Estimation
	bool Estimate(const Matrix &y, const Matrix &A, Matrix &x) const;
	bool Estimate(const Matrix &y, const Matrix &A, Matrix &x, Matrix &Sx, Matrix &N, double &varComp) const;

	// Test and Update with one additional observation y1
	bool TestUpdate(const double y1, const Matrix &A1, Matrix &x, Matrix &Sx, double &varComp, const int nY, const int nAlpha) const;
	bool HypothesisTest(const double y1, const Matrix &A1, const Matrix &x, const Matrix &N, const double varComp, const int nY, const int nAlpha) const;
	bool SequentialEstimate(const double y1, const Matrix &A1, Matrix &x, Matrix &Sx, Matrix &N, double &varComp, const int nY) const;
};

#endif // !defined(AFX_LMSESTIMATOR_H__20976C82_7591_11D5_AB38_00207818AC0F__INCLUDED_)
