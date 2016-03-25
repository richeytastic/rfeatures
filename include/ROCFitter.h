#pragma once
#ifndef RFEATURES_ROC_FITTER
#define RFEATURES_ROC_FITTER

#include <vector>
using std::vector;
typedef unsigned int uint;

namespace RFeatures
{

class ROCFitter
{
public:
    ROCFitter( const vector<double> &posVals, const vector<double> &negVals);

    // Find the true/false positive/negatives over all data given threshold t.
    void getMetrics( double t, uint &tp, uint &fn, uint &tn, uint &fp) const;

    // Collect *steps* many false positive ratios and true positive ratio datums.
    // Returns the area under the curve (AUC). Chart should be plotted with fprs as
    // the independent variable and tprs as the dependent variable.
    double getROCData( uint steps, vector<double> &fprs, vector<double> &tprs) const;

private:
    double maxThresh, minThresh;
    vector<double> posVals; // Responses from positive examples (TPs >= t, FNs < t)
    vector<double> negVals; // Responses from negative examples (TNs < t, FPs >= t)
};  // end class

}   // end namespace

#endif
