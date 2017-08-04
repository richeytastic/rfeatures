/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

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
