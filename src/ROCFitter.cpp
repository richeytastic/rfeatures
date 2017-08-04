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

#include "ROCFitter.h"
using RFeatures::ROCFitter;
#include <cfloat>



ROCFitter::ROCFitter( const vector<double> &pvs, const vector<double> &nvs)
    : maxThresh(FLT_MIN), minThresh(FLT_MAX)
{
    // Find the min/max thresholds
    for ( uint i = 0; i < pvs.size(); ++i)
    {
        const double &m = pvs[i];
        if ( m < minThresh) minThresh = m;
        if ( m > maxThresh) maxThresh = m;
        posVals.push_back( m);
    }   // end for

    for ( uint i = 0; i < nvs.size(); ++i)
    {
        const double &m = nvs[i];
        if ( m < minThresh) minThresh = m;
        if ( m > maxThresh) maxThresh = m;
        negVals.push_back( m);
    }   // end for
}   // end ctor



void ROCFitter::getMetrics( double t, uint &tp, uint &fn, uint &tn, uint &fp) const
{
    tp = 0;
    fn = 0;
    tn = 0;
    fp = 0;

    for ( uint i = 0; i < posVals.size(); ++i)
    {
        if ( posVals[i] >= t) tp++;
        else fn++;
    }   // end for

    for ( uint i = 0; i < negVals.size(); ++i)
    {
        if ( negVals[i] < t) tn++;
        else fp++;
    }   // end for
}   // end getMetrics



double ROCFitter::getROCData( uint steps, vector<double> &fprs, vector<double> &tprs) const
{
    fprs.clear();
    tprs.clear();

    double auc = 0;
    double fpr0 = 1, tpr0 = 1;  // Remember previous sides of trapezoids for AUC calc

    const double stepSz = (maxThresh - minThresh)/steps;
    for ( uint k = 0; k <= steps; ++k)
    {
        double t = minThresh + k*stepSz; // This threshold value
        uint tp, fn, tn, fp;
        getMetrics( t, tp, fn, tn, fp);

        double fpr = (double)fp/(fp+tn);
        double tpr = (double)tp/(tp+fn);
        tprs.push_back(tpr);
        fprs.push_back(fpr);

        // Sum the trapezoids for the area under the curve
        auc += (fpr0 - fpr) * (tpr + tpr0)/2;
        tpr0 = tpr;
        fpr0 = fpr;
    }   // end for

    return auc;
}   // end getROCData
