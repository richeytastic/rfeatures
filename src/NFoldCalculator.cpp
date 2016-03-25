#include "NFoldCalculator.h"

void RFeatures::calcValidationFoldRange( int N, int foldi, int NFOLDs, int& s0, int& r0)
{
    const int baseSz = N / NFOLDs;
    const int remSz = N % NFOLDs;
    r0 = foldi < remSz ? baseSz+1 : baseSz; // The first remSz folds have size baseSz+1
    s0 = foldi * (baseSz+1);
    if ( foldi > remSz)
        s0 -= foldi - remSz;
}   // end calcValidationFoldRange


void RFeatures::calcTrainingFoldRange( int N, int foldi, int NFOLDs, int& s0, int& r0, int& s1, int& r1)
{
    int vi, vr;
    calcValidationFoldRange( N, foldi, NFOLDs, vi, vr);
    s0 = 0;
    r0 = vi - s0;
    s1 = vi + vr;
    r1 = N - s1;
}   // end calcTrainingFoldRange

