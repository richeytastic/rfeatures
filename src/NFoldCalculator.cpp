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

