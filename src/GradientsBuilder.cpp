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

#include "GradientsBuilder.h"
using RFeatures::GradientsBuilder;


void GradientsBuilder::setPixelGradient( int row, int col, double mag, double theta, double binRads, std::vector<cv::Mat_<double> >& gradients)
{
    const int nbins = (int)gradients.size();
    // Discretise theta into a bin, find the bins to the left and the right, and the
    // proportions of the gradient magnitude that each bin has for this pixel.
    const double thetaDiv = theta/binRads;
    const int cbin = int(thetaDiv) % nbins;       // Centre bin index in [0,nbins)
    const int rbin = (cbin + 1) % nbins;             // Right bin index
    const int lbin = (cbin +nbins - 1) % nbins;       // Left bin index
    const double rprop = std::max<double>(thetaDiv - cbin, 0);
    const double lprop = std::max<double>(1.0 - rprop, 0);
    gradients[lbin].ptr<double>(row)[col] += lprop*mag;
    gradients[cbin].ptr<double>(row)[col] += mag;
    gradients[rbin].ptr<double>(row)[col] += rprop*mag;
}   // end setPixelGradient
