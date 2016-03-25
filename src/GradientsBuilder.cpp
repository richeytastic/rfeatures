#include "GradientsBuilder.h"
using RFeatures::GradientsBuilder;


void GradientsBuilder::setPixelGradient( int row, int col, double mag, double theta, double binRads, std::vector<cv::Mat_<double> >& gradients)
{
    const int nbins = gradients.size();
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
