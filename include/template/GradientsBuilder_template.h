
template <typename T>
double GradientsBuilder::calcGradient( const cv::Mat_<T> &img, int ridx, int cidx, double &theta)
{
    const int channels = img.channels();

    // Get the horizontal and vertical gradients from the channels of img
    // and find the channel gradient with the largest delta. Then use this
    // channel to find the gradient of the pixel.
    double hg1 = RFeatures::calcHorizontalGrad<T>( img, ridx, cidx, 0);
    double vg1 = RFeatures::calcVerticalGrad<T>( img, ridx, cidx, 0);

    double mag0 = pow(hg1,2) + pow(vg1,2);
    double mag = mag0;
    double hg = hg1;
    double vg = vg1;

    for ( int k = 1; k < channels; ++k)
    {
        hg1 = RFeatures::calcHorizontalGrad<T>( img, ridx, cidx, k);
        vg1 = RFeatures::calcVerticalGrad<T>( img, ridx, cidx, k);
        mag0 = pow(hg1,2) + pow(vg1,2);

        if ( mag0 > mag)
        {
            mag = mag0;
            hg = hg1;
            vg = vg1;
        }   // end if
    }   // end for

    theta = atan2( vg, hg) + CV_PI; // theta in [0,2pi]
    return sqrt(mag)/2;
}   // end calcGradient



template <typename T>
void GradientsBuilder::calculateGradients( const cv::Mat_<T> &img, int nbs, bool dd, std::vector<cv::Mat_<double> >& gradients)
{
    gradients.resize(nbs);
    for ( int i = 0; i < nbs; ++i)
        gradients[i] = cv::Mat_<double>::zeros( img.size());

    double binRads = CV_PI / nbs;
    if ( dd)    // Over full circle (2*pi/nbs)
        binRads *= 2;

    static const double EPS = 1e-9;
    const int nrows = img.rows;
    const int ncols = img.cols;
    for ( int i = 0; i < nrows; ++i)
    {
        for ( int j = 0; j < ncols; ++j)
        {
            double theta;
            double mag = GradientsBuilder::calcGradient( img, i, j, theta);
            if ( mag >= EPS)
                GradientsBuilder::setPixelGradient( i, j, mag, theta, binRads, gradients);
        }   // end for - cols
    }   // end for - rows
}   // end calculateGradients
