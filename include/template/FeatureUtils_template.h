template <typename T>
ostream& operator<<( ostream &os, const cv::Size_<T> &s)
{
    return os << s.width << "," << s.height;
}   // end operator<<


template <typename T>
istream& operator>>( istream &is, cv::Size_<T> &s)
{
    char ch;
    return is >> s.width >> ch >> s.height;
}   // end operator>>


template <typename T>
ostream& operator<<( ostream &os, const cv::Rect_<T> &r)
{
    return os << "[" << r.x << "," << r.y << "," << r.width << "," << r.height << "]";
}   // end operator<<


template <typename T>
istream& operator>>( istream &is, cv::Rect_<T> &r)
{
    char ch;
    return is >> ch >> r.x >> ch >> r.y >> ch >> r.width >> ch >> r.height >> ch;
}   // end operator>>



template <typename T>
double calcHorizontalGrad( const cv::Mat_<T> &image, int row, int col, int ch)
{
    const int channels = image.channels();
    double grad = 0;

    assert( ch >= 0 && ch < channels);
    if ( col > 0 && col < image.cols - 1)
        grad = (double)image(row,col+1)[ch] - (double)image(row,col-1)[ch];
    else if ( col == 0) // At left edge
        grad = (double)image(row,col+1)[ch] - (double)image(row,col)[ch];
    else    // At right edge
        grad = (double)image(row,col)[ch] - (double)image(row,col-1)[ch];

    return grad;
}   // end calcHorizontalGrad



template <typename T>
double calcHorizontalGrad2( const cv::Mat_<T> &image, int row, int col, int ch)
{
    const int channels = image.channels();
    double grad = 0;

    assert( ch >= 0 && ch < channels);
    if ( col > 0 && col < image.cols - 1)
    {
        const double d1 = (double)image(row,col+1)[ch] - (double)image(row,col)[ch];
        const double d2 = (double)image(row,col)[ch] - (double)image(row,col-1)[ch];
        grad = d1 - d2; // Second derivative estimate
    }   // end if
    else if ( col == 0) // At left edge
        grad = (double)image(row,col+1)[ch] - (double)image(row,col)[ch];
    else    // At right edge

    return grad;
}   // end calcHorizontalGrad2



template <typename T>
double calcVerticalGrad( const cv::Mat_<T> &image, int row, int col, int ch)
{
    const int channels = image.channels();
    double grad = 0;

    assert( ch >= 0 && ch < channels);
    if ( row > 0 && row < image.rows - 1)
        grad = (double)image(row+1,col)[ch] - (double)image(row-1,col)[ch];
    else if ( row == 0) // At top row
        grad = (double)image(row+1,col)[ch] - (double)image(row,col)[ch];
    else    // At bottom row
        grad = (double)image(row,col)[ch] - (double)image(row-1,col)[ch];

    return grad;
}   // end calcVerticalGrad



template <typename T>
double calcVerticalGrad2( const cv::Mat_<T> &image, int row, int col, int ch)
{
    const int channels = image.channels();
    double grad = 0;

    assert( ch >= 0 && ch < channels);
    if ( row > 0 && row < image.rows - 1)
    {
        const double d1 = (double)image(row+1,col)[ch] - (double)image(row,col)[ch];
        const double d2 = (double)image(row,col)[ch] - (double)image(row-1,col)[ch];
        grad = d1 - d2; // Second derivative estimate
    }   // end if
    else if ( row == 0) // At top row
        grad = (double)image(row+1,col)[ch] - (double)image(row,col)[ch];
    else    // At bottom row
        grad = (double)image(row,col)[ch] - (double)image(row-1,col)[ch];

    return grad;
}   // end calcVerticalGrad2



template <typename T>
T getIntegralImageSum( const cv::Mat& iimg, const cv::Rect& rct)
{
    const int x1 = rct.x;
    const int y1 = rct.y;
    const int x2 = rct.x + rct.width;
    const int y2 = rct.y + rct.height;
    return iimg.at<T>(y2,x2) - iimg.at<T>(y2,x1) - iimg.at<T>(y1,x2) + iimg.at<T>(y1,x1);
}   // end getIntegralImageSum



template <typename T>
double calcVariance( const cv::Mat& iim, const cv::Mat_<double>& iimsq, const cv::Rect& rct, cv::Mat_<int> maskii)
{
    double sm = (double)getIntegralImageSum<T>( iim, rct);
    int N = rct.area();
    if ( !maskii.empty())
        N = getIntegralImageSum<int>( maskii, rct);
    double mn = sm / N;
    return ( getIntegralImageSum<double>( iimsq, rct) - 2*mn*sm + N*mn*mn) / N;  // Uses binomial theorem
}   // end calcVariance


template <typename T>
cv::Mat_<int> createMaskIntegralImage( const cv::Mat& m, T minv, T maxv, cv::Mat_<byte>& mask)
{
    mask = (m >= minv) & (m <= maxv);
    mask /= 255;    // mask is now zeros and ones (where m is in range)
    cv::Mat iim;
    cv::integral( mask, iim, CV_32S);
    return iim;
}   // end createMaskIntegralImage





