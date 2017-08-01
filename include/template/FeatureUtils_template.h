template <typename T>
T min( T a, T b, T c)
{
    return std::min<T>( a, std::min<T>( b, c));
}   // end min


template <typename T>
T max( T a, T b, T c)
{
    return std::max<T>( a, std::max<T>( b, c));
}   // end max


template <typename T>
T min( T a, T b)
{
    return std::min<T>( a, b);
}   // end min


template <typename T>
T max( T a, T b)
{
    return std::max<T>( a, b);
}   // end max

    
template <typename T>
ostream& operator<<( ostream &os, const cv::Size_<T> &s)
{
    return os << s.width << " " << s.height;
}   // end operator<<


template <typename T>
istream& operator>>( istream &is, cv::Size_<T> &s)
{
    return is >> s.width >> s.height;
}   // end operator>>


template <typename T>
ostream& operator<<( ostream &os, const cv::Rect_<T> &r)
{
    return os << "[" << r.x << " " << r.y << " " << r.width << " " << r.height << "]";
}   // end operator<<


template <typename T>
istream& operator>>( istream &is, cv::Rect_<T> &r)
{
    char ch;
    return is >> ch >> r.x >> r.y >> r.width >> r.height >> ch;
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



template <typename T>
T findSumBetweenPoints( const cv::Mat& dmap, const cv::Point2f& f0, const cv::Point2f& f1, T minv, T maxv, int& count)
{
    assert( dmap.channels() == 1);

    const int ncols = dmap.cols;
    const int nrows = dmap.rows;
    const cv::Point p0( f0.x * ncols, f0.y * nrows);
    const cv::Point p1( f1.x * ncols, f1.y * nrows);

    const int xdiff = p1.x - p0.x;
    const int ydiff = p1.y - p0.y;
    const int nsteps = std::max<int>( abs(xdiff), abs(ydiff));

    const long double xdelta = (long double)(xdiff)/nsteps;
    const long double ydelta = (long double)(ydiff)/nsteps;

    count = 0;
    int x, y;
    T d;
    T dsum = 0;
    for ( int i = 0; i < nsteps; ++i)
    {
        x = p0.x + cvRound(i*xdelta);
        y = p0.y + cvRound(i*ydelta);
        d = dmap.ptr<T>(y)[x];

        if ( d > minv && d < maxv)
        {
            dsum += d;
            count++;
        }   // end if
    }   // end for

    return dsum;
}   // end findSumBetweenPoints
