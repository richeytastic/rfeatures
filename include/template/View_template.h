
template <typename T>
void __createChangeMaps( const cv::Mat& rngImg, cv::Mat& hmap, cv::Mat& vmap, bool doAbs, cv::Mat_<byte> msk)
{
#ifndef NDEBUG
    if ( !msk.empty())
    {
        assert( rngImg.rows == msk.rows);
        assert( rngImg.cols == msk.cols);
    }   // end if
#endif

    // Create absolute difference maps in depth going horizontally and vertically
    hmap = cv::Mat::zeros( rngImg.size(), rngImg.type());   // From left to right
    vmap = cv::Mat::zeros( rngImg.size(), rngImg.type());   // From top to bottom

    const int nrows = rngImg.rows;
    const int ncols = rngImg.cols;

    const T* vprev = rngImg.ptr<T>(0);
    const byte* prevMskRow = NULL;
    if ( !msk.empty())
        prevMskRow = msk.ptr<byte>(0);

    for ( int i = 1; i < nrows; ++i)
    {
        const T* rngRow = rngImg.ptr<T>(i);
        const byte* mskRow = NULL;
        if ( !msk.empty())
            mskRow = msk.ptr<byte>(i);  // Mask row

        T* hrow = hmap.ptr<T>(i);   // Horizontal out
        T* vrow = vmap.ptr<T>(i);   // Vertical out

        for ( int j = 1; j < ncols; ++j)
        {
            // Ignore pixel if not masked at current pixel, above and to the left
            if ( mskRow != NULL && (!mskRow[j] || !prevMskRow[j] || !mskRow[j-1]))
                continue;

            // Set the horizontal change
            hrow[j] = rngRow[j] - rngRow[j-1];
            if ( doAbs)
                hrow[j] = fabsf(hrow[j]);

            // Set the vertical change
            vrow[j] = rngRow[j] - vprev[j];
            if ( doAbs)
                vrow[j] = fabsf(vrow[j]);
        }   // end for

        vprev = rngRow;
        prevMskRow = mskRow;
    }   // end for
}   // end __createChangeMaps
