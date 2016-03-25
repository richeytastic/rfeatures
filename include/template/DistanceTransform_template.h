template <typename T>
void __calcRowDistances( const T* inRow, float* outRow, size_t N, T thresh, bool signedVal, int pow)
{
    const float sval = signedVal ? -1 : 1;    // Whether or not to sign negative distances

    int k0 = -1;    // Index into inRow from the left of a threshold point
    int k1 = N;     // Index into inRow from the right of a threshold point
    const float MAXD = 1E20;
    outRow[N/2] = MAXD;
    // j0 is index from left going right, j1 is index from right going left
    for ( int j0 = 0, j1 = N-1; j0 < N, j1 >= 0; ++j0, --j1)
    {
        if ( j0 < j1)
        {
            outRow[j0] = MAXD;
            outRow[j1] = MAXD;
        }   // end if

        if ( inRow[j0] > thresh)
            k0 = j0;

        if ( inRow[j1] > thresh)
            k1 = j1;

        // Set the positive distance from the target (k0) pixel
        if ( k0 >= 0)
        {
            // Set horizontal distance going right from nearest non-zero cell to the left
            const float d0 = powf(j0 - k0,pow); // k0 <= j0
            if ( fabsf(outRow[j0]) >= d0) // If equal, overwrite with value
                outRow[j0] = d0;  // Always positive
        }   // end if

        // Set the negative distance from the target (k1) pixel
        if ( k1 < N)
        {
            // Set horizontal distance going left from nearest non-zero cell to the right
            const float d1 = powf(k1 - j1,pow); // k1 >= j1
            if ( fabsf(outRow[j1]) > d1)
                outRow[j1] = sval*d1;
        }   // end if
    }   // end for
}   // end __calcRowDistances


template <typename T>
void calcSignedRowDistances( const T* inRow, float* outRow, size_t N, T thresh, int pow)
{
    __calcRowDistances( inRow, outRow, N, thresh, true, pow);
}   // end calcSignedRowDistances


template <typename T>
void calcRowDistances( const T* inRow, float* outRow, size_t N, T thresh, int pow)
{
    __calcRowDistances( inRow, outRow, N, thresh, false, pow);
}   // end calcRowDistances


template <typename T>
cv::Mat_<float> calcSignedDistanceMap( const cv::Mat_<T>& m, T thresh, int pow)
{
    cv::Mat_<float> dmap( m.size());
    const int rows = dmap.rows;
    const int cols = dmap.cols;
    for ( int i = 0; i < rows; ++i)
        __calcRowDistances( (const T*)m.ptr(i), dmap.ptr<float>(i), cols, thresh, true, pow);
    return dmap;
}   // end calcSignedDistanceMap


template <typename T>
cv::Mat_<float> calcDistanceMap( const cv::Mat_<T>& m, T thresh, int pow)
{
    cv::Mat_<float> dmap( m.size());
    const int rows = dmap.rows;
    const int cols = dmap.cols;
    for ( int i = 0; i < rows; ++i)
        __calcRowDistances( (const T*)m.ptr(i), dmap.ptr<float>(i), cols, thresh, false, pow);
    return dmap;
}   // end calcDistanceMap
