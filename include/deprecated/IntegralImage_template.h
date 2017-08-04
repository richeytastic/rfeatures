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

template <typename T>
const int IntegralImage<T>::ALL_CHANNELS = -1;

static const double EPS = 1e-10;


// static
template <typename T>
boost::shared_ptr<IntegralImage<T> > IntegralImage<T>::create( const cv::Size &sz, int channels)
{
    return boost::shared_ptr<IntegralImage<T> >( new IntegralImage<T>( sz, channels));
}   // end create


    
template <typename T>
IntegralImage<T>::IntegralImage( const cv::Mat &img)
    : rows( img.rows), cols( img.cols),
    intImg_( img.rows+1, img.cols+1, CV_64FC(img.channels())),
    rowIdx_( img.rows), colIdx_( img.cols),
    addVec_(NULL)
{
    addVec_ = (int*)calloc( img.rows, sizeof(int));

    // Set the first row and column of the integral image to zero.
    intImg_.row(0) = cv::Scalar::all(0);
    intImg_.col(0) = cv::Scalar::all(0);

    int rows = img.rows;
    int cols = img.cols;
    int channels = img.channels();

    for ( int i = 0; i < rows; ++i)
    {
        const T *srcRow = img.ptr<T>( i);   // Source image row
        double *prvRow = intImg_.ptr<double>( i);      // Integral image previous row
        double *dstRow = intImg_.ptr<double>( i+1);    // Integral image current row
        for ( int j = 0; j < cols; ++j)
        {
            for ( int k = 0; k < channels; ++k)
            {
                int cidx = j*channels+k;    // Column/channel index
                // Value at this pixel= iimg pixel above + iimg pixel on left + image pixel - iimg corner pixel
                // |-|+|
                // |+|+|
                double val = prvRow[cidx + channels] + dstRow[cidx] + srcRow[cidx] - prvRow[cidx];
                if ( val > -EPS && val < EPS) val = 0;  // Rounding error
                dstRow[cidx+channels] = val;
            }   // end for - channels

            addVec_[i] = j+1;   // Denote added value
        }   // end for - rows
    }   // end for - cols
}   // end ctor



template <typename T>
IntegralImage<T>::IntegralImage( const cv::Size &sz, int channels)
    : rows( sz.height), cols( sz.width),
    intImg_( sz.height+1, sz.width+1, CV_64FC(channels)), rowIdx_(0), colIdx_(0),
    addVec_( NULL)
{
    addVec_ = (int*)calloc( rows, sizeof(int));
    // Set the first row and column of the integral image to zero.
    intImg_.row(0) = cv::Scalar::all(0);
    intImg_.col(0) = cv::Scalar::all(0);
}   // end ctor



template <typename T>
IntegralImage<T>::~IntegralImage()
{
    free( addVec_);
}   // end dtor



template <typename T>
bool IntegralImage<T>::addValue( const T *val)
{
    if ( rowIdx_ >= intImg_.rows-1)   // Keep adding values until all rows complete
        return false;

    addToImage( colIdx_, rowIdx_, val);
    updateIndices();

    return true;
}   // end addValue



// private
template <typename T>
void IntegralImage<T>::addToImage( int col, int row, const T *val)
{
    double *prvRow = intImg_.ptr<double>(row);    // Integral image previous row
    double *dstRow = intImg_.ptr<double>(row+1);  // Integral image current row

    const int channels = intImg_.channels();
    const int cidx = col*channels;
    double v;
    for ( int k = 0; k < channels; ++k)
    {
        v = prvRow[cidx+channels+k] + dstRow[cidx+k] + val[k] - prvRow[cidx+k];
        if ( v > -EPS && v < EPS) v = 0;  // Rounding error
        dstRow[cidx+channels+k] = v;
    }   // end for

    addVec_[row]++; // Value added at position
    //assert( addVec_[row] < intImg_.cols);   // Remember that intImg_ is 1 pixel wider
}   // end addToImage




// private
template <typename T>
void IntegralImage<T>::updateIndices()
{
    colIdx_++;   // Increment indices for next call
    if ( colIdx_ == intImg_.cols-1)
    {
        rowIdx_++;
        if ( rowIdx_ != intImg_.rows-1)
            colIdx_ = addVec_[rowIdx_];
    }   // end if
}   // end updateIndices



template <typename T>
bool IntegralImage<T>::setValue( int col, int row, const T *val)
{
    if ( addVec_[row] > col)    // Cannot set a value already set!
        return false;

    if ( ( row == 0 && col == 0)    // at origin
      || ( row == 0 && addVec_[row] == col)  // or on top row and pixel to left is set
      || ( col == 0 && addVec_[row-1] > 0)  // or on left column and pixel above is set
      || ((addVec_[row-1] > col) && ( addVec_[row] == col)))   // or anywhere else and pixel to both the left and above are set
    {
        addToImage( col, row, val);

        // Even though user is advised against it, need to ensure that if addValue is
        // subsequently called, that the rowIdx_ and colIdx_ values remain valid:
        if ( col == colIdx_)
            updateIndices();

        return true;
    }   // end if

    return false;
}   // end setValue



// Calculates over all channels if channel==ALL_CHANNELS
template <typename T>
double IntegralImage<T>::operator()( const cv::Rect &rect, int channel)
                            const throw (ImageOutOfBoundsException)
{
    if ( rect.x < 0 || rect.y < 0 || rect.width < 0 || rect.height < 0
        || (rect.x + rect.width) >= intImg_.cols || (rect.y + rect.height) >= intImg_.rows)
        throw ImageOutOfBoundsException( "cv::Rect is out of image bounds!");

    int nchannels = intImg_.channels();
    if ( channel != ALL_CHANNELS && (channel >= nchannels || channel < 0))
        throw ImageOutOfBoundsException( "Requested image channel does not exist!");

    // Calculate over a single channel or all channels?
    int ch = channel;
    int endChannel = ch+1;
    if ( channel == ALL_CHANNELS)
    {
        ch = 0;
        endChannel = nchannels;
    }   // end if

    double sum = 0;
    for ( int k = ch; k < endChannel; ++k)
    {
        const int lCol = rect.x*nchannels + channel;
        const int rCol = (rect.x + rect.width)*nchannels + channel;
        const double *tRow = intImg_.ptr<double>( rect.y);
        const double *bRow = intImg_.ptr<double>( rect.y + rect.height);
        // |+|-|
        // |-|+|
        double val = bRow[rCol] - bRow[lCol] + tRow[lCol] - tRow[rCol];
        if ( val > -EPS && val < EPS) val = 0;  // Rounding error
        sum += val;
    }   // end for

    return sum;
}   // end operator()



template <typename T>
double IntegralImage<T>::operator()( const cv::Rect &rect) const throw (ImageOutOfBoundsException)
{
    return (*this)( rect, ALL_CHANNELS);
}   // end operator()



template <typename T>
void IntegralImage<T>::operator()( const cv::Rect &rect, double *res) const throw (ImageOutOfBoundsException)
{
    int nchannels = intImg_.channels();
    for ( int k = 0; k < nchannels; ++k)
        res[k] = (*this)( rect, k);
}   // end operator()
