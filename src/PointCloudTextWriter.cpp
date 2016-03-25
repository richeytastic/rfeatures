#include "PointCloudTextWriter.h"
using RFeatures::PointCloudTextWriter;
#include <iostream>
using std::cerr;
using std::ostream;
using std::endl;
#include <iomanip>



PointCloudTextWriter::PointCloudTextWriter( const PointCloud::Ptr &pc, const cv::Rect* r)
    : PointCloudWriter(pc)
{
    if ( pc->isOrganised())
    {
        rect_ = cv::Rect(0,0,pc->cols(), pc->rows());
        if ( r != NULL) // Ensure rectangle falls within organised point cloud dimensions
            rect_ &= *r;
    }   // end if
    else if ( r != 0)
        cerr << "WARNING: Provided rectangular area will be ignored for unorganised point cloud!" << endl;
}   // end ctor



void PointCloudTextWriter::write( ostream &os) const
{
    if ( pcloud_->isOrganised())
        writeOrganised(os);
    else
        writeUnorganised(os);
}  // end write



void PointCloudTextWriter::writeOrganised( ostream& os) const
{
    os << rect_.height << " " << rect_.width << endl;   // Rows and columns

    size_t rowIdx, colIdx;
    double x, y, z;
    byte r, g, b;
    for ( size_t row = rect_.y; row < rect_.y + rect_.height; ++row)
    {
        rowIdx = row - rect_.y;
        for ( size_t col = rect_.x; col < rect_.x + rect_.width; ++col)
        {
            pcloud_->from( row, col, x, y, z, r, g, b);
            colIdx = col - rect_.x;

            os << rowIdx << " " << colIdx << " "
               << std::fixed << std::setprecision(10) << x << " " << y << " " << z << " ";
            os.unsetf( std::ios_base::fixed);
            os << (int)r << " " << (int)g << " " << (int)b << endl;
        }   // end for
    }   // end for
}   // end writeOrganised



void PointCloudTextWriter::writeUnorganised( ostream& os) const
{
    const int npts = pcloud_->size();
    os << npts << endl;  // Number of points

    double x, y, z;
    byte r, g, b;
    for ( int i = 0; i < npts; ++i)
    {
        pcloud_->from( i, x, y, z, r, g, b);
        os << std::fixed << std::setprecision(10) << x << " " << y << " " << z << " ";
        os.unsetf( std::ios_base::fixed);
        os << (int)r << " " << (int)g << " " << (int)b << endl;
    }   // end for
}   // end writeUnorganised
