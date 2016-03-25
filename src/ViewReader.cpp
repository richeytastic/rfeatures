#include "ViewReader.h"
using RFeatures::ViewReader;
//#include "DepthFinder.h"
//using RFeatures::DepthFinder;
#include <cassert>
#include <climits>


ViewReader::ViewReader()
    : m_view( new View())
{
}   // end ctor



void ViewReader::getSize( istream &is, size_t &width, size_t &height)
{
    width = m_view->img2d.cols;
    height = m_view->img2d.rows;
}   // end getSize



void ViewReader::getPoint( istream &is, size_t row, size_t col,
        double &x, double &y, double &z, double &rng, byte &r, byte &g, byte &b)
{
    m_view->points->from( row, col, x, y, z, r, g, b);  // Set x,y,z,r,g,b
    // Calculate the range from the camera plane to this point
    //const cv::Vec3d pt = cv::Vec3d( x, y, z) - m_view->posVec;
    //rng = m_view->focalVec.dot( pt);   // Set rng
    rng = z;    // Just z since points are rectified
    assert( rng >= 0);
    if ( rng > FLT_MAX) rng = FLT_MAX;

    // Set the image in the view (reverse colours for OpenCV)
    cv::Vec3b &pxl = m_view->img2d(row,col);
    pxl[2] = r;
    pxl[1] = g;
    pxl[0] = b;
    m_view->rngImg(row,col) = rng;
    int rngCnt = 0;
    if ( rng > 0) rngCnt = 1;
    m_rngCntImage->addValue( &rngCnt);

    // Pixel is black if point is invalid
    if ( x == 0 && y == 0 && z == 0)
    {
        r = 0; g = 0; b = 0;
    }   // end if
}   // end getPoint


/*
void ViewReader::finishedRead()
{
    if ( m_imgGradsBuilder == NULL || m_rngGradsBuilder == NULL)
        throw ViewReader::Exception( "No image or range gradient params set!");
    m_view->imgGrads = m_imgGradsBuilder->getIntegralGradients();
    m_view->rngGrads = m_rngGradsBuilder->getIntegralGradients();
    m_view->depthFinder = DepthFinder::create( m_rngBuilder->getIntegralRangeImage(), m_rngCntImage);
}   // end finishedRead
*/



void ViewReader::setPosition( const cv::Vec3d &v)
{
    m_view->posVec = v;
}   // end setPosition



void ViewReader::setFocalVector( const cv::Vec3d &focus)
{
    m_view->focalVec = focus / cv::norm(focus);
}   // end setFocalVector



void ViewReader::setUpVector( const cv::Vec3d &uv)
{
    m_view->upVec = uv / cv::norm(uv);
}   // end setUpVector


/*
void ViewReader::setImageGradientsParams( int nbins, bool dd, bool ss, bool gc)
{
    m_imgGradsBuilder = ImageGradientsBuilder::create( nbins, dd, ss, gc);
    PointDataReader::addDataBuilder( m_imgGradsBuilder);
}   // end setImageGradientsParams



void ViewReader::setRangeGradientsParams( int nbins, bool dd, bool ss)
{
    m_rngGradsBuilder = RangeGradientsBuilder::create( nbins, dd, ss);
    PointDataReader::addDataBuilder( m_rngGradsBuilder);
    m_rngBuilder = RangeBuilder::create();
    PointDataReader::addDataBuilder( m_rngBuilder);
}   // end setRangeGradientsParams



void ViewReader::setPointCloud( const PointCloud::Ptr &pc)
{
    m_view->points = pc;
    m_view->img2d.create( pc->rows(), pc->cols());
    m_view->rngImg.create( pc->rows(), pc->cols());
    m_rngCntImage = IntegralImage<int>::create( m_view->img2d.size());
}   // end setPointCloud
*/
