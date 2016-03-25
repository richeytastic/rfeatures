#include <ContentFinder.h>
using namespace RFeatures;


ContentFinder::ContentFinder() : m_threshold(-1.0f)
{
    m_ranges[0] = m_hranges;    // All channels have same range
    m_ranges[1] = m_hranges;
    m_ranges[2] = m_hranges;
}   // end ctor



void ContentFinder::setThreshold( float t)
{
    m_threshold = t;
}   // end setThreshold



void ContentFinder::setHistogram( const cv::MatND &h)
{
    m_histogram = h;
    cv::normalize( m_histogram, m_histogram, 1.0);
}   // end setHistogram



cv::Mat ContentFinder::find( const cv::Mat &img, float minVal, float maxVal, int *channels, int dim)
{
    cv::Mat result;

    m_hranges[0] = minVal;
    m_hranges[1] = maxVal;

    for ( int i = 0; i < dim; ++i)
        m_channels[i] = channels[i];

    cv::calcBackProject( &img, 1, // input image
            channels,   // list of channels used
            m_histogram,    // the histogram we are using
            result,       // the resulting back projection
            m_ranges,   // the range of values
            255.0); // scaling factor

    // Threshold back projection to obtain binary image
    if ( m_threshold > 0.0)
        cv::threshold(result, result, 255 * m_threshold, 255, cv::THRESH_BINARY);

    return result;
}   // end find
