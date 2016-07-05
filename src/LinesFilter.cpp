#include "LinesFilter.h"
#include "FeatureExceptions.h"
using namespace RFeatures;
#include <cmath>
#include <queue>
#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>
using std::cerr;
using std::endl;



LinesFilter::LinesFilter( const Lines &lns, const cv::Mat &ri) : lines(lns)
{
    if ( ri.channels() != 1 || ri.type() != CV_8UC1)
        throw ImageTypeException( "Image supplied to LinesFilter ctor must be single channel grey scale linear range image!");
    rngImg = ri.clone();
}   // end ctor



LinesFilter::~LinesFilter()
{
}   // end dtor



class LineComparator
{
    //typedef boost::unordered_map<const cv::Vec4i*, double> unordered_map;

    //unordered_map stdDevs;  // Cached std devs so recomputations not necessary
    double maxStdDev;
    int stageGap;
    cv::Mat rngImg;

public:
    LineComparator( double msd, int sg, cv::Mat ri) : maxStdDev( msd), stageGap( sg), rngImg(ri) {}


    double getStdDev( const cv::Vec4i &v)
    {
        double xdiff = v[2] - v[0];     // Total change in x
        double ydiff = v[3] - v[1];     // Total change in y

        double len = sqrt(xdiff*xdiff + ydiff*ydiff);   // Length of the line in pixels
        if ( len < 2 * stageGap)   // Ignore lines too short to measure
            return maxStdDev;

        int stages = int(len / stageGap);   // Number of points along the line that we'll check the depth gradient
        double xinc = xdiff / stages;  // Amount to increment x at each stage along the line
        double yinc = ydiff / stages;  // Amount to increment y at each stage along the line

        double* diffs = (double*)cv::fastMalloc( stages * sizeof(double));  // Holds differences between subsequent points
        double meanScale = 1.0 / stages;
        double mean = 0.0;

        // Obtain differences over the length of the line and calculate the mean
        double x = v[0];    // Starting point in x axis of line
        double y = v[1];    // Starting point in y axis of line
        int sid = 0;    // Stage difference ID
        while ( sid < stages)
        {
            double gVal = rngImg.at<uchar>( (int)y, (int)x);
            x += xinc;
            y += yinc;
            diffs[sid] = (double)rngImg.at<uchar>( (int)y, (int)x) - gVal;
            mean += meanScale * diffs[sid++];
        }   // end while

        // Calculate variance and standard deviation
        sid = 0;
        double variance = 0.0;
        while ( sid < stages)
            variance += pow( diffs[sid++] - mean, 2);

        cv::fastFree(diffs);
        return sqrt(variance);
    }   // end getStdDev


    bool operator()( const cv::Vec4i &lhs, const cv::Vec4i &rhs)
    {
        return getStdDev(lhs) > getStdDev(rhs);
    }   // end operator()
};  // end class LineComparator



typedef std::priority_queue<cv::Vec4i, std::vector<cv::Vec4i>, LineComparator> PriQueue;


Lines LinesFilter::filter( double maxStdDev, int stageGap, int filterSz)
{
    // Smooth the range image
    cv::blur( rngImg, rngImg, cv::Size( filterSz ,filterSz));

    LineComparator lineComp( maxStdDev, stageGap, rngImg);

    PriQueue pqLines( lineComp);
    BOOST_FOREACH( cv::Vec4i v, lines)
        pqLines.push(v);

    Lines sortedLines;
    while ( !pqLines.empty() && lineComp.getStdDev( pqLines.top()) < maxStdDev)
    {
        sortedLines.push_back( pqLines.top());
        pqLines.pop();
    }   // end while

    return sortedLines;
}   // end filter
