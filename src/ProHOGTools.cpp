#include "ProHOGTools.h"
using RFeatures::BatchProHOGExtractor;
#include <cassert>
#include <cmath>
#include <iostream>
using std::cerr;
using std::endl;



cv::Mat RFeatures::calcInternalDiffs( const cv::Mat &phog, bool appendOriginal)
{
    assert( phog.isContinuous());   // Required for reshape operation
    const int channels = phog.channels();   // Number of histogram bins

    const cv::Mat inVec = phog.reshape(0,1);    // Single row vector for input (keep number of channels the same)
    assert( inVec.isContinuous());
    
    const int cols = phog.rows * phog.cols;

    const int sz = (cols * (cols - 1)) / 2; // Length of output vector
    cv::Mat vec;
    if ( appendOriginal)
        vec.create( 1, sz + cols, phog.type());   // Include original gradient info in output row vector
    else
        vec.create( 1, sz, phog.type());   // Row vector for output

    int vi = 0; // Index for out vector
    const double *inRow = inVec.ptr<double>(0);
    double *outRow = vec.ptr<double>(0);

    for ( int i = 0; i < cols; ++i)
    {
        const double *e1 = &inRow[i*channels];    // cell 1

        for ( int j = i+1; j < cols; ++j)
        {
            const double *e2 = &inRow[j*channels];   // cell 2

            double *vo = &outRow[vi*channels];   // Out element
            for ( int k = 0; k < channels; ++k)
                vo[k] = fabs( e2[k] - e1[k]);
            vi++;
        }   // end for

        if ( appendOriginal)
        {
            for ( int k = 0; k < channels; ++k) // Append original elements
                outRow[(sz+i)*channels+k] = e1[k];
        }   // end if
    }   // end for

    return vec;
}   // end calcInternalDiffs



BatchProHOGExtractor::BatchProHOGExtractor( const vector<cv::Mat> &imgs, int nb, bool dd, const cv::Size &cd, bool usd, bool ao)
    : _imgs( imgs), _nbins(nb), _dirDep(dd), _cellDims(cd), _useSpatialDiffs(usd), _appendOrig(ao)
{
}   // end ctor



void extractProHOGs( const vector<cv::Mat> *imgs, int idx, int sz,  // Input and range
                     int nbs, const cv::Size cd, bool dd, bool usd, bool ao, // extraction params
                     vector<cv::Mat> *out)  // output
{
    const int endIdx = idx + sz;
    for ( int i = idx; i < endIdx; ++i)
    {
        cv::Mat phg = RFeatures::ProHOG( (*imgs)[i], nbs, dd).createProHOG( cd);
        assert( phg.type() == CV_64FC(4*nbs));
        if ( usd)
            phg = RFeatures::calcInternalDiffs( phg, ao);
        out->push_back( phg);
    }   // end for
}   // end extractProHOGs



void BatchProHOGExtractor::extract( vector<cv::Mat> &phogs)
{
    extractProHOGs( &_imgs, 0, _imgs.size(), _nbins, _cellDims, _dirDep, _useSpatialDiffs, _appendOrig, &phogs);
}   // end extract



void BatchProHOGExtractor::extract_mt( vector<cv::Mat> &phogs)
{
    boost::thread_group tgrp;
    const int lts = boost::thread::hardware_concurrency();

    // Calculate how much of each of the example vector should be calculated by each thread.
    const int segSz = _imgs.size() / lts;
    const int rem = _imgs.size() % lts;

    vector< vector<cv::Mat>* > vecs;
    // Each thread extracts ProHOG features over its portion of the provided instances
    for ( int i = 0; i < lts; ++i)
    {
        const int sz = i < lts - 1 ? segSz : segSz + rem;
        vector<cv::Mat> *phgt = new vector<cv::Mat>;
        vecs.push_back( phgt);
        tgrp.create_thread( boost::bind(
              &extractProHOGs, &_imgs, i*segSz, sz, _nbins, _cellDims,
                                _dirDep, _useSpatialDiffs, _appendOrig, phgt));
    }   // end foreach

    tgrp.join_all();  // All threads also deleted

    BOOST_FOREACH( const vector<cv::Mat> *v, vecs)
    {
        phogs.insert( phogs.end(), v->begin(), v->end());
        delete v;
    }   // end foreach
}   // end extract_mt



void BatchProHOGExtractor::extract( vector<cv::Mat_<float> > &phogs)
{
    vector<cv::Mat> bphogs;
    extract( bphogs);
    BOOST_FOREACH ( const cv::Mat& phog, bphogs)
    {
        const cv::Mat_<float> rowvec = RFeatures::toRowVector(phog);
        assert( rowvec.rows == 1);
        assert( rowvec.cols == phog.channels() * phog.rows * phog.cols);
        phogs.push_back(rowvec);
    }   // end foreach
}   // end extract



void BatchProHOGExtractor::extract_mt( vector<cv::Mat_<float> > &phogs)
{
    vector<cv::Mat> bphogs;
    extract_mt( bphogs);
    BOOST_FOREACH ( const cv::Mat& phog, bphogs)
    {
        const cv::Mat_<float> rowvec = RFeatures::toRowVector(phog);
        assert( rowvec.rows == 1);
        assert( rowvec.cols == phog.channels() * phog.rows * phog.cols);
        phogs.push_back(rowvec);
    }   // end foreach
}   // end extract_mt
