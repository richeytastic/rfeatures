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

#include <ObjModelTools.h>
using RFeatures::VertexWeights;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;
using RFeatures::Edge;
#include <cassert>


cv::Vec3f RFeatures::maximallyExtrudedPoint( const ObjModel* model, int e0, int e1)
{
    DijkstraShortestPathFinder dspf( model);
    dspf.setEndPointVertexIndices( e0, e1);
    std::vector<int> vidxs;
    dspf.findShortestPath( vidxs);
    const int midx = maximallyExtrudedPointIndex( model, vidxs);
    assert( midx >= 0 && midx < (int)vidxs.size());
    return model->vtx( vidxs[midx]);
}   // end maximallyExtrudedPoint


cv::Vec3f RFeatures::maximallyExtrudedPoint( const ObjModelKDTree* kdt, const cv::Vec3f& v0, const cv::Vec3f& v1)
{
    return maximallyExtrudedPoint( kdt->model(), kdt->find(v0), kdt->find(v1));
}   // end maximallyExtrudedPoint


int RFeatures::maximallyExtrudedPointIndex( const ObjModel* model, const std::vector<int>& vidxs)
{
    const int npts = int(vidxs.size());
    assert( npts > 0);
    if ( npts == 0)
        return -1;

    const cv::Vec3f v0 = model->vtx( vidxs[0]);
    const cv::Vec3f v1 = model->vtx( vidxs[npts-1]);
    int maxidx = -1;
    double maxvdist = 0;
    for ( int i = 0; i < npts; ++i)
    {
        const cv::Vec3f v = model->vtx(vidxs[i]);
        const double vdist = cv::norm(v - v0) + cv::norm(v - v1);
        if ( vdist > maxvdist)
        {
            maxvdist = vdist;
            maxidx = i;
        }   // end if
    }   // end for

    return maxidx;
}   // end maximallyExtrudedPointIndex


int RFeatures::oppositePoly( const ObjModel* model, int fid, int vi, int vj)
{
    const IntSet& sfids = model->getSharedFaces( vi, vj);
    if ( sfids.size() <= 1)
        return -1;

    assert( sfids.size() == 2);
    const int bfid = *sfids.begin();
    return bfid != fid ? bfid : *(++sfids.begin());
}   // end oppositePoly


cv::Mat_<double> RFeatures::verticesToCvMat( const ObjModel* model)
{
    const int n = int(model->numVertices());
    cv::Mat_<double> A( 3, n);

    const IntSet& vidxs = model->vertexIds();
    int i = 0;  // Vertex ID from model (incrementing)
    int c = 0;  // Number of columns (and column index into A)
    while ( c < n)
    {
        if ( vidxs.count(i) > 0)
        {
            const cv::Vec3f& v = model->vtx(i);
            cv::Mat vcol = A.col(c);
            vcol.at<double>(0) = v[0];
            vcol.at<double>(1) = v[1];
            vcol.at<double>(2) = v[2];
            c++;
        }   // end if
        i++;
    }   // end while

    return A;
}   // end verticesToCvMat


cv::Mat_<double> RFeatures::weightsToCvMat( const VertexWeights& vw)
{
    const int n = static_cast<int>(vw.size());
    cv::Mat_<double> W( 1, n);
    double* row = W.ptr<double>();

    int i = 0;  // Key (incrementing)
    int c = 0;  // Number of columns (and column index into W)
    while ( c < n)
    {
        if ( vw.count(i) > 0)
            row[c++] = vw.at(i);
        i++;
    }   // end while

    return W;
}   // end weightsToCvMat


cv::Vec3d RFeatures::calcMeanColumnVector( const cv::Mat_<double>& A, cv::Mat_<double> W)
{
    const int n = A.size().width;
    if ( W.empty() || W.size().width != n)
        W = cv::Mat::ones( 1, n, CV_64FC1);

    cv::Vec3d vsum(0,0,0);
    for ( int i = 0; i < n; ++i)
    {
        cv::Mat v = A.col(i);
        double w = W.at<double>(i);

        // Add weighted vector to sum
        vsum[0] += w * v.at<double>(0);
        vsum[1] += w * v.at<double>(1);
        vsum[2] += w * v.at<double>(2);
    }   // end for
    return (1.0/n) * vsum;
}   // end calcMeanColumnVector


double RFeatures::toMean( cv::Mat_<double>& A, const cv::Vec3d& vbar, cv::Mat_<double> W)
{
    const int n = A.size().width;
    if ( W.empty() || W.size().width != n)
        W = cv::Mat::ones( 1, n, CV_64FC1);

    double s = 0;
    for ( int i = 0; i < n; ++i)
    {
        cv::Mat v = A.col(i);

        // Subtract the centroid
        v.at<double>(0) -= vbar[0];
        v.at<double>(1) -= vbar[1];
        v.at<double>(2) -= vbar[2];

        // Weighted scaling
        double w = W.at<double>(i);
        s += pow(w*v.at<double>(0),2) + pow(w*v.at<double>(1),2) + pow(w*v.at<double>(2),2);
    }   // end for
    return sqrt( s/n);
}   // end toMean


void RFeatures::scale( cv::Mat_<double>& A, double s)
{
    const int n = A.size().width;
    for ( int i = 0; i < n; ++i)
    {
        cv::Mat v = A.col(i);
        v.at<double>(0) *= s;
        v.at<double>(1) *= s;
        v.at<double>(2) *= s;
    }   // end for
}   // end scale
