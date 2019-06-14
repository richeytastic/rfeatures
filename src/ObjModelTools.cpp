/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::ObjEdge;
using RFeatures::Triangle3f;
#include <cassert>
#include <cstring>


void RFeatures::join( ObjModel::Ptr mod, const ObjModel* m1, bool txs)
{
    std::unordered_map<int,int> vvmap;  // Map vertex IDs on m1 to new ones on mod
    const IntSet& vidxs = m1->vtxIds();
    for ( int vidx : vidxs)
        vvmap[vidx] = mod->addVertex( m1->vtx(vidx));

    const IntSet& fids = m1->faces();
    for ( int fid : fids)
    {
        const int* fvidxs = m1->fvidxs(fid);
        const int nv0 = vvmap[fvidxs[0]];
        const int nv1 = vvmap[fvidxs[1]];
        const int nv2 = vvmap[fvidxs[2]];
        const int nfid = mod->addFace( nv0, nv1, nv2);

        if ( txs)
        {
            const int mid = m1->faceMaterialId(fid);
            if ( mid >= 0)
            {
                const int* uvids = m1->faceUVs(fid);
                const cv::Vec2f& uv0 = m1->uv(mid, uvids[0]);
                const cv::Vec2f& uv1 = m1->uv(mid, uvids[1]);
                const cv::Vec2f& uv2 = m1->uv(mid, uvids[2]);
                mod->setOrderedFaceUVs( mid, nfid, uv0, uv1, uv2);
            }   // end if
        }   // end if
    }   // end for
}   // end join


ObjModel::Ptr RFeatures::extractRadialArea( const ObjModel* m, int vidx, float r)
{
    const cv::Vec3f& v = m->vtx(vidx);

    ObjModel::Ptr cm = ObjModel::create();
    const IntSet& fids = m->faces();
    for ( int fid : fids)
    {
        const ObjPoly& poly = m->face(fid);
        const cv::Vec3f& v0 = m->vtx(poly[0]);
        const cv::Vec3f& v1 = m->vtx(poly[1]);
        const cv::Vec3f& v2 = m->vtx(poly[2]);

        if (( cv::norm(v-v0) + cv::norm(v-v1) + cv::norm(v-v2)) <= 3*r)
        {
            int j0 = cm->addVertex( v0);
            int j1 = cm->addVertex( v1);
            int j2 = cm->addVertex( v2);
            cm->addFace(j0,j1,j2);
        }   // end if
    }   // end for

    return cm;
}   // end extractRadialArea


ObjModel::Ptr RFeatures::extractSubset( const ObjModel* m, const IntSet& svidxs, int nedges)
{
    nedges = std::max(1, nedges);

    IntSet fidxs;

    // Copy in seed vertices
    IntSet allvidxs = svidxs;
    IntSet avidxs = svidxs;
    IntSet bvidxs;

    IntSet *rvtxs = &avidxs;
    IntSet *nvtxs = &bvidxs;

    for ( int j = 0; j < nedges; ++j)
    {
        for ( int vidx : *rvtxs)
        {
            for ( int fid : m->faces(vidx))
            {
                if ( fidxs.count(fid) > 0)
                    continue;

                fidxs.insert(fid);
                const ObjPoly& poly = m->face(fid);
                for ( int i = 0; i < 3; ++i)
                {
                    if ( allvidxs.count(poly[i]) == 0)
                    {
                        allvidxs.insert(poly[i]);
                        nvtxs->insert(poly[i]);
                    }   // end if
                }   // end for
            }   // end for
        }   // end for

        std::swap( rvtxs, nvtxs);
        nvtxs->clear();
    }   // end for

    ObjModel::Ptr cm = ObjModel::create();
    for ( int f : fidxs)
    {
        const ObjPoly& poly = m->face(f);
        const cv::Vec3f& v0 = m->vtx(poly[0]);
        const cv::Vec3f& v1 = m->vtx(poly[1]);
        const cv::Vec3f& v2 = m->vtx(poly[2]);
        const int j0 = cm->addVertex( v0);
        const int j1 = cm->addVertex( v1);
        const int j2 = cm->addVertex( v2);
        cm->addFace( j0, j1, j2);
    }   // end for

    return cm;
}   // extractSubset


size_t RFeatures::removeDisconnectedVertices( ObjModel::Ptr model)
{
    IntSet rvidxs;
    const IntSet& vidxs = model->vtxIds();
    for ( int vidx : vidxs)
    {
        if ( model->cvtxs(vidx).empty())
            rvidxs.insert(vidx);
    }   // end for

    for ( int vidx : rvidxs)
        model->removeVertex(vidx);
    return rvidxs.size();
}   // end removeDisconnectedVertices


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


cv::Vec3d RFeatures::calcMeanColumnVector( const cv::Mat_<double>& A, cv::Mat_<double> W)
{
    const int n = A.cols;
    if ( W.empty() || W.cols != n)
        W = cv::Mat::ones( 1, n, CV_64FC1);

    cv::Vec3d vsum(0,0,0);
    for ( int i = 0; i < n; ++i)
    {
        // Add weighted vector to sum
        const cv::Mat v = A.col(i);
        const double w = W.at<double>(i);
        vsum[0] += w * v.at<double>(0);
        vsum[1] += w * v.at<double>(1);
        vsum[2] += w * v.at<double>(2);
    }   // end for

    return (1.0/n) * vsum;
}   // end calcMeanColumnVector


double RFeatures::toMean( cv::Mat_<double>& A, const cv::Vec3d& vbar, cv::Mat_<double> W)
{
    const int n = A.cols;
    if ( W.empty() || W.cols != n)
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
        const double w = W.at<double>(i);
        s += pow(w*v.at<double>(0),2)
           + pow(w*v.at<double>(1),2)
           + pow(w*v.at<double>(2),2);
    }   // end for
    return sqrt( s/n);
}   // end toMean


cv::Mat_<double> RFeatures::verticesToCvMat( const ObjModel* mod)
{
    assert(mod->hasSequentialVertexIds());

    const int n = mod->numVtxs();
    cv::Mat_<double> m( 3, n);
    for ( int i = 0; i < n; ++i)
    {
        const cv::Vec3f& v = mod->vtx(i);
        cv::Mat vcol = m.col(i);
        vcol.at<double>(0) = v[0];
        vcol.at<double>(1) = v[1];
        vcol.at<double>(2) = v[2];
    }   // end for

    return m;
}   // end verticesToCvMat


void RFeatures::obtainBasis( const Triangle3f& T, cv::Vec3d& uNr, double& mNr, cv::Vec3d& uAC, double& mAC, cv::Vec3d& uRt, double& mRt)
{
    const cv::Vec3f& pA = T[0];
    const cv::Vec3f& pB = T[1];
    const cv::Vec3f& pC = T[2];
    const double dAC = cv::norm( pA - pC);
    const double areaABC = calcTriangleArea( cv::norm( pA - pB), cv::norm( pB - pC), dAC);
    assert( areaABC > 0.0);

    const cv::Vec3d vAB = pB - pA;

    uAC = (1.0/dAC) * (pC - pA);         // Get unit vector uAC
    cv::normalize( uAC.cross(vAB), uNr); // Calculate the orthogonal vector to T
    cv::normalize( uNr.cross(uAC), uRt); // Find uRt as orthogonal to uNr and uAC

    mNr = sqrt(areaABC);      // Unitless length of the orthogonal vector equal to the sqrt of Area(ABC)
    mAC = dAC / mNr;          // Unitless length of vector AC is the number of mNr that do into its length.
    mRt = vAB.dot(uRt) / mNr; // Unitless length of vector uRt is number of times projected length of vAB onto uRt goes into mNr.
}   // end obtainBasis


// For this mapping to make sense, the vertices in the corresponding indices of pSpace and qSpace must
// share some kind of semantic equivalence.
float RFeatures::mapPosition( const Triangle3f& pSpace, const cv::Vec3f& p, const Triangle3f& qSpace, cv::Vec3f& q)
{
    double mNr, mAC, mRt;
    cv::Vec3d uNr, uAC, uRt;
    obtainBasis( pSpace, uNr, mNr, uAC, mAC, uRt, mRt);

    // Get point p relative to pA and set its position in terms relative to the basis vectors just defined,
    // and with magnitudes as multiples of the new ratio "units" defined above.
    const cv::Vec3d pDelta = p - pSpace[0];
    const cv::Vec3d pRatio( pDelta.dot(uNr) / mNr, pDelta.dot(uAC) / mAC, pDelta.dot(uRt) / mRt);

    // Now do the inverse process using basis vectors and magnitudes calculated from qSpace
    obtainBasis( qSpace, uNr, mNr, uAC, mAC, uRt, mRt);

    const cv::Vec3f qDelta( float(pRatio[0] * mNr), float(pRatio[1] * mAC), float(pRatio[2] * mRt));
    q = qSpace[0] + qDelta;
    return qDelta[0];   // Returns the actual orthogonal distance from the plane of qSpace to q.
}   // end mapPosition

