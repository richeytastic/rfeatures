#include <ObjModelSurfacePointFinder.h>
#include <FeatureUtils.h>       // l2sq
using RFeatures::ObjModelSurfacePointFinder;
using RFeatures::ObjModel;
#include <boost/foreach.hpp>

ObjModelSurfacePointFinder::ObjModelSurfacePointFinder( const ObjModel::Ptr m)
    : _model(m)
{}


double findClosestSurface( const RFeatures::ObjModel::Ptr model, const cv::Vec3f& v, int vidx, IntSet& visitedFaces, int& bfid, cv::Vec3f& fv)
{
    using namespace RFeatures;
    double minsd = l2sq(fv - v);

    // The next vertices to check
    int nv0 = vidx;
    int nv1 = vidx;

    const IntSet& fids = model->getFaceIds(vidx);
    BOOST_FOREACH ( int fid, fids)
    {
        if ( visitedFaces.count(fid) > 0)
            continue;

        visitedFaces.insert(fid);   // Don't check this face again
        const cv::Vec3f u = model->projectToPoly( fid, v);    // Project v into polygon fid
        const double sd = l2sq(u-v); // Repositioned difference
        if ( sd < minsd)
        {
            minsd = sd;
            fv = u;
            model->getFace(fid).getOpposite( vidx, nv0, nv1); // Get the opposite two vertices to check next.
            bfid = fid;
        }   // end if
    }   // end foreach

    if ( nv0 != vidx)
        minsd = findClosestSurface( model, v, nv0, visitedFaces, bfid, fv);

    if ( nv1 != vidx)
        minsd = findClosestSurface( model, v, nv1, visitedFaces, bfid, fv);

    return minsd;
}   // end findClosestSurface


// public
double ObjModelSurfacePointFinder::find( const cv::Vec3f& v, int& vidx, int& bfid, cv::Vec3f& fv) const
{
    double sd;
    // Check if vertex at vidx at same location as v
    if ( _model->vtx(vidx) == v)
    {
        bfid = -1;  // Denote that v is not in the plane of any of the polygons attached to vidx.
        fv = _model->vtx(vidx);
        sd = RFeatures::l2sq(fv - v);
    }   // end if
    else
    {
        IntSet visitedFaces;
        fv = cv::Vec3f( 10e4, 10e4, 10e4) + v;
        sd = findClosestSurface( _model, v, vidx, visitedFaces, bfid, fv);
        vidx = -1;
    }   // end else
    return sd;
}   // end find


