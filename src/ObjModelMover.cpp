#include <ObjModelMover.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelMover;
using RFeatures::ObjModel;

ObjModelMover::ObjModelMover()
    : _tmat( 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1)
{}   // end ctor

ObjModelMover::ObjModelMover( const cv::Matx44d& t) : _tmat(t) {}

ObjModelMover::ObjModelMover( const cv::Vec3d& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,  1)
{}   // end ctor

ObjModelMover::ObjModelMover( const cv::Matx33d& R, const cv::Vec3d& t)
    : _tmat( R(0,0), R(0,1), R(0,2), t[0],
             R(1,0), R(1,1), R(1,2), t[1],
             R(2,0), R(2,1), R(2,2), t[2],
                0,      0,      0,    1)
{}   // end ctor


ObjModelMover::ObjModelMover( const cv::Vec3d& vnorm, const cv::Vec3d& vup, const cv::Vec3d& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,  1)
{
    // Ensure normalized
    cv::Vec3d uvec, zvec;
    cv::normalize( vup, uvec);
    cv::normalize( vnorm, zvec);

    cv::Vec3d xvec = uvec.cross(zvec);
    cv::Vec3d yvec = zvec.cross(xvec);   // Don't trust that vup and vnorm actually are orthogonal

    _tmat(0,0) = xvec[0];
    _tmat(0,1) = yvec[0];
    _tmat(0,2) = zvec[0];

    _tmat(1,0) = xvec[1];
    _tmat(1,1) = yvec[1];
    _tmat(1,2) = zvec[1];

    _tmat(2,0) = xvec[2];
    _tmat(2,1) = yvec[2];
    _tmat(2,2) = zvec[2];
}   // end ctor


ObjModelMover::ObjModelMover( double rads, const cv::Vec3d& axis, const cv::Vec3d& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,   1)
{
    cv::Vec3d u;    // Ensure normalised axis
    cv::normalize( axis, u);
    const double x = u[0];
    const double y = u[1];
    const double z = u[2];
    const double ct = cos(rads);
    const double mct = 1.0-ct;
    const double st = sin(rads);
    const double xst = x*st;
    const double yst = y*st;
    const double zst = z*st;

    // Set the 3x3 upper left submatrix with the rotation params
    _tmat(0,0) = x*x*mct + ct;
    _tmat(0,1) = x*y*mct - zst;
    _tmat(0,2) = x*z*mct + yst;

    _tmat(1,0) = y*x*mct + zst;
    _tmat(1,1) = y*y*mct + ct;
    _tmat(1,2) = y*z*mct - xst;

    _tmat(2,0) = z*x*mct - yst;
    _tmat(2,1) = z*y*mct + xst;
    _tmat(2,2) = z*z*mct + ct;
}   // end ctor


void ObjModelMover::prependTranslation( const cv::Vec3d& t)
{
    const cv::Matx44d T( 1, 0, 0, t[0],
                         0, 1, 0, t[1],
                         0, 0, 1, t[2],
                         0, 0, 0, 1);
    _tmat = T * _tmat;
}   // end prependTranslation


void ObjModelMover::operator()( cv::Vec3d& v) const
{
    const cv::Vec4d nv = _tmat * cv::Vec4d( v[0], v[1], v[2], 1);
    v[0] = nv[0];
    v[1] = nv[1];
    v[2] = nv[2];
}   // end operator()


void ObjModelMover::operator()( cv::Vec3f& v) const
{
    const cv::Vec4d nv = _tmat * cv::Vec4d( v[0], v[1], v[2], 1);
    v[0] = (float)nv[0];
    v[1] = (float)nv[1];
    v[2] = (float)nv[2];
}   // end operator()


void ObjModelMover::operator()( ObjModel::Ptr model) const
{
    const IntSet& vidxs = model->getVertexIds();
    BOOST_FOREACH ( int vidx, vidxs)
    {
        const cv::Vec3f& v = model->getVertex( vidx);
        const cv::Vec4d nv = _tmat * cv::Vec4d( v[0], v[1], v[2], 1);
        model->adjustVertex( vidx, nv[0], nv[1], nv[2]);
    }   // end foreach
}   // end operator()


