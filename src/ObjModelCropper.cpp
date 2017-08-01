#include <ObjModelCropper.h>
using RFeatures::ObjModelCropper;
using RFeatures::ObjModel;


ObjModelCropper::ObjModelCropper( const ObjModel::Ptr m, const cv::Vec3f& originVertex, double radiusThreshold)
        : _m(m), _ov( originVertex), _sqRadiusThreshold(radiusThreshold*radiusThreshold)
{}

double sqdist( const cv::Vec3f& v) { return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];}

bool ObjModelCropper::parseEdge( int fid, int v0, int v1)
{
    return sqdist( _ov - _m->getVertex(v0)) <= _sqRadiusThreshold
        && sqdist( _ov - _m->getVertex(v1)) <= _sqRadiusThreshold;
}   // end parseEdge
