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
#include <cassert>
using RFeatures::StraightPathFinder;
using RFeatures::ObjModel;


namespace {

bool calcCrossing( const ObjModel* model, int a, int b, const cv::Vec3d& rv, const cv::Vec3d& av, cv::Vec3d& cp)
{
    const cv::Vec3d v0 = model->vtx(a);
    const cv::Vec3d v1 = model->vtx(b);
    cp = RFeatures::intersection( v0, v1, av, rv);
    return RFeatures::isPointOnBothLineSegments( v0, v1, av, rv, cp);
}   // end calcCrossing



cv::Vec3d calcPlanePosition( cv::Vec3d n, const cv::Vec3d& sv, const cv::Vec3d& fv)
{
    const cv::Vec3d d = fv - sv;
    cv::Vec3d p;    // Calculate projection vector
    cv::normalize( n.cross(d.cross(n)), p);
    return sv + cv::norm(d)*p;
}   // end calcPlanePosition


cv::Vec3d calcNorm( const ObjModel* model, int c, int h, int j)
{
    cv::Vec3d u = model->vtx(h) - model->vtx(c);
    cv::Vec3d v = model->vtx(j) - model->vtx(c);
    return u.cross(v);
}   // end calcNorm

}   // end namespace


// private
// Return the vertex on f that is nearest to being on the line segment sv,fv.
int StraightPathFinder::getOppositeEdge( const cv::Vec3d& sv, const cv::Vec3d& fv, int f) const
{
    const int* vidxs = _model->getFaceVertices(f);
    const cv::Vec3d v0 = _model->vtx(vidxs[0]);
    const cv::Vec3d v1 = _model->vtx(vidxs[1]);
    const cv::Vec3d v2 = _model->vtx(vidxs[2]);

    double dmin = cosi( v0, sv, fv);
    const double d1 = cosi( v1, sv, fv);
    const double d2 = cosi( v2, sv, fv);

    int c = vidxs[0];
    if ( d1 < dmin)
    {
        c = vidxs[1];
        dmin = d1;
    }   // end if
    if ( d2 < dmin)
        c = vidxs[2];

    return c;
}   // end getOppositeEdge


// private
int StraightPathFinder::findNextRidgeVertex( const cv::Vec3f& pv, int s) const
{
    const cv::Vec3f& sv = _model->vtx(s);
    cv::Vec3f v0, v1;
    cv::normalize( sv - pv, v0);    // Gives direction going in

    double dmax = -1;
    int bvi = -1;
    for ( int vi : _model->cvtxs(s))
    {
        if ( _svtxs.count(vi) > 0)
            continue;

        cv::normalize( _model->vtx(vi) - sv, v1);
        double m = v1.dot(v0);
        if ( m > dmax)
        {
            bvi = vi;
            dmax = m;
        }   // end if
    }   // end for
    return bvi;
}   // end findNextRidgeVertex



StraightPathFinder::StraightPathFinder( const ObjModel* model) : _model(model) {}


bool StraightPathFinder::findPath( const cv::Vec3f& fsv, int sfid,
                                   const cv::Vec3f& ffv, int lfid,
                                   std::list<cv::Vec3f>& pts)
{
    pts.clear();
    pts.push_back(fsv);

    std::cerr << "Start --> finish vertices:  " << fsv << " --> " << ffv << std::endl;
    _dfv = ffv;
    _fT = lfid;
    _svtxs.clear();
    _faces.clear();

    cv::Vec3d cp = fsv;
    cv::Vec3d fv, sv;

    int h, j, c, s;
    s = c = getOppositeEdge( cp, _dfv, sfid);
    _model->poly(sfid).opposite(c, h, j);
    int f = sfid;

    while ( f >= 0 && f != lfid)
    {
        c = _model->poly(f).opposite(h,j);
        _faces.insert(f);
        _svtxs.insert(s);

        sv = cp;
        fv = calcPlanePosition( calcNorm( _model, c, h, j), sv, _dfv);

        bool getOppFace = true;

        // Find the next edge h,j on poly f to unfold
        if ( calcCrossing( _model, c, j, sv, fv, cp))
            s = h = c;
        else if ( calcCrossing( _model, h, c, sv, fv, cp))
            s = j = c;
        else
        {
            // We're on a ridge or valley between polygons so make the crossing point at s.
            // Since we are on a ridge/valley, keep going in the same direction until
            // there's a clear exit via a polygon edge. The last point added helps to give
            // us the direction we should be going in to find the next polygon.
            cp = _model->vtx(s);
            c = findNextRidgeVertex( *pts.rbegin(), s);
            assert( c >= 0);

            // Set the new polygon as one that shares this edge. Select the finish polygon if it's there.
            const IntSet& sfids = _model->getSharedFaces(s,c);
            int f0 = *sfids.begin(); 
            int f1 = sfids.size() > 1 ? *(++sfids.begin()) : f0;

            f = f0;
            if ( _faces.count(f0) > 0 || f1 == lfid)
                f = f1;

            assert( _faces.count(f) == 0);
            _faces.insert(f0);
            _faces.insert(f1);

            s = c;
            _model->poly(f).opposite(c, h, j);
            getOppFace = false;
        }   // end else

        std::cerr << f << std::endl;
        pts.push_back( cp);

        if ( getOppFace)
            f = RFeatures::oppositePoly( _model, f, h, j);   // Next poly to unfold (could be -1)
    }   // end while

    pts.push_back(ffv);
    return f == lfid;
}   // end findPath
