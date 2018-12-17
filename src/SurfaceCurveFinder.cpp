/************************************************************************
 * Copyright (C) 2018 Richard Palmer
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
using RFeatures::SurfaceCurveFinder;
using RFeatures::ObjModel;


namespace {

bool calcCrossing( const ObjModel* model, int a, int b, const cv::Vec3d& rv, const cv::Vec3d& av, cv::Vec3d& cp)
{
    assert( a != b);
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
    cv::Vec3d pp = sv + cv::norm(d)*p;  // The projected point
    assert( RFeatures::cosi( sv, pp, fv) >= 0);
    return pp;
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
int SurfaceCurveFinder::getOppositeEdge( const cv::Vec3d& sv, const cv::Vec3d& fv, int f) const
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



SurfaceCurveFinder::SurfaceCurveFinder( const ObjModel* model) : _model(model) {}


bool SurfaceCurveFinder::findPath( const cv::Vec3f& fsv, int sfid,
                                   const cv::Vec3f& ffv, int lfid,
                                   std::list<cv::Vec3f>& pts)
{
    pts.clear();
    pts.push_back(fsv);

    //std::cerr << " Path start --> finish vertices:  " << fsv << " --> " << ffv << std::endl;
    const cv::Vec3d dfv = ffv;

    IntSet faces;

    cv::Vec3d cp = fsv;
    cv::Vec3d fv, sv;

    int h, j, c, s;
    s = c = getOppositeEdge( cp, dfv, sfid);
    _model->poly(sfid).opposite(c, h, j);
    int f = sfid;

    while ( f >= 0 && f != lfid && l2sq( sv - dfv) > 1e-8)
    {
        c = _model->poly(f).opposite(h,j);
        faces.insert(f);
        /*
        std::cerr << "  F=" << std::setw(6) << std::right << f
                  << "; c=" << std::setw(6) << c
                  << "; h=" << std::setw(6) << h
                  << "; j=" << std::setw(6) << j << std::endl;
        */
        sv = cp;
        const cv::Vec3d tn = calcNorm( _model, c, h, j);    // Calculate this face normal
        fv = calcPlanePosition( tn, sv, dfv);

        // Find the next edge h,j on poly f to unfold
        if ( calcCrossing( _model, c, j, sv, fv, cp))
        {
            s = h;  // Record where h was.
            h = c;  // Move h to c.
        }   // end if
        else if ( calcCrossing( _model, h, c, sv, fv, cp))
        {
            s = j;  // Record where j was.
            j = c;  // Move j to c.
        }   // end else if
        else
        {
            //std::cerr << " Using pseudo-triangle" << std::endl;

            // If neither edge h-c or j-c is crossed, then the algorithm is trying to
            // re-cross edge h-j. To progress, a pseudo triangle is constructed from vertices c,s,t
            // where t is h or j (whichever is closer to fv) and s is the previous position of either h or j.

            // First ensure that vertex h is always closer than vertex j to the endpoint.
            if ( l2sq( _model->vtx(h) - ffv) > l2sq( _model->vtx(j) - ffv))
                std::swap(h,j);

            const cv::Vec3d tn = calcNorm( _model, h, c, s);

            const cv::Vec3d hv = _model->vtx(h);
            // Project sv into the plane of the pseudo triangle or the crossing calculation won't work.
            sv = calcPlanePosition( tn, hv, sv);
            fv = calcPlanePosition( tn, sv, dfv);

            // It now must be the case that a crossing exists on either edge h-c, or h-s.
            int u = j;  // (remember where j was for making sure we obtain the correct real triangle after)
            if ( l2sq( sv - fv) > 1e-8)
            {
                if ( calcCrossing( _model, h, c, sv, fv, cp))
                    j = c;
                else if ( calcCrossing( _model, h, s, sv, fv, cp))
                    j = s;
                else
                    f = -1;
            }   // end if
            else
            {
                //std::cerr << " Path discovery ended with sv == fv" << std::endl;
                f = -1;
            }   // end else

            if ( f >= 0)
            {
                // Since the edge was calculated using a pseudo-triangle, need to get the real
                // triangle on this edge which will be comprised of vertices h,j,u.

                // Set f to the polygon that shares edge h-j and has u as its opposite vertex
                const IntSet& sfids = _model->spolys( h, j);
                f = *sfids.begin();
                if ( sfids.size() > 1 && _model->poly(f).opposite(h,j) != u)
                    f = *(++sfids.begin());
                if ( _model->poly(f).opposite(h,j) != u)    // Stop - unable to match triangle up.
                {
                    //std::cerr << " Non-matched polygon - ending path discovery prematurely!" << std::endl;
                    f = -1;
                }   // end if
            }   // end if
        }   // end else

        if ( f >= 0)
        {
            pts.push_back( cp);
            f = RFeatures::oppositePoly( _model, f, h, j);   // Next poly to unfold (could be -1)
            if ( faces.count(f) > 0)
            {
                //std::cerr << " Loop encountered - ending path discovery prematurely!" << std::endl;
                f = -1;
            }   // end if
        }   // end if
    }   // end while

    pts.push_back(ffv);
    return f == lfid;
}   // end findPath
