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

#include <ObjModelRegionSelector.h>
#include <FeatureUtils.h>
#include <algorithm>
#include <cassert>
using RFeatures::ObjModelRegionSelector;
using RFeatures::ObjModel;


// public static
ObjModelRegionSelector::Ptr ObjModelRegionSelector::create( const ObjModel* model, const cv::Vec3f& ov, int seedVtx)
{
    return Ptr( new ObjModelRegionSelector( model, ov, seedVtx), [](auto d){delete d;});
}   // end create


// private
ObjModelRegionSelector::ObjModelRegionSelector( const ObjModel* model, const cv::Vec3f& ov, int seedVtx)
    : _model(model), _ov( ov), _front( new IntSet), _rad(0)
{
    if ( seedVtx < 0)
        seedVtx = *model->getVertexIds().begin();
    _front->insert(seedVtx);
    setRadius(DBL_MAX);
}  // end ctor


// private
ObjModelRegionSelector::~ObjModelRegionSelector() { delete _front;}


// public
size_t ObjModelRegionSelector::setCentre( const cv::Vec3f& np)
{
    _ov = np;
    return setRadius( _rad);
}   // end setCentre


namespace {

// vflag denotes whether vidx should:
// -1) Be neither on the front or in the body since it is outside the radius threshold (default assumption).
//  0) Stay on the front which requires that vidx is inside the radius threshold but that at least one of its
//     connected vertices are outside the radius threshold, or that vidx is an edge vertex.
//  1) Be in the body because all of its connected vertices are *not outside* the radius threshold.
int testMembership( int vidx, const ObjModel* m, const cv::Vec3f& ov, double R)
{
    using namespace RFeatures;
    const double rval = l2sq( m->vtx(vidx) - ov);
    //std::cerr << vidx << ") |" << m->vtx(vidx) << " - " << ov << "|^2 = " << rval << std::endl;
    if ( rval > R)
        return -1;

    // vidx in body unless a connected vertex is outside radius threshold,
    // or vidx and a connected vertex makes a boundary edge.
    for ( int cv : m->getConnectedVertices(vidx))
    {
        if ( (l2sq( m->vtx(cv) - ov) > R) || (m->getNumSharedFaces( vidx, cv) == 1))
            return 0;   // vidx found to be on the front.
    }   // end for
    return 1;
}   // end testMembership

}   // end namespace


// public
size_t ObjModelRegionSelector::setRadius( double nrad)
{
    _rad = nrad;
    const double R = nrad < sqrt(DBL_MAX) ? nrad*nrad : nrad;

    IntSet* nfront = new IntSet;
    IntSet cfront = *_front; // Front vertices changed in the last iteration
    while ( !cfront.empty())
    {
        int fvidx = *cfront.begin();    // Get the next vertex from the front.
        cfront.erase(fvidx);
        const int vflag = testMembership( fvidx, _model, _ov, R);

        if ( vflag == -1)
        {
            // fvidx is outside the radius threshold so all of its connected vertices that are marked as being
            // in the body now need to be considered in subsequent loop iterations as potential front vertices.
            nfront->erase(fvidx);
            for ( int cv : _model->getConnectedVertices(fvidx))
            {
                if ( _body.count(cv) > 0)
                {
                    _body.erase(cv);
                    cfront.insert(cv);
                    nfront->insert(cv);
                }   // end if
            }   // end foreach
        }   // end if
        else
        {
            if ( vflag == 0)
            {
                nfront->insert(fvidx);  // fvidx will continue to be a front vertex.
                // Adjustments in the boundary that do not cause fvidx to stop being a front vertex, but that may
                // affect the state of its connected vertices (that are currently marked as front vertices) are
                // addressed when the loop gets around to those front vertices.
            }   // end if
            else if ( vflag == 1)
            {
                // fvidx is in the body now and so all of its connected vertices that aren't
                // in the body now need to be considered as potential front vertices.
                nfront->erase(fvidx);
                _body.insert(fvidx);
            }   // end else if

            for ( int cv : _model->getConnectedVertices(fvidx))
            {
                if ( _body.count(cv) == 0 && nfront->count(cv) == 0)
                {
                    cfront.insert(cv);
                    nfront->insert(cv);
                }   // end if
            }   // end foreach
        }   // end else
    }   // end while

    delete _front;
    _front = nfront;
    return _front->size() + _body.size();
}   // end setRadius


namespace {

// Gets the next vertex in the set that's connected to v AND is most distant from ov.
int getNextVertexInSet( const ObjModel* cmodel, const cv::Vec3f& ov, const IntSet& fvidxs, int v)
{
    const IntSet& cvs = cmodel->getConnectedVertices(v);
    v = -1;
    double maxd = 0;
    for ( int cv : cvs)
    {
        if ( fvidxs.count(cv) > 0)
        {
            const double rval = RFeatures::l2sq( cmodel->vtx(cv) - ov);
            if ( rval > maxd)
            {
                v = cv;
                maxd = rval;
            }   // end if
        }   // end if
    }   // end for
    return v;
}   // end getNextVertexInSet

}   // end namespace


// public
size_t ObjModelRegionSelector::getBoundary( std::list<int>& line) const
{
    line.clear();
    if ( _front->empty())
        return 0;

    IntSet fvidxs = *_front;    // Copy out the front vertices
    int v = *fvidxs.begin();

    while ( v >= 0)
    {
        line.push_back(v);
        //std::cerr << line.size() << " / " << _front->size() << std::endl;
        fvidxs.erase(v);
        // Get the next vertex on the front that's connected to v that isn't already added
        v = getNextVertexInSet( _model, _ov, fvidxs, v);
    }   // end while

    return line.size();
}   // end getBoundary


// public
void ObjModelRegionSelector::getRegionFaces( IntSet& cfids) const
{
    cfids.clear();
    for ( int cv : _body)
    {
        const IntSet& fids = _model->getFaceIds(cv);
        std::for_each(std::begin(fids), std::end(fids), [&](int x){cfids.insert(x);});
    }   // end for
}   // end getRegionFaces
