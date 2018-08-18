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
ObjModelRegionSelector::Ptr ObjModelRegionSelector::create( const ObjModel* model, int svtx)
{
    assert( model->getNumFaces() > 0);

    if ( svtx >= 0 && model->getFaceIds(svtx).empty())
    {
        std::cerr << "[ERROR] RFeatures::ObjModelRegionSelector::create: "
                  << "Cannot create a region selector from a vertex with no polygons attached!" << std::endl;
        return nullptr;
    }   // end if

    if ( svtx < 0)
        svtx = model->poly( *model->getFaceIds().begin()).fvindices[0];

    return Ptr( new ObjModelRegionSelector( model, svtx), [](auto d){delete d;});
}   // end create


// private
ObjModelRegionSelector::ObjModelRegionSelector( const ObjModel* model, int svtx)
    : _cv(-1), _offset(0,0,0), _front( new IntSet), _rad(DBL_MAX)
{
    const size_t rval = setCentre( model, svtx, _offset);
    assert( rval > 0);
}  // end ctor


// private
ObjModelRegionSelector::~ObjModelRegionSelector() { delete _front;}


// public
size_t ObjModelRegionSelector::setCentre( const ObjModel* model, int svtx, const cv::Vec3f& offset)
{
    assert( svtx >= 0);
    assert( !model->getFaceIds(svtx).empty());
    if ( model->getFaceIds(svtx).empty())
    {
        std::cerr << "[ERROR] RFeatures::ObjModelRegionSelector::setCentre: "
                  << "Cannot set new centre if vertex has no attached polygons!" << std::endl;
        return 0;
    }   // end if

    _cv = svtx;
    _cf = *model->getFaceIds(svtx).begin();   // Polygon used as local coordinate frame for offset

    // Calculate and return the basis vectors for the newly set vertex and polygon
    cv::Vec3f vi, vj, vk;
    calcBasisVectors( model, vi, vj, vk);
    // offset is given as difference from model->vtx(_cv) but needs to be stored according to basis vectors
    // defined above so that if the model is translated, the offset can still be applied (since the basis
    // vectors will have changed orientation).
    _offset = cv::Vec3f( vi.dot(offset), vj.dot(offset), vk.dot(offset));

    _body.clear();
    _front->clear();
    _front->insert( _cv);
    return setRadius( model, _rad);
}   // end setCentre


// public
cv::Vec3f ObjModelRegionSelector::centre( const ObjModel* model) const
{
    cv::Vec3f vi, vj, vk;
    calcBasisVectors( model, vi, vj, vk);
    // Calculate and return the true offset using the relative offset stored in terms of these basis vectors.
    return model->vtx(_cv) + cv::Vec3f( vi.dot(_offset), vj.dot(_offset), vk.dot(_offset));
}   // end centre


// private
void ObjModelRegionSelector::calcBasisVectors( const ObjModel* model, cv::Vec3f& vi, cv::Vec3f& vj, cv::Vec3f& vk) const
{
    vk = model->calcFaceNorm( _cf, vi, vj);
    // Make orthonormal set (not strictly needed but whatever)
    vj = vi.cross(vk);  // Doesn't matter how this is ordered as long as consistent
}   // end calcBasisVectors


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
size_t ObjModelRegionSelector::setRadius( const ObjModel* model, double nrad)
{
    nrad = std::max(0.0, nrad);
    const double R = nrad < sqrt(DBL_MAX) ? nrad*nrad : nrad;
    const cv::Vec3f ov = centre( model);

    IntSet* nfront = new IntSet;
    IntSet cfront = *_front; // Front vertices changed in the last iteration
    while ( !cfront.empty())
    {
        int fvidx = *cfront.begin();    // Get the next vertex from the front.
        cfront.erase(fvidx);
        const int vflag = testMembership( fvidx, model, ov, R);

        if ( vflag == -1)
        {
            // fvidx is outside the radius threshold so all of its connected vertices
            // *** that are marked as being in the body ***
            // now need to be considered in subsequent loop iterations as potential front vertices.
            nfront->erase(fvidx);
            for ( int cv : model->getConnectedVertices(fvidx))
            {
                if ( _body.count(cv) > 0)
                {
                    _body.erase(cv);
                    cfront.insert(cv);
                    nfront->insert(cv);
                }   // end if
            }   // end foreach

            // If the new front is now empty, the set radius was too small to retain even a single
            // seed vertex (which is necessary).
            if ( nfront->empty() && cfront.empty())
            {
                nrad = cv::norm( model->vtx(fvidx) - ov);
                //std::cerr << "Final radius = " << nrad << std::endl;
                nfront->insert(fvidx);
            }   // end if
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
                // fvidx is in the body now and so all of its connected vertices
                // *** that aren't in the body now ***
                // need to be considered as potential front vertices.
                nfront->erase(fvidx);
                _body.insert(fvidx);
            }   // end else if

            for ( int cv : model->getConnectedVertices(fvidx))
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
    _rad = nrad;

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
size_t ObjModelRegionSelector::boundary( const ObjModel* model, std::list<int>& line) const
{
    line.clear();
    if ( _front->empty())
        return 0;

    const cv::Vec3f ov = centre( model);
    IntSet fvidxs = *_front;    // Copy out the front vertices
    int v = *fvidxs.begin();

    while ( v >= 0)
    {
        line.push_back(v);
        //std::cerr << line.size() << " / " << _front->size() << std::endl;
        fvidxs.erase(v);
        // Get the next vertex on the front that's connected to v that isn't already added
        v = getNextVertexInSet( model, ov, fvidxs, v);
    }   // end while

    return line.size();
}   // end boundary


// public
void ObjModelRegionSelector::selectedFaces( const ObjModel* model, IntSet& cfids) const
{
    cfids.clear();
    for ( int cv : _body)
    {
        const IntSet& fids = model->getFaceIds(cv);
        std::for_each(std::begin(fids), std::end(fids), [&](int x){cfids.insert(x);});
    }   // end for
}   // end selectedFaces
