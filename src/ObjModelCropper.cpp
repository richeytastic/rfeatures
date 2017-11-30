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

#include <ObjModelCropper.h>
#include <FeatureUtils.h>
#include <boost/foreach.hpp>
#include <cassert>
using RFeatures::ObjModelCropper;
using RFeatures::ObjModel;


// private
class ObjModelCropper::Deleter
{ public:
    void operator()( ObjModelCropper* d) { delete d;}
};  // end class


// public static
ObjModelCropper::Ptr ObjModelCropper::create( const ObjModel::Ptr model, const cv::Vec3f& ov, int seedVtx)
{
    return Ptr( new ObjModelCropper( model, ov, seedVtx), Deleter());
}   // end create


// private
ObjModelCropper::ObjModelCropper( const ObjModel::Ptr model, const cv::Vec3f& ov, int seedVtx)
    : _model(model), _ov( ov), _front( new IntSet)
{
    _front->insert(seedVtx);
    adjustRadius( DBL_MAX);
}  // end ctor


// private
ObjModelCropper::~ObjModelCropper() { delete _front;}


// public
size_t ObjModelCropper::adjustRadius( double nrad)
{
    const double radThresh = nrad < sqrt(DBL_MAX) ? nrad*nrad : nrad;

    IntSet* nfront = new IntSet;
    IntSet cfront = *_front; // Front vertices changed in the last iteration
    while ( !cfront.empty())
    {
        int fvidx = *cfront.begin();
        cfront.erase(fvidx);

        int vflag = -1;
        // vflag denotes whether fvidx should:
        // -1) Be neither on the front or in the body since it is outside the radius threshold (default assumption).
        //  0) Stay on front which requires that fvidx is inside the radius threshold but that at least one of its
        //     connected vertices is outside the radius threshold, or that fvidx is an edge vertex.
        //  1) Be in the body because all of its connected vertices are within the radius threshold.
        const IntSet& cvs = _model->getConnectedVertices(fvidx);
        const double rval = RFeatures::l2sq( _model->vtx(fvidx) - _ov);
        if ( rval <= radThresh)
        {
            // fvidx now assumed to be in the body unless at least one of its connected
            // vertices are found outside the radius threshold or fvidx and one of its
            // connected vertices make a boundary edge.
            vflag = 1;
            BOOST_FOREACH ( int cv, cvs)
            {
                if ( (RFeatures::l2sq( _model->vtx(cv) - _ov) > radThresh) || (_model->getNumSharedFaces( fvidx, cv) == 1))
                {
                    vflag = 0; // fvidx found to be on the front.
                    break;     // Don't need to check the remaining connected vertices.
                }   // end if
            }   // end foreach
        }   // end if

        if ( vflag == -1)
        {
            // fvidx is outside the radius threshold so all of its connected vertices that are marked as being
            // in the body, now need to be considered in subsequent loop iterations as potential front vertices.
            nfront->erase(fvidx);
            BOOST_FOREACH ( int cv, cvs)
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

            BOOST_FOREACH ( int cv, cvs)
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
}   // end adjustRadius


// public
size_t ObjModelCropper::getCroppedFaces( IntSet& cfids) const
{
    size_t nadded = 0;
    BOOST_FOREACH ( int cv, _body)
    {
        const IntSet& fids = _model->getFaceIds(cv);
        BOOST_FOREACH ( int fid, fids)
        {
            if ( cfids.count(fid) == 0)
            {
                cfids.insert(fid);
                nadded++;
            }   // end if
        }   // end foreach
    }   // end foreach
    return nadded;
}   // end get
