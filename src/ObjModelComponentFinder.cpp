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

#include <ObjModelComponentFinder.h>
#include <ObjModelMeshTraversalRecorder.h>
using RFeatures::ObjModelComponentFinder;
using RFeatures::ObjModelBoundaryFinder;
using RFeatures::ObjModelTriangleMeshParser;
using RFeatures::ObjModelMeshTraversalRecorder;
using RFeatures::ObjModel;
#include <algorithm>
#include <cassert>


// public
ObjModelComponentFinder::Ptr ObjModelComponentFinder::create( const ObjModelBoundaryFinder::Ptr bf)
{
    return Ptr( new ObjModelComponentFinder(bf), [](auto d){ delete d;});
}   // end create


// private
ObjModelComponentFinder::ObjModelComponentFinder( const ObjModelBoundaryFinder::Ptr bf) : _bf(bf)
{}   // end ctor


// private
ObjModelComponentFinder::~ObjModelComponentFinder()
{
    reset();
}   // end dtor


// private
void ObjModelComponentFinder::reset()
{
    if ( size() > 1) // Only delete if more than 1 component.
        std::for_each( std::begin(_components), std::end(_components), [&](auto s){ delete _cv.at(s); delete s;});
    _components.clear();
    _cv.clear();
    _cw.clear();
    _cb.clear();
    _lb.clear();
}   // end reset

namespace {

void setBounds( const ObjModel* model, int vidx, cv::Vec6i& bounds)
{
    const cv::Vec3f& newv = model->vtx(vidx);

    if ( newv[0] < model->vtx(bounds[0])[0])    // Xmin
        bounds[0] = vidx;
    if ( newv[0] > model->vtx(bounds[1])[0])    // Xmax
        bounds[1] = vidx;

    if ( newv[1] < model->vtx(bounds[2])[1])    // Ymin
        bounds[2] = vidx;
    if ( newv[1] > model->vtx(bounds[3])[1])    // Ymax
        bounds[3] = vidx;

    if ( newv[2] < model->vtx(bounds[4])[2])    // Zmin
        bounds[4] = vidx;
    if ( newv[2] > model->vtx(bounds[5])[2])    // Zmax
        bounds[5] = vidx;
}   // end setBounds


cv::Vec6i findBounds( const IntSet& vidxs, const ObjModel* model)
{
    cv::Vec6i bounds;
    bounds[0] = *vidxs.begin();  // Xmin
    bounds[1] = bounds[0];  // Xmax
    bounds[2] = bounds[0];  // Ymin
    bounds[3] = bounds[0];  // Ymax
    bounds[4] = bounds[0];  // Zmin
    bounds[5] = bounds[0];  // Zmax
    std::for_each( std::begin(vidxs), std::end(vidxs), [&](int v){ setBounds( model, v, bounds);});
    return bounds;
}   // end findBounds


const IntSet* createNewVertexSet( const IntSet* cset, const ObjModel* model)
{
    IntSet *vset = new IntSet;
    for ( int fid : *cset)
    {
        const int* vindices = model->getFaceVertices(fid);
        vset->insert(vindices[0]);
        vset->insert(vindices[1]);
        vset->insert(vindices[2]);
    }   // end for
    return vset;
}   // end createNewVertexSet

}   // end 


// public
size_t ObjModelComponentFinder::findComponents()
{
    const int nbs = (int)_bf->size(); // Num boundaries on the model
    const ObjModel* model = _bf->model();
    reset();

    IntSet allPolys = model->getFaceIds();      // Copy out (for erasing as components found)
    ObjModelTriangleMeshParser parser(model);   // For parsing the model components
    ObjModelMeshTraversalRecorder vrecorder;    // Record the vertices parsed
    parser.addTriangleParser( &vrecorder);
    const IntSet& traversed = vrecorder.traversed();    // Reference to the traversed vertices

    // Look at all the boundaries first
    for ( int c = 0; c < nbs; ++c)
    {
        const std::list<int>& blist = _bf->boundary(c); // Get boundary (list of ordered vertices)
        const int sfidx = *model->getFaceIds(*blist.begin()).begin();  // A polygon attached to first vertex in boundary.

        // If a vertex from this boundary is in set of already parsed vertices, record component it maps to.
        if ( traversed.count(blist.front()) > 0)
        {
            // Find which of the components already parsed that this vertex is in.
            const IntSet* rset = nullptr;
            for ( const IntSet* fset : _components)
            {
                if ( fset->count(sfidx) > 0)
                {
                    rset = fset;
                    break;
                }   // end if
            }   // end for

            assert(rset);
            _cb[rset].insert(c);   // Map boundary c to this component
        }   // end if
        else
        {   // Otherwise, this is a boundary on a new component, and it must be the longest for the component.
            createNewComponent( &parser, sfidx, allPolys);
            const IntSet* cset = _components.back();    // The component just created.
            _cb[cset].insert(c);    // Map boundary index to the just parsed component.
            _lb[cset] = c;          // Set as the longest boundary for the component.
        }   // end else
    }   // end for

    // After parsing all the boundaries, it's possible there are remaining components with no boundary, so check for them.
    while ( !allPolys.empty())
        createNewComponent( &parser, *allPolys.begin(), allPolys);

    // Sort components in descending order of number of polygons.
    std::sort( std::begin(_components), std::end(_components), []( auto p0, auto p1){return p1->size() < p0->size();});
    return _components.size();
}   // end findComponents


// private
void ObjModelComponentFinder::createNewComponent( ObjModelTriangleMeshParser* parser, int sfidx, IntSet& allPolys)
{
    const ObjModel* model = _bf->model();
    IntSet *cset = new IntSet;
    parser->setParseSet(cset);
    parser->parse( sfidx);
    _cv[cset] = createNewVertexSet(cset, model);    // Copy out the component vertices into a new set.
    _cw[cset] = findBounds( *_cv[cset], model);     // Find the spatial bounds for this component
    for ( int f : *cset)
        allPolys.erase(f);
    _components.push_back(cset);
}   // end createNewComponent



// public
const IntSet* ObjModelComponentFinder::componentPolygons( int i) const
{
    if ( i >= (int)_components.size() || i < 0)
        return nullptr;
    return _components.at(i);
}   // end componentPolygons


// public
const IntSet* ObjModelComponentFinder::componentVertices( int i) const
{
    const IntSet* c = componentPolygons(i);
    return ( c == nullptr || _cv.count(c) == 0) ? nullptr : _cv.at(c);
}   // end componentVertices


// public
const cv::Vec6i* ObjModelComponentFinder::componentBounds( int i) const
{
    const IntSet* c = componentPolygons(i);
    return ( c == nullptr || _cv.count(c) == 0) ? nullptr : &_cw.at(c);
}   // end componentBounds


// public
int ObjModelComponentFinder::numComponentBoundaries( int i) const
{
    const IntSet* cb = cboundaries(i);
    return ( cb == nullptr) ? -1 : (int)cb->size();
}   // end numComponentBoundaries


// public
const IntSet* ObjModelComponentFinder::cboundaries( int i) const
{
    const IntSet* c = componentPolygons(i);
    return ( c == nullptr || _cb.count(c) == 0) ? nullptr : &_cb.at(c);
}   // end cboundaries


// public
int ObjModelComponentFinder::lboundary( int i) const
{
    const IntSet* c = componentPolygons(i);
    if ( c == nullptr)         // No component c
        return -2;
    if ( _cb.count(c) == 0) // No boundaries stored on component c
        return -1;
    return _lb.at(c);
}   // end lboundary
