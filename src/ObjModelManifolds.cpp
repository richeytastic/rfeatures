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

#include <ObjModelManifolds.h>
#include <ObjModelTriangleMeshParser.h>
#include <ObjModelCopier.h>
using RFeatures::ObjModelManifolds;
using RFeatures::ObjModelTriangleMeshParser;
using RFeatures::ObjModelManifoldBoundaries;
using RFeatures::ObjManifold;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <algorithm>
#include <iomanip>
#include <cassert>


namespace {

void createEdgeSet( const IntSet& mset, IntSet& eset, const ObjModel* model)
{
#ifndef NDEBUG
    std::unordered_map<int, IntSet> ecounts;    // For checking that mset actually is a manifold
#endif
    for ( int fid : mset)
    {
        const ObjPoly& f = model->face(fid);
        const int e0 = model->edgeId( f[0], f[1]);
        const int e1 = model->edgeId( f[1], f[2]);
        const int e2 = model->edgeId( f[2], f[0]);

#ifndef NDEBUG
        ecounts[e0].insert(fid);
        ecounts[e1].insert(fid);
        ecounts[e2].insert(fid);
#endif

        if ( eset.count(e0) == 0)
            eset.insert(e0);
        else
            eset.erase(e0);

        if ( eset.count(e1) == 0)
            eset.insert(e1);
        else
            eset.erase(e1);

        if ( eset.count(e2) == 0)
            eset.insert(e2);
        else
            eset.erase(e2);
    }   // end for

#ifndef NDEBUG
    int errId = -1;
    for ( const auto& p : ecounts)
    {
        if ( p.second.size() > 2)
        {
            errId = p.first;
            std::cerr << "[ERROR] More than two polygons attached to manifold edge " << p.first << ":" << std::endl;
            for ( int fid : p.second)
            {
                const ObjPoly& f = model->face(fid);
                std::cerr << "    " << std::setw(6) << std::right << f << std::endl;
            }   // end for
        }   // end if
    }   // end for
    assert( errId < 0);
#endif
}   // end createEdgeSet


struct ObjModelBoundaryEdger2 : RFeatures::ObjModelBoundaryParser
{
    explicit ObjModelBoundaryEdger2( const IntSet& polys) : _polys(polys) {}

    bool parseEdge( int fid, const cv::Vec2i& e, int& pnfid) override
    {
        const int v0 = e[0];
        const int v1 = e[1];
        bool crossEdge = false;
        // Only cross if ns == 2 and the polygon on the opposite edge is in the provisional
        // set and one of its opposite edges is shared by no other polygons in the provisional manifold set.
        if ( model->nspolys( v0, v1) == 2)
        {
            // Work out whether we should enter pnfid next (cross v0-->v1) based on how
            // edge connected pnfid is to the set of provisional polygons. If polygon
            // pnfid is connected on both its other edges to polygons in _polys then we
            // don't cross to it meaning that the final set of polygons parsed for
            // removal from the provisional set will not include pnfid (but will include fid).
            pnfid = model->oppositePoly( fid, v0, v1);
            if ( _polys.count( pnfid) > 0)
            {
                assert( pnfid >= 0);
                const ObjPoly& pnfc = model->face(pnfid);
                const int v2 = pnfc.opposite( v0, v1);
                crossEdge = !_testFarEdgeMembership( pnfid, v2, v0) || !_testFarEdgeMembership( pnfid, v2, v1);
            }   // end if
        }   // end if
        assert( !crossEdge || pnfid >= 0);
        return crossEdge;
    }   // end parseEdge

private:
    const IntSet& _polys;

    bool _testFarEdgeMembership( int fc, int v0, int v1) const
    {
        for ( int f : model->spolys( v0, v1))  // Get shared polys from one edge
        {
            if ( f != fc && _polys.count(f) > 0) // Edge connects to a polygon in the provisional set?
                return true;
        }   // end for
        return false;
    }   // end _testFarEdgeMembership
};  // end struct


struct ObjModelBoundaryEdger : RFeatures::ObjModelBoundaryParser
{
    ObjModelBoundaryEdger() : /*_debug(false),*/ _polys(nullptr) {}

    //void setDebug( bool d) { _debug = d;}
    size_t numPolysRemain() const { return _rpolys.size();}
    int nextSeedPoly() const { return *_rpolys.begin();}
    void reset() override { _rpolys = model->faces();}


    bool parseEdge( int fid, const cv::Vec2i& e, int& pnfid) override   // Called multiple times for fid
    {
        const int eid = model->edgeId( e[0], e[1]);
#ifndef NDEBUG
        /*
        if ( _debug)
            std::cerr << "ObjModelBoundaryEdger::parseEdge( PolyId=" << fid << ", EdgeId=" << eid << ")" << std::endl;
        */
        if ( _rpolys.count(fid) == 0)  // Shouldn't happen
            std::cerr << " *** Poly " << fid << " not found in remaining set!" << std::endl;
#endif
        assert( _rpolys.count(fid) > 0);
        // Set this polygon (fid) in the provisional manifold set. It will only be removed from _rpolys after cleaning.
        _polys->insert(fid);

        // Get only the polygons shared on this edge that still remain to be set in a manifold (excluding fid).
        IntSet sfids = model->spolys(e[0], e[1]);    // Copy out
        auto it = sfids.begin();
        while ( it != sfids.end())
        {
            if ( _rpolys.count(*it) == 0 || _polys->count(*it) > 0) // Erase if already in this manifold, or seen previously.
                it = sfids.erase(it);
            else
                ++it;
        }   // end while

        pnfid = -1;

        // sfids now only contains polygon ids not previously found on other manifolds
        // Cross edge only if it's shared by exactly one other polygon.
        const size_t ns = sfids.size();
        if ( ns == 1)
        {
            pnfid = *sfids.begin();
            assert( _rpolys.count( pnfid) > 0);
/*
#ifndef NDEBUG
            if ( _debug)
                std::cerr << " Specified poly to be parsed next = " << pnfid << std::endl;
#endif
*/
        }   // end if

        if ( model->nspolys( eid) > 2)
        {
            // Once parsing is complete, polygons not in the parsed set that
            // are attached to the problem edges are discounted from the polygons attached
            // to those edges to reassess if the edge should be treated as genuine.
            if ( _pedges[eid] == nullptr)
                _pedges[eid] = new IntSet;
            _pedges[eid]->insert(fid);   // Will only need to be looked at further if # of polygons attached to edge eid > 2.
/*
#ifndef NDEBUG
            if ( _debug)
                std::cerr << " Added poly " << fid << " to edge " << eid << std::endl;
#endif
*/
        }   // end if

        return pnfid >= 0;
    }   // end parseEdge


    void setNewManifold( IntSet* polys)//, IntSet* edges)
    {
        _polys = polys;
        assert( _polys);
        assert( _polys->empty());
        assert( _pedges.empty());
    }   // end setNewManifold


    // Call after parsing complete with the provisional set of manifold polygons.
    // Returns the number of polygons in the final manifold.
    size_t cleanProvisionalManifold()
    {
        IntSet srfids;  // Seed polygons to remove from the provisional manifold

        // Go through the problem edges to check the number of manifold polygons attached.
        // Only edges with more than 2 manifold polygons present potential problems.
        while ( !_pedges.empty())
        {
            auto it = _pedges.begin();
            IntSet* efids = it->second;
            // If only two polygons in the manifold, then no problem (and not an edge).
            if ( efids->size() > 2)
            {
                // The edge has more than 2 polygons in the provisional manifold, but we can only have 2.
                // For each polygon attached to the edge, score its flatness with respect to the other
                // triangles it's connected to on its adjacent sides. Only the two triangles with the
                // best flatness metrics are kept. The others form a second set of seed polygons which
                // will be removed from the first.
                const RFeatures::ObjEdge& edge = model->edge( it->first);
                _setFlattestPolyPair( edge[0], edge[1], *efids);
                for ( int f : *efids)
                    srfids.insert( f);
            }   // end if
            delete efids;
            _pedges.erase(it);
        }   // end while

        // Remove selected polygons from the parsed manifold.
        while ( !srfids.empty())
        {
            int sfid = *srfids.begin();
            srfids.erase(sfid);
            if ( _polys->count(sfid) > 0)
                _removeNonManifoldPolys( sfid);
        }   // end while

        for ( int f : *_polys)
            _rpolys.erase(f);

        return _polys->size();
    }   // end cleanProvisionalManifold

private:
    //bool _debug;
    IntSet* _polys;  // Manifold polygons
    IntSet _rpolys;  // Remaining polys on model to find
    std::unordered_map<int, IntSet*> _pedges;   // Problem edges mapped to provisional manifold polygons


    void _removeNonManifoldPolys( int f)
    {
        ObjModelTriangleMeshParser m2parser(model);
        ObjModelBoundaryEdger2 edger( *_polys);
        m2parser.setBoundaryParser( &edger);
        m2parser.parse( f);
        const IntSet& remset = m2parser.parsed();
        // Remove all the parsed polys from _polys
        for ( int fid : remset)
            _polys->erase( fid);
    }   // end _removeNonManifoldPolys


    void _setFlattestPolyPair( int v0, int v1, IntSet& efids)
    {
        double bmetric0 = -DBL_MAX;    // First best flatness score
        double bmetric1 = -DBL_MAX;    // Second best flatness score
        int bi0 = -1;  // The most flat triangle
        int bi1 = -1;  // The second most flat triangle

        cv::Vec3f u10;
        cv::normalize( model->vtx(v1) - model->vtx(v0), u10);    // The edge vector

        for ( int f : efids)
        {
            double fmetric = 0;
            int cnt = 0;

            const int vi = model->face(f).opposite(v0, v1);    // Vertex opposite edge v0-->v1 on triangle fid.

            cv::Vec3f ui0;
            cv::normalize( model->vtx(vi) - model->vtx(v0), ui0);

            //const double fiArea = RFeatures::calcTriangleArea( model->vtx(v0), model->vtx(v1), model->vtx(vi));
            //const cv::Vec3d ufidi = fiArea * ui0.cross(u10); // Normal vector for triangle fid weighted by area
            //double areaTotal = 2*fiArea;
            const cv::Vec3d ufidi = ui0.cross(u10); // Normal vector for triangle fid weighted by area

            // If triangle fid has adjacent triangles on either of its two opposite edges (i.e. v0-->vi, or v1-->vi),
            // then calculate the norms of these triangles to compare against the norm of triangle fid for a flatness
            // metric being the average dot product of the target triangle's norm with these two neighbours' norms.
            // Target triangles without neighbours on the opposite edges have a flatness score of zero since they can't
            // be reached except through the problem edge and therefore should definitely not remain in the manifold.
            
            if ( model->nspolys( v0, vi) == 2)
            {
                const int vj = model->face( model->oppositePoly( f, v0, vi)).opposite(v0, vi);
                cv::Vec3f uj0;
                cv::normalize( model->vtx(vj) - model->vtx(v0), uj0);
                //const double fjArea = RFeatures::calcTriangleArea( model->vtx(v0), model->vtx(v1), model->vtx(vj));
                //const cv::Vec3d ufidj = fjArea * uj0.cross(ui0);
                const cv::Vec3d ufidj = uj0.cross(ui0);
                fmetric = ufidj.dot(ufidi);
                //areaTotal += fjArea;
                cnt++;
            }   // end if

            if ( model->nspolys( v1, vi) == 2)
            {
                const int vk = model->face( model->oppositePoly( f, v1, vi)).opposite(v1, vi);
                cv::Vec3f uki, u1i;
                cv::normalize( model->vtx(vk) - model->vtx(vi), uki);
                cv::normalize( model->vtx(v1) - model->vtx(vi), u1i);
                //const double fkArea = RFeatures::calcTriangleArea( model->vtx(v1), model->vtx(vi), model->vtx(vk));
                //const cv::Vec3d ufidk = fkArea * uki.cross(u1i);
                const cv::Vec3d ufidk = uki.cross(u1i);
                fmetric += ufidk.dot(ufidi);
                //areaTotal += fkArea;
                cnt++;
            }   // end if

            if ( cnt > 1)
                fmetric /= cnt;

            if ( fmetric > bmetric0)
            {
                // Make the first flattest the second flattest
                bi1 = bi0;
                bmetric1 = bmetric0;
                // Set new flattest
                bmetric0 = fmetric;
                bi0 = f;
            }   // end if
            else if ( fmetric > bmetric1)
            {
                bmetric1 = fmetric;
                bi1 = f;
            }   // end else if
        }   // end for

        // Finally, ensure the chosen two flattest triangles for the manifold are removed from efids.
        assert( bi0);
        assert( bi1);
        efids.erase(bi0);
        efids.erase(bi1);
    }   // end _setFlattestPolyPair
};  // end struct

}   // end namespace


void ObjModelManifolds::_findManifolds()
{
    ObjModelTriangleMeshParser mparser(_model.get()); // For parsing the model manifolds
    ObjModelBoundaryEdger boundaryEdger;
    mparser.setBoundaryParser( &boundaryEdger);

    while ( boundaryEdger.numPolysRemain() > 0)
    {
        _manfs.push_back( new ObjManifold( _model.get()));
        ObjManifold& m = *_manfs.back();
        boundaryEdger.setNewManifold( &m._polys);
        mparser.parse( boundaryEdger.nextSeedPoly());
        boundaryEdger.cleanProvisionalManifold();
        createEdgeSet( m._polys, m._edges, _model.get());
    }   // end while

    // Sort manifolds in descending order of number of polygons.
    std::sort( std::begin(_manfs), std::end(_manfs),
            []( const ObjManifold* m0, const ObjManifold* m1){return m1->polygons().size() < m0->polygons().size();});

    // Set reverse lookup for polygon IDs to manifold ID
    const int n = static_cast<int>(_manfs.size());
    for ( int i = 0; i < n; ++i)
        for ( int f : _manfs.at(size_t(i))->polygons())
            _poly2manf[f] = i;
}   // end _findManifolds


ObjModelManifolds::Ptr ObjModelManifolds::create( ObjModel::Ptr m)
{
    Ptr omc( new ObjModelManifolds(m), [](ObjModelManifolds* d){ delete d;});
    omc->_findManifolds();
    return omc;
}   // end create


// private
ObjModelManifolds::ObjModelManifolds( ObjModel::Ptr m) : _model(m) {}


// private
ObjModelManifolds::~ObjModelManifolds()
{
    for ( const ObjManifold* m : _manfs)
        delete m;
}   // end dtor


const ObjManifold* ObjModelManifolds::manifold( int i) const
{
    if ( i < 0 || i >= int(_manfs.size()))
        return nullptr;
    return _manfs.at(size_t(i));
}   // end manifold


// private
ObjManifold::ObjManifold( const ObjModel* m) : _model(m) {}


// private
ObjManifold::ObjManifold( const ObjManifold& m) { *this = m;}


// private
ObjManifold& ObjManifold::operator=( const ObjManifold& m)
{
    _model = m._model;
    _polys = m._polys;
    _edges = m._edges;
    _bnds = m._bnds;
    return *this;
}   // end operator=


ObjModel::Ptr ObjManifold::toObject() const
{
    ObjModelCopier copier( _model);
    for ( int f : _polys)
        copier.add( f);
    return copier.copiedModel();
}   // end toObject


const IntSet& ObjManifold::vertices() const
{
    if ( _verts.empty())
    {
        for ( int fid : _polys)
        {
            const ObjPoly& f = _model->face(fid);
            _verts.insert(f[0]);
            _verts.insert(f[1]);
            _verts.insert(f[2]);
        }   // end for
    }   // end if
    return _verts;
}   // end vertices


int ObjModelManifolds::manifoldId( int f) const
{
    return _poly2manf.count(f) > 0 ? _poly2manf.at(f) : -1;
}   // end manifoldId


const ObjModelManifoldBoundaries& ObjManifold::boundaries() const
{
    // Find the boundaries if not yet obtained
    if ( _bnds.count() == 0 && !edges().empty())
    {
#ifndef NDEBUG
        int nbs = _bnds.sort( _model, edges());
        assert( nbs >= 0);
#else
        _bnds.sort( _model, edges());
#endif
    }   // end if
    return _bnds;
}   // end boundaries


ObjModelManifolds::Ptr ObjModelManifolds::reduceManifolds( int n)
{
    ObjModel::Ptr nmod = ObjModel::create();
    const ObjModel* cmod = nmod.get();
    nmod->copyInMaterials( &*_model, true/*Share materials*/);

    ObjModelManifolds* omm = new ObjModelManifolds( nmod);
    n = std::max( 1, std::min( n, int(count())));
    omm->_manfs.resize(size_t(n));

    for ( int i = 0; i < n; ++i)
    {
        ObjManifold* man = omm->_manfs[size_t(i)] = new ObjManifold(cmod);
        const ObjManifold* sman = manifold(i); // Source manifold on this object

        // Add vertex remapped faces
        std::unordered_map<int,int> vvmap;  // Old to new vertices
        std::unordered_map<int, int> ffmap;  // Map old faces to new
        for ( int fid : sman->_polys)
        {
            const ObjPoly& f = _model->face(fid);
            const int v0 = nmod->addVertex(_model->vtx(f[0]));
            const int v1 = nmod->addVertex(_model->vtx(f[1]));
            const int v2 = nmod->addVertex(_model->vtx(f[2]));

            vvmap[f[0]] = v0;
            vvmap[f[1]] = v1;
            vvmap[f[2]] = v2;

            const int nfid = nmod->addFace( v0, v1, v2);
            ffmap[fid] = nfid;
            man->_polys.insert( nfid);
            omm->_poly2manf[nfid] = i;
        }   // end for

        // Add the texture coords to the new model
        if ( _model->numMats() > 0)
        {
            for ( int fid : sman->_polys)
            {
                const int mid = _model->faceMaterialId(fid);
                if ( mid < 0)
                    continue;

                const cv::Vec2f& uva = _model->faceUV( fid, 0);
                const cv::Vec2f& uvb = _model->faceUV( fid, 1);
                const cv::Vec2f& uvc = _model->faceUV( fid, 2);
                nmod->setOrderedFaceUVs( mid, ffmap[fid], uva, uvb, uvc);
            }   // end for
        }   // end if

        // Add vertex remapped edges
        for ( int eid : sman->_edges)
        {
            const ObjEdge& edge = _model->edge(eid);
            man->_edges.insert( nmod->edgeId( vvmap[edge[0]], vvmap[edge[1]]));
        }   // end for

        // Create a new set of boundaries with remapped vertices.
        man->_bnds = ObjModelManifoldBoundaries( sman->_bnds, vvmap);
    }   // end for

    return Ptr( omm, [](ObjModelManifolds* d){ delete d;});
}   // end reduceManifolds
