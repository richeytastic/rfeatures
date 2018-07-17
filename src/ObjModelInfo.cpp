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

#include <ObjModelInfo.h>
#include <ObjModelCleaner.h>
#include <ObjModelTetrahedronReplacer.h>
using RFeatures::ObjModelInfo;
using RFeatures::ObjModelBoundaryFinder;
using RFeatures::ObjModelComponentFinder;
using RFeatures::ObjModelIntegrityChecker;
using RFeatures::ObjModelTetrahedronReplacer;
using RFeatures::ObjModelCleaner;
using RFeatures::ObjModel;


// public
ObjModelInfo::Ptr ObjModelInfo::create( ObjModel::Ptr m)
{
    Ptr p = Ptr( new ObjModelInfo(m));
    if ( !p->reset(m))
        p = nullptr;
    return p;
}   // end create


// private
ObjModelInfo::ObjModelInfo( ObjModel::Ptr m)
{
    if ( m->getNumMaterials() > 1)  // Merge materials if > 1 texture map
    {
        std::cerr << "[INFO] RFeatures::ObjModelInfo: Merging multiple materials" << std::endl;
        m->mergeMaterials();
    }   // end if
}   // end ctor


// public
bool ObjModelInfo::reset( ObjModel::Ptr m)
{
    if ( !m)
        m = _model;

    if ( !m)
    {
        std::cerr << "[ERROR] RFeatures::ObjModelInfo::reset: null model passed in!" << std::endl;
        return false;
    }   // end if

    std::cerr << "[INFO] RFeatures::ObjModelInfo::reset: Checking pre-clean model integrity..." << std::endl;
    m->showDebug();
    ObjModelIntegrityChecker ic;
    ic.checkIntegrity( m.get());
    std::cerr << ic << std::endl;
    if ( !ic.is2DManifold())
    {
        std::cerr << "[INFO] RFeatures::ObjModelInfo::reset: Non-triangulated manifold - cleaning..." << std::endl;
        clean(m);
        std::cerr << "[INFO] RFeatures::ObjModelInfo::reset: Checking post-clean model integrity..." << std::endl;
        m->showDebug();
        ic.checkIntegrity( m.get());  // Re-check model integrity
        std::cerr << ic << std::endl;
    }   // end if

    bool success = false;
    if ( !ic.is2DManifold())
    {
        std::cerr << "[ERROR] RFeatures::ObjModelInfo::reset: Clean model failed on creation of ObjModelInfo!" << std::endl;
        std::cerr << ic << std::endl;
    }   // end if
    else
    {
        // Set the model and the integrity checker
        _ic = ic;
        _model = m;
        rebuildInfo();   // Re-build from already clean model
        success = true;
    }   // end else

    return success;
}   // end reset


// public
const ObjModelComponentFinder& ObjModelInfo::components() const { return *_cf.get();}
const ObjModelBoundaryFinder& ObjModelInfo::boundaries() const { return *_bf.get();}


// private
void ObjModelInfo::clean( ObjModel::Ptr m)
{
    std::cerr << "[INFO] RFeatures::ObjModelInfo::clean: Cleaning model." << std::endl;
    // Clean the model
    ObjModelCleaner omc( m);
    const int rem3d = omc.remove3D();
    const int rem1d = omc.remove1D();
    if ( rem3d > 0 || rem1d > 0)
    {
        std::cerr << "[INFO] RFeatures::ObjModelInfo::clean: Removed "
                  << rem3d << " 3D and " << rem1d << " 1D vertices" << std::endl;
    }   // end if
    int totRemTV = 0;
    int remTV = 0;
    do
    {
        remTV = ObjModelTetrahedronReplacer( m).removeTetrahedrons();
        totRemTV += remTV;
    } while ( remTV > 0);

    if ( totRemTV > 0)
    {
        std::cerr << "[INFO] RFeatures::ObjModelInfo::clean: Removed/replaced "
                  << totRemTV << " tetrahedron peaks" << std::endl;
    }   // end if
}   // end clean


// public
void ObjModelInfo::rebuildInfo()
{
    _bf = ObjModelBoundaryFinder::create( _model.get());
    const int nb = (int)_bf->findOrderedBoundaryVertices( _ic.flatEdges());
    _cf = ObjModelComponentFinder::create( _bf);
    const int nc = (int)_cf->findComponents();

    std::cerr << "[INFO] RFeatures::ObjModelInfo::rebuildInfo: Found " << nc << " components and " << nb << " boundaries" << std::endl;
    for ( int i = 0; i < nc; ++i)
    {
        const IntSet* cbs = _cf->cboundaries(i);
        std::cerr << "[INFO] RFeatures::ObjModelInfo::rebuildInfo: Component " << i;
        if ( cbs)
        {
            std::cerr << " has " << cbs->size() << " boundaries:" << std::endl;
            for ( int b : *cbs)
            {
                const std::list<int>& blist = _bf->boundary(b);
                std::cerr << "  Boundary (" << b << ") has " << blist.size() << " vertices with endpoints "
                          << *blist.begin() << ", " << *blist.rbegin() << std::endl;
            }   // end for
        }   // end if
        else
            std::cerr << " has no bounding edge." << std::endl;
    }   // end for
}   // end rebuildInfo
