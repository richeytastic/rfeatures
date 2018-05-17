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
#include <ObjModelIntegrityChecker.h>
#include <ObjModelTetrahedronReplacer.h>
using RFeatures::ObjModelInfo;
using RFeatures::ObjModelIntegrityChecker;
using RFeatures::ObjModelComponentFinder;
using RFeatures::ObjModelBoundaryFinder;
using RFeatures::ObjModelTetrahedronReplacer;
using RFeatures::ObjModelCleaner;
using RFeatures::ObjModel;


ObjModelInfo::Ptr ObjModelInfo::create( ObjModel::Ptr m)
{
    Ptr p = Ptr( new ObjModelInfo(m));
    if ( !p->isClean())
        p->clean();         // Clean the model first (then rebuild info)
    else
        p->rebuildInfo();   // Re-build from already clean model

    if ( !p->isClean())
    {
        std::cerr << "[WARNING] RFeatures::ObjModelInfo::create: Failed to clean model on creation of ObjModelInfo!" << std::endl;
        p = NULL;
    }   // end else
    return p;
}   // end create


// private
ObjModelInfo::ObjModelInfo( ObjModel::Ptr m) : _model(m), _ic(*m.get())
{
    checkIntegrity();
}   // end ctor


// private
bool ObjModelInfo::checkIntegrity()
{
    std::cerr << "[INFO] RFeatures::ObjModelInfo::checkIntegrity:" << std::endl;
    _ic.checkIntegrity();
    std::cerr << _ic << std::endl;
    return isClean();
}   // end checkIntegrity


bool ObjModelInfo::isClean() const { return _ic.is2DManifold();}


bool ObjModelInfo::clean()
{
    std::cerr << "[INFO] RFeatures::ObjModelInfo::clean: Cleaning model." << std::endl;
    // Clean the model
    ObjModelCleaner omc( _model);
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
        remTV = ObjModelTetrahedronReplacer( _model).removeTetrahedrons();
        totRemTV += remTV;
    } while ( remTV > 0);
    if ( totRemTV > 0)
        std::cerr << "[INFO] RFeatures::ObjModelInfo::clean: Removed/replaced " << totRemTV << " tetrahedron peaks" << std::endl;

    _bf = NULL;
    _cf = NULL;
    if ( checkIntegrity())
        rebuildInfo();
    return isClean();
}   // end clean


void ObjModelInfo::rebuildInfo()
{
    _bf = ObjModelBoundaryFinder::create( _model);
    const int nb = (int)_bf->findOrderedBoundaryVertices( _ic.flatEdges());
    _cf = ObjModelComponentFinder::create( _bf);
    const int nc = (int)_cf->findComponents();

    std::cerr << "[INFO] RFeatures::ObjModelInfo::clean: Found " << nc << " model components and " << nb << " boundaries" << std::endl;
    for ( int i = 0; i < nc; ++i)
    {
        const IntSet* cbs = _cf->cboundaries(i);
        std::cerr << "[INFO] RFeatures::ObjModelInfo::clean: Component " << i;
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
            std::cerr << " is a surface without a boundary." << std::endl;
    }   // end for
}   // end rebuildInfo


const ObjModelComponentFinder& ObjModelInfo::components() const { return *_cf.get();}
const ObjModelBoundaryFinder& ObjModelInfo::boundaries() const { return *_bf.get();}
