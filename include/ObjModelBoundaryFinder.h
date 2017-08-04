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

#ifndef RFEATURES_OBJ_MODEL_BOUNDARY_FINDER_H
#define RFEATURES_OBJ_MODEL_BOUNDARY_FINDER_H

/**
 * Find 2D boundaries on the model with vertices in connected order.
 */
#include "ObjModelTriangleMeshParser.h"
#include <list>

namespace RFeatures
{

class rFeatures_EXPORT ObjModelBoundaryFinder : public ObjModelBoundaryParser
{
public:
    // If bverts is not NULL, it should point to a set of unique vertex IDs
    // that are on some specified boundary. The complete boundary of connected
    // vertices must be specified or mesh parsing will "leek" through.
    // When bverts is NULL, the actual model boundaries will be found.
    ObjModelBoundaryFinder( const ObjModel::Ptr, const std::list<int>* bverts=NULL);
    virtual ~ObjModelBoundaryFinder();

    void reset( const std::list<int>* bverts=NULL);

    size_t getNumBoundaries() const;

    // Sort the discovered boundaries in order of length (default max length first).
    void sortBoundaries( bool maxFirst=true);

    const std::list<int>& getBoundary( int b) const;

protected:
    virtual bool parseEdge( int fid, int v0, int v1);

private:
    const ObjModel::Ptr _model;
    class Boundaries;
    Boundaries *_boundaries;
    boost::unordered_map<int,int> _bverts;  // Each vertex maps to the next in order
};  // end class

}   // end namespace

#endif
