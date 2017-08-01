#include <ObjModelMeshMaker.h>
#include <ObjModelKDTree.h>
using RFeatures::ObjModel;
using RFeatures::ObjModelMeshMaker;
using RFeatures::ObjModelKDTree;
#include <cassert>


ObjModelMeshMaker::ObjModelMeshMaker( ObjModel::Ptr m) : _model(m)
{}  // end ctor


int ObjModelMeshMaker::operator()()
{
    ObjModel::Ptr model = _model;
    int nfaces = (int)model->getNumFaces();
    if ( nfaces > 0)
        return nfaces;

    if ( model->getNumVertices() < 3)
        return nfaces;

    const ObjModelKDTree::Ptr kdtree = ObjModelKDTree::create(model);

    std::vector<int> vns(2);
    int vi = *model->getVertexIds().begin(); // Starting vertex

    while ( true)
    {
        // Find vj as the nearest vertex to vi
        int nfound = kdtree->findn( model->vtx(vi), vns);
        assert(nfound == 2);
        int vj = vns[0] == vi ? vns[1] : vns[0];

        // Find vk as the nearest vertex to vj that isn't vi

    }   // end while

    return nfaces;
}   // end operator
