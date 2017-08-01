#include "ObjModelMeshTraversalRecorder.h"
using RFeatures::ObjModelMeshTraversalRecorder;

// public
void ObjModelMeshTraversalRecorder::reset()
{
    _uvidxs.clear();
}   // end reset

// protected
void ObjModelMeshTraversalRecorder::parseTriangle( int f, int r, int a, int b)
{
    _uvidxs.insert(r);
    _uvidxs.insert(a);
    _uvidxs.insert(b);
}   // end parseTriangle
