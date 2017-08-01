#ifndef RFEATURES_NOCTREE_H
#define RFEATURES_NOCTREE_H

/**
 * Divides a search space into N dimensions for efficient search of an N-dimensional point.
 * Runs in O(N) in worst case and O(logN) in best case.
 * Richard Palmer.
 */
#include <cstring>
#include <cmath>

namespace RFeatures
{
// Usually N=3 for 3 spatial dimension (use N=2 for Quadtree).
// T must be a simple signed intrinsic type (e.g. float, double, int).
template <typename T, int N=3>
class NOctree
{
public:
    explicit NOctree( const T p[N]);
    virtual ~NOctree();

    void insert( const T p[N]);

    // Find the nearest point to p. Returns p if no points in octree.
    const T* findNearest( const T p[N]) const;

    const T* point() const;
    void point( T p[N]) const;          // Copy out this NOctree cell's point.

private:
    T _p[N];
    NOctree *_cells[2<<N];

    NOctree* find( const T p[N]); // Find the NOctree leaf closest to the given point
    NOctree( const NOctree&);
    void operator=( const NOctree&);
};  // end class


#include "template/NOctree_template.h"
}   // end namespace

#endif
