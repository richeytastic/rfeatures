template <typename T, int N>
NOctree<T,N>::NOctree( const T p[N])
{
    memcpy( _p, p, N*sizeof(T));
    const int M = 0x1 << N;
    for ( int i = 0; i < M; ++i)
        _cells[i] = NULL;
}   // end ctor
    

template <typename T, int N>
NOctree<T,N>::~NOctree()
{
    const int M = 0x1 << N;
    for ( int i = 0; i < M; ++i)
    {
        if ( _cells[i])
            delete _cells[i];
    }   // end for
}   // end dtor


template <typename T, int N>
const T* NOctree<T,N>::point() const
{
    return _p;
}   // end point


template <typename T, int N>
void NOctree<T,N>::point( T p[N]) const
{
    memcpy( p, _p, N*sizeof(T));
}   // end point


// Get the cell index for p given point q;
template <typename T, int N>
int getCellIndex( const T *q, const T *p)
{
    int index = 0x0;
    int bmask = 0x1 << N;
    for ( int i = 0; i < N; ++i)
    {
        bmask >>= 1;
        if ( p[i] > q[i])
            index |= bmask;
    }   // end for
    return index;
}   // end getCellIndex


template <typename T, int N>
double sqDistance( const T *p, const T *q)
{
    double d = 0.0;
    for ( int i = 0; i < N; ++i)
        d += pow( p[i] - q[i], 2);
    return d;
}   // end sqDistance


template <typename T, int N>
const T* NOctree<T,N>::findNearest( const T p[N]) const
{
    const NOctree<T,N> *node = const_cast<NOctree<T,N>*>(this)->find( p);
    if ( !node)
        return p;
    return node->point();
}   // end findNearest


// private
template <typename T, int N>
NOctree<T,N>* NOctree<T,N>::find( const T p[N])
{
    //std::cerr << "Looking for (" << p[0] << "," << p[1] << ") in (" << _p[0] << "," << _p[1] << ") @ " << this << std::endl;
    const int i = getCellIndex<T,N>( _p, p);
    if ( _cells[i] == NULL) // Point in discovered direction doesn't exist so return this cell.
        return this;

    // If the search point is closest to the current cell, return this Octree cell.
    if ( sqDistance<T,N>( _p, p) < sqDistance<T,N>( _cells[i]->_p, p))
        return this;

    return _cells[i]->find( p);
}   // end find


template <typename T, int N>
void NOctree<T,N>::insert( const T p[N])
{
    NOctree<T,N> *octree = find( p);
    const int i = getCellIndex<T,N>( octree->_p, p);

    NOctree<T,N>* tmp = octree->_cells[i];
    octree->_cells[i] = new NOctree( p);
    if ( tmp != NULL)
    {
        const int j = getCellIndex<T,N>( p, tmp->_p);
        octree->_cells[i]->_cells[j] = tmp;
    }   // end else
}   // end insert


