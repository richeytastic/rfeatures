template <typename T, int M>
Key<T,M>::Key( double v[M], int pw)
{
    const double m = pow(10,pw);
    for ( int i = 0; i < M; ++i)
    {
        _ielems[i] = (T)round(v[i]*m);
        //assert( signbit(double(_ielems[i])) == signbit(v[i]));  // Underflow check
    }   // end for
}   // end ctor

template <typename T, int M>
Key<T,M>::Key( float v[M], int pw)
{
    const float m = powf(10,pw);
    for ( int i = 0; i < M; ++i)
    {
        _ielems[i] = (T)roundf(v[i]*m);
        //assert( signbit(float(_ielems[i])) == signbit(v[i]));   // Underflow check
    }   // end for
}   // end ctor

template <typename T, int M>
bool Key<T,M>::operator==( const Key<T,M>& k) const
{
    for ( int i = 0; i < M; ++i)
        if ( _ielems[i] != k._ielems[i])
            return false;
    return true;
}   // end operator==

template <typename T, int M>
std::ostream& operator<<( std::ostream& os, const Key<T,M>& k)
{
    os << k[0];
    for ( int i = 1; i < M; ++i)
        os << " " << k[i];
    return os;
}   // end operator<<
