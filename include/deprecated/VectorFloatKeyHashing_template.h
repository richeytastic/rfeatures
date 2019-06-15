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
    const float m = powf(10.0f,(float)pw);
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
