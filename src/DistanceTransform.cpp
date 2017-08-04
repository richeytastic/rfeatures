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

#include "DistanceTransform.h"
using RFeatures::DistanceTransform;

#define INF INT_MAX >> 2
#define SQUARE( x)( (x)*(x))


void calcPhase1Distance( int* inArray, int n, int stepSz)
{
    const int maxn = n * stepSz;
    int d = 0, dsum = INF;  // Large enough!
    int a = -1, k;
    for ( int i = 0; i < maxn; i += stepSz)
    {
        if ( inArray[i] == 0) // FG point found
        {
            if ( a > -1)    // Point to work backwards to writing in distances
                a = (a + i) / 2;

            d = -1; dsum = 0;
            k = i;
            while ( k > a)
            {
                inArray[k] = dsum;
                k -= stepSz;
                d += 2; dsum += d;
            }   // end while

            a = i;  // Last index of FG point
            d = 1; dsum = 1;
        }   // end if
        else
        {
            inArray[i] = dsum;
            d += 2; dsum += d;
        }   // end else
    }   // end for
}   // end calcPhase1Distance



struct Point
{
    static int MINi;

    int idx;    // Initial origin of point (index in array)
    int lxb;    // Left hand exclusive bound of region where this point is closest
    int dp;     // Orthogonally offset (to the array direction) squared Euclidean distance
    int hp;     // Distance of point from array origin (idx*idx + dp) : for fast FCALCQ calculation

    Point() : idx(0), lxb(-1), dp(0), hp(0) {}
    Point( int p, int d=0, int l=MINi) : idx(p), lxb(l), dp(d), hp(p*p + d) {}

    void update( int i, int d, int l=MINi)
    {
        idx = i;
        lxb = l;
        dp = d;
        hp = i*i + d;
    }   // end update

    // Square distance of this point to lateral point i
    int sqdist( int i) const
    {
        return SQUARE(idx-i) + dp;
    }   // end sqdist

    bool isFG() const
    {
        return dp <= 1;
    }   // end ifFG
};  // end struct


int Point::MINi = -1;


// Update from R2L, calculating the squared Euclidean distances for points not already having values <= 1 (i.e. FG points).
// Has upper bound complexity O(n), but works faster the more FG points are present up to a minimum of O(1) for a single entry
// on the npStack specifying the left most FG point of a full row of FG points (i.e. the point at position 0). In that case, j
// is immediately set to -1, the inner while loop is avoided entirely, and the outer loop ends after the first and only iteration.
void updatePointsR2L( const Point* const* npStack, int np, int* f, int j)
{
    int sedsum, sedadd, dx, LXB;
    const Point* tp;

    for ( int i = np-1; i >= 0; --i)
    {
        tp = npStack[i];
        dx = tp->idx;
        LXB = tp->lxb;

        sedsum = tp->dp + SQUARE(j - dx);
        sedadd = 2*(j - dx)-1;

        dx = MAX(LXB,dx);
        while ( j > dx)
        {
            f[j--] = sedsum;
            sedsum -= sedadd;
            sedadd -= 2;
        }   // end while

        // Skip past skip LXBs for FG points
        if ( tp->isFG() && i > 0 && npStack[i-1]->isFG() && LXB < npStack[i-1]->idx)
            j = LXB;
        else
        {
            while ( j > LXB)
            {
                f[j--] = sedsum;
                sedsum -= sedadd;
                sedadd -= 2;
            }   // end while
        }   // end else
    }   // end for
}   // end updatePointsR2L


// Requires hi = i*i + di*di
// Always ensure i > j
#define FCALCQ( i, hi, j, hj)( ((hi) < (hj)) ? -1 : ((hi) - (hj)) / (2*((i) - (j))))


// Find where in the npStack, qq should actually sit.
void updateNpStack( const int* f, int MAXj, Point** npStack, int& pidx, int& plast, Point* qq)
{
    Point* pp = npStack[plast];

    // If p is a skip point for a row with the same points, then we also need to check
    // the split point calculated as the base of the triangle from qq->idx with
    // hypotenuse = pp->dp, and adjacent side = qq->dp. Only if the extent of this
    // base is beyond (to the left) of pp->idx do we pop p off the stack.
    int t = FCALCQ( qq->idx, qq->hp, pp->idx, pp->hp);
    while ( t <= pp->lxb && plast > 0)
    {
        pp = npStack[--plast];
        t = FCALCQ( qq->idx, qq->hp, pp->idx, pp->hp);
    }   // end while

    if ( t < 0) // plast == 0
    {
        npStack[0] = qq;
        qq->lxb = Point::MINi;
        pidx = 0;
    }   // end if
    // Ignore qq if it's only closest to points off the end of the array, or the first point
    // that's supposed to be closest to qq is actually closest to its own orthogonally located point.
    else if (( t > MAXj) || (t >= qq->idx && f[t+1] <= qq->sqdist(t+1)))
        return;
    else
    {
        qq->lxb = t;
        npStack[++plast] = qq;
        if ( plast < pidx)  // Test when this occurs
            pidx = plast;
    }   // end else
}   // end updateNpStack



int ignoreFromLeft( int* f, int n)
{
    int i = 0;
    // Ignore initial INF points from the left
    while (i < n && f[i] >= INF)
        i++;

    if ( i == 0) // Ignore initial FG points from the left.
    {
        while (i < n && f[i] < 2)
            i++;
        Point::MINi = i-1;
        if ( i < n) // Ensure i starts at the first FG point
            i = MAX(0,i-1);
    }   // end if

    return i;
}   // end ignoreFromLeft



void transform2D( cv::Mat_<int>& mat)
{
    const int rows = mat.rows;
    const int n = mat.cols;

    const int TOPN = n-1;

    Point* cpoints = (Point*)cv::fastMalloc( n * sizeof(Point));   // Generated candidate points (not all used in npStack)
    Point** npStack = (Point**)cv::fastMalloc( n * sizeof(Point*));  // Pointers to nearest points
    Point* q = NULL;    // Inclusive top of cpoints (newest point added)
    Point* p = NULL;    // The current nearest point

    int pidx = 0;       // Index into npStack - current nearest point (p + 1 = fLIDX)
    int plast = 0;      // Index into npStack - inclusive upper bound (last point added). plast == p if no fwd points set
    int i, MAXj, di;

    int* f = mat.ptr<int>();
    for ( int row = 0; row < rows; ++row, f+= n)
    {
        q = NULL, p = NULL;
        pidx = 0, plast = 0;
        i = ignoreFromLeft( f, n);
        if ( i < n) // Create the first closest point which is either a FG point or a regular point
        {
            q = cpoints;
            q->update(i, f[i]);
            npStack[0] = q;
            i++;
        }   // end if

        MAXj = TOPN;   // Max inclusive index for updates using R2L function
        while (MAXj > i && f[MAXj] <= 1)    // Ignore initial FG points from the right.
            MAXj--;
        MAXj = MIN(TOPN, MAXj+1);

        for ( ; i <= MAXj; ++i)
        {
            di = f[i];

            if ( plast > pidx && i > npStack[pidx+1]->lxb)
                pidx++;

            if ( di >= INF)
                continue;

            q++;
            q->update( i, di);    // Get the next candidate point at entry i
            p = npStack[pidx];    // Ensure p is up to date

            // Deal with FG regions (iterate over their extent and leave them be), recording just the endpoints for the R2L update.
            if ( di < 2)
            {
                if ( f[i-1] > 1)
                { // Left endpoint - reset the forward points list and set fg as the currently closest point.
                    plast = pidx;
                    updateNpStack( f, MAXj, npStack, pidx, plast, q); // Find the right LXB for pidx
                }   // end if
                else if ( f[i+1] > 1)
                { // Right endpoint
                    // Make a new point that gives a LXB that skips back to the index of the last given point p
                    // and resets the forward points list.
                    q->lxb = p->idx-1;
                    pidx = ++plast;
                    npStack[pidx] = q;
                }   // end if
                continue;
            }   // end if

            updateNpStack( f, MAXj, npStack, pidx, plast, q);
        }   // end for

        if ( q == NULL)
            continue;

        if ( f[MAXj] <= 1)
            MAXj--;
        updatePointsR2L( npStack, plast+1, f, MAXj);
    }   // end for - rows

    cv::fastFree(cpoints);
    cv::fastFree(npStack);
}   // end transform2D



void DistanceTransform::inplace( cv::Mat_<int>& im)
{
    const int nrows = im.rows;
    const int ncols = im.cols;
    int* fcol = im.ptr<int>();
    for ( int i = 0; i < ncols; ++i, ++fcol)
        calcPhase1Distance( fcol, nrows, ncols);

    transform2D(im);
}   // end inplace
