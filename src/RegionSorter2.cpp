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

#include "RegionSorter2.h"
using RFeatures::RegionSorter2;

#include <list>
using std::list;
#include <cmath>
#include <cassert>
#include <iostream>


struct Node
{
    // Dimensions of this rectangle
    const cv::Rect rect;
    const double val;

    Node( const cv::Rect& rct, double v)
        : rect(rct), val(v)
    {}   // end ctor
};  // end struct


// Quadrant of a square area
struct Quad
{
    Quad* parent;
    uint id;    // Placement of quad according to parent 1, 2, 3, 4 (root quad has id = 0 and 0 parent)

    uint x;
    uint y;
    uint dim;
    cv::Rect rect;  // For convenience
    uint hdim;

    Quad* q1;
    Quad* q2;
    Quad* q3;
    Quad* q4;

    list<Node*> nodes;
    list<double> vals;
    double totVal;  // Total value of nodes intersecting with this quadrant
    double max; // Maximum value that can be found by following this quadrant to its children
    list<Quad*> topList;    // Child quads ordered according to max value (front = max, back = min)

    Quad( Quad* p, uint id_, uint x_, uint y_, uint d_)
        : parent(p), id(id_), x(x_), y(y_), dim(d_),
          rect(x,y,dim,dim), hdim(d_/2), q1(0), q2(0), q3(0), q4(0)
    {
        assert( d_ > 0);
        totVal = 0;
        max = 0;
    }   // end ctor

    ~Quad()
    {
        if ( q1 != 0)
            delete q1;
        if ( q2 != 0)
            delete q2;
        if ( q3 != 0)
            delete q3;
        if ( q4 != 0)
            delete q4;
    }   // end dtor


    void orderChildQuads()
    {
        topList.clear();

        // Quadrant 1
        if ( q1 != 0)
            topList.push_front(q1);
  
        // Quadrant 4 
        if ( q4 != 0)
        {
            if ( topList.empty() || q4->max > (*topList.begin())->max)
                topList.push_front(q4);
            else
                topList.push_back(q4);
        }   // end if

        // Quadrant 2
        if ( q2 != 0)
        {
            list<Quad*>::iterator it = topList.begin();
            if ( topList.empty() || q2->max > (*it)->max)
                topList.push_front(q2); // At front
            else if ( q2->max <= (*topList.rbegin())->max)
                topList.push_back(q2);  // At back
            else    // In middle position
                topList.insert( (++it), q2);
        }   // end if

        // Quadrant 3
        if ( q3 != 0)
        {
            list<Quad*>::iterator it = topList.begin();
            if ( topList.empty() || q3->max > (*it)->max)
                topList.push_front(q3); // At front
            else if ( q3->max <= (*topList.rbegin())->max)
                topList.push_back(q3);  // At back
            else if ( topList.size() == 2)  // In middle position
                topList.insert( (++it), q3);
            else
            {   // Choose between position 2 and 3 in topList
                ++it;
                if ( q3->max > (*it)->max)
                    topList.insert(it,q3);  // Position 2;
                else
                {
                    ++it;
                    topList.insert(it,q3);  // Position 3;
                }   // end else
            }   // end else
        }   // end if
    }   // end orderChildQuads


    double addNode( Node* n)
    {
        // If the given node completely overlaps this quad, point to it without involving child quads.
        if ( (n->rect & rect) == rect)
        {
            nodes.push_back(n);
            vals.push_back(n->val);
            totVal += n->val;
        }   // end if
        else
        {
            // Otherwise, find the quadrants that the node intersects with and add them recursively.
            uint xnpw = n->rect.x + n->rect.width;    // Right edge of node's rectangle
            uint ynph = n->rect.y + n->rect.height;    // Bottom edge of node's rectangle

            const uint xphd = x+hdim; // Vertical midline of this quad
            const uint yphd = y+hdim; // Horizontal midline of this quad

            // Quadrant 1 and/or 4
            if ( xnpw > xphd)
            {
                // Quadrant 1
                if ( n->rect.y < yphd)
                {
                    if ( q1 == 0)
                        q1 = new Quad( this, 1, xphd, y, hdim);
                    q1->addNode(n);
                }   // end if

                // Quadrant 4
                if ( ynph > yphd)
                {
                    if ( q4 == 0)
                        q4 = new Quad( this, 4, xphd, yphd, hdim);
                    q4->addNode(n);
                }   // end if
            }   // end if

            // Quadrant 2 and/or 3
            if ( n->rect.x < xphd)
            {
                // Quadrant 2
                if ( n->rect.y < yphd)
                {
                    if ( q2 == 0)
                        q2 = new Quad( this, 2, x, y, hdim);
                    q2->addNode(n);
                }   // end if

                // Quadrant 3
                if ( ynph > yphd)
                {
                    if ( q3 == 0)
                        q3 = new Quad( this, 3, x, yphd, hdim);
                    q3->addNode(n);
                }   // end if
            }   // end if

            orderChildQuads();
        }   // end else

        max = totVal;
        if ( !topList.empty())
            max += (*topList.begin())->max;

        return max;
    }   // end addNode



    double combineNodes( cv::Rect& outRect)
    {
        if ( nodes.empty())
            return 0;

        list<Node*>::iterator it = nodes.begin();

        Node* n = NULL;
        double v = 0;

        if ( outRect.area() == 0)
        {
            n = *it;
            outRect = n->rect;
            v = n->val;
            it++;
        }   // end if

        while ( it != nodes.end())
        {
            n = *it;
            outRect &= n->rect;
            it++;
            v += n->val;
        }   // end while

        return v;
    }   // end combineNodes



    double findMax( cv::Rect& outRect)
    {
        // Get the intersection of the nodes in this quadrant
        if ( topList.empty())
            return combineNodes( outRect);
        double v = (*topList.begin())->findMax( outRect);
        // Combine with the nodes that are set in this quadrant
        return v + combineNodes( outRect);
    }   // end findMax
};  // end Quad



class RegionSorter_impl : public RegionSorter2
{
public:
    RegionSorter_impl( uint d)
    {
        if ( fabs(log2(d) - (int)log2(d)) > 0)
        {
            std::cerr << "ERROR: RegionSorter_impl can only work with square images with power of 2 dimensions!" << std::endl;
            assert( fabs(log2(d) - (int)log2(d)) == 0.0);
        }   // end if

        root = new Quad(0, 0, 0, 0, d);
    }   // end ctor


    ~RegionSorter_impl()
    {
        delete root;
        for ( list<Node*>::iterator it = nodes.begin(); it != nodes.end(); ++it)
            delete *it;
    }   // end dtor


    virtual double add( const cv::Rect& rct, double v)
    {
        Node* n = new Node( rct, v);
        nodes.push_back(n);
        return root->addNode(n);
    }   // end add


    virtual double findMax( cv::Rect& outRect) const
    {
        outRect.width = 0;
        outRect.height = 0;
        double v = root->findMax( outRect);
        return v;
    }   // end findMax


    Quad* root;
    list<Node*> nodes;  // Added nodes
};  // end class



RegionSorter2* RegionSorter2::create( uint dim)
{
    return new RegionSorter_impl(dim);
}   // end create



