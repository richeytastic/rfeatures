#include "RegionSorter.h"
using RFeatures::RegionSorter;
#include <iostream>
#include <cmath>
#include <cassert>
#include <cfloat>
#include <algorithm>


namespace
{

/*
void makeDensityRectUnion( cv::Rect& rct, double& density, const cv::Rect& r, double dens)
{
    if ( rct.area() == 0)
    {
        rct = r;
        density = dens;
        return;
    }   // end if

    double as = density*rct.area() + dens*r.area() - ((density+dens)/2)*(rct&r).area();
    rct |= r;   // Make smallest fitting rectangle that contains rect and r
    density = as / rct.area();
}   // end makeDensityRectUnion
*/


// Returns true iff r1 and r2 are vertically, horizontally or diagonally
// adjacent (i.e. corners touch) or they intersect.
bool areAdjacentOrIntersect( const cv::Rect& r1, const cv::Rect& r2)
{
    if ( r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0)
        return false;
    const bool hAdj = ((r1.x + r1.width >= r2.x) && (r2.x >= r1.x)) || ((r2.x + r2.width >= r1.x) && (r1.x >= r2.x));
    const bool vAdj = ((r1.y + r1.height >= r2.y) && (r2.y >= r1.y)) || ((r2.y + r2.height >= r1.y) && (r1.y >= r2.y));
    return hAdj && vAdj;
}   // end areAdjacentOrIntersect



struct Node
{
    Node( uint x, uint y, uint d) : rect_(x, y, d, d), hdim_(d/2), yhd_(y+hdim_), xhd_(x+hdim_),
        val_(0), min_(0), max_(0), q1_(NULL), q2_(NULL), q3_(NULL), q4_(NULL)
    {
        assert( d > 0);
        //nodeId_ = nodeCount++;    // Keep track of number of nodes constructed
    }   // end ctor


    ~Node() // Recursively delete child nodes
    {
        if ( q1_ != NULL)
            delete q1_;
        if ( q2_ != NULL)
            delete q2_;
        if ( q3_ != NULL)
            delete q3_;
        if ( q4_ != NULL)
            delete q4_;
    }   // end dtor


    void reorderChildNodes()
    {
        topList_.clear();

        // Quadrant 1
        if ( q1_ != NULL)
            topList_.push_front(q1_);

        // Quadrant 4 
        if ( q4_ != NULL)
        {
            if ( topList_.empty() || q4_->max_ > (*topList_.begin())->max_)
                topList_.push_front(q4_);
            else
                topList_.push_back(q4_);
        }   // end if

        // Quadrant 2
        if ( q2_ != NULL)
        {
            list<Node*>::iterator it = topList_.begin();
            if ( topList_.empty() || q2_->max_ > (*it)->max_)
                topList_.push_front(q2_); // At front
            else if ( q2_->max_ <= (*topList_.rbegin())->max_)
                topList_.push_back(q2_);  // At back
            else    // In middle position
                topList_.insert( (++it), q2_);
        }   // end if

        // Quadrant 3
        if ( q3_ != NULL)
        {
            list<Node*>::iterator it = topList_.begin();
            if ( topList_.empty() || q3_->max_ > (*it)->max_)
                topList_.push_front(q3_); // At front
            else if ( q3_->max_ <= (*topList_.rbegin())->max_)
                topList_.push_back(q3_);  // At back
            else if ( topList_.size() == 2)  // In middle position
                topList_.insert( (++it), q3_);
            else
            {   // Choose between position 2 and 3 in topList
                ++it;
                if ( q3_->max_ > (*it)->max_)
                    topList_.insert(it,q3_);  // Position 2;
                else
                {
                    ++it;
                    topList_.insert(it,q3_);  // Position 3;
                }   // end else
            }   // end else
        }   // end if
    }   // end reorderChildNodes



    void add( const cv::Rect& r, double v)
    {
        // Just add the value if given area exactly fits this node.
        if ( r == rect_)
            val_ += v;
        else
        {
            // Find the child nodes that contain this rectangle for value v and pass the operation down
            // returning the maximum value than can be achieved from this node by going to a child node.
            // Max values that can be found in child nodes are ordered in topList (reorderChildNodes).
            const uint x1w = r.x + r.width;
            const uint y1h = r.y + r.height;

            uint uh = r.height; // Default height for quadrants 1 and 2
            uint bh = r.height; // Default height for quadrants 3 and 4
            // If rectangle crosses centre line, need to split height
            if ( y1h > yhd_ && r.y < yhd_)
            {
                uh = yhd_ - r.y;
                bh = y1h - yhd_;
            }   // end if

            uint by = r.y;
            if ( yhd_ > r.y)
                by = yhd_;

            if ( x1w > xhd_)    // Quadrants 1 and 4
            {
                // Width for q1 and q4
                uint nw = r.width;    // Default width
                uint rx = r.x;
                if ( r.x < xhd_)  // Width crosses centre line so only use portion in right two quadrants
                {
                    nw = x1w - xhd_;
                    rx = xhd_;
                }   // end if

                if ( r.y < yhd_)  // Quadrant 1
                {
                    if ( q1_ == NULL)
                        q1_ = new Node( xhd_, rect_.y, hdim_);
                    q1_->add( cv::Rect(rx, r.y, nw, uh), v);
                }   // end if

                if ( y1h > yhd_) // Quadrant 4
                {
                    if ( q4_ == NULL)
                        q4_ = new Node( xhd_, yhd_, hdim_);
                    q4_->add( cv::Rect(rx, by, nw, bh), v);
                }   // end if
            }   // end if

            if ( r.x < xhd_)  // Quadrants 2 and 3
            {
                // Width for q2 and q3
                uint nw = r.width;    // Default width
                if ( x1w > xhd_) // Width crosses centre line so only use portion in left two quadrants
                    nw = xhd_ - r.x;

                if ( r.y < yhd_)  // Quadrant 2
                {
                    if ( q2_ == NULL)
                        q2_ = new Node( rect_.x, rect_.y, hdim_);
                    q2_->add( cv::Rect(r.x, r.y, nw, uh), v);
                }   // end if

                if ( y1h > yhd_) // Quadrant 3
                {
                    if ( q3_ == NULL)
                        q3_ = new Node( rect_.x, yhd_, hdim_);
                    q3_->add( cv::Rect(r.x, by, nw, bh), v);
                }   // end if
            }   // end else

            reorderChildNodes();
        }   // end else

        max_ = min_ = val_;
        if ( !topList_.empty())
        {
            max_ += (*topList_.begin())->max_;
            min_ += (*topList_.rbegin())->min_;
        }   // end if
    }   // end add



    double findMax( list<cv::Rect>& regions) const
    {
        // Recursively descend through the graph to get the maximum valued rectangle.
        if ( topList_.empty())
        {
            regions.push_back( rect_);
            return val_; // == max_
        }   // end if

        list<Node*>::const_iterator it = topList_.begin();
        double m = (*it)->findMax( regions) + val_;

        it++;
        while ( it != topList_.end())
        {
            if ( (*it)->max_ + val_ == m)
            {
                (*it)->findMax( regions);
                it++;
            }   // end if
            else
                break;
        }   // end if

        return max_;
    }   // end findMax



    double findMin( list<cv::Rect>& regions) const
    {
        // Recursively descend through the graph to get the minimum valued rectangle.
        if ( topList_.empty())
        {
            regions.push_back( rect_);
            return val_; // == min_
        }   // end if

        list<Node*>::const_reverse_iterator it = topList_.rbegin();
        double m = (*it)->findMin( regions) + val_;

        it++;
        while ( it != topList_.rend())
        {
            if ( (*it)->min_ + val_ == m)
            {
                (*it)->findMin( regions);
                it++;
            }   // end if
            else
                break;
        }   // end if

        return min_;
    }   // end findMin



    double removeMax( list<cv::Rect>& regions)
    {
        if ( topList_.empty())
            return val_; // == max_

        // Get all the rectangles from the child nodes having the same value
        double cval;
        do
        {
            Node* cn = *topList_.begin();
            cval = cn->removeMax( regions) + val_;

            if ( cval == cn->max_ + val_)
            {
                regions.push_back(cn->rect_);
                topList_.erase(topList_.begin()); // Node cn
            }   // end if
            else if ( topList_.size() > 1)   // Otherwise, reorder the children to ensure max is in first position
            {
                // Current "max" node may need to trickle down the topList
                topList_.erase(topList_.begin());
                list<Node*>::iterator it = topList_.begin();
                while ( it != topList_.end() && (*it)->max_ > cn->max_)
                    it++;
                topList_.insert(it, cn);
            }   // end else

            max_ = val_;
            if ( !topList_.empty())
               max_ += (*topList_.begin())->max_;
        } while ( !topList_.empty() && cval == max_);

        return cval;
    }   // end removeMax


    const cv::Rect rect_;   // Rectangular area represented by this node
    const uint hdim_;       // half dim (dim/2)
    const uint yhd_;        // rect_.y + hdim
    const uint xhd_;        // rect_.x + hdim

    double val_;        // Value of the region represented by this node
    double min_;        // Min value that can be found by following child nodes
    double max_;        // Max value that can be found by following child nodes

    // |2|1|
    // |3|4|
    Node* q1_;
    Node* q2_;
    Node* q3_;
    Node* q4_;

    // The child quadrants ordered from the highest value (front) to lowest (back)
    list<Node*> topList_;

    //uint nodeId_;
    //static uint nodeCount;
};  // end struct


//uint Node::nodeCount = 0;
}   // end namespace



double Log2( double n)
{
    return log(n)/log(2);
}   // end Log2



class RegionSorter::RegionSorter_impl : public RegionSorter
{
public:
    explicit RegionSorter_impl( uint dim) : RegionSorter(0), root(0)
    {
        if ( fabs(Log2(dim) - (int)Log2(dim)) > 0)
        {
            std::cerr << "ERROR: RegionSorter can only work with square images with power of 2 dims!" << std::endl;
            assert(fabs(Log2(dim) - (int)Log2(dim)) == 0.0);
        }   // end if

        root = new Node(0,0,dim);
    }   // end ctor


    ~RegionSorter_impl()
    {
        delete root;
    }   // end dtor


    virtual double* add( const cv::Rect& r, double v)
    {
        if ( r.area() > 0)
            root->add( r, v);
        minMax[0] = root->min_;
        minMax[1] = root->max_;
        return minMax;
    }   // end add


    virtual double findMax( list<cv::Rect>& rects) const
    {
        return root->findMax( rects);
    }   // end findMax


    virtual double findMin( list<cv::Rect>& rects) const
    {
        return root->findMin( rects);
    }   // end findMin


    virtual double removeMax( list<cv::Rect>& rects)
    {
        return root->removeMax( rects);
    }   // end removeMax


private:
    Node* root;
    double minMax[2];   // min and max added so far
};  // end class



RegionSorter::RegionSorter( uint dim)
{
    if ( dim == 0)  // Not allowed and to prevent recursion
        return;
    pimpl_ = boost::shared_ptr<RegionSorter_impl>( new RegionSorter_impl( dim));
}   // end ctor


RegionSorter::~RegionSorter()
{
}   // end dtor


double* RegionSorter::add( const cv::Rect& r, double v)
{
    return pimpl_->add(r,v);
}   // end add


double RegionSorter::findMax( list<cv::Rect>& regions) const
{
    return pimpl_->findMax(regions);
}   // end findMax


double RegionSorter::findMin( list<cv::Rect>& regions) const
{
    return pimpl_->findMin(regions);
}   // end findMin


double RegionSorter::removeMax( list<cv::Rect>& regions)
{
    return pimpl_->removeMax(regions);
}   // end removeMax
