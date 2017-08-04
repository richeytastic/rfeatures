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

/**
 * Allow arbitrary rectangles to be drawn, moved and resized.
 * Richard Palmer
 * October 2012
 */

#ifndef RFEATURES_RECTANGLE_MANAGER_H
#define RFEATURES_RECTANGLE_MANAGER_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

namespace RFeatures
{

class rFeatures_EXPORT RectangleManager
{
public:
    typedef boost::shared_ptr<RectangleManager> Ptr;
    static Ptr create();
    RectangleManager();

    // Start creating a new rectangle from the given point.
    void createNew( int x, int y);

    // Shifts the rectangle by the desired number of positive or negative pixels
    void shiftVertically( int pixels);
    void shiftHorizontally( int pixels);

    // Resizes the rectangle by the desired number of positive or negative pixels.
    // Vertical resizing always moves the bottom edge of the rectangle, while
    // horizontal resizing always moves the right edge of the rectangle.
    void resizeVertically( int pixels);
    void resizeHorizontally( int pixels);

    // All subsequent calls to update will move the position of the
    // rectangle from the coordinates provided here.
    void move( int x, int y);

    // All subsequent calls to update will resize the rectangle
    // with edges to move defined by the provided coordinates.
    void resize( int x, int y);

    // As resize but only allows resizing in the vertical direction.
    void resizeVert( int x, int y);

    // As resize but only allows resizing in the horizontal direction.
    void resizeHorz( int x, int y);

    void stop();   // Sets current mode to NONE so that calls to update do nothing
    void reset();  // Resets the rectangle position and dimensions

    // Updates the rectangle using the provided coordinates
    // according to the current mode of operation.
    void update( int x, int y);

    // Returns true if rectangle is currently being worked on
    inline bool working() const { return workMode != NONE;}

    // Returns true if rectangle is currently being moved
    inline bool isMoving() const { return workMode == MOVE;}

    // Returns true if rectangle is currently being drawn
    inline bool isDrawing() const { return workMode == DRAW;}

    // Returns true if rectangle is currently being resized
    inline bool isResizing() const { return (workMode & (RESIZE | RESIZE_VERT | RESIZE_HORZ)) > 0;}

    // Returns true iff the provided coordinates intersect with the rectangle.
    bool intersects( int x, int y) const;

    // Returns true iff the provided coordinates intersect with one of the
    // corners of the rectangle within tol pixels.
    bool onCorner( int x, int y, int tol=0) const;

    // Returns true iff the provided coordinates intersects with either
    // the top or the bottom edge within tol pixels.
    bool onHorizontalEdge( int x, int y, int tol=0) const;

    // Returns true iff the provided coordinates intersects with either
    // the left or the right edge within tol pixels.
    bool onVerticalEdge( int x, int y, int tol=0) const;

    // Check for intersection with individual edges.
    bool onLeftEdge( int x, int y, int tol=0) const;
    bool onRightEdge( int x, int y, int tol=0) const;
    bool onTopEdge( int x, int y, int tol=0) const;
    bool onBottomEdge( int x, int y, int tol=0) const;

    // Get the drawn rectangle
    inline cv::Rect getRectangle() const { return rect;}

    // Return an extract from the provided image given by the drawn rectangle.
    cv::Mat getExtract( const cv::Mat &img) const;

private:
    // Modes of operation for working with the rectangle:
    enum WorkMode
    {
        NONE = 0,           // Do nothing to the current rectangle
        MOVE = 1,           // Moving fixed dimension rectangle to new position
        DRAW = 2,           // Drawing the rectangle dimensions at fixed position
        RESIZE = 4,         // Resizing both vertically and horizontally
        RESIZE_VERT = 8,    // Resizing vertically
        RESIZE_HORZ = 16,    // Resizing horizontally
    };  // end enum

    cv::Rect rect; // Rectangle being drawn by user.
    cv::Point2i anchor;  // Record x,y coord from previous call to update
    bool rsTop, rsLeft; // Sides to resize (top OR bottom, left OR right)
    WorkMode workMode;  // Mode of rectangle draw operation

    void resizeVert(int y);
    void resizeHorz(int x);

    bool withinHeight(int y, int tol) const;
    bool withinWidth(int x, int tol) const;
};  // end class

}   // end namespace

#endif
