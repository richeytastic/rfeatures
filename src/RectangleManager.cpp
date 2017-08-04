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

#include "RectangleManager.h"
using RFeatures::RectangleManager;
#include <cmath>
#include <cassert>


RectangleManager::Ptr RectangleManager::create()
{
    return RectangleManager::Ptr( new RectangleManager());
}   // end create


RectangleManager::RectangleManager()
{
    reset();
}   // end ctor


void RectangleManager::createNew( int x, int y)
{
    reset();
    anchor.x = x;
    anchor.y = y;
    workMode = DRAW;
}   // end createNew


void RectangleManager::shiftVertically( int pxls)
{
    move( anchor.x, anchor.y);
    update( anchor.x, anchor.y+pxls);
    stop();
}   // end shiftVertically


void RectangleManager::shiftHorizontally( int pxls)
{
    move( anchor.x, anchor.y);
    update( anchor.x+pxls, anchor.y);
    stop();
}   // end shiftHorizontally


void RectangleManager::resizeVertically( int pxls)
{
    const int newHeight = rect.height + pxls;
    rect.height = newHeight < 1 ? 1 : newHeight;
}   // end resizeVertically


void RectangleManager::resizeHorizontally( int pxls)
{
    const int newWidth = rect.width + pxls;
    rect.width = newWidth < 1 ? 1 : newWidth;
}   // end resizeHorizontally


void RectangleManager::move( int x, int y)
{
    anchor.x = x;
    anchor.y = y;
    rsTop = abs(y - rect.y) < abs(y - rect.y - rect.height);
    rsLeft = abs(x - rect.x) < abs(x - rect.x - rect.width);
    workMode = MOVE;
}   // end move


void RectangleManager::resize( int x, int y)
{
    move( x, y);
    workMode = RESIZE;
}   // end resize


void RectangleManager::resizeVert( int x, int y)
{
    move( x, y);
    workMode = RESIZE_VERT;
}   // end resizeVert


void RectangleManager::resizeHorz( int x, int y)
{
    move( x, y);
    workMode = RESIZE_HORZ;
}   // end resizeHorz


void RectangleManager::stop()
{
    workMode = NONE;
}   // end stop


void RectangleManager::reset()
{
    anchor.x = -1;
    anchor.y = -1;
    rect.x = -1;
    rect.y = -1;
    rect.width = 0;
    rect.height = 0;
    workMode = NONE;
}   // end reset


void RectangleManager::update( int x, int y)
{
    switch ( workMode)
    {
        case MOVE:
            rect.x += x - anchor.x;
            rect.y += y - anchor.y;
            anchor.x = x;
            anchor.y = y;
            break;
        case DRAW:
            rect.height = abs(y - anchor.y);
            rect.y = y;
            if ( y > anchor.y) rect.y = anchor.y;
            rect.width = abs(x - anchor.x);
            rect.x = x;
            if ( x > anchor.x) rect.x = anchor.x;
            break;
        case RESIZE:
            resizeVert(y);
            resizeHorz(x);
            break;
        case RESIZE_VERT:
            resizeVert(y);
            break;
        case RESIZE_HORZ:
            resizeHorz(x);
            break;
        default:    // NONE
            break;
    }   // end switch
}   // end update


bool RectangleManager::intersects( int x, int y) const
{
    return x >= rect.x && x < (rect.x + rect.width) && y >= rect.y && y < (rect.y + rect.height);
}   // end intersects


bool RectangleManager::onCorner( int x, int y, int tol) const
{
    return onHorizontalEdge(x,y,tol) && onVerticalEdge(x,y,tol);
}   // end onCorner


bool RectangleManager::onHorizontalEdge( int x, int y, int tol) const
{
    if ( tol < 0) tol = 0;
    return (abs(y - rect.y) <= tol || abs(y - rect.y - rect.height) <= tol) && withinWidth(x,tol);
}   // end onHorizontalEdge


bool RectangleManager::onVerticalEdge( int x, int y, int tol) const
{
    if ( tol < 0) tol = 0;
    return (abs(x - (rect.x + rect.width)) <= tol || abs(x - rect.x) <= tol) && withinHeight(y,tol);
}   // end onVerticalEdge


bool RectangleManager::onRightEdge( int x, int y, int tol) const
{
    if ( tol < 0) tol = 0;
    return abs(x - (rect.x + rect.width)) <= tol && withinHeight(y,tol);
}   // end onRightEdge


bool RectangleManager::onLeftEdge( int x, int y, int tol) const
{
    if ( tol < 0) tol = 0;
    return abs(x - rect.x) <= tol && withinHeight(y,tol);
}   // end onLeftEdge


bool RectangleManager::onTopEdge( int x, int y, int tol) const
{
    if ( tol < 0) tol = 0;
    return abs(y - rect.y) <= tol && withinWidth(x,tol);
}   // end onTopEdge


bool RectangleManager::onBottomEdge( int x, int y, int tol) const
{
    if ( tol < 0) tol = 0;
    return abs(y - rect.y - rect.height) <= tol && withinWidth(x,tol);
}   // end onBottomEdge


bool RectangleManager::withinHeight( int y, int tol) const
{
    return y >= (rect.y - tol) && y < (rect.y + rect.height + tol);
}   // end withinHeight


bool RectangleManager::withinWidth( int x, int tol) const
{
    return x >= (rect.x - tol) && x < (rect.x + rect.width + tol);
}   // end withinWidth


cv::Mat RectangleManager::getExtract( const cv::Mat &img) const
{
    cv::Rect rct( 0, 0, img.cols, img.rows);
    rct &= rect;    // Intersection
    return img(rct);
}   // end getExtract


void RectangleManager::resizeVert( int y)
{
    // The edge closest to y will snap to it
    //if ( abs(y - rect.y) < abs(y - rect.y - rect.height))
    if ( rsTop)
    {   // Top edge needs to move
        if ( y >= rect.y + rect.height) // Limit y
            y = rect.y + rect.height - 1;
        rect.height = rect.y + rect.height - y;
        rect.y = y;
    }   // end if
    else
        rect.height = y - rect.y;

    if ( rect.height < 1)
        rect.height = 1;
}   // end resizeVert


void RectangleManager::resizeHorz( int x)
{
    // Edge closest to x snaps to it
    //if ( abs(x - rect.x) < abs(x - rect.x - rect.width))
    if ( rsLeft)
    {   // Left edge needs to move
        if ( x >= rect.x + rect.width)  // Limit x
            x = rect.x + rect.width - 1;
        rect.width = rect.x + rect.width - x;
        rect.x = x;
    }   // end if
    else
        rect.width = x - rect.x;

    if ( rect.width < 1)
        rect.width = 1;
}   // end resizeHorz
