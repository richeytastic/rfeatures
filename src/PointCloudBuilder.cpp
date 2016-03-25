#include "PointCloudBuilder.h"
using RFeatures::PointCloudBuilder;
#include <cassert>


PointCloudBuilder::Ptr PointCloudBuilder::create()
{
    PointCloudBuilder *pcb = new PointCloudBuilder;
    return PointCloudBuilder::Ptr( pcb);
}   // end create


PointCloudBuilder::PointCloudBuilder() : m_dataCount(0)
{
}   // end ctor



void PointCloudBuilder::reset( int width, int height)
{
    m_pc = PointCloud::create( width, height);
    m_dataCount = 0;
}   // end reset



void PointCloudBuilder::setPointPos( int row, int col, double x, double y, double z)
{
    m_row1 = row;
    m_col1 = col;
    m_x = x;
    m_y = y;
    m_z = z;
    m_dataCount++;
    if ( m_dataCount == 2)    // Add the point if we've received x,y,z and r,g,b
        addPoint();
}   // end setPointRange



void PointCloudBuilder::setPointCol( int row, int col, byte r, byte g, byte b)
{
    m_row2 = row;
    m_col2 = col;
    m_r = r;
    m_g = g;
    m_b = b;
    m_dataCount++;
    if ( m_dataCount == 2)    // Add the point if we've received x,y,z and r,g,b
        addPoint();
}   // end setPointCol



void PointCloudBuilder::addPoint()
{
    assert( m_row1 == m_row2 && m_col1 == m_col2);  // Ensure that x,y,z point corresponds to r,g,b
    m_pc->set( m_row1, m_col1, m_x, m_y, m_z, m_r, m_g, m_b);
    m_dataCount = 0;  // Ready for next point
}   // end addPoint
