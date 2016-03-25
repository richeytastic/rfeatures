#include "LASReader.h"
using RFeatures::LASReader;
using RFeatures::LASException;
#include <fstream>

typedef liblas::Color LASColour;


LASReader::LASReader( const string& fname)
    : _fname(fname), _minTime(DBL_MAX), _maxTime(0), _maxX(-DBL_MAX), _minX(DBL_MAX), _maxY(-DBL_MAX),
     _minY(DBL_MAX), _maxZ(-DBL_MAX), _minZ(DBL_MAX), _offset(0,0,0), _scale(0,0,0)
{
}   // end ctor


// private
void LASReader::testPointBounds( const cv::Vec3d& p)
{
    if (p[0] < _minX) _minX = p[0];
    if (p[0] > _maxX) _maxX = p[0];
    if (p[1] < _minY) _minY = p[1];
    if (p[1] > _maxY) _maxY = p[1];
    if (p[2] < _minZ) _minZ = p[2];
    if (p[2] > _maxZ) _maxZ = p[2];
}   // end testPointBounds



cv::Vec3d readPointCoords( const LASPoint& pt, const cv::Vec3d& scale, const cv::Vec3d& offset)
{
    //const cv::Vec3d p = cv::Vec3d( pt.GetX(), pt.GetY(), pt.GetZ()).mul(_scale) + _offset;
    cv::Vec3d p( pt.GetX(), pt.GetY(), pt.GetZ()); // Original LAS point
    p -= offset;   // Ensure points are relative to offset
    p = p.mul(scale);  // Scaled point
    return p;
}   // end readPointCoords


cv::Vec3b readPointColour( const LASPoint& pt)
{
    const LASColour lasColour = pt.GetColor();
    const uint8_t r = int(255 * lasColour.GetRed() / 65536 + 0.5);
    const uint8_t g = int(255 * lasColour.GetGreen() / 65536 + 0.5);
    const uint8_t b = int(255 * lasColour.GetBlue() / 65536 + 0.5);
    return cv::Vec3b(r,g,b);
}   // end readPointColour


void LASReader::readPointTime( const LASPoint& pt)
{
    double pointTime = pt.GetTime();
    if ( pointTime < _minTime)
        _minTime = pointTime;
    if ( pointTime > _maxTime)
        _maxTime = pointTime;
}   // end readPointTime



// private
void LASReader::readPointFormat0( const LASPoint& pt, PointCloud::Ptr pcloud)  // 20 bytes
{
    const cv::Vec3d p = readPointCoords( pt, _scale, _offset);
    pcloud->add( p[0], p[1], p[2], 255, 255, 255);
    testPointBounds(p);
}   // end readPointFormat0


// private
void LASReader::readPointFormat1( const LASPoint& pt, PointCloud::Ptr pcloud)  // 28 bytes (same as version 0 but with time)
{
    readPointTime(pt);
    const cv::Vec3d p = readPointCoords( pt, _scale, _offset);
    pcloud->add( p[0], p[1], p[2], 255, 255, 255);
    testPointBounds(p);
}   // end readPointFormat1


// private
void LASReader::readPointFormat2( const LASPoint& pt, PointCloud::Ptr pcloud)  // 26 bytes (same as version 0 but with r, g, b)
{
    const cv::Vec3d p = readPointCoords( pt, _scale, _offset);
    const cv::Vec3b colour = readPointColour(pt);
    pcloud->add( p[0], p[1], p[2], colour[0], colour[1], colour[2]);
    testPointBounds(p);
}   // end readPointFormat2




// private
void LASReader::readPointFormat3( const LASPoint& pt, PointCloud::Ptr pcloud)  // 34 bytes (same as version 2 but with time)
{
    readPointTime(pt);
    const cv::Vec3d p = readPointCoords( pt, _scale, _offset);
    const cv::Vec3b colour = readPointColour(pt);
    pcloud->add( p[0], p[1], p[2], colour[0], colour[1], colour[2]);
    testPointBounds(p);
}   // end readPointFormat3



PointCloud::Ptr LASReader::read() throw (LASException)
{
    std::ifstream ifs;
    ifs.open( _fname.c_str(), std::ios::in | std::ios::binary);
    if ( !ifs.good())
        return PointCloud::Ptr();

    liblas::ReaderFactory readerFactory;    // ReaderFactory takes into account compressed files
    liblas::Reader reader = readerFactory.CreateWithStream( ifs);
    const liblas::Header& header = reader.GetHeader();
    const liblas::PointFormatName pointFormat = header.GetDataFormatId();

    _offset = cv::Vec3d( header.GetOffsetX(), header.GetOffsetY(), header.GetOffsetZ());
    _scale = cv::Vec3d( header.GetScaleX(), header.GetScaleY(), header.GetScaleZ());

    PointCloud::Ptr pcloud = PointCloud::create();  // Unstructured!

    while ( reader.ReadNextPoint())
    {
        const LASPoint &pt = reader.GetPoint();

        switch ( pointFormat)
        {
            case liblas::ePointFormat0:
                readPointFormat0( pt, pcloud);
                break;
            case liblas::ePointFormat1:
                readPointFormat1( pt, pcloud);
                break;
            case liblas::ePointFormat2:
                readPointFormat2( pt, pcloud);
                break;
            case liblas::ePointFormat3:
                readPointFormat3( pt, pcloud);
                break;
            default:
                throw LASException( "Unknown point format!");
                break;
        }   // end switch
    }   // end while

    ifs.close();
    return pcloud;
}   // end read

