#include "PointCloud_impl.h"
using RFeatures::PointCloud_impl;
#include <sstream>


size_t PointCloud_impl::getWidth() const
{
    return _width;
}   // end getWidth


size_t PointCloud_impl::getHeight() const
{
    return _height;
}   // end getHeight


bool PointCloud_impl::isOrganised() const
{
    return _isOrganised;
}   // end isOrganised


void PointCloud_impl::resize( size_t rows, size_t cols)
{
    if ( rows < 1)
        rows = 1;
    _height = rows;
    _width = cols;
    _pdata.resize( rows * cols);
}   // end resize


size_t PointCloud_impl::resize( size_t rows)
{
    if ( rows < 1)
        rows = 1;
    size_t cols = size() / rows;
    if ( cols * rows < size())
        rows++;

    this->resize( rows, cols);
    return rows;
}   // end resize


size_t PointCloud_impl::size() const
{
    return _pdata.size();
}   // end size


void PointCloud_impl::add( double x, double y, double z, byte r, byte g, byte b) throw (PointCloudException)
{
    if ( _isOrganised)
        throw PointCloudException( "Cannot use PointCloud::add method for organised point clouds!");
    PointXYZRGB pt;
    PointCloud::packPoint( pt, x, y, z, r, g, b);
    _pdata.push_back( pt);
}   // end add


void PointCloud_impl::set( size_t row, size_t col, double x, double y, double z, byte r, byte g, byte b) throw (PointCloudException)
{
    if ( !_isOrganised)
        throw PointCloudException( "Cannot use PointCloud::set method for unorganised point clouds!");
    checkIndices( "set", row, col);
    size_t pos = _width * row + col;
    PointCloud::packPoint( _pdata.at(pos), x, y, z, r, g, b);
}   // end set


const PointXYZRGB& PointCloud_impl::at( size_t row, size_t col) const throw (PointCloudException)
{
    checkIndices( "at (const)", row, col);
    size_t pos = _width * row + col;
    return _pdata.at(pos);
}   // end at


PointXYZRGB& PointCloud_impl::at( size_t row, size_t col) throw (PointCloudException)
{
    checkIndices( "at", row, col);
    size_t pos = _width * row + col;
    return _pdata.at(pos);
}   // end at



const PointXYZRGB& PointCloud_impl::at( size_t idx) const throw (PointCloudException)
{
    if ( idx >= _pdata.size())
    {
        std::ostringstream oss;
        oss << "Index " << idx << " for PointCloud::from is out of bounds; ";
        oss << "index must be < " << _pdata.size();
        throw PointCloudException( oss.str());
    }   // end if

    return _pdata.at(idx);
}   // end at


PointXYZRGB& PointCloud_impl::at( size_t idx) throw (PointCloudException)
{
    if ( idx >= _pdata.size())
    {
        std::ostringstream oss;
        oss << "Index " << idx << " for PointCloud::from is out of bounds; ";
        oss << "index must be < " << _pdata.size();
        throw PointCloudException( oss.str());
    }   // end if

    return _pdata.at(idx);
}   // end at


void PointCloud_impl::from( size_t row, size_t col, double &x, double &y, double &z, byte &r, byte &g, byte &b) const
                    throw (PointCloudException)
{
    if ( !_isOrganised)
        throw PointCloudException( "Cannot use PointCloud::from (organised) method for unorganised point clouds!");
    checkIndices( "from (organised)", row, col);
    size_t pos = _width * row + col;
    PointCloud::unpackPoint( _pdata.at(pos), x, y, z, r, g, b);
}   // end from


void PointCloud_impl::from( size_t row, size_t col, float &x, float &y, float &z, byte &r, byte &g, byte &b) const
                    throw (PointCloudException)
{
    if ( !_isOrganised)
        throw PointCloudException( "Cannot use PointCloud::from (organised) method for unorganised point clouds!");
    checkIndices( "from (organised)", row, col);
    size_t pos = _width * row + col;
    PointCloud::unpackPoint( _pdata.at(pos), x, y, z, r, g, b);
}   // end from


void PointCloud_impl::from( size_t idx, double &x, double &y, double &z, byte &r, byte &g, byte &b) const
                    throw (PointCloudException)
{
    if ( idx >= _pdata.size())
    {
        std::ostringstream oss;
        oss << "Index " << idx << " for PointCloud::from is out of bounds; ";
        oss << "index must be < " << _pdata.size();
        throw PointCloudException( oss.str());
    }   // end if
    PointCloud::unpackPoint( _pdata.at(idx), x, y, z, r, g, b);
}   // end from


void PointCloud_impl::from( size_t idx, float &x, float &y, float &z, byte &r, byte &g, byte &b) const
                    throw (PointCloudException)
{
    if ( idx >= _pdata.size())
    {
        std::ostringstream oss;
        oss << "Index " << idx << " for PointCloud::from is out of bounds; ";
        oss << "index must be < " << _pdata.size();
        throw PointCloudException( oss.str());
    }   // end if
    PointCloud::unpackPoint( _pdata.at(idx), x, y, z, r, g, b);
}   // end from


void PointCloud_impl::setColour( size_t row, size_t col, byte r, byte g, byte b) throw (PointCloudException)
{
    checkIndices( "setColour", row, col);
    size_t pos = _width * row + col;
    PointXYZRGB &pt = _pdata.at(pos);
    PointCloud::packPoint( pt, pt.x, pt.y, pt.z, r, g, b);
}   // end setColour


//const std::vector<PointXYZRGB>& PointCloud_impl::getRaw() const
//{
//    return _pdata;
//}   // end getRaw


void PointCloud_impl::init( size_t width, size_t height)
{
    //_pcloud = pcl::PointCloud<PointXYZRGB>::Ptr( new pcl::PointCloud<PointXYZRGB>());
    resize( height, width);
}   // end init


PointCloud_impl::PointCloud_impl()
{
    init( 0, 1);
    _isOrganised = false;
}   // end ctor


PointCloud_impl::PointCloud_impl( size_t width, size_t height)
{
    init( width, height);
    _isOrganised = true;
}   // end ctor



// pack r/g/b into rgb
void packRGB( PointXYZRGB &p, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    p.rgb = *reinterpret_cast<float*>(&rgb);
}   // end packRGB



// unpack rgb into r/g/b
void unpackRGB( const PointXYZRGB &p, uint8_t &r, uint8_t &g, uint8_t &b)
{
    const uint32_t rgb = *reinterpret_cast<const int*>(&p.rgb);
    r = (rgb >> 16) & 0x0000ff;
    g = (rgb >> 8)  & 0x0000ff;
    b = (rgb)       & 0x0000ff;
}   // end unpackRGB



ostream& RFeatures::operator<<( ostream &os, const PointCloud::Ptr &pcptr)
{
    // Cast down to this implemented type
    PointCloud_impl* ptr = dynamic_cast<PointCloud_impl*>( pcptr.get());
    /*
    boost::archive::binary_oarchive oa( os);
    oa << *ptr;
    return os;
    */

    os << ptr->_height << " " << ptr->_width << std::endl;
    const std::vector<PointXYZRGB>& pc = ptr->_pdata;

    uint8_t r, g, b;
    const int sz = (int)pc.size();
    for ( int i = 0; i < sz; ++i)
    {
        const PointXYZRGB& pt = pc.at(i);

        // Write the x,y,z
        os.write( (char*)&pt.x, sizeof(float));
        os.write( (char*)&pt.y, sizeof(float));
        os.write( (char*)&pt.z, sizeof(float));

        // Write the colour
        unpackRGB( pt, r, g, b);
        os.write( (char*)&r, sizeof(byte));
        os.write( (char*)&g, sizeof(byte));
        os.write( (char*)&b, sizeof(byte));
    }   // end for

    os << std::endl;
    return os;
}   // end operator<<



istream& RFeatures::operator>>( istream &is, PointCloud::Ptr &pcptr)
{
    if ( pcptr == NULL)
        pcptr = PointCloud::create();

    // Cast down to this implemented type
    PointCloud_impl* ptr = dynamic_cast<PointCloud_impl*>( pcptr.get());
    /*
    boost::archive::binary_iarchive ia( is);
    ia >> *ptr;
    return is;
    */

    std::string ln;
    std::getline( is, ln);
    std::istringstream iss(ln);
    size_t height, width;
    iss >> height >> width;
    ptr->init( width, height);
    ptr->_isOrganised = width > 0;

    std::vector<PointXYZRGB>& pc = ptr->_pdata;

    uint8_t r, g, b;
    const int sz = int(width) * int(height);
    for ( int i = 0; i < sz; ++i)
    {
        PointXYZRGB &p = pc.at(i);

        // Read the x,y,z
        is.read( (char*)&p.x, sizeof(float));
        is.read( (char*)&p.y, sizeof(float));
        is.read( (char*)&p.z, sizeof(float));

        // Read the colour
        is.read( (char*)&r, sizeof(byte));
        is.read( (char*)&g, sizeof(byte));
        is.read( (char*)&b, sizeof(byte));
        packRGB( p, r, g, b);
    }   // end for

    std::getline( is, ln);  // Read end of line
    return is;
}   // end operator>>



void PointCloud_impl::checkIndices( const std::string &mth, size_t row, size_t col) const throw (PointCloudException)
{
    if ( row >= _height || col >= _width)
    {
        std::ostringstream oss;
        oss << "Indices (" << row << ", " << col << ") for PointCloud::" << mth << " are invalid; ";
        oss << "indices must be < (" << _height << ", " << _width << ")";
        throw PointCloudException( oss.str());
    }   // end if
}   // end checkIndices


