#include <PointCloudTextReader.h>
using RFeatures::PointCloudTextReader;
#include <sstream>
#include <iostream>


PointCloudTextReader::PointCloudTextReader()
{}   // end ctor



void PointCloudTextReader::read( istream &is)
{
    string ln;
    size_t rows, cols;
    std::getline( is, ln);
    std::istringstream ciss(ln);
    ciss >> rows >> cols;

    const int numLines = rows * cols; // Number of lines to read in
    PointCloudReader::createPointCloud(rows,cols);

    size_t row, col;
    double x, y, z;
    byte r, g, b;

    for ( int i = 0; i < numLines; ++i)
    {
        if ( std::getline( is, ln) && !ln.empty())
        {
            std::istringstream iss( ln);
            iss >> row >> col >> x >> y >> z >> r >> g >> b;
            PointCloudReader::setPoint( row, col, x, y, z, r, g, b);
        }   // end if
        else
        {
            std::cerr << "ERROR: Unable to read in point text data from file!" << std::endl;
            is.setstate( std::ios::failbit);
            break;
        }   // end else
    }   // end while
}   // end read
