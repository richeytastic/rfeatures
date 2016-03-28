#include <cstdlib>
#include <iostream>
using namespace std;

#include "PointCloud.h"
using namespace rlib;


void printPointCloudDims( const PointCloud::Ptr pc)
{
    cout << "Height = " << pc->rows() << endl;
    cout << "Width = " << pc->cols() << endl;
    cout << "Size = " << pc->size() << endl << endl;
}   // end printPointCloudDims



int main( int argc, char **argv)
{
    PointCloud::Ptr pc( PointCloud::create());

    // Height = 1, Width = Size = 20
    for ( int i = 0; i < 20; ++i)
        pc->add( 10, 11, 12, 240, 230, 20);
    printPointCloudDims( pc);

    pc->resize(3,6);

    // Height = 4, Width = 6, Size = 24
    for ( size_t y = 0; y < 4; ++y)
        for ( size_t x = 0; x < 6; ++x)
            pc->set( y, x, 10, 11, 12, 240, 230, 20);
    printPointCloudDims( pc);

    return EXIT_SUCCESS;
}   // end main
