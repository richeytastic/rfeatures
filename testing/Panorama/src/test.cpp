#include <Panorama.h>
using RFeatures::Panorama;
#include <cstdlib>
#include <exception>
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <fstream>



void saveView( const View::Ptr v, string panoFileName, string sface, float maxRange)
{
    string panoDir = panoFileName.substr(0,panoFileName.size() - 5);    // Remove extension
    panoDir = panoDir.substr( panoDir.find_last_of('/') + 1);   // Create dir from filename only

    system( ("mkdir -p " + panoDir).c_str());
    std::ostringstream coss;
    coss << panoDir << "/" << sface << "_colour.png";
    std::ostringstream doss;
    doss << panoDir << "/" << sface << "_depth.png";
    cv::imwrite( coss.str(), v->img2d);
    const cv::Mat_<byte> dimg = RFeatures::makeDisplayableRangeMap( v->rngImg, 0, maxRange);
    cv::imwrite( doss.str(), dimg);
}   // end saveView



Panorama::Ptr loadPano( const string fname)
{
    Panorama::Ptr pano;

    std::ifstream ifs;
    try
    {
        ifs.open(fname.c_str());
        pano = Panorama::Ptr( new Panorama());
        ifs >> *pano;
    }   // end try
    catch ( const std::exception& e)
    {
        cerr << "Unable to load panorama!" << endl;
        cerr << e.what() << endl;
    }   // end catch

    if ( ifs.is_open())
        ifs.close();

    return pano;
}   // end loadPano



int processPano( const Panorama::Ptr pano, string panoFileName, float maxRange)
{
    int retCode = EXIT_SUCCESS;
    if ( pano == NULL)
    {
        cerr << "NULL panorama supplied to processPano()" << endl;
        retCode == EXIT_FAILURE;
    }   // end if
    else
    {
        const View::Ptr v0 = pano->getFrontView();
        const View::Ptr v1 = pano->getLeftView();
        const View::Ptr v2 = pano->getRearView();
        const View::Ptr v3 = pano->getRightView();
        saveView( v0, panoFileName, "FRONT", maxRange);
        saveView( v1, panoFileName, "LEFT", maxRange);
        saveView( v2, panoFileName, "REAR", maxRange);
        saveView( v3, panoFileName, "RIGHT", maxRange);
    }   // end if

    return retCode;
}   // end processPano



int main( int argc, char** argv)
{
    int exitCode = EXIT_SUCCESS;

    if ( argc < 3)
    {
        cerr << "Supply panorama filename and max depth to display range at." << endl;
        exitCode = EXIT_FAILURE;
    }   // end if
    else
    {
        const Panorama::Ptr pano = loadPano( argv[1]);
        float maxRange = strtof( argv[2], 0);
        exitCode = processPano(pano, argv[1], maxRange);
    }   // end else

    return exitCode;
}   // end main

