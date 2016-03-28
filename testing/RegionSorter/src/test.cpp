#include <RegionSorter.h>
using RFeatures::RegionSorter;
#include <iostream>
#include <cstdlib>
#include <sys/time.h>


RegionSorter* makeTiny()
{
    RegionSorter* rs = new RegionSorter(4);
    double* max;

    max = rs->add( cv::Rect(0, 1, 3, 3), 4);
    std::cout << "Max value now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(2, 0, 2, 3), 6);
    std::cout << "Max value now = " << max[1] << std::endl;

    return rs;
}   // end makeTiny



RegionSorter* makeSmall()
{
    RegionSorter* rs = new RegionSorter(8);
    double* max;

    //max = rs->add( 0, 0, 3, 3, 2);
    //std::cout << "Max value now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(2, 1, 2, 2), 5);
    std::cout << "Max value now now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(1, 2, 3, 2), 3);
    std::cout << "Max value now now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(2, 0, 2, 2), 2);
    std::cout << "Max value now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(0, 1, 2, 3), 1);
    std::cout << "Max value now = " << max[1] << std::endl;

    return rs;
}   // end makeSmall



RegionSorter* makeLarge()
{
    RegionSorter* rs = new RegionSorter(32);
    double* max;

    max = rs->add( cv::Rect(10, 5, 15, 15), 3);
    std::cout << "Max value now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(21, 5, 11, 7), 4);
    std::cout << "Max value now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(5, 10, 15, 20), 4);
    std::cout << "Max value now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(15, 15, 15, 10), 7);
    std::cout << "Max value now = " << max[1] << std::endl;

    max = rs->add( cv::Rect(14, 2, 9, 15), 2);
    //max = rs->add( cv::Rect(15, 15, 5, 2), 2);
    std::cout << "Max value now = " << max[1] << std::endl;

    return rs;
}   // end makeLarge



RegionSorter* makeHuge()
{
    RegionSorter* rs = new RegionSorter(512);
    double* max;

    const uint height = 30;
    const uint width = 40;

    const uint rows = 512 - height;
    const uint cols = 512 - width;

    cv::Rect r(0,0,width,height);
    srand(1);
    for ( int y = 0; y < rows; y += 5)
    {
        r.y = y;
        for ( int x = 0; x < cols; x += 5)
        {
            r.x = x;
            double v = (double)rand() / RAND_MAX * 10;
            max = rs->add(r, v);
        }   // end for
    }   // end for

    return rs;
}   // end makeHuge



void doTest()
{
    RegionSorter* rs = makeLarge();

    timeval st, et;
    double tusecs = 0;
    gettimeofday( &st, NULL);

    for ( int i = 0; i < 100; ++i)
    {
        list<cv::Rect> rects;
        double fmax = rs->removeMax( rects);
        for ( list<cv::Rect>::const_iterator it = rects.begin(); it != rects.end(); ++it)
        {
            const cv::Rect& rct = *it;
            std::cout << "Found " << fmax
                      << " at X:" << rct.x << ", Y:" << rct.y
                      << ", W:" << rct.width << ", H:" << rct.height << std::endl;
        }   // end for
    }   // end for

    gettimeofday( &et, NULL);
    tusecs = ((et.tv_sec - st.tv_sec) * 1000000 + ( et.tv_usec - st.tv_usec));
    std::cout << "Got max value locations in " << tusecs/1000 << " msecs" << std::endl;

    delete rs;
}   // end doTest



int main()
{
    for ( int i = 0; i < 10; ++i)
        doTest();
    return EXIT_SUCCESS;
}   // end main
