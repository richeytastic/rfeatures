#include <RegionSorter2.h>
using RFeatures::RegionSorter2;
#include <iostream>
#include <cstdlib>
#include <sys/time.h>


RegionSorter2* makeTiny()
{
    RegionSorter2* rs = RegionSorter2::create(4);
    double max = 0;

    max = rs->add( cv::Rect(0, 1, 3, 3), 4);
    std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(2, 0, 2, 3), 6);
    std::cout << "Max value = " << max << std::endl;

    return rs;
}   // end makeTiny



RegionSorter2* makeSmall()
{
    RegionSorter2* rs = RegionSorter2::create(8);
    double max = 0;

    //max = rs->add( 0, 0, 3, 3, 2);
    //std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(2, 1, 2, 2), 5);
    std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(1, 2, 3, 2), 3);
    std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(2, 0, 2, 2), 2);
    std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(0, 1, 2, 3), 1);
    std::cout << "Max value = " << max << std::endl;

    return rs;
}   // end makeSmall



RegionSorter2* makeLarge()
{
    RegionSorter2* rs = RegionSorter2::create(32);
    double max = 0;

    max = rs->add( cv::Rect(10, 5, 15, 15), 3);
    std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(21, 5, 11, 7), 4);
    std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(5, 10, 15, 20), 4);
    std::cout << "Max value = " << max << std::endl;

    max = rs->add( cv::Rect(15, 15, 15, 10), 7);
    std::cout << "Max value = " << max << std::endl;

    //max = rs->add( 14, 2, 9, 15, 2);
    max = rs->add( cv::Rect(15, 15, 5, 2), 2);
    std::cout << "Max value = " << max << std::endl;

    return rs;
}   // end makeLarge



RegionSorter2* makeHuge()
{
    RegionSorter2* rs = RegionSorter2::create(512);
    double max = 0;

    const uint height = 30;
    const uint width = 40;

    const uint rows = 512 - height;
    const uint cols = 512 - width;

    srand(1);
    for ( int y = 0; y < rows; y += 7)
    {
        for ( int x = 0; x < cols; x += 7)
        {
            double v = (double)rand() / RAND_MAX * 10;
            max = rs->add( cv::Rect(x, y, width, height), v);
        }   // end for
    }   // end for

    return rs;
}   // end makeHuge



int main()
{
    RegionSorter2* rs = makeTiny();

    /*
    int count = 0;
    timeval st, et;
    double tmsecs = 0;

    for ( int i = 0; i < 4; ++i)
    {
        list<RegionSorter::Rect> region;
        gettimeofday( &st, NULL);
        double fmax = rs->removeMax( region);
        gettimeofday( &et, NULL);
        tmsecs += ((et.tv_sec - st.tv_sec) * 1000000 + ( et.tv_usec - st.tv_usec)) / 1000;

        count++;
        std::cout << "Max = " << fmax << ", over:" << std::endl;
        for ( list<RegionSorter::Rect>::const_iterator it = region.begin(); it != region.end(); ++it)
        {
            RegionSorter::Rect r = *it;
            std::cout << "X:" << r.x << ", Y:" << r.y << ", W:" << r.width << ", H:" << r.height << std::endl;
        }   // end for
    }   // end for

    std::cout << "Got " << count << " max value locations in " << tmsecs << " msecs" << std::endl;

    */
    cv::Rect r;
    double val = rs->findMax( r);
    std::cout << "Value = " << val << " at X:" << r.x << ", Y:" << r.y << ", W:" << r.width << ", H:" << r.height << std::endl;

    delete rs;
    return 0;
}   // end main
