#include <DepthSegmenter.h>
using RFeatures::DepthSegmenter;
#include <DataLoader.h>
#include <FeatureUtils.h>
#include <IntensityIndexer.h>
#include <SilhouetteFeature.h>
#include <ProHOG.h>
#include <DescriptorStatistics.h>

#include <string>
using std::string;
#include <boost/foreach.hpp>
#include <vector>
#include <cstdlib>
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <fstream>



cv::Mat_<byte> createDifferenceMap( const cv::Mat_<byte>& timg)
{
    cv::Mat_<byte> outMap( timg.size());
    const int rows = timg.rows;
    const int cols = timg.cols;
    for ( int i = 0; i < rows; ++i)
    {
        for ( int j = 0; j < cols; ++j)
        {
            const double hGrad = fabs(RFeatures::calcHorizontalGrad<cv::Vec<byte,1> >( timg, i, j, 0));
            const double vGrad = fabs(RFeatures::calcVerticalGrad<cv::Vec<byte,1> >( timg, i, j, 0));
            if ( hGrad > 0 || vGrad > 0)
            {
                const double mag = sqrt(hGrad*hGrad + vGrad*vGrad);
                const double angle = 2*atan2(vGrad,hGrad) / M_PI;   // value in [0,1]
                //cerr << angle << endl;
                outMap.at<byte>(i,j) = angle*255;
            }   // end if
            else
                outMap.at<byte>(i,j) = 0;
        }   // end for - cols
    }   // end for - rows
    return outMap;
}   // end createDifferenceMap




int main( int argc, char **argv)
{
    if ( argc != 3)
    {
        cerr << "Provide panorama directory and ground truth file" << endl;
        return EXIT_FAILURE;
    }   // end if

    const string panoDir = argv[1];
    const string examplesFile = argv[2];
    std::vector<RFeatures::Instance> instances;
    RFeatures::DataLoader( panoDir).loadInstances( examplesFile, instances);

    RFeatures::DescriptorStatistics dstats;

    BOOST_FOREACH ( RFeatures::Instance& instance, instances)
    {
        // Get the original image subregion
        const cv::Mat colourImage = instance.colourImg( instance.boundBox);

        // Get equalised lightness image
        const cv::Mat_<float> cieLabLightImg = RFeatures::getCIELabChannels( colourImage)[0];
        cv::Mat_<byte> labImg;
        cieLabLightImg.convertTo( labImg, CV_8U, 255./100,0);
        cv::equalizeHist(labImg,labImg);
        //RFeatures::showImage( labImg, "Equalised lightness", false);

        // Get original depth map
        const cv::Mat_<float> dimg = instance.depthImg( instance.boundBox);
        const cv::Mat d2img = RFeatures::convertForDisplay( dimg, true);
        //RFeatures::showImage( d2img, "Original depth", false);

        // Get the segmented depth (most common depth values)
        DepthSegmenter segmenter( instance.depthImg);
        segmenter.setMinMaxFromSubRect( instance.boundBox);
        cv::Mat_<byte> dmask = segmenter.calcMostCommonDepthMask( instance.boundBox);

        // Use most common depth as mask to segment equalised lightness image
        //dmask = cv::Mat_<byte>::ones( dmask.size()) * 255; // DEBUG - REMOVE MASK
        RFeatures::showImage( labImg, "Equalised, lightness", false);
        labImg &= dmask;
        RFeatures::showImage( labImg, "Equalised, segmented lightness", true);

        /*
        // Band reduced number of intensity values indexed by frequency - ignoring the zero values
        const int numLevs = 2;
        const cv::Mat_<byte> indexedImg = RFeatures::IntensityIndexer( labImg, numLevs)( cv::Rect(0,0,0,0), dmask);
        RFeatures::showImage( indexedImg, "Indexed image", false);

        const cv::Mat_<byte> diffMap = createDifferenceMap( indexedImg);    // Difference map (edges of bands)
        RFeatures::showImage( diffMap, "Difference map", true);
        */

        cv::Mat_<float> dmaskfloat;
        dmask.convertTo( dmaskfloat, CV_32F);
        const cv::Mat_<float> depthProfileFloatMap = dimg & dmaskfloat;
        double mn, mx;
        cv::minMaxLoc( depthProfileFloatMap, &mn, &mx);

        cv::Mat depthProfileByteMap;
        depthProfileFloatMap.convertTo( depthProfileByteMap, CV_8U, 255./mx);
        //RFeatures::showImage( depthProfileByteMap, "Depth profile map", false);

        // Create the descriptor
        //RFeatures::showImage( dmask, "Most common depth mask", true);
        const cv::Mat_<float> f1 = RFeatures::SilhouetteFeature( dmask, 4)();   // Descriptor
        const cv::Size cellDims(2,2);
        const cv::Mat_<float> f2 = RFeatures::ProHOG( 3, true, cellDims).extract( labImg);   // Descriptor

        const cv::Mat_<float> f = RFeatures::combine( f1, f2);
        dstats << f;
        RFeatures::writeDescriptor( cout, f);
        cout << endl;
    }   // end foreach

    // Print statistics
    cerr << dstats << endl;

    return EXIT_SUCCESS;
}   // end main
