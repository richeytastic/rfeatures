#include "GrabCutsOperator.h"
using RFeatures::GrabCutsOperator;
#include <algorithm>


GrabCutsOperator::GrabCutsOperator( const cv::Mat &img, const cv::Rect &fg, int its)
    : fgArea(fg), iters(std::max<int>(1,its))
{
    image = img.clone();
}   // end ctor


cv::Mat GrabCutsOperator::findSegmentation() const
{
    cv::Mat segmentation;
    cv::Mat bgModel, fgModel;   // Internally used by cv::grabCut
    cv::grabCut( image, segmentation, fgArea, bgModel, fgModel, iters, cv::GC_INIT_WITH_RECT);
    return segmentation;
}   // end findSegmentation



cv::Mat GrabCutsOperator::getBinaryForeground( const cv::Mat &segmentation) const
{
    //cv::compare( segmentation, cv::GC_PR_FGD, segmentation, cv::CMP_EQ);
    cv::Mat segImg;
    segImg = (segmentation & 0x01) * 255;  // Works since cv::GC_FGD == 1 and cv::GC_PR_FGD == 3
    return segImg;
}   // end getBinaryForeground



cv::Mat GrabCutsOperator::getImageForeground( const cv::Mat &segmentation) const
{
    cv::Mat binImg = getBinaryForeground( segmentation);
    cv::Mat foreground( image.size(), CV_8UC3, cv::Scalar(0,0,0));
    image.copyTo( foreground, binImg);
    return foreground;
}   // end getImageForeground
