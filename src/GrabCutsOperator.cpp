/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

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
