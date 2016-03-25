#include <ImageProcess.h>
using namespace RFeatures;



cv::Mat RFeatures::swapEndBytes( const cv::Mat &img, cv::Mat &out)
{
    out.create( img.size(), img.type());

    int channels = img.channels();
    int nl = img.rows;
    int stepSize = img.cols * channels;
    const uchar* inScanline = NULL;
    uchar* outScanline = NULL;
    int modRes = 0;

    for ( int j = 0; j < nl; ++j)
    {
        inScanline = img.ptr<const uchar>(j);
        outScanline = out.ptr<uchar>(j);

        for ( int i = 0; i < stepSize; ++i)
        {
            modRes = i % channels;
            if ( modRes == 0)   // Put the right input byte into the left output byte
                outScanline[i] = inScanline[i-1 + channels];
            else if ( modRes == (channels - 1)) // Put the left input byte into the right output byte
                outScanline[i] = inScanline[i+1 - channels];
            else    // Not a left or right byte so just copy down normally
                outScanline[i] = inScanline[i];
        }   // end for
    }   // end for

    return out;
}  // end swapEndBytes



void RFeatures::colourReduce( const cv::Mat &img, cv::Mat &out, int div)
{
    out.create(img.rows,img.cols,img.type());

    int nl = img.rows;
    int nc = img.cols * img.channels(); // Total number of elements per line

    for ( int j = 0; j < nl; ++j)
    {
        const uchar* indata = img.ptr<uchar>(j);
        uchar* outdata = out.ptr<uchar>(j);
        for ( int i = 0; i < nc; ++i)
        {
            outdata[i] = indata[i]/div * div + div/2;    // NB - integer division
        }   // end for
    }   // end for
}   // end colourReduce 



// How to do sharpening without explicitly using a convolution filter and cv::filter2D
void RFeatures::sharpen_OLD( const cv::Mat &img, cv::Mat &out)
{
    out.create( img.size(), img.type());    // Allocate if necessary

    int channels = img.channels();
    int nc = img.cols * channels;

    for ( int j = 1; j < img.rows-1; ++j) // All rows except first and last
    {
        const uchar* previous = img.ptr<const uchar>(j-1); // Previous row
        const uchar* current = img.ptr<const uchar>(j); // Current row
        const uchar* next = img.ptr<const uchar>(j+1);  // Next row
        uchar* output = out.ptr<uchar>(j);  // Output row

        for ( int i = channels; i < nc - channels; ++i)   // All columns except first and last
        {
            uchar v = 5*current[i] - current[i-channels] - current[i+channels] - previous[i] - next[i];
            *output++ = cv::saturate_cast<uchar>(v);
        }   // end for
    }   // end for

    // Set the unprocesses pixels to 0
    cv::Scalar s(0);
    if (img.channels() == 3)
        s = cv::Scalar(0,0,0);
    out.row(0).setTo( s);
    out.row(out.rows-1).setTo( s);
    out.col(0).setTo( s);
    out.col(out.cols-1).setTo( s);
}   // end sharpen_OLD



void RFeatures::sharpen( const cv::Mat &img, cv::Mat &out)
{
    // Construct kernel (all entries initialised to 0)
    cv::Mat kernel(3,3,CV_32F,cv::Scalar(0));
    kernel.at<float>(1,1) = 5.0;
    kernel.at<float>(0,1) = -1.0;
    kernel.at<float>(2,1) = -1.0;
    kernel.at<float>(1,0) = -1.0;
    kernel.at<float>(1,2) = -1.0;
    // Filter the image
    cv::filter2D( img, out, img.depth(), kernel);
}   // end sharpen
