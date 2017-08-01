#include "ImageType.h"
using RFeatures::ImageType;
#include "EDTFeatureExtractor.h"
#include <cassert>
#include <algorithm>


bool RFeatures::checkFXImageTypeMismatch( const FeatureExtractor* fx, const cv::Mat img)
{
    boost::unordered_set<ImageType> matchingTypes;
    getMatchingImageTypes( img, matchingTypes);
    return matchingTypes.count( fx->getImageType()) > 0;
}   // end checkFXImageTypeMismatch

bool RFeatures::checkFXImageTypeMismatch( const FeatureExtractor::Ptr fx, const cv::Mat img)
{
    return checkFXImageTypeMismatch( fx.get(), img);
}   // end checkFXImageTypeMismatch


int RFeatures::getMatchingImageTypes( const cv::Mat img, boost::unordered_set<ImageType>& imgTypes)
{
    int rvals = -1;
    double mn, mx;
    const int cvtype = img.type();
    switch ( cvtype)
    {
        case CV_8UC1:
            imgTypes.insert( Grey);
            rvals = 1;
            break;
        case CV_8UC3:
            imgTypes.insert( BGR);
            rvals = 1;
            break;
        case CV_32FC1:
            // Check correct range
            cv::minMaxLoc( img, &mn, &mx);
            if ( mn >= 0 && mx <= 1)
            {
                imgTypes.insert( Light);
                imgTypes.insert( Depth);
                imgTypes.insert( EDT);
                rvals = 3;
            }   // end if
            break;
        case CV_32FC3:
            imgTypes.insert( CIELab);
            rvals = 1;
            break;
        default:
            rvals = -1;
    }   // end switch

    return rvals;
}   // end getMatchingImageTypes



int RFeatures::getConvertibleImageTypes( int cvtype, boost::unordered_set<ImageType>& imgTypes)
{
    switch ( cvtype)
    {
        case CV_8UC3:
            imgTypes.insert(BGR);
            imgTypes.insert(Grey);
            imgTypes.insert(Light);
            imgTypes.insert(CIELab);
            break;
        case CV_8UC1:
            imgTypes.insert(Grey);
            break;
        case CV_32FC1:
            imgTypes.insert(Depth);
            imgTypes.insert(EDT);
            break;
        default:
            //no-op
            break;
    }   // end switch
    return (int)imgTypes.size();
}   // end getConvertibleImageTypes


int RFeatures::getConvertibleImageTypes( const cv::Mat img, boost::unordered_set<ImageType>& imgTypes)
{
    return getConvertibleImageTypes( img.type(), imgTypes);
}   // end getConvertibleImageTypes



cv::Mat RFeatures::createImageType( ImageType imgType, const View::Ptr v, cv::Rect rct)
{
    const cv::Size vsz = v->size();
    const cv::Rect imgRct(0,0, vsz.width, vsz.height);
    if ( rct.area() == 0)
        rct = imgRct;

    rct = imgRct & rct; // Ensure rct is within image bounds for desired type

    cv::Mat outm;
    if ( imgType == BGR || imgType == Grey || imgType == Light || imgType == CIELab)
        outm = RFeatures::createImageType( imgType, v->img2d( rct));
    else if ( imgType == Depth || imgType == EDT)
        outm = RFeatures::createImageType( imgType, v->rngImg( rct));

    return outm;
}   // end createImageType



cv::Mat RFeatures::createImageType( ImageType imgType, cv::Mat img) throw (ImageTypeException)
{
    cv::Mat m;
    cv::Mat_<cv::Vec3f> tmp;    // for cielab
    cv::Mat_<byte> bem; // Binary edge map for EDT
    switch ( imgType)
    {
        case BGR:
            if ( img.type() != CV_8UC3)
                throw ImageTypeException( "[EXCEPTION] RFeatures::createImageType: Given image is not already CV_8UC3!");
            m = img.clone();
            break;
        case Grey:
            if ( img.type() != CV_8UC3 && img.type() != CV_8UC1)
                throw ImageTypeException( "[EXCEPTION] RFeatures::createImageType: Cannot make ImageType::Grey from non CV_8UC3 image!");
            m = RFeatures::flatten( img);
            break;
        case Light:
            if ( img.type() != CV_8UC3)
                throw ImageTypeException( "[EXCEPTION] RFeatures::createImageType: Cannot make ImageType::Light from non CV_8UC3 image!");
            m = RFeatures::getLightness( img, 1.0);
            break;
        case CIELab:
            if ( img.type() != CV_8UC3)
                throw ImageTypeException( "[EXCEPTION] RFeatures::createImageType: Cannot make ImageType::CIELab from non CV_8UC3 image!");
            img.convertTo( tmp, CV_32F, 1.0/255);
            cv::cvtColor( tmp, m, CV_BGR2Lab);
            break;
        case Depth:
            if ( img.type() != CV_32FC1)
                throw ImageTypeException( "[EXCEPTION] RFeatures::createImageType: Cannot make ImageType::Depth from non CV_32FC1 image!");
            m = RFeatures::truncateAndScale( img, MAX_RANGE_M, 1);
            break;
        case EDT:
            if ( img.type() != CV_32FC1)
                throw ImageTypeException( "[EXCEPTION] RFeatures::createImageType: Cannot make ImageType::EDT from non CV_32FC1 image!");
            m = RFeatures::truncateAndScale( img, MAX_RANGE_M, 1);
            bem = RFeatures::EDTFeatureExtractor::createBinaryEdgeMap( m, 15, 10);
            //RFeatures::showImage( RFeatures::convertForDisplay( bem), "EDT: Binary Edge Map", true); // DEBUG
            m =  RFeatures::EDTFeature( bem)();
            break;
        default:
            throw ImageTypeException( "[EXCEPTION] RFeatures::createImageType: Invalid ImageType (ENUM DEFINITION ERROR!)");
    }   // end switch

    return m;
}   // end createImageType



ImageType RFeatures::parseImageType( std::istream& ss) throw (ImageTypeException)
{
    string stype;
    ss >> stype;
    std::transform( stype.begin(), stype.end(), stype.begin(), ::tolower);  // Make lower case

    ImageType rtype;
    if ( stype == "bgr" || stype == "rgb")
        rtype = BGR;
    else if ( stype == "grey")
        rtype = Grey;
    else if ( stype == "light")
        rtype = Light;
    else if ( stype == "cielab")
        rtype = CIELab;
    else if ( stype == "depth")
        rtype = Depth;
    else if ( stype == "edt")
        rtype = EDT;
    else
        throw ImageTypeException( "[EXCEPTION] RFeatures::parseImageType: Invalid ImageType for parse string: " + stype);

    return rtype;
}   // end parseImageType



string RFeatures::toString( ImageType imgType) throw (ImageTypeException)
{
    string s;

    switch ( imgType)
    {
        case BGR:
            s = "BGR";
            break;
        case Grey:
            s = "Grey";
            break;
        case Light:
            s = "Light";
            break;
        case CIELab:
            s = "CIELab";
            break;
        case Depth:
            s = "Depth";
            break;
        case EDT:
            s = "EDT";
            break;
        default:
            throw ImageTypeException( "[EXCEPTION] RFeatures::toString: Invalid ImageType (ENUM DEFINITION ERROR!)");
    }   // end switch

    return s;
}   // end toString
