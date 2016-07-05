#include "GroundTruthRecords.h"
using RFeatures::GroundTruthRecords;
using RFeatures::View;
using RFeatures::ImageType;
#include <cassert>
#include <fstream>
#include <sstream>


void RFeatures::ViewGT::getExtracts( std::vector<cv::Mat>& exs, ImageType imgType, bool useHorzFlipped) const
{
    BOOST_FOREACH ( const cv::Rect& rct, grndTrth)
    {
        const cv::Mat ex = RFeatures::createImageType( imgType, view, rct);
        exs.push_back( ex);
        if ( useHorzFlipped)
        {
            cv::Mat hex;
            cv::flip( ex, hex, 1);
            exs.push_back(hex);
        }   // end if
    }   // end foreach
}   // end getExtracts



// public
GroundTruthRecords::GroundTruthRecords( const std::string& panoDir, bool viewsAsImages, bool cv)
    : _panoDir(panoDir), _viewsAsImages( viewsAsImages), _cacheViews(cv), _totalPosExs(0)
{}   // end ctor

// public
bool GroundTruthRecords::areViewsCached() const { return _cacheViews;}


// public
cv::Size2f GroundTruthRecords::getRealWorldMeanObjectDims( bool measureFromBase)
{
    double totWidth = 0;
    double totHeight = 0;
    for ( int i = 0; i < _views.size(); ++i)
    {
        const ViewGT v = getView(i);
        RFeatures::PatchRanger pranger( v.view->rngImg);
        BOOST_FOREACH ( const cv::Rect& gt, v.grndTrth)
        {
            const cv::Size2f realSz = pranger( gt, measureFromBase);
            totWidth += realSz.width;
            totHeight += realSz.height;
        }   // end for
    }   // end for

    return cv::Size2f( float(totWidth / _totalPosExs), float(totHeight / _totalPosExs));
}   // end getRealWorldMeanObjectDims



View::Ptr loadView( const std::string& panoDir, const std::string& panoId, int viewFace, bool viewsAsImages)
{
    View::Ptr view;
    if ( viewsAsImages)
    {
        boost::filesystem::path inpano( panoDir);
        inpano /= panoId;
        const std::string faceStr = RFeatures::PanoramaReader::getViewFaceString(viewFace);

        boost::filesystem::path colourImgFile = inpano;
        colourImgFile /= faceStr + "_colour.png";
        boost::filesystem::path depthImgFile = inpano;
        depthImgFile /= faceStr + "_depth.png";
        cv::Mat_<cv::Vec3b> img = cv::imread( colourImgFile.string(), CV_LOAD_IMAGE_COLOR);
        cv::Mat rimg = cv::imread( depthImgFile.string(), CV_LOAD_IMAGE_ANYDEPTH);
        assert( rimg.type() == CV_16UC1);
        cv::Mat_<float> frimg;
        rimg.convertTo( frimg, CV_32F, 1./100);
        view = View::create( img, frimg);
    }   // end if
    else
    {
        RFeatures::PanoramaReader preader(panoDir);
        view = preader.readViewFace( panoId, viewFace);
    }   // end else

    return view;
}   // end loadView


View::Ptr loadView( const std::string& panoDir, const std::string& viewStr, bool viewsAsImages)
{
    string panoId;
    int viewFace;
    RFeatures::PanoramaReader::splitViewString( viewStr, panoId, viewFace);
    return loadView( panoDir, panoId, viewFace, viewsAsImages);
}   // end loadView


// public
const std::vector<int>& GroundTruthRecords::getGroundTruthOverlaps() const { return _posOverlaps;}
int GroundTruthRecords::getNumViews() const { return (int)_views.size();}


// public
const RFeatures::ViewGT GroundTruthRecords::getView( int viewId)
{
    assert( viewId >= 0 && viewId < _views.size());
    if ( _views[viewId].view == NULL)
    {
        ViewGT& v = _views[viewId];
        v.view = loadView( _panoDir, v.vstr, _viewsAsImages);
        v.win = v.view->size();   // Ensure the view dims are up to date for this view
    }   // end if

    ViewGT retv = _views[viewId];   // Copy for return
    if ( !_cacheViews)  // Don't cache the view after returning if set not to cache
        _views[viewId].view.reset();
    return retv;
}   // end getView



// public
int GroundTruthRecords::loadPosExtracts( const std::string& gtfile, const cv::Size& minPxlDims, std::ostream* os)
{
    if ( os)
        *os << "Loading ground truth extracts for positive class..." << std::endl;

    _views.clear();
    _posOverlaps.clear();
    _negExtracts.clear();

    _posPanoViews.clear();
    _negPanoViews.clear();
    _negPanoIds.clear();

    boost::unordered_map<std::string, int> loadedViews; // View strings to view IDs
    _totalPosExs = 0;

    try
    {
        std::ifstream ifs( gtfile.c_str());
        std::string ln;
        while ( std::getline(ifs, ln))
        {
            RFeatures::FeatureRecord frec;
            std::istringstream iss( ln);
            iss >> frec;
            if ( frec.dataId.empty())
                continue;

            // If the bounding box of this example is too small, we skip it
            if ( frec.boundingBox.width < minPxlDims.width || frec.boundingBox.height < minPxlDims.height)
                continue;

            const int viewNo = rlib::cnv<int>( frec.viewInfo);  // 0, 1, 2, 3
            _posPanoViews[frec.dataId].insert( viewNo); // Set this pano view as unavailable for extraction of negative examples

            const std::string vs = RFeatures::PanoramaReader::createViewString( frec.dataId, viewNo);
            if ( !loadedViews.count(vs))
            {
                _views.resize( _views.size()+1);
                ViewGT& v = *_views.rbegin();
                loadedViews[vs] = v.id = (int)_views.size()-1;
                v.win.height = v.win.width = 0;
                v.vstr = vs;
            }   // end if
            ViewGT& thisView = _views[loadedViews[vs]];

            // Check overlap with existing extracts for this view and record if found
            bool addGroundTruth = true;
            BOOST_FOREACH ( const cv::Rect& gtbox, thisView.grndTrth)
            {
                if (( gtbox & frec.boundingBox).area() > 0)
                {
                    addGroundTruth = false;
                    _posOverlaps.push_back(thisView.id);
                    break;
                }   // end if
            }   // end foreach

            if ( addGroundTruth)
            {
                _totalPosExs++;
                thisView.grndTrth.push_back( frec.boundingBox);
            }   // end if
        }   // end while
    }   // end try
    catch ( const std::exception& e)
    {
        std::cerr << "[ERROR] GroundTruthRecords::load: Unable to read in ground truth data!" << std::endl;
        std::cerr << e.what() << std::endl;
        return -1;
    }   // end catch

    // Set the possible negative views we can get random data from
    // Parse the full panorama directory and set panos for negatives if their views are not used for positives
    typedef boost::filesystem::directory_iterator DirItr;
    DirItr endIt;
    for ( DirItr it( _panoDir); it != endIt; ++it)
    {
        const std::string fname = it->path().filename().string();
        std::string panoId = fname; // Is _viewsAsImages is true, this will be the panoID
        if ( !_viewsAsImages) // Drop the .pano extension to get the panorama ID used in the ground truth file (first column)
            panoId = fname.substr( 0, fname.size() - std::string(".pano").size());

        // Is this a possible negative pano?
        if ( !_posPanoViews.count(panoId))  // If no views of this panorama are used for any of the positives, we can select negs from any view
        {
            _negPanoViews[panoId].insert(0);    // Front
            _negPanoViews[panoId].insert(1);    // Left
            _negPanoViews[panoId].insert(2);    // Rear
            _negPanoViews[panoId].insert(3);    // Right
        }   // end if
        else
        {   // But if it is used for the positives, we can only use negative extracts from the views not used by the positives
            const boost::unordered_set<int>& posViews = _posPanoViews[panoId];
            for ( int vno = 0; vno < 4; ++vno)
                if ( !posViews.count(vno))
                    _negPanoViews[panoId].insert(vno);
        }   // end else

        // Did we add this pano ID as a possible pano for negative extracts? (For randomly selecting)
        if ( _negPanoViews.count(panoId))
            _negPanoIds.push_back(panoId);
    }   // end for

    return _totalPosExs;
}   // end loadPosExtracts



// private
int GroundTruthRecords::loadNegs( int negMultiple, bool useHorzFlipped, ImageType imgType, const RFeatures::FeatureExtractor::Ptr fx, std::ostream* os)
{
    if ( os)
        *os << "Loading non-class views to select from for negative extracts pool..." << std::endl;

    if ( fx != NULL)
        imgType = fx->getImageType();

    _negExtracts.clear();
    int reqNegs = negMultiple * _totalPosExs;   // Required min number of negatives for the pool

    rlib::Random rnd(1);
    while ( reqNegs > 0)
    {
        // Get a random negative pano ID and view face
        const std::string& panoId = _negPanoIds[ rnd.getRandomInt() % _negPanoIds.size()];
        const boost::unordered_set<int>& availViews = _negPanoViews[panoId];
        int rndViewFace = rnd.getRandomInt() % 4;
        while ( !availViews.count(rndViewFace))
            rndViewFace = (rndViewFace + 1) % 4;

        // Load the view - note that negative views aren't ever cached!
        const View::Ptr nview = loadView( _panoDir, panoId, rndViewFace, _viewsAsImages);

        // Get a bunch of random extracts from positive views (do this to maintain aspect ratios between positives and negatives)
        const ViewGT& vgt = _views[ rnd.getRandomInt() % _views.size()];
        BOOST_FOREACH ( const cv::Rect& rct, vgt.grndTrth)
        {
            const cv::Mat ex = RFeatures::createImageType( imgType, nview, rct);
            if ( fx == NULL)
            {
                _negExtracts.push_back( ex);    // Raw extract
                if ( useHorzFlipped)
                {
                    cv::Mat hex;
                    cv::flip( ex, hex, 1);
                    _negExtracts.push_back(hex);
                }   // end if
            }   // end if
            else
            {
                // Resize the extract if it's too small for the feature extractor
                const cv::Size& minDims = fx->getMinSamplingDims();
                cv::Mat inm;
                if ( ex.cols < minDims.width || ex.rows < minDims.height)
                    cv::resize( ex, inm, minDims);
                else
                    inm = ex;

                _negExtracts.push_back( fx->extract(inm));   // Feature vector

                if ( useHorzFlipped)
                {
                    cv::Mat hex;
                    cv::flip( inm, hex, 1);
                    _negExtracts.push_back( fx->extract(hex));
                }   // end if
            }   // end if

            reqNegs--;
        }   // end for
    }   // end while

    return (int)_negExtracts.size();
}   // end loadNegs



// public
int GroundTruthRecords::loadNegExtracts( int negMultiple, bool useHorzFlipped, ImageType imgType, std::ostream* os)
{
    return loadNegs( negMultiple, useHorzFlipped, imgType, RFeatures::FeatureExtractor::Ptr(), os);
}   // end loadNegExtracts


// public
int GroundTruthRecords::loadNegFeatureVectors( int negMultiple, bool useHorzFlipped, const RFeatures::FeatureExtractor::Ptr fx, std::ostream* os)
{
    return loadNegs( negMultiple, useHorzFlipped, fx->getImageType(), fx, os);
}   // end loadNegFeatureVectors



const std::vector<cv::Mat>& GroundTruthRecords::getNegs() const { return _negExtracts;}
