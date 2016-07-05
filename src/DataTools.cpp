#include "DataTools.h"
using RFeatures::FeatureWriter;
using RFeatures::FeatureRecord;
#include <iostream>
#include <sstream>
#include <fstream>
#include <ctime>
#include <cassert>
#include <cstdlib>



string toClassName( const string& fn)
{
    string fname = fn;
    boost::algorithm::trim( fname);
    // Strip any extension
    fname = fname.substr(0, fname.find_first_of('.'));
    boost::algorithm::to_lower( fname);   // Convert to lower case
    boost::algorithm::replace_all( fname, "_", " ");   // Replace underscores
    return fname;
}   // end toClassName


string classToFileName( const string& cn)
{
    string cname = cn;
    boost::algorithm::trim( cname);
    boost::algorithm::to_lower( cname);   // Convert to lower case
    boost::algorithm::replace_all( cname, " ", "_");   // Replace spaces
    return cname + ".dat";
}   // end classToFileName



const char SEP = '|';   // Field separator in bounding box file
const char COMMENT = '#';   // indicates that line should be commented out of bounding box file


ostream& RFeatures::operator<<( ostream &os, const FeatureRecord &r)
{
    os << r.dataId << SEP << r.viewInfo << SEP << r.className << SEP << r.exampleId << SEP;
    const cv::Rect &bb = r.boundingBox;
    os << bb.x << SEP << bb.y << SEP << bb.width << SEP << bb.height << std::endl;
    return os;
}   // end operator<<



istream& RFeatures::operator>>( istream &is, FeatureRecord &r)
{
    r.dataId = "";
    r.viewInfo = "";
    r.className = "";

    r.exampleId = 0;
    r.boundingBox.x = 0;
    r.boundingBox.y = 0;
    r.boundingBox.width = 0;
    r.boundingBox.height = 0;
    string exId, x, y, w, h;

    string ln;
    std::getline( is, ln);

    int recNum = 0;
    for ( int i = 0; i < ln.length(); ++i)
    {
        const char c = ln[i];
        if ( c == SEP)
            recNum++;        
        else if ( ln[i] == COMMENT) // Ignore remainder of line
            break;
        else
        {
            switch ( recNum)
            {
                case 0: r.dataId += c; break;
                case 1: r.viewInfo += c; break;
                case 2: r.className += c; break;
                case 3: exId += c; break;
                case 4: x += c; break;
                case 5: y += c; break;
                case 6: w += c; break;
                case 7: h += c; break;
                default: break;
            }   // end switch
        }   // end else
    }   // end for

    if ( !exId.empty())
    {
        std::istringstream iss_exampleId( exId);
        iss_exampleId >> r.exampleId;
    }   // end if

    if ( !x.empty())
    {
        std::istringstream iss_x( x);
        iss_x >> r.boundingBox.x;
    }   // end if

    if ( !y.empty())
    {
        std::istringstream iss_y( y);
        iss_y >> r.boundingBox.y;
    }   // end if

    if ( !w.empty())
    {
        std::istringstream iss_w( w);
        iss_w >> r.boundingBox.width;
    }   // end if

    if ( !h.empty())
    {
        std::istringstream iss_h( h);
        iss_h >> r.boundingBox.height;
    }   // end if

    return is;
}   // end operator>>



void readFeatureRecords( const string& featFile, list<FeatureRecord*>& recs)
{
    std::ifstream ifs;
    try
    {
        ifs.open( featFile.c_str());
        while ( ifs)
        {
            FeatureRecord* f = new FeatureRecord;
            ifs >> *f;
            if ( f->exampleId != 0)
                recs.push_back(f);
            else
                delete f;
        }   // end while
    }   // end try
    catch ( const std::exception& e)
    {
        std::cerr << "ERROR: Unable to read in feature records from " << featFile << std::endl;
        std::cerr << e.what() << std::endl;
    }   // end catch

    if ( ifs.is_open())
        ifs.close();
}   // end readFeatureRecords



int RFeatures::loadFeatureRecords( const std::string& gtfile, std::vector<FeatureRecord>& frecs)
{
    int loadCount = 0;
    try
    {
        std::ifstream ifs( gtfile.c_str());
        while ( ifs)
        {
            FeatureRecord f;
            ifs >> f;
            if ( f.exampleId != 0)
            {
                loadCount++;
                frecs.push_back(f);
            }   // end if
        }   // end while
        ifs.close();
    }   // end try
    catch ( const std::exception& e)
    {
        std::cerr << "ERROR: Unable to read in feature records from " << gtfile << std::endl;
        std::cerr << e.what() << std::endl;
        loadCount = -1;
    }   // end catch

    return loadCount;
}   // end loadFeatureRecords



/*********************************/


FeatureWriter::FeatureWriter( const string &fname)
{
    if ( !setGroundTruthDir( fname))
        throw std::invalid_argument( "FeatureWriter ctor arg not a directory!");

    dataId_ = "-";
    viewInfo_ = "-";
    className_ = "-";
}   // end ctor



// private
bool FeatureWriter::setGroundTruthDir( const string& gtdir)
{
    if ( !boost::filesystem::is_directory( gtdir))
        return false;

    featDir_ = gtdir;

    // Get all the existing classes and features
    using namespace boost::filesystem;
    directory_iterator endItr;
    for ( directory_iterator i( featDir_); i != endItr; ++i)
    {
        const string nm = i->path().filename().string();
        const int lastDot = (int)nm.find_last_of('.');
        if ( lastDot != string::npos && nm.substr( lastDot) == ".dat")
        {
            const string tn = toClassName( nm);

            // Get the feature records from this file
            list<FeatureRecord*>* recs = new list<FeatureRecord*>;
            readFeatureRecords( i->path().string(), *recs);
            classRecs_[tn] = recs;

            // Check the data ID of each new feature to map to dataRecs_
            BOOST_FOREACH ( FeatureRecord* f, *recs)
            {
                if ( dataRecs_.count(f->dataId) == 0)
                    dataRecs_[f->dataId] = new list<FeatureRecord*>;
                dataRecs_[f->dataId]->push_back(f);
            }   // end foreach
        }   // end if
    }   // end for

    return true;
}   // end setGroundTruthDir


string FeatureWriter::getGroundTruthDir() const
{
    return featDir_.generic_string();
}   // end getGroundTruthDir



FeatureWriter::~FeatureWriter()
{
    writeData();

    typedef std::pair<string, list<FeatureRecord*>* > RecPair;
    BOOST_FOREACH ( const RecPair& rp, classRecs_)
    {
        list<FeatureRecord*>* recs = rp.second;
        BOOST_FOREACH ( const FeatureRecord* f, *recs)
            delete f;
        delete recs;
    }   // end foreach

    BOOST_FOREACH ( const RecPair& rp, dataRecs_)
        delete rp.second;
}   // end dtor


// private
FeatureRecord* FeatureWriter::createNewFeatureRecord( const cv::Rect &bbox)
{
    FeatureRecord *r = new FeatureRecord;
    r->dataId = dataId_;
    r->viewInfo = viewInfo_;
    r->className = toClassName( className_);
    r->boundingBox = bbox;
    r->exampleId = int(time(0));
    return r;
}   // end createNewFeatureRecord


// private
int FeatureWriter::writeData() const
{
    int fwritecount = 0;

    typedef std::pair<string, list<FeatureRecord*>* > RecPair;
    BOOST_FOREACH ( const RecPair& rp, classRecs_)
    {
        const list<FeatureRecord*>* recs = rp.second;
        assert( recs != NULL);

        boost::filesystem::path trainingFile = featDir_;
        trainingFile /= classToFileName( rp.first);
        ofstream recstream( trainingFile.string().c_str());
        if ( recstream.is_open())
        {
            BOOST_FOREACH ( const FeatureRecord* f, *recs)
            {
                assert( f != NULL);
                recstream << *f;
                fwritecount++;
            }   // end foreach
            recstream.close();
        }   // end if
        else
        {
            std::cerr << "ERROR: Unable to write features for class " << rp.first << " to disk!" << std::endl;
            fwritecount = -1;
            break;
        }   // end else
    }   // end foreach

    return fwritecount;
}   // end writeData



void FeatureWriter::setClass( const string &cn)
{
    if ( classToFileName(cn).empty())
    {
        className_ = "-";
        throw std::invalid_argument("FeatureWriter::setClass - empty argument string!");
    }   // end if

    className_ = toClassName(cn);   // Current class name
    if ( classRecs_.count(className_) == 0) // Ensure added
        classRecs_[className_] = new list<FeatureRecord*>;
}   // end setClass



void FeatureWriter::setDataId( const string &did)
{
    dataId_ = did;
    if ( dataId_.empty())
        dataId_ = "-";
    if ( dataRecs_.count(dataId_) == 0) // Ensure added
        dataRecs_[dataId_] = new list<FeatureRecord*>;
}   // end setDataId



void FeatureWriter::setViewInfo( const string &vinfo)
{
    viewInfo_ = vinfo;
    if ( viewInfo_.empty())
        viewInfo_ = "-";
}   // end setViewInfo



int FeatureWriter::recordInstance( const cv::Rect& rct)
{
    FeatureRecord* fr = createNewFeatureRecord(rct);
    classRecs_[className_]->push_back(fr);
    dataRecs_[fr->dataId]->push_back(fr);
    return fr->exampleId;
}   // end recordInstance



int FeatureWriter::eraseInstance( const cv::Rect& rct, const string& dataId, const string& viewInfo)
{
    if ( dataRecs_.count(dataId) == 0)
        return 0;

    int numerased = 0;
    list<FeatureRecord*>& recs = *dataRecs_.at(dataId);
    list<FeatureRecord*>::iterator i = recs.begin();
    while ( i != recs.end())
    {
        FeatureRecord* r = *i;
        const string cname = r->className;

        if ( r->boundingBox == rct && r->viewInfo == viewInfo)
        {
            // Erase pointer from the classRecs_ list
            list<FeatureRecord*>::iterator j = classRecs_.at(cname)->begin();
            while ( j != classRecs_.at(cname)->end())
            {
                if ( *j == r)
                    j = classRecs_.at(cname)->erase(j);
                else
                    j++;
            }   // end while

            delete r;
            i = recs.erase(i);
            numerased++;
        }   // end if
        else
            i++;
    }   // end while

    return numerased;
}   // end eraseInstance



void FeatureWriter::findRecords( const string &dataId, vector<FeatureRecord> &recs) const
{
    if ( dataRecs_.count(dataId) == 0)
        return;

    const list<FeatureRecord*>* srecs = dataRecs_.at(dataId);
    BOOST_FOREACH ( const FeatureRecord* f, *srecs)
    {
        if ( f->className == className_)
            recs.push_back( *f);
    }   // end foreach
}   // end findRecords



bool FeatureWriter::classExists( const string &cname) const
{
    return classRecs_.count( toClassName(cname)) == 1;
}   // end classExists



void FeatureWriter::getExistingClassNames( vector<string> &cns) const
{
    typedef std::pair<string, list<FeatureRecord*>* > RecPair;
    BOOST_FOREACH( const RecPair& rp, classRecs_)
        cns.push_back(rp.first);
}   // end getExistingClassNames



string FeatureWriter::getSaveLocation() const
{
    boost::filesystem::path recFile = featDir_;
    recFile /= classToFileName( className_);
    return recFile.string();
}   // end getSaveLocation


/***************************************/


using RFeatures::PanoramaReader;

// public static
View::Ptr PanoramaReader::getViewFace( const Panorama::Ptr pano, const string &face)
{
    std::istringstream iss(face);
    int faceNum = 0;
    iss >> faceNum;
    return PanoramaReader::getViewFace( pano, faceNum);
}   // end getViewFace


// public static
View::Ptr PanoramaReader::getViewFace( const Panorama::Ptr pano, int face)
{
    View::Ptr vface;
    if ( pano != NULL)
    {
        switch ( face)
        {
            case 0: vface = pano->getFrontView(); break;
            case 1: vface = pano->getLeftView(); break;
            case 2: vface = pano->getRearView(); break;
            case 3: vface = pano->getRightView(); break;
        }   // end switch
    }   // end if
    return vface;
}   // end getViewFace


// public static
std::string PanoramaReader::getViewFaceString( int faceId)
{
    assert( faceId >= 0 && faceId <= 3);
    std::string vface;
    switch ( faceId)
    {
        case 0: vface = "FRONT"; break;
        case 1: vface = "LEFT"; break;
        case 2: vface = "REAR"; break;
        case 3: vface = "RIGHT"; break;
    }   // end switch
    return vface;
}   // end getViewFaceString



PanoramaReader::PanoramaReader( const string &panoDir)
    : panoDir_( panoDir)
{}   // end ctor



// public static
Panorama::Ptr PanoramaReader::read( const string& filename)
{
    Panorama::Ptr pano;

    try
    {
        std::ifstream panoFile( filename.c_str());
        pano = Panorama::Ptr( new Panorama);
        panoFile >> *pano;  // throws if pano doesn't exist
        panoFile.close();
    }   // end try
    catch ( const std::exception &e)
    {
        pano.reset();
        std::cerr << "ERROR (PanoramaReader::read): couldn't read panorama from " << filename << std::endl;
        std::cerr << e.what() << std::endl;
    }   // end catch

    return pano;
}   // end read


// public
View::Ptr PanoramaReader::readView( const string& panoViewId)
{
    string panoId;
    int viewFace;
    PanoramaReader::splitViewString( panoViewId, panoId, viewFace);
    return this->readViewFace( panoId, viewFace);
}   // end readView



// public
View::Ptr PanoramaReader::readViewFace( const string &panoId, const string &face)
{
    if ( panoId == lastPanoId_)
        return PanoramaReader::getViewFace( pano_, face);

    string panoFileName = panoDir_ + "/" + panoId;
    // Check if .pano extension already on panoId for loading file
    string extPart = "";
    if ( panoId.size() > 5)
        extPart = panoId.substr( panoId.size()-5);
    if ( extPart != ".pano")
        panoFileName += ".pano";

    Panorama::Ptr pano = PanoramaReader::read( panoFileName);
    View::Ptr v;
    if ( pano != NULL)
    {
        pano_ = pano;
        lastPanoId_ = panoId;
        v = PanoramaReader::getViewFace( pano_, face);
    }   // end if

    return v;
}   // end readViewFace



// public
View::Ptr PanoramaReader::readViewFace( const string& panoId, int face)
{
    std::ostringstream oss;
    oss << face;
    return readViewFace( panoId, oss.str());
}   // end readViewFace



// public
std::string PanoramaReader::createViewString( const std::string& panoId, int face)
{
    assert( face >=0 && face <= 3);
    std::ostringstream oss;
    oss << face;
    return panoId + "-" + oss.str();
}   // end createViewString


// public
void PanoramaReader::splitViewString( const std::string& viewStr, std::string& panoId, int& face)
{
    const int dashPos = (int)viewStr.find_last_of('-');
    panoId = viewStr.substr(0,dashPos);
    face = rlib::cnv<int>( viewStr.substr(dashPos+1,1));
    assert( face >= 0 && face <= 3);
}   // end splitViewString



using RFeatures::DataRecord;
cv::Size2f RFeatures::readDataRecords( const string &panoDir, const string &bbFile,
                        vector<DataRecord> &records, float maxDepth, const cv::Size *minSz)
{
    using namespace std;
    cv::Size2f avgSize;
    int addCount = 0;

    ifstream ifs;
    try
    {
        PanoramaReader preader( panoDir);
        ifs.open( bbFile.c_str());

        boost::unordered_map<string, int> classIdMap;
        classIdMap[""] = 0; // No name for object has class ID of 0
        int classIds = 1;   // Indices of object class IDs

        string ln;
        while ( getline( ifs, ln))
        {
            if ( ln.empty() || ln[0] == '#')
                continue;

            FeatureRecord fr;
            fr.dataId = "";
            istringstream iss(ln);
            iss >> fr;  // Read the example
            if ( fr.dataId.empty()) // Skip the line if not read in as a FeatureRecord
                continue;

            // Ignore example if pixel dimensions are too small
            if ( minSz != NULL)
            {
                if ( fr.boundingBox.width <= minSz->width || fr.boundingBox.height <= minSz->height)
                {
                    cerr << "Example too small - skipping" << endl;
                    continue;
                }   // end if
            }   // end if

            // Get the relevant view
            const View::Ptr view = preader.readViewFace( fr.dataId, fr.viewInfo);
            if ( view == NULL)
            {
                cerr << "EXAMPLE IGNORED: Panorama " << fr.dataId << " could not be found!" << endl;
                continue;
            }   // end if

            const cv::Rect viewRect( 0, 0, view->img2d.cols, view->img2d.rows);
            fr.boundingBox &= viewRect; // ASSUME EXAMPLE MEANS TO BE CONTAINED (ONLY WHOLE EXAMPLES GIVEN)

            // Get the class ID for the class name
            if ( classIdMap.count( fr.className) == 0 && !fr.className.empty())
                classIdMap[fr.className] = classIds++;
            const int classId = classIdMap.at(fr.className);

            // Get the actual size of the feature as measured from the base
            //double depth = view->depthFinder->getMeanDepthAlongBase( fr.boundingBox, maxDepth);
            float depth = maxDepth;
            static const float MIN_DEPTH = 1;
            // Assume that the actual depth of the feature is the closest object in the given bounding box (but not zero)
            for ( int i = 0; i < fr.boundingBox.height; ++i)
            {
                for ( int j = 0; j < fr.boundingBox.width; ++j)
                {
                    const cv::Point pt( j+fr.boundingBox.x, i+fr.boundingBox.y);
                    const float ptDepth = view->rngImg.at<float>(pt);
                    if ( ptDepth < depth && ptDepth > 1)
                        depth = ptDepth;
                }   // end for
            }   // end for

            if ( depth <= MIN_DEPTH)
            {
                cerr << "EXAMPLE IGNORED: Example " << fr.exampleId << " has invalid depth" << endl;
                continue;
            }   // end if

            // Estimate the actual size of the object in the scene
            const float wProp = (float)fr.boundingBox.width / viewRect.width;
            const float hProp = (float)fr.boundingBox.height / viewRect.height;
            const cv::Size2f actSz( wProp * 2*depth, hProp * 2*depth);  // Assume 90 degree FoV

            DataRecord rec;
            rec.classId = classId;
            rec.size = actSz;
            rec.view = view;
            rec.feature = fr;
            records.push_back(rec);

            avgSize.width += actSz.width;
            avgSize.height += actSz.height;
            addCount++;
        }   // end while
    }   // end try
    catch ( const std::exception& e)
    {
        cerr << "ERROR: reading from bounding box file" << endl;
        cerr << e.what() << endl;
    }   // end catch

    if ( ifs.is_open())
        ifs.close();

    if ( addCount > 0)
    {
        avgSize.width /= addCount;
        avgSize.height /= addCount;
    }   // end if

    return avgSize;
}   // end readDataRecords

