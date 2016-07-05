#include "DataLoader.h"
using RFeatures::Instance;
using RFeatures::DataLoader;



// public static
DataLoader::Ptr DataLoader::create( const string& panoDir, ProgressDelegate* pd)
{
    return DataLoader::Ptr( new DataLoader( panoDir, pd));
}   // end create


// public
void DataLoader::setProgressDelegate( ProgressDelegate* pd)
{
    _pupdater = pd;
}   // end setProgressDelegate



// private
DataLoader::DataLoader( const string& panoDir, ProgressDelegate* pd)
    : _panoReader(panoDir),  _pupdater(pd)
{}   // end ctor


// private
DataLoader::~DataLoader()
{
    _thread.join();
}   // end dtor



// Returns true iff images for the view are available
bool DataLoader::setImagesFromView( const string& panoId, const string& viewId)
{
    // Do we already have this view loaded?
    const string id = panoId + "_" + viewId;
    if ( _views.count( id) == 0)
    {
        // Get the relevant view
        const View::Ptr view = _panoReader.readViewFace( panoId, viewId);
        if ( view == NULL)
            return false;
        _views[id] = view;
    }   // end if

    return true;
}   // end setImagesFromView



void readFeatureRecords( const string& bbFile, vector<RFeatures::FeatureRecord>& frecs)
{
    std::ifstream ifs;

    try
    {
        ifs.open( bbFile.c_str());

        string ln;
        while ( std::getline( ifs, ln))
        {
            if ( ln.empty())
                continue;

            RFeatures::FeatureRecord fr;
            fr.dataId = "";
            std::istringstream iss(ln);
            iss >> fr;  // Read the example
            if ( fr.dataId.empty()) // Skip the line if not read in as a FeatureRecord
                continue;

            frecs.push_back(fr);
        }   // end while
    }   // end try
    catch ( const std::exception& e)
    {
        std::cerr << "ERROR (DataLoader): reading from bounding box file " << bbFile << std::endl;
        std::cerr << e.what() << std::endl;
    }   // end catch

    if ( ifs.is_open())
        ifs.close();
}   // end readFeatureRecords



// private (used as thread function)
void DataLoader::createInstances( const vector<RFeatures::FeatureRecord>* frecs)
{
    const int numRecs = (int)frecs->size();
    for ( int i = 0; i < numRecs; ++i)
    {
        // Call progress updater if set
        if ( _pupdater != NULL)
        {
            const float propDone = float(i+1)/numRecs;
            _pupdater->updateProgress( propDone);
        }   // end if

        const RFeatures::FeatureRecord& fr = frecs->at(i);

        if ( !setImagesFromView( fr.dataId, fr.viewInfo))
        {
            std::cerr << "ERROR (DataLoader): Panorama " << fr.dataId << " could not be found!" << std::endl;
            continue;
        }   // end if

        Instance instance;
        instance.viewId = fr.dataId + "_" + fr.viewInfo;
        instance.view = _views[instance.viewId];

        // Bounding box as given in example file might sit partially outside the view.
        // In this case, we restrict the bounding box to be within the view.
        const int cols = instance.view->img2d.cols;
        const int rows = instance.view->img2d.rows;
        const cv::Rect viewRect( 0, 0, cols, rows);
        const cv::Rect modBox = fr.boundingBox & viewRect;
        if ( modBox != fr.boundingBox)
        {
            std::cerr << "Instance " << fr.exampleId << " bounding box partially outside view --> setting within view";
            const int widthDelta = modBox.width - fr.boundingBox.width;
            const int heightDelta = modBox.height - fr.boundingBox.height;
            std::cerr << "\n\t(pixel change width, height: " << widthDelta << ", " << heightDelta << ")" << std::endl;
        }   // end if

        instance.boundBox = modBox;
        _recs.push_back(instance);
    }   // end for
}   // end createInstances



// public
void DataLoader::load( const string& exFile)
{
    _thread.join(); // Ensure previous operation finished
    _frecs.clear();
    _recs.clear();
    readFeatureRecords( exFile, _frecs); // Blocks but fast (no images loaded)
    _thread = boost::thread( &DataLoader::createInstances, this, &_frecs);
}   // end load



// public
int DataLoader::getInstances( vector<Instance>& instances)
{
    _thread.join(); // Ensure finished
    instances.insert( instances.end(), _recs.begin(), _recs.end());
    return (int)_recs.size();
}   // end getInstances



// public
const string& DataLoader::getPanoDirectory() const
{
    return _panoReader.getPanoDirectory();
}   // end getPanoDirectory
