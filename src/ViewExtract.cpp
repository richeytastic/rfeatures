#include "ViewExtract.h"
using RFeatures::ViewExtract;
#include "PointCloudTextWriter.h"
#include "PointCloudTextReader.h"


ViewExtract::ViewExtract( const PointCloud::Ptr data, const cv::Rect& rct)
    : data_(data), bbox_(rct)
{
}   // end ctor



ViewExtract::ViewExtract()
{   // Used by operator>>
}   // end ctor



void ViewExtract::setModelName( const string& nm)
{
    modelName_ = nm;
}   // end setModelName


void ViewExtract::setPartName( const string& nm)
{
    partName_ = nm;
}   // end setPartName


void ViewExtract::setAspectInfo( const string& ai)
{
    aspectInfo_ = ai;
}   // end setAspectInfo


void ViewExtract::setPosVec( const cv::Vec3d &pv)
{
    posVec_ = pv;
}   // end setPosVec


void ViewExtract::setDirVec( const cv::Vec3d &dv)
{
    dirVec_ = dv;
}   // end setDirVec


void ViewExtract::setUpVec( const cv::Vec3d &uv)
{
    upVec_ = uv;
}   // end setUpVec



cv::Size2f ViewExtract::calcRealSize( double focLen) const
{
    int xpxl = bbox_.width / 2;
    int ypxl = bbox_.height / 2;
    float x, y, z;
    byte r, g, b;

    int pxlCount = 0;
    double depthSum = 0;
    for ( int xp = xpxl - 1; xp <= xpxl + 1; ++xp)
    {
        for ( int yp = ypxl - 1; yp <= ypxl + 1; ++yp)
        {
            data_->from( yp, xp, x, y, z, r, g, b);
            if ( z > 0)
            {
                pxlCount++;
                depthSum += z;
            }   // end if
        }   // end for
    }   // end for

    if ( pxlCount == 0)
    {
        std::cerr << "ERROR: Can't estimate depth of ViewExtract - no depth data at 9 centre pixels!" << std::endl;
        assert( pxlCount > 0);
    }   // end if
   
    const double meanDepth = depthSum / pxlCount;
    double scale = meanDepth / focLen;  // Size estimated by similar triangles
    return cv::Size2f( (float)(bbox_.width * scale), (float)(bbox_.height * scale));
}   // end calcRealSize



void ViewExtract::save( const char *vfile) const
{
    std::ofstream ofs( vfile);
    ofs << *this;
    ofs.close();
}   // end save



// static
ViewExtract::Ptr ViewExtract::load( const char *vfile)
{
    ViewExtract::Ptr vep;

    ViewExtract *ve = NULL;
    try
    {
        std::ifstream ifs( vfile);
        ve = new ViewExtract;
        ifs >> *ve;
        ifs.close();
        vep = ViewExtract::Ptr( ve);
    }   // end try
    catch ( const std::exception &e)
    {
        std::cerr << "Unable to load ViewExtract: " << vfile << std::endl;
        if ( ve != NULL)
            delete ve;
        vep.reset();
    }   // end catch

    return vep;
}   // end load



// static
void ViewExtract::loadDir( const char *vdir, vector<ViewExtract::Ptr> &views)
{
    using namespace boost::filesystem;
    directory_iterator endIt;

    for ( directory_iterator it( vdir); it != endIt; ++it)
    {
        if ( !is_regular_file( it->status()))
            continue;

        const char *vfile = it->path().string().c_str();
        ViewExtract::Ptr vp = ViewExtract::load( vfile);
        if ( vp != NULL)
            views.push_back(vp);
    }   // end for
}   // end loadDir



std::ostream& RFeatures::operator<<( std::ostream& os, const ViewExtract& ee)
{
    using std::endl;
    os << "MODEL: " << ee.modelName_ << endl;
    os << "PART: " << ee.partName_ << endl;
    os << "ASPECT: " << ee.aspectInfo_ << endl;
    os << "POS: " << ee.posVec_[0] << " " << ee.posVec_[1] << " " << ee.posVec_[2] << endl;
    os << "DIR: " << ee.dirVec_[0] << " " << ee.dirVec_[1] << " " << ee.dirVec_[2] << endl;
    os << "UP: " << ee.upVec_[0] << " " << ee.upVec_[1] << " " << ee.upVec_[2] << endl;
    os << "DATA:" << endl;
    RFeatures::PointCloudTextWriter pctw( ee.data_, &ee.bbox_);
    os << pctw;
    return os;
}   // end operator<<



std::istream& RFeatures::operator>>( std::istream& is, ViewExtract& ee)
{
    string label;

    // Model info
    is >> label >> ee.modelName_;
    assert( label == "MODEL:");
    is >> label >> ee.partName_;
    assert( label == "PART:");
    is >> label >> ee.aspectInfo_;
    assert( label == "ASPECT:");

    // View vectors
    cv::Vec3d pv, dv, uv;
    is >> label >> pv[0] >> pv[1] >> pv[2];
    assert( label == "POS:");
    is >> label >> dv[0] >> dv[1] >> dv[2];
    assert( label == "DIR:");
    is >> label >> uv[0] >> uv[1] >> uv[2];
    assert( label == "UP:");
    ee.setPosVec(pv);
    ee.setDirVec(dv);
    ee.setUpVec(uv);

    // Bounding box
    is >> label;    // DATA:
    assert( label == "DATA:");
    std::getline( is, label);   // Discard end of line

    // Get the point cloud
    RFeatures::PointCloudTextReader pcdr;
    is >> pcdr;
    ee.data_ = pcdr.getPointCloud();
    ee.bbox_ = cv::Rect(0,0,ee.data_->cols(), ee.data_->rows());
    return is;
}   // end operator>>
