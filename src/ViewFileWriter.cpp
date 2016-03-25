#include <ViewFileWriter.h>
using RFeatures::ViewFileWriter;
#include <iostream>
using std::ostream;
using std::endl;



ViewFileWriter::ViewFileWriter( const View::Ptr &vp) : ViewWriter(vp)
{}   // end ctor



void ViewFileWriter::write( ostream &os) const
{
    // View position
    const cv::Vec3d &pv = m_view->posVec;
    os << pv[0] << " " << pv[1] << " " << pv[2] << endl;

    // View focal vector
    const cv::Vec3d &fv = m_view->focalVec;
    os << fv[0] << " " << fv[1] << " " << fv[2] << endl;

    // View up vector
    const cv::Vec3d &uv = m_view->upVec;
    os << uv[0] << " " << uv[1] << " " << uv[2] << endl;

    // Image gradients parameters
    os << (m_view->imgGrads->channels() - 1) << " " << m_view->imgDirDep
       << " " << m_view->imgSpatialSmooth << " " << m_view->sqrtGammaCorrect << endl;

    // Range gradients parameters
    os << (m_view->rngGrads->channels() - 1) << " " << m_view->rngDirDep
       << " " << m_view->rngSpatialSmooth << endl;

    // Point cloud
    os << m_view->points << endl;
}  // end write
