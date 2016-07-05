#include "KeypointsConverter.h"
using RFeatures::KeypointsConverter;


KeypointsConverter::KeypointsConverter( const Keypoints &kpts, const cv::Mat_<cv::Vec3f> ref)
    : m_ref(ref), m_kpts( kpts)
{
}   // end ctor



cv::Mat_<cv::Vec3f> KeypointsConverter::get3DKeypoints() const
{
    cv::Mat_<cv::Vec3f> kpcloud;
    BOOST_FOREACH( cv::KeyPoint kp, m_kpts)
    {
        const cv::Vec3f& v = m_ref.at<cv::Vec3f>( (int)kp.pt.y, (int)kp.pt.x);
        // Invalid points (points too distant) are at 0,0,0 (the camera position)
        if ( v[0] != 0.0 && v[1] != 0.0 && v[2] != 0.0)
            kpcloud.push_back(v);
    }   // end foreach
    return kpcloud;
}   // end get3DKeypoints



Keypoints KeypointsConverter::cullNullDepthKeypoints() const
{
    Keypoints kpts;
    BOOST_FOREACH( cv::KeyPoint kp, m_kpts)
    {
        const cv::Vec3f& v = m_ref.at<cv::Vec3f>( (int)kp.pt.y, (int)kp.pt.x);
        // Invalid points (points too distant) are at 0,0,0 (the camera position)
        if ( v[0] != 0.0 && v[1] != 0.0 && v[2] != 0.0)
            kpts.push_back( kp);
    }   // end foreach
    return kpts;
}   // end cullNullDepthKeypoints

