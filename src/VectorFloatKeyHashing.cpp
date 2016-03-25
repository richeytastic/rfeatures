#include "VectorFloatKeyHashing.h"

cv::Vec6i RFeatures::concatToInt( const cv::Vec3f& u, const cv::Vec3f& v, int pw)
{
    const double m = pow(10,pw);
    return cv::Vec6i( int(u[0]*m), int(u[1]*m), int(u[2]*m), int(v[0]*m), int(v[1]*m), int(v[2]*m));
}   // end concatToInt

cv::Vec3i RFeatures::toInt( const cv::Vec3f& u, int pw)
{
    const double m = pow(10,pw);
    return cv::Vec3i( int(u[0]*m), int(u[1]*m), int(u[2]*m));
}   // end toInt

cv::Vec2i RFeatures::toInt( const cv::Vec2f& u, int pw)
{
    const double m = pow(10,pw);
    return cv::Vec2i( int(u[0]*m), int(u[1]*m));
}   // end toInt


size_t RFeatures::HashVec6i::operator()( const cv::Vec6i& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u[0]); boost::hash_combine( seed, u[1]); boost::hash_combine( seed, u[2]);
    boost::hash_combine( seed, u[3]); boost::hash_combine( seed, u[4]); boost::hash_combine( seed, u[5]);
    return seed;
}   // end operator()

size_t RFeatures::HashVec3i::operator()( const cv::Vec3i& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u[0]); boost::hash_combine( seed, u[1]); boost::hash_combine( seed, u[2]);
    return seed;
}   // end operator()

size_t RFeatures::HashVec2i::operator()( const cv::Vec2i& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u[0]); boost::hash_combine( seed, u[1]);
    return seed;
}   // end operator()
