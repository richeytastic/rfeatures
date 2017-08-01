#include "VectorFloatKeyHashing.h"
#include <boost/functional/hash.hpp>

struct Rounder
{
    explicit Rounder( int p) : m( pow(10,p)) {}
    double operator()( double v) const { return round(v*m)/m;}
private:
    double m;
};  // end struct

double RFeatures::roundDP( double v, int p)
{
    return Rounder( p)(v);
}   // end roundDP


using RFeatures::Key6L;
using RFeatures::Key3L;
using RFeatures::Key2L;


Key6L RFeatures::concatToKey( const cv::Vec3f& u, const cv::Vec3f& v, int pw)
{
    float fkey[6] = { u[0], u[1], u[2], v[0], v[1], v[2]};
    return Key6L( fkey, pw);
}   // end concatToKey

Key3L RFeatures::toKey( double x, double y, double z, int pw)
{
    double dkey[3] = {x,y,z};
    return Key3L( dkey, pw);
}   // end toKey

Key3L RFeatures::toKey( const cv::Vec3f& u, int pw)
{
    float fkey[3] = {u[0],u[1],u[2]};
    return Key3L( fkey, pw);
}   // end toKey

Key2L RFeatures::toKey( double x, double y, int pw)
{
    double dkey[2] = {x,y};
    return Key2L( dkey, pw);
}   // end toKey

Key2L RFeatures::toKey( const cv::Vec2f& u, int pw)
{
    float fkey[2] = {u[0],u[1]};
    return Key2L( fkey, pw);
}   // end toKey


size_t RFeatures::HashKey6L::operator()( const Key6L& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u[0]); boost::hash_combine( seed, u[1]); boost::hash_combine( seed, u[2]);
    boost::hash_combine( seed, u[3]); boost::hash_combine( seed, u[4]); boost::hash_combine( seed, u[5]);
    return seed;
}   // end operator()

size_t RFeatures::HashKey3L::operator()( const Key3L& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u[0]); boost::hash_combine( seed, u[1]); boost::hash_combine( seed, u[2]);
    return seed;
}   // end operator()

size_t RFeatures::HashKey2L::operator()( const Key2L& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u[0]); boost::hash_combine( seed, u[1]);
    return seed;
}   // end operator()
