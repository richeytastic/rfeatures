#pragma once
#ifndef RFEATURES_AAM_READER_H
#define RFEATURES_AAM_READER_H

#include "RFeatures.h"
#include "PointCloud.h"
using RFeatures::PointCloud;
#include "Panorama.h"
using RFeatures::Panorama;
#include "LASReader.h"


namespace RFeatures
{

// Describes a capture location in AAM data
struct rFeatures_EXPORT CapturePoint
{
    std::string imgFile;     // Associated image for (camera 1)
    double timestamp;   // Time at image capture
    cv::Vec3d pos;      // Vehicle IMU location
    double yaw, pitch, roll;    // Degrees of vehicle heading
};  // end struct

rFeatures_EXPORT ostream& operator<<( ostream& os, const CapturePoint& m);


class rFeatures_EXPORT AAMException : public std::runtime_error
{
public:
    explicit AAMException( const std::string& msg) : std::runtime_error(msg) {}
};  // end class


class rFeatures_EXPORT AAMInfoReader
{
public:
    AAMInfoReader( const std::string& metaFile) throw (AAMException);
    ~AAMInfoReader();

    const std::vector<const CapturePoint*>& getCapturePoints() const;
    int getNumCapturePoints() const { return (int)_capturePoints.size();}

private:
    std::vector<const CapturePoint*> _capturePoints;
};  // end class


class rFeatures_EXPORT AAMReader
{
public:
    // Throws Exception if unable to read given file.
    AAMReader( const AAMInfoReader& aamInfo, const std::string& lasFile) throw (AAMException);

    const PointCloud::Ptr getLASCloud() const;

    // Number of panoramas that may be produced from the LAS data.
    // If the LAS data doesn't contain any capture points, 0 is returned.
    int getNumPanos() const;

    // Creates a panorama for the given index in range [0,getNumPanos() )
    // NULL is returned if the panorama for this location lacks sufficient
    // points in one of the faces. NULL is returned if idx is out of range.
    Panorama::Ptr createPanorama( int idx) const;

private:
    const AAMInfoReader& _cps; // The AAM capture points info
    PointCloud::Ptr _pcloud;   // The LAS data
    cv::Vec3d _offset, _scale;
    std::vector<int> _cIndices;
};  // end class


}   // end namespace

#endif
