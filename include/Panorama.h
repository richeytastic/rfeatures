/**
 * A combination of the front, left, rear and right View objects
 * making up a complete 360 degree panorama, plus orientation
 * information for the front face of the panorama.
 *
 * Richard Palmer
 * March 2013
 */

#pragma once
#ifndef RFEATURES_PANORAMA_H
#define RFEATURES_PANORAMA_H

#include "View.h"
using RFeatures::View;

#include <iostream>
using std::istream;
using std::ostream;
#include <string>
using std::string;
#include <stdexcept>
#include "rFeatures_Export.h"

namespace RFeatures
{

class rFeatures_EXPORT Panorama
{
public:
    typedef boost::shared_ptr<Panorama> Ptr;

    struct Exception : public std::runtime_error
    { explicit Exception( const string &err) : std::runtime_error(err){} };  // end class

    Panorama( const cv::Vec3d &posVec, const cv::Vec3d &upVec, double yaw, double pitch, double roll);
    Panorama(); // Use with operator>>

    // Yaw, pitch and roll values should correspond to the front view of the panorama
    double getYaw() const { return yaw_;}
    double getPitch() const { return pitch_;}
    double getRoll() const { return roll_;}

    void setFront( const View::Ptr);
    void setLeft( const View::Ptr);
    void setRear( const View::Ptr);
    void setRight( const View::Ptr);

    View::Ptr getFrontView() const { return front_;}
    View::Ptr getLeftView() const { return left_;}
    View::Ptr getRearView() const { return rear_;}
    View::Ptr getRightView() const { return right_;}

private:
    cv::Vec3d posVec_;
    cv::Vec3d upVec_;
    double yaw_, pitch_, roll_;

    View::Ptr front_;
    View::Ptr left_;
    View::Ptr rear_;
    View::Ptr right_;

    friend ostream& operator<<( ostream&, const Panorama&);
    friend istream& operator>>( istream&, Panorama&);
};  // end class

ostream& operator<<( ostream&, const Panorama&);
istream& operator>>( istream&, Panorama&);

}   // end namespace


#endif
