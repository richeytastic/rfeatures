/*
 * This class manages caching and reordering of 3D points received out of order of the
 * image pixel order of a view (e.g. because of asynchronous receipt of data over a
 * network connection). Reordering of the received pixels is necessary to allow the
 * correct creation of integral image based view data (which requires appending pixel
 * data in an orderly sequence from top left to bottom right in the image).
 *
 * Richard Palmer
 * 2012
 */

#pragma once
#ifndef RFEATURES_VIEW_RECEIVER_H
#define RFEATURES_VIEW_RECEIVER_H

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <list>
using std::list;

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

#include "View.h"
using RFeatures::View;
#include "PointCloud.h"
using RFeatures::PointCloud;

#include <Exceptions.h> // rlib


namespace RFeatures
{

class rFeatures_EXPORT ViewReceiver
{
public:
    typedef boost::shared_ptr<ViewReceiver> Ptr;
    static Ptr create( ViewBuilder::Ptr vb, double maxPointRange=100);
    ~ViewReceiver();

    // Rectify the absolute point in space p by reconciling it with the image pixel coordinates x,y
    // and the camera position and focal vector. The third element of the returned vector may be used as
    // a range value. InvalidVectorException is thrown if the provided focal vector causes the
    // calculated vector to point rearward. The field of view is assumed to be 90 degrees.
    static cv::Vec3d rectify90Point( const cv::Vec3d &p, int x, int y, const cv::Size &viewDims,
                                     const cv::Vec3d &camPos, const cv::Vec3d &focus)
                                     throw (rlib::InvalidVectorException);

    // Return the ViewBuilder object.
    inline ViewBuilder::Ptr builder() const { return viewBuilder_;}

    // Hand off received point data that might be out of order. Parameter vectors must be non-NULL
    // and the same length. This function assumes that every pixel in the image has a corresponding
    // location. The given 3-D point locations are absolute in space. Because some 3-D points may
    // not be available for some pixels, the corresponding 3-D points for those pixels should be
    // set to X=0,Y=0,Z=0 before calling this function. These zero points are not added to the
    // point cloud returned from this function, but they ARE added to the internal View object
    // (via the member ViewBuilder provided via the constructor). Point cloud data may thus be
    // more efficiently visualised using an aggregation of PointCloud objects returned from calls
    // to this function, while referential integrity of the View's point cloud by its image is
    // maintained by ensuring a point (possibly a zero point) exists for every image pixel in the
    // View's organised (structured) point cloud.
    //
    // When points are added to the View (via the member ViewBuilder object), the absolute position
    // of the view is first subtracted from each point so that the view's point cloud is made up of
    // points having locations in space relative to the view's location. The range of each (relative)
    // point from the view's camera plane is calculated as the scalar product of the view's normalised
    // focal vector with the (relative) point. The points are then rectified to align with the world
    // X,Y,Z axes so that lateral position is given in the X,Y plane and depth is given in the Z axis.
    //
    // This function returns the points added on this call (if any), exluding possible zero points
    // as mentioned above. This returned point cloud may also include points that existed in the
    // cache from previous calls to this function but were found to be out of order at those times.
    //
    // The parameter pointers may be deleted (and will be deleted eventually) so that their state
    // is undefined upon function return (they should not be used again).
    PointCloud::Ptr receivePointData( const vector<cv::Point>* pxls, const vector<cv::Vec3d>* locs);

    // Returns true iff all points have been received for the view (according to the size
    // of the view's expected point cloud) and all cache data has been exhausted and returned
    // via a call to receivePointData.
    bool isFinished() const;

    // Return the proportion of points received so far [0,1]
    double getProportionReceived() const;

private:
    const double MAX_POINT_RANGE;   // Max range at which points can be added to the view (default 100m)
    ViewBuilder::Ptr viewBuilder_;  // The actual view data (being built)
    int nxtPxlPos_; // Track next pixel position expected

    struct DataChunk;
    list<DataChunk*> cache_; // Cached pixels and their associated points

    // Takes in received pointers to point cloud data and tests if they are next in order of
    // processing. If they are, no changes are made to the out parameters. If the parameters
    // are found to be out of order however, they are instead set to reference the appropriate
    // entry from the cache on return. The function returns true only if the out parameters
    // are set to some in-order data on return (whether provided as the parameters or found
    // in the cache). The parameter data should only be used to add to the view when this
    // function returns true.
    bool checkCache( const vector<cv::Point>*&, const vector<cv::Vec3d>*&);

    // Ctor
    ViewReceiver( ViewBuilder::Ptr vb, double maxPointRange);
    // Non-copyable
    ViewReceiver(const ViewReceiver&);
    void operator=(const ViewReceiver&);
};  // end class


}	// end namespace

#endif
