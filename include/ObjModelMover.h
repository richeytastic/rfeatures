#ifndef RFEATURES_OBJ_MODEL_MOVER_H
#define RFEATURES_OBJ_MODEL_MOVER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelMover
{
public:
    ObjModelMover();
    explicit ObjModelMover( const cv::Vec3d& translation);        // Create translation matrix only (no rotation).
    explicit ObjModelMover( const cv::Matx44d& transform);        // Set entire transformation matrix directly.

    // The following constructors allow for the optional incorporatation of a translatation AFTER the desired rotation.

    // Set rotation submatrix directly with (optional) subsequent translation.
    ObjModelMover( const cv::Matx33d& rotMat, const cv::Vec3d& t=cv::Vec3d(0,0,0));
    // Rotation from positive Z (normal) and positive Y (up) vectors with (optional) subsequent translation.
    ObjModelMover( const cv::Vec3d& posZ, const cv::Vec3d& posY, const cv::Vec3d& t=cv::Vec3d(0,0,0));
    // Rotation from angle and axis with (optional) subsequent translation.
    ObjModelMover( double radians, const cv::Vec3d& axis, const cv::Vec3d& t=cv::Vec3d(0,0,0));

    void prependTranslation( const cv::Vec3d&); // Perform a transform prior to the existing transform.

    inline const cv::Matx44d& getTransformMatrix() const { return _tmat;}
    inline const cv::Matx44d& operator()() const { return _tmat;}

    void operator()( ObjModel::Ptr) const;  // Move the provided object (adjust location of all of its vertices).
    void operator()( cv::Vec3d& v) const;   // Transform a single vertex
    void operator()( cv::Vec3f& v) const;   // Transform a single vertex

private:
    cv::Matx44d _tmat;  // Transformation matrix as homogeneous coordinates
};  // end class

}   // end namespace

#endif
