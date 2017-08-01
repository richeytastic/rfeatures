#ifndef RFEATURES_OBJ_MODEL_FACE_ANGLE_CALCULATOR_H
#define RFEATURES_OBJ_MODEL_FACE_ANGLE_CALCULATOR_H

/**
 * Calculates the inner angles of the polygonal faces of ObjModels.
 */
#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

typedef boost::unordered_map<int, double> VertexAngles; // Inner angles of vertices within a face
typedef boost::unordered_map<int, VertexAngles> FaceAngles;


class rFeatures_EXPORT ObjModelFaceAngleCalculator : public ObjModelTriangleParser
{
public:
    explicit ObjModelFaceAngleCalculator( const ObjModel::Ptr);

    const ObjModel::Ptr getObject() const { return _model;}
    void reset();

    // Calculate the inner angle at v0 of the triangle defined by the three vertices.
    static double calcAngle( const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2);

    void calcFaceAngles( int fid); // Face angles not updated until this function called.
    const FaceAngles& getFaceAngles() const { return _faces;}

    // Return the inner angle of vidx inside triangle fid.
    // Returns negative if vidx not in referenced face.
    double operator()( int fid, int vidx) const;

protected:
    virtual void parseTriangle( int fid, int vroot, int va, int vb);

private:
    const ObjModel::Ptr _model;
    FaceAngles _faces;
};  // end class

}   // end namespace

#endif
