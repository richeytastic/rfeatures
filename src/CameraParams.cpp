/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#include <CameraParams.h>
#include <Eigen/Geometry>
using RFeatures::CameraParams;
#include <cassert>

CameraParams::CameraParams() : pos(0,0,1), focus(0,0,0), up(0,1,0), fov(30)
{
}   // end ctor


CameraParams::CameraParams( const cv::Vec3f& p, const cv::Vec3f& f, const cv::Vec3f& u, double fv)
    : pos(p), focus(f), up(u), fov(fv)
{
    cv::normalize( u, up);  // Ensure up vector is unit length
    assert( fov > 0.0 && fov <= 180.0);
    cv::Vec3f tvec;
    cv::normalize( focus - pos, tvec);
    const double orthval = tvec.dot(up);    // Should be zero
    assert( fabs(orthval) < 0.0005);    // up vector must always be orthogonal to focus - position vector
}  // end ctor


// public
void CameraParams::setPositionFromFocus( double r, const cv::Vec3f* newfocus)
{
    if ( newfocus)
        focus = *newfocus;
    cv::Vec3f dirVec;
    cv::normalize( pos - focus, dirVec);
    pos = float(r)*dirVec + focus;
}   // end setPositionFromFocus


// public
void CameraParams::rotateAboutUpAxis( float degs)
{
    Eigen::Vector3f axis( up[0], up[1], up[2]);
    axis.normalize();
    Eigen::Quaternionf r;
    r = Eigen::AngleAxisf( degs * (float)(CV_PI/180), axis);
    r.normalize();

    Eigen::Quaternionf qpos;
    qpos.w() = 0;
    qpos.vec() = Eigen::Vector3f( pos[0], pos[1], pos[2]);

    Eigen::Quaternionf rotPos = r * qpos * r.inverse();
    Eigen::Vector3f newPos = rotPos.vec();
    pos[0] = newPos[0];
    pos[1] = newPos[1];
    pos[2] = newPos[2];
}   // end rotateAboutUpAxis


// public
cv::Point2f CameraParams::project( const cv::Vec3f& t) const
{
    assert( cv::norm(up) == 1);

    cv::Vec3d nrmFocVec;    // Relative to the camera position
    cv::normalize( focus - pos, nrmFocVec);
    assert( nrmFocVec.dot(up) == 0);    // up vector must always be orthogonal to focus - position vector

    cv::Vec3d rt = t - pos; // Target position relative to the camera
    const double ty = up.dot(rt);  // Amount up (+Y) (real world coord metric)

    cv::Vec3d rightVec;
    cv::normalize( nrmFocVec.cross(up), rightVec);
    const double tx = rightVec.dot(rt); // Amount right (+X) (real world coord metric)

    // The view plane is assumed here to have unit height (can be scaled later by client)
    const double rfov = CV_PI * fov / 180;  // Convert degrees to radians
    const double focLen = 0.5/tan(rfov/2);

    const double realDistance = distance( t);
    double viewPlaneUp = focLen * ty/realDistance;  // From centre of view plane
    double viewPlaneRight = focLen * tx/realDistance;   // From centre of view plane

    // Translate offsets to be from upper left instead of centre
    return cv::Point2f( float(viewPlaneRight + 0.5), float( 0.5 - viewPlaneUp));
}   // end project


// public
cv::Point CameraParams::project( const cv::Vec3f& t, const cv::Size2i& dims) const
{
    const cv::Point2f pp = project( t);
    return cv::Point( (int)cvRound(dims.width * pp.x), (int)cvRound(dims.height * pp.y));
}   // end project


// public
cv::Vec3f CameraParams::project( const cv::Point2f& p, double Z) const
{
    const double S = Z/calcFocalLength();
    return cv::Vec3f( float(S*p.x), float(S*p.y), float(Z));
}   // end project


// public
double CameraParams::distance( const cv::Vec3f& t) const
{
    cv::Vec3d nrmFocVec;
    cv::normalize( focus - pos, nrmFocVec);
    return (t - pos).dot( nrmFocVec);
}   // end distance


// public
double CameraParams::fovRads() const
{
    return fov * CV_PI/180;
}   // end fovRads


// public
double CameraParams::calcFocalLength() const
{
    const double hfov = fovRads()/2;
    return cos(hfov)/sin(hfov);
}   // end calcFocalLength


std::ostream& operator<<( std::ostream& os, const CameraParams& cp)
{
    os << "CAM_POS:   " << cp.pos << std::endl;
    os << "CAM_FOCUS: " << cp.focus << std::endl;
    os << "CAM_UP:    " << cp.up << std::endl;
    os << "CAM_FOV:   " << cp.fov << std::endl;
    return os;
}   // end operator
