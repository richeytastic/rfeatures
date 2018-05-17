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
