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

#include <ViewFileReader.h>
using RFeatures::ViewFileReader;
#include <fstream>


ViewFileReader::ViewFileReader()
{}   // end ctor



void ViewFileReader::read( istream &is)
{
    cv::Vec3d posVec, focalVec, upVec;

    // Get the view's position
    is >> posVec[0] >> posVec[1] >> posVec[2];
    setPosition( posVec);

    // View's focal vector
    is >> focalVec[0] >> focalVec[1] >> focalVec[2];
    setFocalVector( focalVec);

    // View's up vector
    is >> upVec[0] >> upVec[1] >> upVec[2];
    setUpVector( upVec);

    int nbins;
    bool dd, ss, gc;    // Direction dependence, spatial smoothing and gamma correction flags
    is >> nbins >> dd >> ss >> gc;  // For 2D image
    setImageGradientsParams( nbins, dd, ss, gc);

    is >> nbins >> dd >> ss;    // For range image
    setRangeGradientsParams( nbins, dd, ss);

    string eol; // Discard end of line
    std::getline(is,eol);

    PointCloud::Ptr pcloud = PointCloud::create();
    is >> pcloud;
    setPointCloud( pcloud);
}   // end read



// static
View::Ptr ViewFileReader::load( const char *vfile)
{
    ViewFileReader reader;
    std::ifstream ifs( vfile);
    ifs >> reader;
    ifs.close();
    return reader.getView();
}   // end load
