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

#include "Panorama.h"
using RFeatures::Panorama;

Panorama::Panorama( const cv::Vec3d &pv, const cv::Vec3d &uv, double y, double p, double r)
    : posVec_(pv), upVec_(uv), yaw_(y), pitch_(p), roll_(r)
{}   // end ctor


Panorama::Panorama()
{}   // end ctor



void Panorama::setFront( const View::Ptr v)
{
    front_ = v;
}   // end setFront


void Panorama::setLeft( const View::Ptr v)
{
    left_ = v;
}   // end setLeft


void Panorama::setRear( const View::Ptr v)
{
    rear_ = v;
}   // end setRear


void Panorama::setRight( const View::Ptr v)
{
    right_ = v;
}   // end setRight


ostream& RFeatures::operator<<( ostream &os, const Panorama &p)
{
    using std::cerr;
    using std::endl;

    if ( p.front_ == NULL || p.left_ == NULL || p.rear_ == NULL || p.right_ == NULL)
    {
        cerr << "ERROR: Cannot save Panorama - missing Views!" << endl;
        return os;
    }   // end if

    os << "POS: " << p.posVec_[0] << " " << p.posVec_[1] << " " << p.posVec_[2] << endl;
    os << "UP: " << p.upVec_[0] << " " << p.upVec_[1] << " " << p.upVec_[2] << endl;
    os << "YAW: " << p.yaw_ << endl;
    os << "PITCH: " << p.pitch_ << endl;
    os << "ROLL: " << p.roll_ << endl;
    os << p.front_ << endl;
    os << p.left_ << endl;
    os << p.rear_ << endl;
    os << p.right_ << endl;

    return os;
}   // end operator<<


istream& RFeatures::operator>>( istream &is, Panorama &p)
{
    using std::getline;

    cv::Vec3d posVec, upVec;
    double yaw, pitch, roll;

    string ln, tok;

    is >> tok;
    if ( tok == "POS:") is >> posVec[0] >> posVec[1] >> posVec[2];
    else throw Panorama::Exception( "Could not read POS token from Panorama input stream!");

    is >> tok;
    if ( tok == "UP:") is >> upVec[0] >> upVec[1] >> upVec[2];
    else throw Panorama::Exception( "Could not read UP token from Panorama input stream!");

    is >> tok;
    if ( tok == "YAW:") is >> yaw;
    else throw Panorama::Exception( "Could not read YAW token from Panorama input stream!");

    is >> tok;
    if ( tok == "PITCH:") is >> pitch;
    else throw Panorama::Exception( "Could not read PITCH token from Panorama input stream!");

    is >> tok;
    if ( tok == "ROLL:") is >> roll;
    else throw Panorama::Exception( "Could not read ROLL token from Panorama input stream!");

    getline( is, ln);   // Get the end of line

    View::Ptr vfront, vleft, vrear, vright;
    is >> vfront;
    assert( vfront != NULL);
    is >> vleft;
    assert( vleft != NULL);
    is >> vrear;
    assert( vrear != NULL);
    is >> vright;
    assert( vright != NULL);

    p = Panorama( posVec, upVec, yaw, pitch, roll);
    p.setFront( vfront);
    p.setLeft( vleft);
    p.setRear( vrear);
    p.setRight( vright);
    
    return is;
}   // end operator>>
