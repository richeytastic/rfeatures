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

#include "PatchDescriptor.h"
using RFeatures::PatchDescriptor;
#include "FeatureUtils.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
using std::cerr;
using std::endl;


PatchDescriptor::PatchDescriptor( const cv::Mat_<float> v)
{
    assert( importFormatted(v));
}   // end ctor


PatchDescriptor::PatchDescriptor( const cv::Vec2f ofs)
{
    setOffset(ofs);
}   // end ctor


void PatchDescriptor::setOffset( const cv::Vec2f& ofs)
{
    _offset = ofs;
}   // end setOffset



int PatchDescriptor::addFeatureVector( const cv::Mat_<float> fv)
{
    _featureVecs.push_back(fv);
    return (int)_featureVecs.size();
}   // end addFeatureVector



int PatchDescriptor::addFeatureVectors( const list<cv::Mat_<float> >& fvs)
{
    int i = (int)_featureVecs.size();
    _featureVecs.resize( _featureVecs.size() + fvs.size());
    for ( list<cv::Mat_<float> >::const_iterator fv = fvs.begin(); fv != fvs.end(); ++fv)
        _featureVecs[i++] = *fv;
    return (int)_featureVecs.size();
}   // end addFeatureVectors



int PatchDescriptor::addRowFeatureVectors( const cv::Mat_<float> fv)
{
    const int nrows = fv.rows;
    for ( int i = 0; i < nrows; ++i)
    {
        const cv::Mat_<float> row = fv.row(i);
        _featureVecs.push_back(row);
    }   // end for
    return (int)_featureVecs.size();
}   // end addRowFeatureVectors



int PatchDescriptor::addFeatureVector( const float* sp, const float* ep)
{
    assert( sp != NULL && ep != NULL);
    cv::Mat_<float> fvec;
    while ( sp != ep)
    {
        fvec.push_back(*sp);
        sp++;
    }   // end while
    return addFeatureVector( fvec.t()); // Add as row vector
}   // end addFeatureVector



float PatchDescriptor::val( int fv, int i) const
{
    const cv::Mat_<float>& f = _featureVecs[fv];
    return f.at<float>(i / f.cols, i % f.cols);
}   // end val



// private
bool PatchDescriptor::importFormatted( const cv::Mat_<float> pf)
{
    //const int totalLen = (int)pf.total();
    cv::Mat_<float> d = pf;
    if ( d.cols == 1)
        d = d.t();   // Ensure d is a row vector

    if ( d.rows != 1)
    {
        cerr << "PatchDescriptor::importFormatted: only accepts single row/column cv::Mat_<float>" << endl;
        return false;
    }   // end if

    if ( d.cols < 5)
    {
        cerr << "PatchDescriptor::importFormatted: accepted vector must be at least 5 floats long" << endl;
        cerr << "offset_X offset_Y numFVs size_fv0 fv0_0 [fv0_1 fv0_2 ... fv0_size_fv0-1 size_fv1 fv1_0 ...]" << endl;
        return false;
    }   // end if

    const float* arr = d.ptr<float>(0);
    setOffset( cv::Vec2f( arr[0], arr[1]));
    const int numFVs = int(arr[2]);
    int didx = 3;
    for ( int i = 0; i < numFVs; ++i)   // Read in each descriptor
    {
        const int fvLen = (int)arr[didx++];
        cv::Mat_<float> fv(1, fvLen);
        memcpy( fv.ptr<float>(), &arr[didx], fvLen*sizeof(float));
        addFeatureVector(fv);
        didx += fvLen;
    }   // end for

    return true;
}   // end importFormatted



ostream& RFeatures::operator<<( ostream& os, const PatchDescriptor& pf)
{
    const cv::Vec2f& offset = pf.getOffset();
    RFeatures::writeDescriptor( os, (cv::Mat_<float>)offset);

    const int numFVs = pf.getNumFeatureVectors();
    // third number denotes the number of feature vectors stored
    os << " " << numFVs;
    for ( int i = 0; i < numFVs; ++i)
    {
        // Each feature vector is prepended by its length
        const cv::Mat_<float> fv = pf.getFeatureVector(i);
        os << " " << fv.total() << " ";
        RFeatures::writeDescriptor( os, fv);
    }   // end for

    return os;
}   // end operator<<



istream& RFeatures::operator>>( istream& is, PatchDescriptor& pf)
{
    const cv::Mat_<float> d = RFeatures::readDescriptor( is, true);
    if ( !pf.importFormatted(d))
        is.setstate( std::ios_base::failbit);
    return is;
}   // end operator>>



int PatchDescriptor::load( const string& featFile, vector<PatchDescriptor::Ptr>& pds)
{
    const cv::Mat_<float> pfs = RFeatures::readDescriptors( featFile, false);
    const int numPFs = pfs.rows;
    for ( int i = 0; i < numPFs; ++i)
    {
        PatchDescriptor::Ptr pd( new PatchDescriptor( pfs.row(i)));
        pds.push_back( pd);
    }   // end for
    return numPFs;
}   // end load

