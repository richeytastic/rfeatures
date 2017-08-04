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

#ifndef RFEATURES_OBJECT_MODEL_FUNCTION_MAPPER_H
#define RFEATURES_OBJECT_MODEL_FUNCTION_MAPPER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelFunctionMapper
{
public:
    ObjModelFunctionMapper( int rows, int cols,                     // Mapping resolution
                            double xoffset=0.0, double yoffset=0.0, // Function origin
                            double xscale=1.0, double yscale=1.0);  // Function scale parameters

    // Provide literal z values at x,y column,row indices instead of function(x,y) calculated.
    explicit ObjModelFunctionMapper( const cv::Mat_<double>& literalZ);

    virtual ~ObjModelFunctionMapper();

    ObjModel::Ptr operator()(); // Create and return the model.
    ObjModel::Ptr getModel();   // Return the model created as a result of operator().

    // Textures the model. Returns false if model not yet created or texture empty.
    bool textureMap( const std::string& txfile);
    bool textureMap( const cv::Mat& m);

    // Returned matrix contains function mapped values after call to operator().
    // If constructed using second constructor, this simply returns the parameter
    // to the second constructor.
    inline const cv::Mat_<double>& getFunctionMap() const { return _zvals;}

protected:
    virtual double calcZ( double x, double y) const;    // Default is to return 0 for every input.

private:
    cv::Mat_<double> _zvals;
    const bool _useLiteral;
    cv::Mat_<int> _vidxs;
    double _xoffset, _yoffset;
    double _xscale, _yscale;
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
