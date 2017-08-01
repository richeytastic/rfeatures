/**
 * Uses PCA to orient the given face in space.
 */
#ifndef RFEATURES_OBJ_MODEL_ORIENTER_H
#define RFEATURES_OBJ_MODEL_ORIENTER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelOrienter
{
public:
    explicit ObjModelOrienter( ObjModel::Ptr);

    // Orient the model using PCA. Returns the mean of the original position
    // to translate model back if necessary.
    cv::Vec3f orientPCA();

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
