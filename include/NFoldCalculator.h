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

#ifndef RFEATURES_NFOLD_CALCULATOR_H
#define RFEATURES_NFOLD_CALCULATOR_H

#include <vector>
#include <boost/unordered_map.hpp>
#include "rFeatures_Export.h"

namespace RFeatures
{

// For a total of N things to divide up (e.g., images, feature vectors etc), find at foldi from [0,NFOLDs)
// the start of the range and the size of the range (set respectively as s0 and r0).
rFeatures_EXPORT void calcValidationFoldRange( int N, int foldi, int NFOLDs, int& s0, int& r0);

// Does the same calculation as calcValidationFoldRange, but sets the start indices and range sizes
// for the portions not within the fold (i.e. the training range). Note that there is one portion
// on either side of the validation range. s0 and r1 are set as the start index and range for the
// first chunk, and s1 and r1 are set for the start index and range for the second chunk.
// Note that s0 is always 0, but that r0 may also be 0 meaning that the training range extends
// only from s1 for r1 elements.
rFeatures_EXPORT void calcTrainingFoldRange( int N, int foldi, int NFOLDs, int& s0, int& r0, int& s1, int& r1);


template <typename T>
class NFoldCalculator
{
public:
    // Convenience function copies the elements of in to out that match the validation fold range.
    // Returns the number of elements (the size of the validation range) copied from vin to vout.
    static int calcValidationFoldSet( int foldi, int NFOLDs, const std::vector<T>& vin, std::vector<T>& vout);

    // Convenience function for calcTrainingFoldRange (this one's a little bit more convenient though!)
    // Returns the number of elements in the training set copied from vin to vout.
    static int calcTrainingFoldSet( int foldi, int NFOLDs, const std::vector<T>& vin, std::vector<T>& vout);

    // Given foldi in [0,NFOLDs), set the positive/negative extracts for the training set.
    // Returns the number set in exs or -1 if foldi not in [0,NFOLDs).
    // These functions should be used for both object classification (tests over extracts
    // returned by the get[Pos|Neg]Validation functions), or for object detection (tests
    // over view IDs returned by calcFoldRange with N == views total).
    int getTrainingExs( int foldi, int NFOLDs, std::vector<T>& exs) const;

    // If given the same foldi and NFOLDs parameters as in the above functions, user can be assured that
    // the extracts set in the exs parameters are not also in the sets returned by the calls to the
    // training functions above. Extracts from the same view are not used for both validation and
    // training (because of contextual clues in bounding boxes from examples in the same image).
    int getValidationExs( int foldi, int NFOLDs, std::vector<T>& exs) const;

    void mapToView( int viewid, const T& x);
    void mapToView( int viewId, const std::vector<T>& x);

private:
    boost::unordered_map<int, std::vector<T> > _viewExs;    // T extracts mapped to view IDs

    // Get the extracts mapped to the views in the range [startViewId,startViewId+count)
    // Returns the number of extracts written into exs.
    int getFromViewRange( int startViewId, int count, std::vector<T>& exs) const;
};  // end class


#include "../include/template/NFoldCalculator_template.h"
}   // end namespace

#endif
