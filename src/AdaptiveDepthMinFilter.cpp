#include "AdaptiveDepthMinFilter.h"
using RFeatures::AdaptiveDepthMinFilter;
using RFeatures::AdaptiveDepthPatchScanner;
using RFeatures::PatchProcessor;
using RFeatures::PatchRanger;

#include <cassert>
#include <algorithm>
#include <vector>
using std::vector;
#include <list>
using std::list;


struct PatchInfo
{
    PatchInfo( float d, const cv::Rect& pRct) : depth(d), patchRct(pRct), _flipped(0) {}

    void flipAnchor( int rows, int cols)
    {
        if ( _flipped)
            return;
        patchRct.x = cols - patchRct.x - patchRct.width + 1;
        patchRct.y = rows - patchRct.y - patchRct.height + 1;
        _flipped = 1;
    }   // end flipAnchor

    const float depth;
    cv::Rect patchRct;  // Absolute location of patch

private:
    bool _flipped;
};  // end struct



void updateImagePatchRefs( const PatchInfo* patchi, int row, int col, cv::Mat_<const PatchInfo*> imgPatches)
{
    // Find the first point on the next row down indicating the start of a
    // patchi row (as long as the patch still extends that far vertically).
    const cv::Point np0( patchi->patchRct.x, row+1);
    if ( np0.y < imgPatches.rows && patchi->patchRct.contains(np0))
        imgPatches(np0) = patchi;

    // Set the next point along in this same row to refer to patchi if still valid in width 
    const cv::Point np1( col+1, row);    // Next point to the right
    if ( np1.x < imgPatches.cols && patchi->patchRct.contains(np1))
        imgPatches(np1) = patchi;
}   // end updateImagePatchRefs



class DownScanner : private PatchProcessor
{
public:
    DownScanner( const cv::Mat_<float> rImg, const cv::Size2f rps, list<const PatchInfo*>& patches)
        : _rngImg(rImg), _imgRct(0,0,rImg.cols,rImg.rows), _rpatchSz(rps), _patchList(patches)
    {}   // end ctor

    cv::Mat_<float> operator()( float minRng, float maxRng)
    {
        _tImg.create( _rngImg.size());
        _imgPatches = cv::Mat_<const PatchInfo*>::zeros( _rngImg.size()); // 2D array of pointers to pixel corresponding patches
        AdaptiveDepthPatchScanner adps( _rngImg, _rpatchSz, this);
        adps.scan( minRng, maxRng);
        return _tImg;
    }   // end operator()

    cv::Mat_<const PatchInfo*> getImagePatches() const
    {
        return _imgPatches;
    }   // end getImagePatches

private:
    virtual void process( const cv::Point& p, float pdepth, const cv::Rect& patchRct)
    {
        // Get the existing patch carried forward along the row
        const PatchInfo* patchi = _imgPatches(p);

        if ( patchRct.area() > 0 && (patchi == 0 || pdepth <= patchi->depth))   // No patch yet or pdepth is smaller (closer)
        {
            const cv::Rect prct = _imgRct & patchRct;
            patchi = _imgPatches(p) = new PatchInfo( pdepth, prct);
            _patchList.push_back(patchi);
        }   // end if

        if ( patchi != 0)
        {
            updateImagePatchRefs( patchi, p.y, p.x, _imgPatches);
            pdepth = patchi->depth;
        }   // end if

        _tImg(p) = pdepth;
    }   // end process

    const cv::Mat_<float> _rngImg;
    const cv::Rect _imgRct;
    const cv::Size2f _rpatchSz;
    list<const PatchInfo*> _patchList;  // List of generated patches
    cv::Mat_<float> _tImg;
    cv::Mat_<const PatchInfo*> _imgPatches; // Pointers to patches at pixels
};  // end class



class UpScanner
{
public:
    UpScanner( cv::Mat_<const PatchInfo*> imgPatches, cv::Mat_<float>& inImg)
        : _imgPatches(imgPatches), _outImg(inImg)
    {}   // end ctor

    cv::Mat_<float> process()
    {
        cv::Mat_<float> outImg = _outImg;
        cv::Mat_<const PatchInfo*> patches = _imgPatches;

        const int rows = outImg.rows;
        const int cols = outImg.cols;
        for ( int i = 0; i < rows; ++i)
        {
            PatchInfo** pirow = patches.ptr<PatchInfo*>(i);
            float* outRow = outImg.ptr<float>(i);

            PatchInfo* prevColPatch = 0;
            for ( int j = 0; j < cols; ++j)
            {
                PatchInfo* ptch = pirow[j];

                // Get the patch previous to the current one in this row that's still valid (overlaps this pixel)
                if ( j > 0)
                {
                    prevColPatch = pirow[j-1];  // prevColPatch will already be flipped from previous iteration
                    // If prevColPatch has expired in width at this column index, we set it back to NULL
                    if ( prevColPatch != 0 && (j >= (prevColPatch->patchRct.x + prevColPatch->patchRct.width)))
                        prevColPatch = 0;
                }   // end if

                // Use previous patch if no patch at this pixel or the depth of ptch is >= prevColPatch
                if ( ptch == 0 || (prevColPatch != 0 && ptch->depth >= prevColPatch->depth))
                    ptch = prevColPatch;

                float depth = 0;
                if ( ptch != 0)
                {
                    // The x,y rectangle coordinates of the patch need flipping because they
                    // actually reference the bottom right corner of the patch.
                    ptch->flipAnchor( rows, cols); // Doesn't flip if already done so

                    if (ptch != 0)
                    {
                        updateImagePatchRefs( ptch, i, j, patches);
                        depth = ptch->depth;
                    }   // end if
                }   // end if

                outRow[j] = depth;
            }   // end for - cols
        }   // end for - rows

        return outImg;
    }   // end process

private:
    cv::Mat_<const PatchInfo*> _imgPatches; // Pointers to patches at pixels
    cv::Mat_<float> _outImg;
};  // end class




AdaptiveDepthMinFilter::AdaptiveDepthMinFilter( const cv::Mat_<float> depthMap, const cv::Size2f& rps)
    : _rngImg(depthMap), _rpatchSz(rps)
{ }   // end ctor


cv::Mat_<float> AdaptiveDepthMinFilter::filter( float minRng, float maxRng)
{
    list<const PatchInfo*> patchList;
    DownScanner downScanner( _rngImg, _rpatchSz, patchList);
    const cv::Mat_<float> downResult = downScanner( minRng, maxRng);

    const cv::Mat_<PatchInfo*> downImgPatches = downScanner.getImagePatches();
  
    // Flip around the X and Y axis (rotate 180 degrees)
    cv::Mat_<float> upInput;
    cv::flip( downResult, upInput, -1);
    cv::Mat_<PatchInfo*> upImgPatches;
    cv::flip( downImgPatches, upImgPatches, -1);

    UpScanner upScanner( upImgPatches, upInput);
    const cv::Mat_<float> upResult = upScanner.process();
    cv::Mat_<float> outputImg;
    cv::flip( upResult, outputImg, -1);  // Rotate the right way up again

    // Delete the patches created by DownScanner
    for ( list<const PatchInfo*>::const_iterator i = patchList.begin(); i != patchList.end(); ++i)
        delete *i;

    return outputImg;
}   // end filter

