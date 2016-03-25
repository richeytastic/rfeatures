// public static
template <typename T>
int NFoldCalculator<T>::calcValidationFoldSet( int foldi, int NFOLDs, const std::vector<T>& vin, std::vector<T>& vout)
{
    int vi, vr;
    calcValidationFoldRange( vin.size(), foldi, NFOLDs, vi, vr);
    for ( int i = vi; i < (vi+vr); ++i)
        vout.push_back(vin[i]);
    return vr;
}   // end calcValidationFoldSet


// public static
template <typename T>
int NFoldCalculator<T>::calcTrainingFoldSet( int foldi, int NFOLDs, const std::vector<T>& vin, std::vector<T>& vout)
{
    int t0, r0, t1, r1;
    calcTrainingFoldRange( vin.size(), foldi, NFOLDs, t0, r0, t1, r1);
    for ( int i = t0; i < (t0+r0); ++i)
        vout.push_back(vin[i]);
    for ( int i = t1; i < (t1+r1); ++i)
        vout.push_back(vin[i]);
    return r1 + r0;
}   // end calcTrainingFoldSet


// protected
template <typename T>
void NFoldCalculator<T>::mapToView( int viewId, const T& x)
{
    _viewExs[viewId].push_back(x);
}   // end mapToView

// protected
template <typename T>
void NFoldCalculator<T>::mapToView( int viewId, const std::vector<T>& xs)
{
    _viewExs[viewId].insert( _viewExs[viewId].end(), xs.begin(), xs.end());
}   // end mapToView


// private
template <typename T>
int NFoldCalculator<T>::getFromViewRange( int si, int ri, std::vector<T>& exs) const
{
    int addedCount = 0;
    for ( int i = si; i < si+ri; ++i)
    {
        const std::vector<T>& exsi = _viewExs.at(i); // Extract IDs for this view
        exs.insert( exs.end(), exsi.begin(), exsi.end());
        addedCount += exsi.size();
    }   // end for
    return addedCount;
}   // end getFromViewRange


// public
template <typename T>
int NFoldCalculator<T>::getTrainingExs( int foldi, int NFOLDs, std::vector<T>& exs) const
{
    int t0, r0, t1, r1;
    calcTrainingFoldRange( _viewExs.size(), foldi, NFOLDs, t0, r0, t1, r1);
    int addCnt = getFromViewRange( t0, r0, exs);
    addCnt += getFromViewRange( t1, r1, exs);
    return addCnt;
}   // end getTrainingExs


// public
template <typename T>
int NFoldCalculator<T>::getValidationExs( int foldi, int NFOLDs, std::vector<T>& exs) const
{
    int vi, vr;
    calcValidationFoldRange( _viewExs.size(), foldi, NFOLDs, vi, vr);
    return getFromViewRange( vi, vr, exs);
}   // end getValidationExs
