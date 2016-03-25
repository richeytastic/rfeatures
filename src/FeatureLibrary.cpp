#include "FeatureLibrary.h"
using RFeatures::FeatureLibrary;
#include "ImageType.h"
#include <algorithm>
#include <cassert>
#include <fstream>


FeatureLibrary::Ptr FeatureLibrary::S_singleton;    // Static singleton


// public static
vector<FX> FeatureLibrary::readFXs( const std::string& featCfgFile)
{
    FeatureLibrary::Ptr fxlib = FeatureLibrary::get();
    std::ifstream ifs( featCfgFile.c_str());
    vector<FX> fxs;
    fxlib->build( ifs, fxs);
    ifs.close();
    return fxs;
}   // end readFXs



// public static
FeatureLibrary::Ptr FeatureLibrary::get()
{
    if ( S_singleton == NULL)
        S_singleton = FeatureLibrary::Ptr( new FeatureLibrary);
    return S_singleton;
}   // end get



void FeatureLibrary::registerFX( FX fx)
{
    string fxString = fx->getTypeString();
    std::transform( fxString.begin(), fxString.end(), fxString.begin(), ::tolower);   // make lower case
    _fxMap[fxString] = fx;
}   // end registerFX



FX FeatureLibrary::build( const string& fxspec) const throw (ExtractorTypeException)
{
    std::istringstream iss(fxspec);
    string fxType, paramStr;
    iss >> fxType;
    std::transform( fxType.begin(), fxType.end(), fxType.begin(), ::tolower);
    ImageType imgType = RFeatures::parseImageType( iss);
    std::getline( iss, paramStr);

    if ( !_fxMap.count(fxType))
        throw ExtractorTypeException( "[EXCEPTION] FeatureLibrary::build: Unregistered feature extractor type!");

    // Check if the requested feature extractor
    vector<ImageType> vimgTypes;
    _fxMap.at(fxType)->getValidImageTypes( vimgTypes);
    bool foundImageType = false;
    for ( int i = 0; i < (int)vimgTypes.size(); ++i)
    {
        if ( imgType == vimgTypes[i])
        {
            foundImageType = true;
            break;
        }   // end if
    }   // end for

    if (!foundImageType)
    {
        const string simgType = RFeatures::toString( imgType);
        const string fxString = _fxMap.at(fxType)->getTypeString();
        throw ExtractorTypeException( "[EXCEPTION] FeatureLibrary::build: "
                + fxString + " feature extractors do not accept " + simgType + " image types!");
    }   // end if

    _fxMap.at(fxType)->setImageType( imgType);
    return _fxMap.at(fxType)->createNew( paramStr);   // May throw if paramStr no good
}   // end build



size_t FeatureLibrary::build( const vector<string>& fxspecs, vector<FX>& fxs) const throw (ExtractorTypeException)
{
    size_t fxcount = 0;
    BOOST_FOREACH ( const string& fxspec, fxspecs)
    {
        // Ignore empty lines or lines starting with '#'
        if ( fxspec.empty() || fxspec[0] == '#')
            continue;

        fxs.push_back( build( fxspec));
        fxcount++;
    }   // end for
    return fxcount;
}   // end build



size_t FeatureLibrary::build( istream& is, vector<FX>& fxs) const throw (ExtractorTypeException)
{
    size_t fxcount = 0;
    string fxspec;
    while ( std::getline( is, fxspec))
    {
        // Ignore empty lines or lines starting with '#'
        if ( fxspec.empty() || fxspec[0] == '#')
            continue;

        FX fx = build( fxspec);
        assert( fx != NULL);
        fxs.push_back( fx);
        fxcount++;
    }   // end while
    return fxcount;
}   // end build
