#include "FeaturePreProcessor.h"
using RFeatures::FeaturePreProcessor;


FeaturePreProcessor::Ptr FeaturePreProcessor::preProcessViews( const vector<FX>& fxs, vector<Instance>& instances, ProgressDelegate* pd)
{
    FeaturePreProcessor::Ptr fxpp( new FeaturePreProcessor( fxs));
    fxpp->process( instances, pd);  // Blocks if pd is NULL
    return fxpp;
}   // end preProcessViews



// private
FeaturePreProcessor::FeaturePreProcessor( const vector<FX>& fxs)
    : _fxs(fxs) // Feature extractor template objects copied in
{
}   // end ctor



FeaturePreProcessor::~FeaturePreProcessor()
{
    _threadGrp.join_all();
}   // end dtor



void processViewFxs( const string& viewId, const View::Ptr view,
                              boost::unordered_map<string, vector<FX> >& viewFxs,
                              const vector<FX>& fxs)
{
    BOOST_FOREACH ( const FX fx, fxs)
        viewFxs[viewId].push_back( fx->preProcess( view));
}   // end processViewFxs



// boost thread function
void FeaturePreProcessor::threadFunc( Instance* instances, int numInstances, ProgressDelegate* pd)
{
    for ( int i = 0; i < numInstances; ++i)
    {
        Instance& instance = *instances;

        { boost::mutex::scoped_lock lock(_mutex);
            if ( !_viewFxs.count(instance.viewId))   // Pre-process feature extractors for this view
                processViewFxs( instance.viewId, instance.view, _viewFxs, _fxs);
            instance.fxs = _viewFxs.at(instance.viewId);  // Copy in vector contents
        }   // end lock

        instances++;

        if ( pd != NULL)
        {
            const float propComp = float(i+1)/numInstances;
            pd->updateProgress( propComp);
        }   // end if
    }   // end for
}   // end threadFunc



// public
void FeaturePreProcessor::process( vector<Instance>& instances, ProgressDelegate* pd)
{
    _viewFxs.clear();

    const int numInstances = instances.size();

    if ( pd == NULL)
    {
        threadFunc( &instances[0], numInstances, NULL); // blocks
        return;
    }   // end if

    _threadGrp.join_all();  // Ensure previous operation (if any) finished

    const int numThreads = pd->getNumThreads(); // How many threads do we want to use?
    const int segSz = numInstances / numThreads;
    int segRm = numInstances % numThreads;

    // Start a bunch of new threads
    int stIdx = 0;
    for ( int i = 0; i < numThreads; ++i)
    {
        // Set the number of instances for this thread
        int sz = segSz;
        if ( segRm > 0)
        {
            sz++;
            segRm--;
        }   // end if

        //boost::thread* ithread = new boost::thread( boost::bind( &threadFunc, &instances[stIdx], sz, &_viewFxs, &_mutex, &_fxs, pd));
        boost::thread* ithread = new boost::thread( boost::bind( &FeaturePreProcessor::threadFunc, this, &instances[stIdx], sz, pd));
        _threadGrp.add_thread( ithread);

        stIdx += sz;    // For next round
    }   // end for
}   // end process

