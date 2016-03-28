#include <VectorDistribution.h>
using RFeatures::VectorDistribution;
#include <iostream>
using std::cerr;
using std::endl;
#include <cstdlib>
#include <Random.h>



void printVecs( const vector<float>& vec, const float* pb)
{
    const int sz = vec.size();
    for ( int i = 0; i < sz; ++i)
        cerr << "[" << vec[i] << ":" << pb[i] << "] ";
    cerr << endl;
}   // end printVecs



int main( int argc, char** argv)
{
    const int numVs = 2000000;
    const int numBins = 10;
    const int vlen = 10;

    rlib::Random rndGen;

    VectorDistribution vd( vlen, numBins, 0, 1);

    // Construct the vectors and send to the vector distribution
    for ( int i = 0; i < numVs; ++i)
    {
        vector<float> vec(vlen);
        for ( int j = 0; j < vlen; ++j)
        {
            vec[j] = 1;
            while ( vec[j] == 1)
                vec[j] = rndGen.getRandom();  // Between 0 and 1
        }   // end for
        vd.addVector(vec);
    }   // end for
    cerr << "Created distribution" << endl;

    // Test 10 vectors
    for ( int i = 0; i < 10; ++i)
    {
        vector<float> vec(vlen);
        for ( int j = 0; j < vlen; ++j)
            vec[j] = rndGen.getRandom();  // Between 0 and 1

        float pb[vlen];
        vd.getLikelihood( vec, pb);

        printVecs( vec, pb);
    }   // end for

    return EXIT_SUCCESS;
}   // end main
