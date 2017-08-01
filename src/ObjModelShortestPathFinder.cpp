#include <ObjModelShortestPathFinder.h>
#include <DijkstraShortestPathFinder.h>
#include <ObjModelPolyUnfolder.h>
#include <ObjModelFaceAngleCalculator.h>
#include <cassert>
using RFeatures::DijkstraShortestPathFinder;
using RFeatures::ObjModelShortestPathFinder;
using RFeatures::ObjModelPolyUnfolder;
using RFeatures::ObjModelFaceAngleCalculator;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;


// Get the other integer from fids that isn't fid, or just return the singular entry if fids.size() == 1.
int getOther( const ObjModel::Ptr model, int ev0, int ev1, int fid)
{
    const IntSet& fids = model->getSharedFaces( ev0, ev1);
    assert( fids.size() <= 2);
    int fid2 = *fids.begin();
    if ( fids.size() > 1 && fid2 == fid)
        return *(++fids.begin());    // Get the other
    return fid2;
}   // end getOther


// Returns the sum of the inner angles of the sequence of triangles or 0 if the sequence didn't complete
// which implies that the sequence would have to be > 1 triangle in order to complete. P should be
// set to the triangle ID from the pair on the edge given by ev0-->ev1 that acts as the start barrier.
// That is, the sequence is generated from triangle T where triangles T and P are the two triangles that
// share edge ev0-->ev1. In some cases, edge ev0-->ev1 will only be associated with a single triangle,
// in which case P should be set as -1 (since triangle IDs start at 0).
// On return, tseq will have always had at least one entry appended to it unless P was set to the ID
// of the sole triangle associated with edge ev0-->ev1 (shouldn't happen).
double getEdgePairTriangleSequence( const ObjModel::Ptr model, int ev0, int ev1, int ev2, std::vector<int>& tseq, int P)
{
    const ObjModelFaceAngleCalculator faceAngle(model);
    double angleSum = 0.0; // Sum of angles in face sequence tseq

    int T = -1;
    int v2 = ev0;
    while ( v2 != ev2)
    {
        T = getOther( model, v2, ev1, P);
        if ( T == P)
            break;

        angleSum += faceAngle( T, ev1);
        tset.push_back(T);
        v2 = model->getFace(T).getOpposite( v2, ev1);
        P = T;
    }   // end while

    // If v2 isn't ev2, then the sequence of triangles failed to join the edges and we need to
    // check the sequence starting from the other triangle associated with edge ev0-->ev1.
    if ( v2 != ev2 && !tseq.empty())
    {
        tseq.erase( (++tseq.begin()), tseq.end());  // Keep just the first triangle
        angleSum = 0.0;
    }   // end if

    return angleSum;
}   // end getEdgePairTriangleSequence


// Get the sequence of triangles on the triangulated mesh that are associated with the edge sequence ev0-->ev1-->ev2
// so that the sum of the inner angles of these triangles is < PI. If no joining face sequence exists (due to ev1 being
// an "incomplete" edge vertex), the adjoining faces are identified. Returns the number of triangles appended to tseq.
int getLeastAngleSumTriangleSet( const ObjModel::Ptr model, int ev0, int ev1, int ev2, std::vector<int>& tseq)
{
    std::vector<int> tseq0, tseq1;
    const IntSet& sfids = model->getSharedFaces( ev0, ev1);
    if ( sfid.size() == 1)
        getEdgePairTriangleSequence( model, ev0, ev1, ev2, tseq0, -1);
    else
    {
        int P1 = *(++sfids.begin());
        double asum = getEdgePairTriangleSequence( model, ev0, ev1, ev2, tseq0, P1);    // tseq0 starts with P0
        assert( !tseq0.empty());

        // If tseq0 is a single triangle, it either doesn't cover the edge, or it does (and its angle must be < CV_PI).
        // In either case, it needs to be at the end of tseq.
        if ( tseq.empty() || tseq.back() != tseq0[0])
            tseq.push_back(tseq0);



        if ( asum == 0.0 || asum >= CV_PI)
        {
            int P0 = *sfids.begin();
            asum = getEdgePairTriangleSequence( model, ev0, ev1, ev2, tseq1, P0);   // tseq1 starts with P1
        }   // end if

    }   // end else
}   // end getLeastAngleSumTriangleSet



void findFaceSequence( const ObjModel::Ptr model, const std::vector<int>& vidxs, std::vector<int>& faceSeq)
{
    const int nvs = (int)vidxs.size();
    int ev1 = vidxs[1];
    int ev0 = vidxs[0];

    for ( int i = 2; i < nvs; ++i)
    {
        int ev1 = vidxs[i];
        int ev0 = vidxs[i-1];

    }   // end for
}	// end findFaceSequence


ObjModelShortestPathFinder::ObjModelShortestPathFinder( const ObjModelKDTree::Ptr kdt) : _kdtree {}


int ObjModelShortestPathFinder::operator()( const cv::Vec3f& v0, const cv::Vec3f& v1, std::vector<cv::Vec3f>& pts) const
{
    // Create the path by moving the vertices found via Dijkstra's shortest path within polygons surfaces.
    const ObjModel::Ptr model = _kdtree->getObject();
    DijkstraShortestPathFinder dspf( model);
    // Find the starting vertices of the endpoints
    const int vidx0 = _kdtree->find(v0);
    const int vidx1 = _kdtree->find(v1);
    dspf.setEndPointVertexIndices( vidx0, vidx1);
    std::vector<int> vidxs;
    dspf.findShortestPath( vidxs);

	// Unfold all the polygons around the shortest path.
	ObjModelPolyUnfolder polyUnfolder(model, faceSeq[0]);
	const int npolys = (int)faceSeq.size();
	for ( int i = 1; i < npolys; ++i)
	{
		polyUnfolder.unfoldAlongEdge( faceSeq[i], ev0, ev1);
	}	// end for

	// Find the specific polygon sequence around the vertex path that we 
	std::vector<int> faceSeq;
	findFaceSequence( model, vidxs, faceSeq);
	



    pts.push_back(v0);

    // Find the viable traversable faces between vidx0 and the next path edge
    IntSet facePathSet;

    cv::Vec3f p = model->projectToPoly( fid, v);
    pts.push_back(p);

    pts.push_back(v1);
}   // end operator()


int ObjModelShortestPathFinder::operator()( int v0, int v1, std::vector<cv::Vec3f>& pts) const
{
    const ObjModel::Ptr model = _kdtree->getObject();
	const cv::Vec3f& vec0 = model->vtx(v0);
	const cv::Vec3f& vec1 = model->vtx(v1);
	return operator()( vec0, vec1, pts);
}	// end operator()


