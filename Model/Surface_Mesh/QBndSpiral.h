#ifndef _CCL_QBNDSPIRAL
#define _CCL_QBNDSPIRAL

#define TOL		1.0e-5

class QMeshPatch;
class PQP_Model;

class QBndSpiral
{
public:
	QBndSpiral(void);
	~QBndSpiral(void);

	static double GenerateBndDistMap(QMeshPatch *mesh, int patchID=-1);			//	Discrete version - by Multiple Source Dijekstra algorithm
	static double GenerateBndDistMap2(QMeshPatch *mesh, short refinedNodeNum);	//	Approximate version - by refined edge-nodes

	static void GenerateHarmonicMap(QMeshPatch *mesh, bool bCot=false);			//	Generate Harmonic field	(the value of anchor points has been given in the "BoundaryDist")

	static QMeshPatch* GenerateIsocurves(QMeshPatch *mesh, double maxValue, double offsetValue, int patchID=-1);
	static QMeshPatch* GenerateSpiralCurves(QMeshPatch *mesh, double maxValue, double offsetValue, int patchID=-1, bool bWithSmoothing=true);	// by particle movement
	static QMeshPatch* GenerateSpiralCurves2(QMeshPatch *mesh, double maxValue, double offsetValue, int patchID=-1, bool bWithSmoothing=true);	// by Geodesic path

	static double CompAverageEdgeLength(QMeshPatch *mesh);
	static bool AnalyzeBndDistMap(QMeshPatch *mesh, double maxValue, double offsetValue, int maxPatchID);

	static int CheckMaxPatchID(QMeshPatch *mesh);

private:
	static bool _generateIsocurve(QMeshPatch *mesh, double isoValue, GLKObList *contourNodeList, int patchID=-1);	
					//	return value -	true: all nodes are linked in a contour with simple topology
					//					false: the contour has complex topology
	static void _reIndexNodeOnContourByStartPoint(GLKObList *contourNodeList, double closestPntToStart[]);
	static void _createEdgesByNodeList(GLKObList *nodeList, GLKObList *edgeList, bool bPeriodic);
	static void _resamplingContourCurve(GLKObList *contourNodeList, int pntsNum, double* &xPos, double* &yPos, double* &zPos);
	static void _resamplingContourCurve(GLKObList *contourNodeList, int pntsNum, double* &xPos, double* &yPos, double* &zPos, QMeshFace **&faceArray);
	static void _smoothingSpiralCurve(GLKObList *contourNodeList, double relaxationFactor, int iterNum);
	static bool _pushingPntToPlaceWithGivenBDMValue(double targetFieldValue, double inputPnt[], QMeshFace *currentFace);
	static void _saveCurveFile(GLKObList *contourNodeList, char *filename);

	static void _pqpInitialization(QMeshPatch *mesh);
	static QMeshFace* _pqpClosestPntQuery(double queryPnt[], double closestPt[], double &dist);
	static void _pqpMemoryRelease();

	static bool _attachCotLaplacianWeight(QMeshPatch *mesh);
	static void _releaseCotLaplacianWeight(QMeshPatch *mesh);
	static bool _cotLaplacianWeight(QMeshNode *node, QMeshEdge *edge, double &cotA, double &cotB);
	static double _nodeVoronoiArea(QMeshNode *node);
	static double _squareDist(const double ver1[3], const double ver2[3]);
	static double _cotan(const double len, const double len1, const double len2, const bool bSqLen/*false*/);
	static double _area(const double ver1[3], const double ver2[3], const double ver3[3]);
	static double _cosine(const double len, const double len1, const double len2, const bool bSqLen/*false*/);

	static void _3DCoordToBarycentricCoord(double pos[], double ver0[], double ver1[], double ver2[], double &alpha, double &beta);
	static void _barycentricCoordTo3DCoord(double alpha, double beta, double ver0[], double ver1[], double ver2[], double pos[]);
};
#endif