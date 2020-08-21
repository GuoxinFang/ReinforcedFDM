#ifndef	_CCL_VOX_SET_OPERATION
#define	_CCL_VOX_SET_OPERATION

#include "VOXSetStructure.h"
#include "BSPTreeStructure.h"
#include "../QMeshLib/PolygenMesh.h"
#include "../Library/PQPLib/PQP.h"


class GLKArray;
class QuadTrglMesh;

typedef struct convexHullSet {
	int faceNum;	double *normalVec;	double *offset;
	int vertNum;	double *vertPos;
	unsigned int *faceTable;	//	Note that: the index starts from '1'
}CONVEXHULLSET;

class VOXSetOperation
{
public:
	VOXSetOperation(void);
	~VOXSetOperation(void);

	static VOXELSET* ConstructVoxelSetFromBSPSolid(BSPTREE *bspTree, float boundingBox[], int res);
	static VOXELSET* ConstructVoxelSetFromBSPSolid(BSPTREE *bspTree, float boundingBox[], float width, bool bClippingByPlatform);
	static void FreeVoxelSet(VOXELSET *voxSet);

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//  The following function generates the layer decomposition by convex-peeling governed convex-growing
	static int ComputeConvexPeelingGovenedConvexGrowingAMOrder(VOXELSET *voxSet, short nPrintDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename = NULL);

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//  The following function generates the layer decomposition by convex-peeling
	static int ComputeConvexPeelingOrder(VOXELSET *voxSet, short nPrintDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename = NULL);

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//  The following function has the critical convex-analysis added
	static int ComputeAdditiveManufacturingOrder3(VOXELSET *voxSet, short nPrintDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename = NULL);

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//  The following function has separated the following steps: 1) layer decomposition and 2) in-layer tool-path planning
	static int ComputeAdditiveManufacturingOrder2(VOXELSET *voxSet, short nPrintDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename = NULL);

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//  The following function contains the first implementation of convex-front growing
	static int ComputeAdditiveManufacturingOrder(VOXELSET *voxSet, short nPrintDir, int baseTolerance, char *filename,
		short nStrategy, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename = NULL);
	//	strategy:	1 - the basic flooding scheme by the accumulation neighbor table
	//				2 - the convex front advancing scheme with single convex-front
	//				... ...
	//				
	//	return:		the number of fabricated voxels

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//  The following function contains the tetrahedral field governed growing order calculation - for SIGGRAPH 2020 project, written by Guoxin Fang 10.26.2019
	static int ComputeTetrahedralFieldGovenedGrowingAMOrder(VOXELSET *voxSet, QuadTrglMesh *platformMesh, short nDir);

	static int GetLayerNumber(VOXELSET *voxSet);
	static bool	OutputFabricationInfoFile(VOXELSET *voxSet, UINT *fabVoxOrder, float *fabOrients, UINT *fabVoxLayerIndex, int fabVoxNum, char *filename);
	static bool	InputFabricationInfoFile(UINT *&fabVoxOrder, float *&fabOrients, UINT *&fabVoxLayerIndex, int &fabVoxNum, float origin[], UINT res[], float &width, char *filename);

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	static int AddingSupportingStructure(VOXELSET *voxSet);

private:
	//-----------------------------------------------------------------------------------------------------------------------
	//	The simplest motion in a curved layer
	static void _motionPlanningInLayerPrimary(VOXELSET *voxSet, short nPrintDir, GLKArray *currentFront, UINT currentLayerIndex,
		float *fabOrients, UINT *fabVoxOrder, UINT *fabVoxLayerIndex, int &fabVoxNum);
	//-----------------------------------------------------------------------------------------------------------------------
	//	More advanced motion in a curved layer 
	static void _motionPlanningInLayer(VOXELSET *voxSet, unsigned int *voxGrid, short nPrintDir, GLKArray *currentFront, UINT currentLayerIndex,
		CONVEXHULLSET *pConvexHull, float *fabOrients, UINT *fabVoxOrder, UINT *fabVoxLayerIndex, int &fabVoxNum);
	static void _boundaryVoxelPlanning(VOXELSET *voxSet, unsigned int *voxGrid, UINT currentLayerIndex, GLKArray *boundaryOfFab);
	static void _determineOrientByReliableMaterialAccumulation(VOXELSET *voxSet, unsigned int *voxGrid, short nPrintDir, UINT currentLayerIndex,
		VOXELSETNode *currentNode, float orient[]);
	static void _determineOrientByClosestPntOnConvexFront(VOXELSET *voxSet, VOXELSETNode *currentNode, CONVEXHULLSET *pConvexHull, float orient[]);
	static void _tuningOrient(float orient[], float materialAccuPreferredDir[]);

	//-----------------------------------------------------------------------------------------------------------------------
	//	Functions for determine the voxels in the next layer
	static void _searchVoxelsInNextLayerByFlooding(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
		GLKArray *currentFront, GLKArray *nextFront, int layerCounter, bool bConventional);

	static void _searchVoxelsInNextLayerByStressFieldGuideFlooding(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
		GLKArray *currentFront, GLKArray *nextFront, int layerCounter, int stressFieldCounter, bool bConventional);

	static void _searchVoxelsInNextLayerByConvexFront(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
		GLKArray *currentFront, CONVEXHULLSET *currentConvexFront, GLKArray *nextFront, int layerCounter, double distTolerance);
	static bool _searchVoxelsInNextLayerByConvexFrontAndGovernField(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
		unsigned int *voxGovernFieldValue, unsigned int currentIsoValue,
		GLKArray *currentFront, CONVEXHULLSET *currentConvexFront, GLKArray *nextFront, int layerCounter, double distTolerance);
	static void _searchVoxelsOnBoundary(VOXELSET *voxSet, unsigned int *voxGrid, GLKArray *resBndVoxSet);

	//-----------------------------------------------------------------------------------------------------------------------
	//	Functions for convex-hull computation
	static CONVEXHULLSET* _constructNewConvexFront(VOXELSET *voxSet, GLKArray *newVoxFront, CONVEXHULLSET *currentConvexFront, QuadTrglMesh *platformMesh = NULL);
	static CONVEXHULLSET* _mallocMemoryConvexHull(int faceNum, int vertNum);
	static void _freeMemoryConvexHull(CONVEXHULLSET *&pConvexHull);     // after releasing the memory, NULL will be assigned to pConvexHull
	static bool _isPntInsideConvexHull(CONVEXHULLSET *pConvexHull, double pnt[]);
	static bool _isConvexHullCritical(VOXELSET *voxSet, CONVEXHULLSET *pConvexHull);
	static int _countNumOfCriticalVoxel(VOXELSET *voxSet, CONVEXHULLSET *pCurrentConvexFront);
	static double _compDistanceToConvexFront(double pos[], CONVEXHULLSET *pCurrentConvexFront);
	static void _findVoxelsOnConvexFront(GLKArray *inputVoxSet, CONVEXHULLSET *pCurrentConvexFront, GLKArray *resVoxSet, double thresholdDist, VOXELSET *voxSet);

	//-----------------------------------------------------------------------------------------------------------------------
	//	For clustering a curved player into separated regions
	static void	_floodingRegionInCurvedLayer(VOXELSET *voxSet, unsigned int *voxGrid, bool *bVoxFlag, VOXELSETNode *seedNode,
		UINT currentLayerIndex, int voxelNumInLayer, GLKArray *regionOfFab);

	static bool _isVoxelExisting(unsigned int *voxGrid, UINT res[], int i, int j, int k);
	static bool _isBoundaryVoxel(unsigned int *voxGrid, UINT res[], int i, int j, int k);

	static VOXELSETNode* _getNodeByIndex(VOXELSET *voxSet, unsigned int *voxGrid, int ii, int jj, int kk);
	static void _indexToIJK(UINT res[], int index, int &i, int &j, int &k);
	static int _ijkToIndex(UINT res[], int i, int j, int k);
	static void _ijkToPosition(VOXELSET *voxSet, int i, int j, int k, float pos[]);

	//-----------------------------------------------------------------------------------------------------------------------
	//	For outputting information to take robot-assisted manufacturing
	static bool _outputConvexFrontFile(CONVEXHULLSET *currentConvexFront, char *convexFileName);
	static bool _outputLayerPathFile(int currentLayerIndex, int fabVoxNum, float *fabOrients, UINT *fabVoxOrder, VOXELSET *voxSet, unsigned int *voxGrid, char *layerPathFileName);
	static void _mapCoordIntoRoboCoord(float xx, float yy, float zz, float &outputX, float &outputY, float &outputZ);

public:
	//------------------------------------------------------------------------------------------------------------------------
	//  The following function is written by Guoxin Fang
	static void voxelVisualization(PolygenMesh* voxelMesh, VOXELSET *voxSet);
	static void voxelVisualization_layerUpdate(PolygenMesh* voxelMesh, VOXELSET *voxSet);
	static void voxelVisualizationwithPlatform(PolygenMesh* voxelMesh, VOXELSET *voxSet, QMeshPatch* platform);
	static void layerInformationUpdate(PolygenMesh* voxelMesh, VOXELSET *voxSet);
	static void transformVoxeltoMeshField(VOXELSET *voxSet, QMeshPatch *patch);
	static void transformMeshFieldValuetoVoxel(VOXELSET *voxSet, QMeshPatch *tetPatch, QMeshPatch *voxelPatch);
	static void expandingVoxelSet(VOXELSET *voxSet);

	static int ComputeAdditiveManufacturingOrderwithShadowPrevention(VOXELSET *voxSet, short nDir, int baseTolerance, QuadTrglMesh *platformMesh);
	//void seperateVoxelSetHalfbyPCA(VOXELSET *voxSet, GLKArray *processVoxelSet, std::vector<int> &left_index, std::vector<int> &right_index);
	static int checkifVoxelSetgetShadowed(VOXELSET *voxSet, GLKArray *checkVoxelSet,
		CONVEXHULLSET *currentConvexFront, QuadTrglMesh *platformMesh, double tolerance);

private:
	static bool _isVoxelExisting(VOXELSET *voxSet, int ii, int jj, int kk);
	static bool _isVoxelExisting_layerupdate(VOXELSET *voxSet, int ii, int jj, int kk);
	static void getVoxelPosbyIndex(VOXELSET *voxSet, VOXELSETNode *currentNode, double x, double y, double z);
	
	static bool detectVoxelInsideTetrahedral(Eigen::Vector3d &nodePos, Eigen::MatrixXd &tetPos);
	static bool directSearchMethodfromVoxeltoTet(QMeshPatch *tetPatch, Eigen::Vector3d &voxelPos);
	static double ScTP(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c);
	static double computeVoxelValuebyTetrahedral(Eigen::Vector3d &nodePos, Eigen::MatrixXd &tetPos, Eigen::Vector4d &tetValue);
	static bool detectVoxelOutsideMesh(Eigen::Vector3d &nodePos, PQP_Model *pqpModel, QMeshPatch *tetPatch);
	static double computeVoxelValuebyNearestTriangle(Eigen::Vector3d &nodePos, PQP_Model *pqpModel, QMeshPatch *tetPatch);
	//------------------------------------------------------------------------------------------------------------------------
};

const int neighborNum = 18;
const int neighborDelta[][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,-1,0 },{ 0,0,-1 },{ 0,0,1 },
{ -1,1,0 },{ -1,-1,0 },{ 1,-1,0 },{ 1,1,0 },
{ -1,0,1 },{ -1,0,-1 },{ 1,0,-1 },{ 1,0,1 },
{ 0,-1,1 },{ 0,-1,-1 },{ 0,1,-1 },{ 0,1,1 } };
const float neighborDist[] = { 1,1,1,1,1,1,
1.414214f, 1.414214f, 1.414214f, 1.414214f,
1.414214f, 1.414214f, 1.414214f, 1.414214f,
1.414214f, 1.414214f, 1.414214f, 1.414214f };

const int faceNeighborNum = 6;
const int faceNeighborDelta[][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,-1,0 },{ 0,0,-1 },{ 0,0,1 } };		// This is the most safe strategy

#endif
