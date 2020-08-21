#ifndef	_CCL_BSP_SOLID_OPERATION
#define	_CCL_BSP_SOLID_OPERATION

#define UINT	unsigned int
#define UCHAR	unsigned char

#include <stdio.h>
#include "PMBody.h"
#include "BSPTreeStructure.h"
#include "../GLKLib/GLKObList.h"
#define CPU_TREETRAVEL_THREAD_NUMBER	64

class BSPSolidOperation
{
public:
	BSPSolidOperation(void);
	~BSPSolidOperation(void);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Main Functions for construction and operations
	static void BSPTreeConstructionFromBRep(QuadTrglMesh *mesh, BSPTREE *&treePtr, float boundingBox[],
		int maxLevelAllowed = -1, bool bOrthogonalClipping = true);
	static void BooleanOperation(BSPTREE *treeAptr, float solidABndBox[],
		BSPTREE *treeBptr, float solidBBndBox[], short nOperationType,		// nOperationType:	0 -- union
																			//					1 -- intersection
																			//					2 -- subtraction
																			//					3 -- inversed-subtraction
		BSPTREE *&treeResPtr, float resBndBox[]);
	static void BSPTreeCollapse(BSPTREE *treePtr);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Main Functions for transformation and scaling
	static void BSPSolidTranslation(BSPTREE *treePtr, float boundingBox[], float tx, float ty, float tz);
	static void BSPSolidFlipping(short nDir, BSPTREE *treePtr, float boundingBox[]);
	static void BSPSolidRotation(BSPTREE *treePtr, float boundingBox[], float axisVec[], float angle);
	static void BSPSolidScaling(BSPTREE *treePtr, float boundingBox[], float ratio);
	//--------------------------------------------------------------------------------------------------------
	static void cudaBSPSolidTranslation(BSPTREE *treePtr, float boundingBox[], float tx, float ty, float tz);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//	Sevice Functions
	//
	//--------------------------------------------------------------------------------------------------------
	static void BPTFileImport(BSPTREE *&treePtr, char *filename, float boundingBox[]);
	static void BPTFileExport(BSPTREE *treePtr, char *filename, float boundingBox[]);
	static void BSPFileImport(BSPTREE *&treePtr, char *filename, float boundingBox[], bool bWithPrint = true);
	static void BSPFileExport(BSPTREE *treePtr, char *filename, float boundingBox[]);
	static void CopyDevTreeToHost(BSPTREE *&hostTreePtr, BSPTREE *devTreePtr);
	static void CopyHostTreeToDev(BSPTREE *&devTreePtr, BSPTREE *hostTreePtr);
	//--------------------------------------------------------------------------------------------------------
	static void StatisticsOfBSPTree(BSPTREE *treePtr);
	static int CountLevelNumber(BSPTREE *treePtr, int currentNodeID = 1, int nodeLevel = 0);
	//--------------------------------------------------------------------------------------------------------
	static bool IsPntInsideBSPSolid(BSPTREE *treePtr, float pnt[]);
	static void _bspTreeFreeMemory(BSPTREE *&ptrBSPTree);
private:
	//------------------------------------------------------------------------------------------------------------------------------
	//	Functions for B-rep to BSP conversion
	static void _constructBSPNode(BSPTREEArrayNode *treeNode, BSPTREEArrayNode *bspTreeNodeArray, UINT &bspTreeNodeArraySize,
		int level, float lastClippingPlane[],
		int trglNum, TRGLForBSP *trglArray, BSPTREEArrayPlane *bspPlaneArray, UINT &bspPlaneArraySize,
		float boundingBox[], int maxLevelAllowed, bool bOrthogonalClipping);
	static bool _splittingTrgl(TrglForBSP *trgl, BSPTREEArrayPlane *bspPlaneArray, float aa, float bb, float cc, float dd, short &nStatus,
		TrglForBSP *&trgl1, bool &trgl1Above, 					//	nStatus -	0 (on the plane) 
		TrglForBSP *&trgl2, bool &trgl2Above, 					//				1 (below the plane)	
		TrglForBSP *&trgl3, bool &trgl3Above);					//				2 (above the plane)

																//////////////////////////////////////////////////////////////////////////////////////////////////////////
																//	Functions for tree collapse
	static UINT _bspSubTreeCollapse(BSPTREE *treePtr, UINT currentNodeID,
		GLKArray **bspHashingArray, UINT *newIDArray, UINT &newIDCounter);
	static bool _isSameSubTreeExist(GLKArray **bspHashingArray, UINT planeID, UINT leftChildID, UINT rightChildID, int &indexInHashingTable);
	//------------------------------------------------------------------------------------------------------------------------------
	static void _removeRedundancyOnTree(BSPTREE *treePtr);
	static void _specifyVisitFlagOfTreeNodesPlanes(BSPTREE *treePtr, UINT currentNodeID, bool *bPlaneVisited, bool *bNodeVisited);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Functions for Boolean operation
	static void _booleanOperationOnBoundingBox(float solidABndBox[], float solidBBndBox[], short nOperationType, float resBndBox[]);
	//------------------------------------------------------------------------------------------------------------------------------
	static UINT	_booleanOperationByNaiveMerge(BSPTREE *treeAptr, BSPTREE *treeBptr, short nOperationType, BSPTREE *&treeResPtr);
	static void	_expandMergedTree(BSPTREE *treePtr);
	static UINT _expandMergedTreeSubRoutine(UINT indexInMergedTree, BSPTREE *treePtr, UINT &currentNum, BSPTREEArrayNode* newNodeArray);
	static void	_reduceMergedTree(float computingBndBox[], BSPTREE *treePtr, UINT stIndexForFeasibleCheck = 0);
	static UINT _reduceMergedTreeSubRoutine(UINT indexInMergedTree, BSPTREE *treePtr, UINT &currentNum, BSPTREEArrayNode* newNodeArray,
		UINT stackSize, BSPTREETravelStackNode *stackForTraversal, float computingBndBox[], float feaPnt[], UINT stIndexForFeasibleCheck);
	//------------------------------------------------------------------------------------------------------------------------------
	static bool _detectOverlapping(float computingBndBox[], BSPTREE *treeAptr, BSPTREETravelStackNode *stackForTreeA, UINT stackSizeA,
		BSPTREE *treeBptr, BSPTREETravelStackNode *stackForTreeB, UINT stackSizeB);
	static bool _detectFeasibleRegion(float computingBndBox[], BSPTREE *trPtr, BSPTREETravelStackNode *stackForTreeTraversal, UINT stackSize, float feaPnt[]);
	static bool _linearProgrammingBySeidelMethod(unsigned int dimension, unsigned int constraintsnum, double constraints[],
		double objectivevector[], double lowerbound[], double upperbound[], double result[]);
	static bool _linearProgrammingBySeidelMethodFor3D(unsigned int constraintsnum, double constraints[],
		double objectivevector[], double lowerbound[],
		double upperbound[], double result[]);
	//------------------------------------------------------------------------------------------------------------------------------
	static bool _recomputeFeasiblePntByLinearProgramming(float computingBndBox[], BSPTREE *trPtr,	//	If NOT feasible, the reture will be FALSE.	
		BSPTREETravelStackNode *stackForTreeTraversal, UINT stackSize, float feaPnt[]);	//	The last plane in stack is the newly added one.
																						//------------------------------------------------------------------------------------------------------------------------------
																						//	The following function -- "_recomputeFeasibleByLPwithPrior" -- compute the feasible point with the prior that:
																						//	
																						//		If the input feaPnt[] is not feasible, the infeasibility is ONLY led by the newly added half-space.
																						//		Based on this prior, the new feaPnt[] only need to be found on this newly added plane.
																						//		If no such point can be found on the plane, the region defined by this set of half-spaces is infeasible.
																						//	
																						//------------------------------------------------------------------------------------------------------------------------------
	static bool _recomputeFeasibleByLPwithTreePrior(float computingBndBox[], BSPTREE *trPtr,		//	If NOT feasible, the reture will be FALSE.	
		BSPTREETravelStackNode *stackForTreeTraversal, UINT stackSize, float feaPnt[]);	//	The last plane in stack is the newly added one.
	static bool _clippingPolygonByPlane(float *pnts, int pntsNum, float abcd[], float stPnt[], float edPnt[]);
	static bool _clippingSegmentByPlane(float stPnt[], float edPnt[], float abcd[], float &resAlpha, bool &bStPntAbove, bool &bEdPntAbove);
	static void _determineHalfSpaceByIndex(BSPTREETravelStackNode *stackForTreeTraversal, BSPTREE *trPtr, UINT index, float abcd[]);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Service Functions for tree manipulation
	static void _compBoundingBox(int trglNum, TrglForBSP *trglArray, float bndBox[]);
	static bool _compPlaneEquation(float p1[], float p2[], float p3[], float &aa, float &bb, float &cc, float &dd);
	static float _compArea(float p1[], float p2[], float p3[]);
	//------------------------------------------------------------------------------------------------------------------------------
	static void _bptNodeImport(BSPTREEArrayNode *currentNode, GLKArray *bspPlaneArray, GLKArray *bspTreeNodeArray, FILE *fp);
	static void _bptNodeExport(BSPTREE *treePtr, int currentNodeID, FILE *fp);
	//------------------------------------------------------------------------------------------------------------------------------
	static void _countNodeNumberInDifferentLevels(BSPTREE *treePtr, int *nodeNumInLevels, int currentNodeID = 1, int currentLevel = 0);
	static int _countLeafNodeNumberInDifferentLevels(BSPTREE *treePtr, int currentNodeID = 1);
};

#endif
