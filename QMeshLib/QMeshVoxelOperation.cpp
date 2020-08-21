#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "PMBody.h"

#include "../GLKLib/GLKGraph.h"
#include "../GLKLib/GLKGeometry.h"

#include "../Library/QHull/qhull_a.h"

#include "QMeshVoxelOperation.h"
#include "BSPTreeOperation.h"

#include "BSTree.h"
#include <omp.h>


#define MAXALLOWED_TUNINGANGLE_ON_NOZZLE	30

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

#define MIN(a,b)	((a)<(b))?(a):(b)
#define MAX(a,b)	((a)>(b))?(a):(b)

#define CROSS_PRODUCT(vec1,vec2,resVec)		{	resVec[0]=vec1[1]*vec2[2]-vec1[2]*vec2[1];	\
												resVec[1]=vec1[2]*vec2[0]-vec1[0]*vec2[2];	\
												resVec[2]=vec1[0]*vec2[1]-vec1[1]*vec2[0];	}
#define DOT_PRODUCT(vec1,vec2)				(vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2])

#define DISTANCE(vec1,vec2)	((vec1[0]-vec2[0])*(vec1[0]-vec2[0])+(vec1[1]-vec2[1])*(vec1[1]-vec2[1])+(vec1[2]-vec2[2])*(vec1[2]-vec2[2]))

VOXSetOperation::VOXSetOperation(void)
{
}

VOXSetOperation::~VOXSetOperation(void)
{
}

int VOXSetOperation::ComputeConvexPeelingGovenedConvexGrowingAMOrder
	(VOXELSET *voxSet, short nDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename)
{
	unsigned int *voxGrid;  unsigned int *voxGovernFieldValue;     int i, temp;         FILE *fp = NULL;
	GLKArray *currentFront, *nextFront;		UINT layerCounter;
	float *fabOrients;		UINT *fabVoxOrder, *fabVoxLayerIndex;		int fabVoxNum;
	CONVEXHULLSET *currentConvexFront, *nextConvexFront;
	//  double thresholdDist=0.86602540378*(double)(voxSet->width);
	double thresholdDist = (double)(voxSet->width);

	if (platformMesh == NULL) {
		printf("Error: the peeling based convex front computation cannot find a platform mesh file!\n");
		return 0;
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of arrays and grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	voxGovernFieldValue = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->nodeNum);
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);  // 0 value is specified for those places with no voxel
	memset(voxGovernFieldValue, 0, sizeof(unsigned int)*voxSet->nodeNum);                         // 0 value is specified for undefined field value
	for (i = 0; i<voxSet->nodeNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res,
			voxSet->nodeArray[i].posIndex[0],
			voxSet->nodeArray[i].posIndex[1],
			voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}
	//---------------------------------------------------------------------------------------------------------------
	temp = voxSet->m_res[(nDir + 1) % 3] * voxSet->m_res[(nDir + 2) % 3];
	currentFront = new GLKArray(temp, temp, 0);
	nextFront = new GLKArray(temp, temp, 0);
	//---------------------------------------------------------------------------------------------------------------
	fabOrients = (float*)malloc(sizeof(float)*(voxSet->nodeNum) * 3);
	fabVoxOrder = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum) * 3);
	fabVoxLayerIndex = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum));
	//---------------------------------------------------------------------------------------------------------------
	if (nDir<0) nDir = 0;
	if (nDir>2) nDir = 2;

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Find the seed region of peeling - the last layer of materials
	printf("--------------------------------------------------------------------\n");
	currentConvexFront = nextConvexFront = NULL;    currentFront->RemoveAll();  nextFront->RemoveAll();
	for (i = 0; i<voxSet->nodeNum; i++) { voxSet->nodeArray[i].layerIndex = 0; }
	_searchVoxelsOnBoundary(voxSet, voxGrid, nextFront);
	printf("The boundary layer has %d voxels\n", nextFront->GetSize());
	currentConvexFront = _constructNewConvexFront(voxSet, nextFront, NULL, platformMesh);
	_findVoxelsOnConvexFront(nextFront, currentConvexFront, currentFront, thresholdDist, voxSet);
	nextFront->RemoveAll();
	//printf("Initial front has %d voxels\n",currentFront->GetSize());
	printf("--------------------------------------------------------------------\n");

	//---------------------------------------------------------------------------------------------------------------
	//	Step 3: Layer decomposition and store the result by specifying the value of "layerIndex" for "nodeArray[...]"
	layerCounter = 1;
	while (currentFront->GetSize()>0) {
		printf("Peeling Layer %d: the num of voxels is %d\n", (int)layerCounter, currentFront->GetSize());

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: Assign the "layerIndex" of voxels on the current-front
		for (i = 0; i<currentFront->GetSize(); i++) {
			VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));
			currentNode->layerIndex = layerCounter;
		}

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Search voxels in the next front
		currentFront->RemoveAll();
		_searchVoxelsOnBoundary(voxSet, voxGrid, currentFront);
		if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
		currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
		_findVoxelsOnConvexFront(currentFront, currentConvexFront, nextFront, thresholdDist, voxSet);

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 3: Assign the current-front by the next-front
		GLKArray *tempArray = currentFront;
		currentFront = nextFront;     nextFront = tempArray;    nextFront->RemoveAll();

		layerCounter++;
	}
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	//  Inverse the order of layers by converting peeling into growing
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].layerIndex == 0) continue;
		voxSet->nodeArray[i].layerIndex = layerCounter - (voxSet->nodeArray[i].layerIndex);
	}
	//---------------------------------------------------------------------------------------------------------------
	printf("Total layer number: %d\n", layerCounter - 1);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 4: Convex-growing based on the governing field values
	printf("\n\nComputing the layers by convex-growing\n");
	printf("-----------------------------------------------------\n");
	//---------------------------------------------------------------------------------------------------------------
	//  Sub-Step 1: fill field-value and other initialization steps
	for (i = 0; i<voxSet->nodeNum; i++) { voxGovernFieldValue[i] = voxSet->nodeArray[i].layerIndex; }
	currentFront->RemoveAll();      nextFront->RemoveAll();
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	if (nextConvexFront != NULL) _freeMemoryConvexHull(nextConvexFront);
	if (platformMesh != NULL) currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
	unsigned int maxIsovalue = layerCounter;
	//---------------------------------------------------------------------------------------------------------------
	//  Sub-Step 2: find the seed region of flooding - the first layer of materials
	UINT lowest = voxSet->m_res[nDir];
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir]<lowest) lowest = voxSet->nodeArray[i].posIndex[nDir];
		voxSet->nodeArray[i].layerIndex = 0;    // stands for the voxels that have not been processed
	}
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir] <= lowest) {
			currentFront->Add(&(voxSet->nodeArray[i]));
			voxSet->nodeArray[i].layerIndex = 1;
		}
	}
	//---------------------------------------------------------------------------------------------------------------
	//  Sub-Step 3: convex-growing based layer decomposition -- store the result by specifying the value of "layerIndex" for "nodeArray[...]"
	unsigned int currentIsoValue = 3;
	layerCounter = 1;
	while (currentIsoValue <= maxIsovalue) {
		printf("Current Iso-value=%d\n", (int)currentIsoValue);
		while (currentFront->GetSize()>0) {
			//-----------------------------------------------------------------------------------------------------------
			//  Search voxels in the next front
			bool bIncremental = _searchVoxelsInNextLayerByConvexFrontAndGovernField(voxSet, nDir, voxGrid,
				voxGovernFieldValue, currentIsoValue,
				currentFront, currentConvexFront, nextFront, layerCounter, thresholdDist);

			//-----------------------------------------------------------------------------------------------------------
			//  When no voxel is found in the next front, the iso-value needs to be increased.
			if (!bIncremental) break;

			//-----------------------------------------------------------------------------------------------------------
			//  Update the new convex-front
			if (currentConvexFront != NULL) {
				//-------------------------------------------------------------------------------------------------------
				// As the platform mesh has been included in previous convex-front, it becomes redundancy here and can be neglected.
				//	"nextConvexFront=_constructNewConvexFront(voxSet, nextFront, currentConvexFront, platformMesh);"
				nextConvexFront = _constructNewConvexFront(voxSet, nextFront, currentConvexFront, NULL);
				_freeMemoryConvexHull(currentConvexFront);
				currentConvexFront = nextConvexFront;
			}
			else {
				//-------------------------------------------------------------------------------------------------------
				// To avoid input co-planar points, the first convex-front must be formed by two layers
				for (i = 0; i<nextFront->GetSize(); i++) { currentFront->Add(nextFront->GetAt(i)); }
				currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
			}

			//-----------------------------------------------------------------------------------------------------------
			//  Switch the current-front and the next-front
			GLKArray *tempArray = currentFront;
			currentFront = nextFront;     nextFront = tempArray;    nextFront->RemoveAll();

			layerCounter++;
		}

		currentIsoValue += 3;
	}
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 5: Generate the information of fabrication - including the motion planning in layer
	//              (storing the information into "fabVoxOrder,fabOrients,fabVoxLayerIndex,fabVoxNum")
	
	//---------------------------------------------------------------------------------------------------------------
	//if (resRoboAMFilename != NULL) {
	//	char projFileName[1000];    sprintf(projFileName, "%s%s.amproj", folderLocation, resRoboAMFilename);
	//	fp = fopen(projFileName, "w");
	//}
	////---------------------------------------------------------------------------------------------------------------
	//fabVoxNum = 0;
	//for (UINT nlayerIndex = 1; nlayerIndex <= layerCounter; nlayerIndex++) {
	//	currentFront->RemoveAll();
	//	//-----------------------------------------------------------------------------------------------------------
	//	//  Sub-Step 1: search voxels belonging to the current layer and update the convex front
	//	for (i = 0; i<voxSet->nodeNum; i++) {
	//		if (voxSet->nodeArray[i].layerIndex == nlayerIndex) currentFront->Add(&(voxSet->nodeArray[i]));
	//	}
	//	nextConvexFront = _constructNewConvexFront(voxSet, currentFront, currentConvexFront, platformMesh);
	//	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//	currentConvexFront = nextConvexFront;	nextConvexFront = NULL;
	//	//-----------------------------------------------------------------------------------------------------------
	//	if (resRoboAMFilename != NULL) {
	//		if (currentConvexFront != NULL) {
	//			char convexFileName[1000];
	//			fprintf(fp, "%s%d.cov\n", resRoboAMFilename, nlayerIndex);
	//			sprintf(convexFileName, "%s%s%d.cov", folderLocation, resRoboAMFilename, nlayerIndex);
	//			_outputConvexFrontFile(currentConvexFront, convexFileName);
	//		}
	//		else
	//			fprintf(fp, "NULL\n");
	//	}
	//	//-----------------------------------------------------------------------------------------------------------
	//	//  Sub-Step 2: Processing voxels in the current front - e.g., determine the following information
	//	//		1) the order of material accumulation
	//	//		2) the preferred orientation of nozzle
	//	int beforeFabVoxNum = fabVoxNum;
	//	_motionPlanningInLayer(voxSet, voxGrid, nDir, currentFront, nlayerIndex, currentConvexFront, fabOrients, fabVoxOrder, fabVoxLayerIndex, fabVoxNum);
	//	printf("Motion planning for layer %d with %d voxels\n", (int)nlayerIndex, (int)(currentFront->GetSize()));
	//	//-----------------------------------------------------------------------------------------------------------
	//	if (resRoboAMFilename != NULL) {
	//		if (fabVoxNum != beforeFabVoxNum) {
	//			char layerPathFileName[1000];
	//			fprintf(fp, "%s%d.lph\n", resRoboAMFilename, nlayerIndex);
	//			sprintf(layerPathFileName, "%s%s%d.lph", folderLocation, resRoboAMFilename, nlayerIndex);
	//			_outputLayerPathFile(layerCounter, fabVoxNum - beforeFabVoxNum, &(fabOrients[beforeFabVoxNum * 3]), &(fabVoxOrder[beforeFabVoxNum * 3]), voxSet, voxGrid, layerPathFileName);
	//		}
	//		else
	//			fprintf(fp, "NULL\n");
	//	}
	//}
	//if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	////---------------------------------------------------------------------------------------------------------------
	//fprintf(fp, "TOTAL_LAYER_NUM: %d\n", layerCounter - 1);
	//fclose(fp);
	//printf("%d voxels have been fabricated\n", fabVoxNum);

	////---------------------------------------------------------------------------------------------------------------
	////	Step 6: Output file for the information of fabrication
	
	//OutputFabricationInfoFile(voxSet, fabVoxOrder, fabOrients, fabVoxLayerIndex, fabVoxNum, filename);
	
	//---------------------------------------------------------------------------------------------------------------
	//	Step 7: Free the memory
	free(voxGrid);          free(voxGovernFieldValue);
	delete currentFront;	delete nextFront;
	//---------------------------------------------------------------------------------------------------------------
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	free(fabOrients);		free(fabVoxOrder);		free(fabVoxLayerIndex);

	printf("\n------------------------------------------------------------------\n");
	printf("%d voxels can be fabricated in %d curved-layers\n", fabVoxNum, (int)layerCounter);
	if (fabVoxNum != voxSet->nodeNum) {
		printf("\n================================================\n");
		printf("Warning: some voxels cannot be fabricated!\n");
		printf("================================================\n\n");
	}

	return fabVoxNum;
}

int VOXSetOperation::ComputeTetrahedralFieldGovenedGrowingAMOrder
	(VOXELSET *voxSet, QuadTrglMesh *platformMesh, short nDir) {

	unsigned int *voxGrid;  unsigned int *voxGovernFieldValue;     int i, temp;         FILE *fp = NULL;
	GLKArray *currentFront, *nextFront;		UINT layerCounter;
	float *fabOrients;		UINT *fabVoxOrder, *fabVoxLayerIndex;		int fabVoxNum;
	CONVEXHULLSET *currentConvexFront, *nextConvexFront;
	//  double thresholdDist=0.86602540378*(double)(voxSet->width);
	double thresholdDist = (double)(voxSet->width);

	if (platformMesh == NULL) {
		printf("Error: the peeling based convex front computation cannot find a platform mesh file!\n");
		return 0;
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of arrays and grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	voxGovernFieldValue = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->nodeNum);
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);  // 0 value is specified for those places with no voxel
	memset(voxGovernFieldValue, 0, sizeof(unsigned int)*voxSet->nodeNum);                         // 0 value is specified for undefined field value
	for (i = 0; i < voxSet->nodeNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res,
			voxSet->nodeArray[i].posIndex[0],
			voxSet->nodeArray[i].posIndex[1],
			voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}
	//---------------------------------------------------------------------------------------------------------------
	temp = voxSet->m_res[(nDir + 1) % 3] * voxSet->m_res[(nDir + 2) % 3];
	currentFront = new GLKArray(temp, temp, 0);
	nextFront = new GLKArray(temp, temp, 0);
	//---------------------------------------------------------------------------------------------------------------
	fabOrients = (float*)malloc(sizeof(float)*(voxSet->nodeNum) * 3);
	fabVoxOrder = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum) * 3);
	fabVoxLayerIndex = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum));
	//---------------------------------------------------------------------------------------------------------------
	if (nDir < 0) nDir = 0;
	if (nDir > 2) nDir = 2;

	//-------------------The initial voxel layer information is already transformed from tetrahedral mesh------------
	//for (i = 0; i < voxSet->nodeNum; i++) { voxGovernFieldValue[i] = voxSet->nodeArray[i].stressFieldLayerIndex; }

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Convex-growing based on the tetrahedral growing field values
	printf("\n\nBegin computing the layers by direct growing\n");
	printf("-----------------------------------------------------\n");
	//---------------------------------------------------------------------------------------------------------------
	//  Sub-Step 1: fill field-value and other initialization steps
	/*currentFront->RemoveAll();      nextFront->RemoveAll();
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	if (nextConvexFront != NULL) _freeMemoryConvexHull(nextConvexFront);
	if (platformMesh != NULL) currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
	unsigned int maxIsovalue = layerCounter;*/
	//---------------------------------------------------------------------------------------------------------------
	//  Sub-Step 2: find the seed region of flooding - the first layer of materials

	unsigned int currentIsoValue = 1;
	currentFront->RemoveAll();      nextFront->RemoveAll();

	UINT lowest = voxSet->m_res[nDir];
	for (i = 0; i < voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir] < lowest) lowest = voxSet->nodeArray[i].posIndex[nDir];
		voxSet->nodeArray[i].layerIndex = 0;    // stands for the voxels that have not been processed
	}

	for (i = 0; i < voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir] <= lowest) {
			currentFront->Add(&(voxSet->nodeArray[i]));
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	//  Sub-Step 3:

	int maxIsovalue = 50; layerCounter = 1;
	for (int i = 0; i < maxIsovalue; i++) {

		while (currentFront->GetSize() > 0) {
			_searchVoxelsInNextLayerByStressFieldGuideFlooding(voxSet, nDir, voxGrid, currentFront, nextFront, layerCounter, i+1, false);
			//-----------------------------------------------------------------------------------------------------------
			//  Switch the current-front and the next-front
			GLKArray *tempArray = currentFront;
			currentFront = nextFront;     nextFront = tempArray;    nextFront->RemoveAll();

			layerCounter++;
			//std::cout << "Now the size of current front is: " << currentFront->GetSize() << std::endl;
		}

		currentFront->RemoveAll();      nextFront->RemoveAll();
		for (int j = 0; j < voxSet->nodeNum; j++) {
			if (voxSet->nodeArray[j].stressFieldLayerIndex == i+1 ) {
				currentFront->Add(&(voxSet->nodeArray[j]));
			}
		}
		//std::cout << "The next section current front is: " << currentFront->GetSize() << std::endl;
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 7: Free the memory
	free(voxGrid);          free(voxGovernFieldValue);
	delete currentFront;	delete nextFront;
	
	printf("\n------------------------------------------------------------------\n");
	printf("%d voxels can be fabricated in %d curved-layers\n", fabVoxNum, (int)layerCounter);
	if (fabVoxNum != voxSet->nodeNum) {
		printf("\n================================================\n");
		printf("Warning: some voxels cannot be fabricated!\n");
		printf("================================================\n\n");
	}

	return fabVoxNum;

}

bool VOXSetOperation::_searchVoxelsInNextLayerByConvexFrontAndGovernField(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
	unsigned int *voxGovernFieldValue, unsigned int currentIsoValue,
	GLKArray *currentFront, CONVEXHULLSET *currentConvexFront, GLKArray *nextFront, int layerCounter, double distTolerance)
{
	bool bIncremental = false;
	int i, j, num, index, ii, jj, kk;	double pnt[3];

	//-------------------------------------------------------------------------------------------------------------------
	//	Rules for the voxels to be accumulated:
	//		Rule 1) - they must be the neighbors of the current voxel-front
	//		Rule 2) - these voxels must be outside the current convex-front to be collision-free
	//		Rule 3) - the field values of these voxels must be less or equal to the "currentIsoValue"
	num = currentFront->GetSize();
	for (i = 0; i < num; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));

		for (j = 0; j < neighborNum; j++) {
			ii = (int)(currentNode->posIndex[0]) + neighborDelta[j][0];
			jj = (int)(currentNode->posIndex[1]) + neighborDelta[j][1];
			kk = (int)(currentNode->posIndex[2]) + neighborDelta[j][2];
			VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid, ii, jj, kk);
			if (neighborNode == NULL || neighborNode->layerIndex != 0) // There is no voxel or the voxel has been processed.
				continue;
			index = (int)(voxGrid[_ijkToIndex(voxSet->m_res, ii, jj, kk)]) - 1;
			if (voxGovernFieldValue[index] > currentIsoValue) continue;

			if (currentConvexFront != NULL) {
				pnt[0] = voxSet->origin[0] + ((float)(neighborNode->posIndex[0]) + 0.5f)*(voxSet->width);
				pnt[1] = voxSet->origin[1] + ((float)(neighborNode->posIndex[1]) + 0.5f)*(voxSet->width);
				pnt[2] = voxSet->origin[2] + ((float)(neighborNode->posIndex[2]) + 0.5f)*(voxSet->width);
				if (_isPntInsideConvexHull(currentConvexFront, pnt)) {   // Checking if the rule no.2 is not obeyed.
					if (_compDistanceToConvexFront(pnt, currentConvexFront) > distTolerance)
						continue;
				}
			}

			nextFront->Add(neighborNode);	neighborNode->layerIndex = layerCounter + 1;
		}
	}

	if (nextFront->GetSize() > 0) {
		bIncremental = true;
		//-------------------------------------------------------------------------------------------------------------------
		//  Other boundary-voxels in the currentFront can also be the part of new front
		//
		//      Condition 1: on the boundary of already fabricated voxel-set
		//      Condition 2: not far-inside the current convex-front
		//
		//  Note that -- the condition 2 is used to reduce the number of voxels in the currentFront (i.e., without it will make the program slower -- need to be verified)
		//
		for (i = 0; i < num; i++) {
			VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));
			bool bBoundary = false;
			for (j = 0; j < neighborNum; j++) {
				ii = (int)(currentNode->posIndex[0]) + neighborDelta[j][0];
				jj = (int)(currentNode->posIndex[1]) + neighborDelta[j][1];
				kk = (int)(currentNode->posIndex[2]) + neighborDelta[j][2];
				VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid, ii, jj, kk);
				if (neighborNode == NULL || neighborNode->layerIndex == 0)  // There is no voxel or the voxel has NOT been processed.
				{
					bBoundary = true; break;
				}
			}
			if (bBoundary) {
				pnt[0] = voxSet->origin[0] + ((float)(currentNode->posIndex[0]) + 0.5f)*(voxSet->width);
				pnt[1] = voxSet->origin[1] + ((float)(currentNode->posIndex[1]) + 0.5f)*(voxSet->width);
				pnt[2] = voxSet->origin[2] + ((float)(currentNode->posIndex[2]) + 0.5f)*(voxSet->width);
				if (_compDistanceToConvexFront(pnt, currentConvexFront) < distTolerance*2.0)
					//if (_compDistanceToConvexFront(pnt, currentConvexFront)<distTolerance)
					nextFront->Add(currentNode);
			}
		}
	}

	return bIncremental;
}


int VOXSetOperation::ComputeConvexPeelingOrder
(VOXELSET *voxSet, short nDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename)
{
	unsigned int *voxGrid;	int i, temp;     FILE *fp = NULL;
	GLKArray *currentFront, *nextFront;		UINT layerCounter;
	float *fabOrients;		UINT *fabVoxOrder, *fabVoxLayerIndex;		int fabVoxNum;
	CONVEXHULLSET *currentConvexFront, *nextConvexFront;
	//  double thresholdDist=0.86602540378*(double)(voxSet->width);
	double thresholdDist = (double)(voxSet->width);

	if (platformMesh == NULL) {
		printf("Error: the peeling based convex front computation cannot find a platform mesh file!\n");
		return 0;
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of arrays and grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]); // 0 value is specified for those places with no voxel
	for (i = 0; i<voxSet->nodeNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res,
			voxSet->nodeArray[i].posIndex[0],
			voxSet->nodeArray[i].posIndex[1],
			voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}
	//---------------------------------------------------------------------------------------------------------------
	temp = voxSet->m_res[(nDir + 1) % 3] * voxSet->m_res[(nDir + 2) % 3];
	currentFront = new GLKArray(temp, temp, 0);
	nextFront = new GLKArray(temp, temp, 0);
	std::cout << temp << std::endl;
	//---------------------------------------------------------------------------------------------------------------
	fabOrients = (float*)malloc(sizeof(float)*(voxSet->nodeNum) * 3);
	fabVoxOrder = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum) * 3);
	fabVoxLayerIndex = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum));
	//---------------------------------------------------------------------------------------------------------------
	if (nDir<0) nDir = 0;
	if (nDir>2) nDir = 2;

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Find the seed region of peeling - the last layer of materials
	printf("--------------------------------------------------------------------\n");
	currentConvexFront = nextConvexFront = NULL;    currentFront->RemoveAll();  nextFront->RemoveAll();
	for (i = 0; i<voxSet->nodeNum; i++) { voxSet->nodeArray[i].layerIndex = 0; }
	_searchVoxelsOnBoundary(voxSet, voxGrid, nextFront);
	printf("The boundary layer has %d voxels\n", nextFront->GetSize());
	currentConvexFront = _constructNewConvexFront(voxSet, nextFront, NULL, platformMesh);
	_findVoxelsOnConvexFront(nextFront, currentConvexFront, currentFront, thresholdDist, voxSet);
	nextFront->RemoveAll();
	//printf("Initial front has %d voxels\n",currentFront->GetSize());
	printf("--------------------------------------------------------------------\n");

	//---------------------------------------------------------------------------------------------------------------
	//	Step 3: Layer decomposition and store the result by specifying the value of "layerIndex" for "nodeArray[...]"
	layerCounter = 1;
	while (currentFront->GetSize()>0) {
		printf("Peeling Layer %d: the num of voxels is %d\n", (int)layerCounter, currentFront->GetSize());

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: Assign the "layerIndex" of voxels on the current-front
		for (i = 0; i<currentFront->GetSize(); i++) {
			VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));
			currentNode->layerIndex = layerCounter;
		}

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Search voxels in the next front
		currentFront->RemoveAll();
		_searchVoxelsOnBoundary(voxSet, voxGrid, currentFront);
		if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
		currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
		_findVoxelsOnConvexFront(currentFront, currentConvexFront, nextFront, thresholdDist, voxSet);

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 3: Assign the current-front by the next-front
		GLKArray *tempArray = currentFront;
		currentFront = nextFront;     nextFront = tempArray;    nextFront->RemoveAll();

		layerCounter++;
	}
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	//  Inverse the order of layers by converting peeling into growing
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].layerIndex == 0) continue;
		voxSet->nodeArray[i].layerIndex = layerCounter - (voxSet->nodeArray[i].layerIndex);
	}
	//---------------------------------------------------------------------------------------------------------------
	printf("Total layer number: %d\n", layerCounter - 1);
	std::cout << voxSet->nodeNum << " Voxel in total" << std::endl << std::endl;
	//---------------------------------------------------------------------------------------------------------------
	//	Step 4: Generate the information of fabrication - including the motion planning in layer
	//              (storing the information into "fabVoxOrder,fabOrients,fabVoxLayerIndex,fabVoxNum")
	//---------------------------------------------------------------------------------------------------------------
	//if (resRoboAMFilename != NULL) {
	//	char projFileName[1000];    sprintf(projFileName, "%s%s.amproj", folderLocation, resRoboAMFilename);
	//	fp = fopen(projFileName, "w");
	//}
	////---------------------------------------------------------------------------------------------------------------
	//fabVoxNum = 0;
	//for (UINT nlayerIndex = 1; nlayerIndex <= layerCounter; nlayerIndex++) {
	//	currentFront->RemoveAll();
	//	//-----------------------------------------------------------------------------------------------------------
	//	//  Sub-Step 1: search voxels belonging to the current layer and update the convex front
	//	for (i = 0; i<voxSet->nodeNum; i++) {
	//		if (voxSet->nodeArray[i].layerIndex == nlayerIndex) currentFront->Add(&(voxSet->nodeArray[i]));
	//	}
	//	nextConvexFront = _constructNewConvexFront(voxSet, currentFront, currentConvexFront, platformMesh);
	//	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//	currentConvexFront = nextConvexFront;	nextConvexFront = NULL;
	//	//-----------------------------------------------------------------------------------------------------------
	//	if (resRoboAMFilename != NULL) {
	//		if (currentConvexFront != NULL) {
	//			char convexFileName[1000];
	//			fprintf(fp, "%s%d.cov\n", resRoboAMFilename, nlayerIndex);
	//			sprintf(convexFileName, "%s%s%d.cov", folderLocation, resRoboAMFilename, nlayerIndex);
	//			_outputConvexFrontFile(currentConvexFront, convexFileName);
	//		}
	//		else
	//			fprintf(fp, "NULL\n");
	//	}
	//	//-----------------------------------------------------------------------------------------------------------
	//	//  Sub-Step 2: Processing voxesl in the current front - e.g., determine the following information
	//	//		1) the order of material accumulation
	//	//		2) the preferred orientation of nozzle
	//	int beforeFabVoxNum = fabVoxNum;
	//	_motionPlanningInLayer(voxSet, voxGrid, nDir, currentFront, nlayerIndex, currentConvexFront, fabOrients, fabVoxOrder, fabVoxLayerIndex, fabVoxNum);
	//	printf("Motion planning for layer %d with %d voxels\n", (int)nlayerIndex, (int)(currentFront->GetSize()));
	//	//-----------------------------------------------------------------------------------------------------------
	//	if (resRoboAMFilename != NULL) {
	//		if (fabVoxNum != beforeFabVoxNum) {
	//			char layerPathFileName[1000];
	//			fprintf(fp, "%s%d.lph\n", resRoboAMFilename, nlayerIndex);
	//			sprintf(layerPathFileName, "%s%s%d.lph", folderLocation, resRoboAMFilename, nlayerIndex);
	//			_outputLayerPathFile(layerCounter, fabVoxNum - beforeFabVoxNum, &(fabOrients[beforeFabVoxNum * 3]), &(fabVoxOrder[beforeFabVoxNum * 3]), voxSet, voxGrid, layerPathFileName);
	//		}
	//		else
	//			fprintf(fp, "NULL\n");
	//	}
	//}
	//if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	////---------------------------------------------------------------------------------------------------------------
	//fprintf(fp, "TOTAL_LAYER_NUM: %d\n", layerCounter - 1);
	//fclose(fp);
	//printf("%d voxels have been fabricated\n", fabVoxNum);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 5: Output file for the information of fabrication
	//OutputFabricationInfoFile(voxSet, fabVoxOrder, fabOrients, fabVoxLayerIndex, fabVoxNum, filename);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 6: Free the memory
	free(voxGrid);
	delete currentFront;	delete nextFront;
	//---------------------------------------------------------------------------------------------------------------
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	free(fabOrients);		free(fabVoxOrder);		free(fabVoxLayerIndex);

	//printf("\n------------------------------------------------------------------\n");
	//printf("%d voxels can be fabricated in %d curved-layers\n", fabVoxNum, (int)layerCounter);
	//if (fabVoxNum != voxSet->nodeNum) {
	//	printf("\n================================================\n");
	//	printf("Warning: some voxels cannot be fabricated!\n");
	//	printf("================================================\n\n");
	//}

	return fabVoxNum;
}

int VOXSetOperation::ComputeAdditiveManufacturingOrder3(VOXELSET *voxSet, short nDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename)
{
	unsigned int *voxGrid;	int i, temp;     FILE *fp = NULL;
	GLKArray *currentFront, *nextFront;		UINT layerCounter;
	float *fabOrients;		UINT *fabVoxOrder, *fabVoxLayerIndex;		int fabVoxNum;
	CONVEXHULLSET *currentConvexFront, *nextConvexFront;
	const int baseTolerance = 0;
	double distTolerance = 0.86602540378*(voxSet->width);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of arrays and grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]); // 0 value is specified for those places with no voxel
	for (i = 0; i<voxSet->nodeNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res,
			voxSet->nodeArray[i].posIndex[0],
			voxSet->nodeArray[i].posIndex[1],
			voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}
	//---------------------------------------------------------------------------------------------------------------
	temp = voxSet->m_res[(nDir + 1) % 3] * voxSet->m_res[(nDir + 2) % 3];
	currentFront = new GLKArray(temp, temp, 0);
	nextFront = new GLKArray(temp, temp, 0);
	//---------------------------------------------------------------------------------------------------------------
	fabOrients = (float*)malloc(sizeof(float)*(voxSet->nodeNum) * 3);
	fabVoxOrder = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum) * 3);
	fabVoxLayerIndex = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum));
	//---------------------------------------------------------------------------------------------------------------
	if (nDir<0) nDir = 0;
	if (nDir>2) nDir = 2;
	//---------------------------------------------------------------------------------------------------------------
	currentConvexFront = nextConvexFront = NULL;
	if (platformMesh != NULL) currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Find the seed region of flooding - the first layer of materials
	UINT lowest = voxSet->m_res[nDir];
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir]<lowest) lowest = voxSet->nodeArray[i].posIndex[nDir];
		voxSet->nodeArray[i].layerIndex = 0;  // stands for the voxels that have not been processed
	}
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir] <= lowest + baseTolerance) {
			currentFront->Add(&(voxSet->nodeArray[i]));
			voxSet->nodeArray[i].layerIndex = 1;
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 3: Layer decomposition and store the result by specifying the value of "layerIndex" for "nodeArray[...]"
	layerCounter = 1;
	while (currentFront->GetSize()>0) {
		printf("Layer %d: the num of voxels is %d\n", (int)layerCounter, currentFront->GetSize());

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: Search voxels in the next front
		_searchVoxelsInNextLayerByConvexFront(voxSet, nDir, voxGrid, currentFront, currentConvexFront, nextFront, layerCounter, distTolerance);
		if (currentConvexFront != NULL) {
			//----------------------------------------------------------------------------------------------------------------------
			// As the platform mesh has been included in previous convex-front, it becomes redundancy here and can be neglected.
			//	"nextConvexFront=_constructNewConvexFront(voxSet, nextFront, currentConvexFront, platformMesh);"
			nextConvexFront = _constructNewConvexFront(voxSet, nextFront, currentConvexFront, NULL);

			_freeMemoryConvexHull(currentConvexFront);
			currentConvexFront = nextConvexFront;
		}
		else {
			//---------------------------------------------------------------------------------------------------
			// To avoid input co-planar points, the first convex-front must be formed by two layers
			for (i = 0; i<nextFront->GetSize(); i++) { currentFront->Add(nextFront->GetAt(i)); }
			currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
		}

		int nCriticalVoxelNum = _countNumOfCriticalVoxel(voxSet, currentConvexFront);

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Switch the current-front and the next-front
		GLKArray *tempArray = currentFront;
		currentFront = nextFront;     nextFront = tempArray;
		//---------------------------------------------------------------------------------------------
		nextFront->RemoveAll();

		layerCounter++;

		if (nCriticalVoxelNum>10) {
			printf("Warning: critical layer is found: with %d missed voxels\n", nCriticalVoxelNum);
			//            break;
		}
	}
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 4: Generate the information of fabrication - including the motion planning in layer
	//              (storing the information into "fabVoxOrder,fabOrients,fabVoxLayerIndex,fabVoxNum")
	//---------------------------------------------------------------------------------------------------------------
	if (resRoboAMFilename != NULL) {
		char projFileName[1000];    sprintf(projFileName, "%s%s.amproj", folderLocation, resRoboAMFilename);
		fp = fopen(projFileName, "w");
	}
	//---------------------------------------------------------------------------------------------------------------
	fabVoxNum = 0;
	for (UINT nlayerIndex = 1; nlayerIndex <= layerCounter; nlayerIndex++) {
		currentFront->RemoveAll();

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: search voxels belonging to the current layer and update the convex front
		for (i = 0; i<voxSet->nodeNum; i++) {
			if (voxSet->nodeArray[i].layerIndex == nlayerIndex) currentFront->Add(&(voxSet->nodeArray[i]));
		}
		nextConvexFront = _constructNewConvexFront(voxSet, currentFront, currentConvexFront, platformMesh);
		if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
		currentConvexFront = nextConvexFront;	nextConvexFront = NULL;
		//-----------------------------------------------------------------------------------------------------------
		if (resRoboAMFilename != NULL) {
			if (currentConvexFront != NULL) {
				char convexFileName[1000];
				fprintf(fp, "%s%d.cov\n", resRoboAMFilename, nlayerIndex);
				sprintf(convexFileName, "%s%s%d.cov", folderLocation, resRoboAMFilename, nlayerIndex);
				_outputConvexFrontFile(currentConvexFront, convexFileName);
			}
			else
				fprintf(fp, "NULL\n");
		}

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Processing voxels in the current front - e.g., determine the following information
		//		1) the order of material accumulation
		//		2) the preferred orientation of nozzle
		int beforeFabVoxNum = fabVoxNum;
		_motionPlanningInLayer(voxSet, voxGrid, nDir, currentFront, nlayerIndex, currentConvexFront, fabOrients, fabVoxOrder, fabVoxLayerIndex, fabVoxNum);
		printf("Motion planning for layer %d with %d voxels\n", (int)nlayerIndex, (int)(currentFront->GetSize()));
		//-----------------------------------------------------------------------------------------------------------
		if (resRoboAMFilename != NULL) {
			if (fabVoxNum != beforeFabVoxNum) {
				char layerPathFileName[1000];
				fprintf(fp, "%s%d.lph\n", resRoboAMFilename, nlayerIndex);
				sprintf(layerPathFileName, "%s%s%d.lph", folderLocation, resRoboAMFilename, nlayerIndex);
				_outputLayerPathFile(layerCounter, fabVoxNum - beforeFabVoxNum, &(fabOrients[beforeFabVoxNum * 3]), &(fabVoxOrder[beforeFabVoxNum * 3]), voxSet, voxGrid, layerPathFileName);
			}
			else
				fprintf(fp, "NULL\n");
		}
	}
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	fprintf(fp, "TOTAL_LAYER_NUM: %d\n", layerCounter - 1);
	fclose(fp);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 5: Output file for the information of fabrication
	OutputFabricationInfoFile(voxSet, fabVoxOrder, fabOrients, fabVoxLayerIndex, fabVoxNum, filename);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 6: Free the memory
	free(voxGrid);
	delete currentFront;	delete nextFront;
	//---------------------------------------------------------------------------------------------------------------
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	free(fabOrients);		free(fabVoxOrder);		free(fabVoxLayerIndex);

	printf("\n------------------------------------------------------------------\n");
	printf("%d voxels can be fabricated in %d curved-layers\n", fabVoxNum, (int)layerCounter);
	if (fabVoxNum != voxSet->nodeNum) {
		printf("\n================================================\n");
		printf("Warning: some voxels cannot be fabricated!\n");
		printf("================================================\n\n");
	}

	return fabVoxNum;
}

int VOXSetOperation::ComputeAdditiveManufacturingOrder2(VOXELSET *voxSet, short nDir, char *filename, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename)
{
	unsigned int *voxGrid;	int i, temp;     FILE *fp = NULL;
	GLKArray *currentFront, *nextFront;		UINT layerCounter;
	float *fabOrients;		UINT *fabVoxOrder, *fabVoxLayerIndex;		int fabVoxNum;
	CONVEXHULLSET *currentConvexFront, *nextConvexFront;
	const int baseTolerance = 0;
	double distTolerance = 0.86602540378*(voxSet->width);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of arrays and grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]); // 0 value is specified for those places with no voxel
	for (i = 0; i<voxSet->nodeNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res,
			voxSet->nodeArray[i].posIndex[0],
			voxSet->nodeArray[i].posIndex[1],
			voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}
	//---------------------------------------------------------------------------------------------------------------
	temp = voxSet->m_res[(nDir + 1) % 3] * voxSet->m_res[(nDir + 2) % 3];
	currentFront = new GLKArray(temp, temp, 0);
	nextFront = new GLKArray(temp, temp, 0);
	//---------------------------------------------------------------------------------------------------------------
	fabOrients = (float*)malloc(sizeof(float)*(voxSet->nodeNum) * 3);
	fabVoxOrder = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum) * 3);
	fabVoxLayerIndex = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum));
	//---------------------------------------------------------------------------------------------------------------
	if (nDir<0) nDir = 0;
	if (nDir>2) nDir = 2;
	//---------------------------------------------------------------------------------------------------------------
	currentConvexFront = nextConvexFront = NULL;
	if (platformMesh != NULL) currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Find the seed region of flooding - the first layer of materials
	UINT lowest = voxSet->m_res[nDir];
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir]<lowest) lowest = voxSet->nodeArray[i].posIndex[nDir];
		voxSet->nodeArray[i].layerIndex = 0;  // stands for the voxels that have not been processed
	}
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir] <= lowest + baseTolerance) {
			currentFront->Add(&(voxSet->nodeArray[i]));
			voxSet->nodeArray[i].layerIndex = 1;
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 3: Layer decomposition and store the result by specifying the value of "layerIndex" for "nodeArray[...]"
	layerCounter = 1;
	while (currentFront->GetSize()>0) {
		printf("Layer %d: the num of voxels is %d\n", (int)layerCounter, currentFront->GetSize());

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: Search voxels in the next front
		_searchVoxelsInNextLayerByConvexFront(voxSet, nDir, voxGrid, currentFront, currentConvexFront, nextFront, layerCounter, distTolerance);
		if (currentConvexFront != NULL) {
			//----------------------------------------------------------------------------------------------------------------------
			// As the platform mesh has been included in previous convex-front, it becomes redundancy here and can be neglected.
			//	"nextConvexFront=_constructNewConvexFront(voxSet, nextFront, currentConvexFront, platformMesh);"
			nextConvexFront = _constructNewConvexFront(voxSet, nextFront, currentConvexFront, NULL);

			_freeMemoryConvexHull(currentConvexFront);
			currentConvexFront = nextConvexFront;
		}
		else {
			//---------------------------------------------------------------------------------------------------
			// To avoid input co-planar points, the first convex-front must be formed by two layers
			for (i = 0; i<nextFront->GetSize(); i++) { currentFront->Add(nextFront->GetAt(i)); }
			currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
		}

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Switch the current-front and the next-front
		GLKArray *tempArray = currentFront;
		currentFront = nextFront;     nextFront = tempArray;
		//---------------------------------------------------------------------------------------------
		nextFront->RemoveAll();

		layerCounter++;
	}
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 4: Generate the information of fabrication - including the motion planning in layer
	//              (storing the information into "fabVoxOrder,fabOrients,fabVoxLayerIndex,fabVoxNum")
	//---------------------------------------------------------------------------------------------------------------
	if (resRoboAMFilename != NULL) {
		char projFileName[1000];    sprintf(projFileName, "%s%s.amproj", folderLocation, resRoboAMFilename);
		fp = fopen(projFileName, "w");
	}
	//---------------------------------------------------------------------------------------------------------------
	fabVoxNum = 0;
	for (UINT nlayerIndex = 1; nlayerIndex <= layerCounter; nlayerIndex++) {
		currentFront->RemoveAll();

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: search voxels belonging to the current layer and update the convex front
		for (i = 0; i<voxSet->nodeNum; i++) {
			if (voxSet->nodeArray[i].layerIndex == nlayerIndex) currentFront->Add(&(voxSet->nodeArray[i]));
		}
		nextConvexFront = _constructNewConvexFront(voxSet, currentFront, currentConvexFront, platformMesh);
		if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
		currentConvexFront = nextConvexFront;	nextConvexFront = NULL;
		//-----------------------------------------------------------------------------------------------------------
		if (resRoboAMFilename != NULL) {
			if (currentConvexFront != NULL) {
				char convexFileName[1000];
				fprintf(fp, "%s%d.cov\n", resRoboAMFilename, nlayerIndex);
				sprintf(convexFileName, "%s%s%d.cov", folderLocation, resRoboAMFilename, nlayerIndex);
				_outputConvexFrontFile(currentConvexFront, convexFileName);
			}
			else
				fprintf(fp, "NULL\n");
		}

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Processing voxesl in the current front - e.g., determine the following information
		//		1) the order of material accumulation
		//		2) the preferred orientation of nozzle
		int beforeFabVoxNum = fabVoxNum;
		_motionPlanningInLayer(voxSet, voxGrid, nDir, currentFront, nlayerIndex, currentConvexFront, fabOrients, fabVoxOrder, fabVoxLayerIndex, fabVoxNum);
		printf("Motion planning for layer %d with %d voxels\n", (int)nlayerIndex, (int)(currentFront->GetSize()));
		//-----------------------------------------------------------------------------------------------------------
		if (resRoboAMFilename != NULL) {
			if (fabVoxNum != beforeFabVoxNum) {
				char layerPathFileName[1000];
				fprintf(fp, "%s%d.lph\n", resRoboAMFilename, nlayerIndex);
				sprintf(layerPathFileName, "%s%s%d.lph", folderLocation, resRoboAMFilename, nlayerIndex);
				_outputLayerPathFile(layerCounter, fabVoxNum - beforeFabVoxNum, &(fabOrients[beforeFabVoxNum * 3]), &(fabVoxOrder[beforeFabVoxNum * 3]), voxSet, voxGrid, layerPathFileName);
			}
			else
				fprintf(fp, "NULL\n");
		}


	}
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	fprintf(fp, "TOTAL_LAYER_NUM: %d\n", layerCounter - 1);
	fclose(fp);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 5: Output file for the information of fabrication
	OutputFabricationInfoFile(voxSet, fabVoxOrder, fabOrients, fabVoxLayerIndex, fabVoxNum, filename);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 6: Free the memory
	free(voxGrid);
	delete currentFront;	delete nextFront;
	//---------------------------------------------------------------------------------------------------------------
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	free(fabOrients);		free(fabVoxOrder);		free(fabVoxLayerIndex);

	printf("\n------------------------------------------------------------------\n");
	printf("%d voxels can be fabricated in %d curved-layers\n", fabVoxNum, (int)layerCounter);
	if (fabVoxNum != voxSet->nodeNum) {
		printf("\n================================================\n");
		printf("Warning: some voxels cannot be fabricated!\n");
		printf("================================================\n\n");
	}

	return fabVoxNum;
}

int VOXSetOperation::ComputeAdditiveManufacturingOrder(VOXELSET *voxSet, short nDir, int baseTolerance, char *filename,
	short nStrategy, QuadTrglMesh *platformMesh, char *folderLocation, char *resRoboAMFilename)
{
	unsigned int *voxGrid;	int i, temp;     FILE *fp = NULL;
	GLKArray *currentFront, *nextFront;		UINT layerCounter;
	float *fabOrients;		UINT *fabVoxOrder, *fabVoxLayerIndex;		int fabVoxNum;
	CONVEXHULLSET *currentConvexFront, *nextConvexFront;
	double distTolerance = 0.86602540378*(voxSet->width);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of arrays and grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]); // 0 value is specified for those places with no voxel
	for (i = 0; i<voxSet->nodeNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res,
			voxSet->nodeArray[i].posIndex[0],
			voxSet->nodeArray[i].posIndex[1],
			voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}
	//---------------------------------------------------------------------------------------------------------------
	temp = voxSet->m_res[(nDir + 1) % 3] * voxSet->m_res[(nDir + 2) % 3];
	currentFront = new GLKArray(temp, temp, 0);
	nextFront = new GLKArray(temp, temp, 0);
	//---------------------------------------------------------------------------------------------------------------
	fabOrients = (float*)malloc(sizeof(float)*(voxSet->nodeNum) * 3);
	fabVoxOrder = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum) * 3);
	fabVoxLayerIndex = (UINT*)malloc(sizeof(UINT)*(voxSet->nodeNum));
	//---------------------------------------------------------------------------------------------------------------
	if (nDir<0) nDir = 0;
	if (nDir>2) nDir = 2;
	//---------------------------------------------------------------------------------------------------------------
	/*if (resRoboAMFilename != NULL) {
		char projFileName[1000];    sprintf(projFileName, "%s%s.amproj", folderLocation, resRoboAMFilename);
		fp = fopen(projFileName, "w");
	}*/
	//---------------------------------------------------------------------------------------------------------------
	currentConvexFront = nextConvexFront = NULL;
	if (platformMesh != NULL) currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
	//nextConvexFront = _constructNewConvexFront(voxSet, nextFront, currentConvexFront, NULL);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Find the seed region of flooding - the first layer of materials
	UINT lowest = voxSet->m_res[nDir];
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir]<lowest) lowest = voxSet->nodeArray[i].posIndex[nDir];
		voxSet->nodeArray[i].layerIndex = 0;  // stands for the voxels that have not been processed
	}
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir] <= lowest + baseTolerance) {
			currentFront->Add(&(voxSet->nodeArray[i]));
			voxSet->nodeArray[i].layerIndex = 1;
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 3: Progressively flooding the regions into the whole model
	layerCounter = 1;
	fabVoxNum = 0;
	while (currentFront->GetSize()>0) {
		printf("Layer %d: the num of voxels is %d\n", (int)layerCounter, currentFront->GetSize());

		//if (resRoboAMFilename != NULL) {
		//	if (currentConvexFront != NULL) {
		//		char convexFileName[1000];
		//		fprintf(fp, "%s%d.cov\n", resRoboAMFilename, layerCounter);
		//		sprintf(convexFileName, "%s%s%d.cov", folderLocation, resRoboAMFilename, layerCounter);
		//		//if (layerCounter==1 || layerCounter==76)
		//		_outputConvexFrontFile(currentConvexFront, convexFileName);
		//	}
		//	else
		//		fprintf(fp, "NULL\n");
		//}

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: Processing voxels in the current front - e.g., determine the following information
		//		1) the order of material accumulation
		//		2) the preferred orientation of nozzle
		int beforeFabVoxNum = fabVoxNum;
		//_motionPlanningInLayer(voxSet, voxGrid, nDir, currentFront, layerCounter, currentConvexFront, fabOrients, fabVoxOrder, fabVoxLayerIndex, fabVoxNum);
		/*if (resRoboAMFilename != NULL) {
			if (fabVoxNum != beforeFabVoxNum) {
				char layerPathFileName[1000];
				fprintf(fp, "%s%d.lph\n", resRoboAMFilename, layerCounter);
				sprintf(layerPathFileName, "%s%s%d.lph", folderLocation, resRoboAMFilename, layerCounter);
				_outputLayerPathFile(layerCounter, fabVoxNum - beforeFabVoxNum, &(fabOrients[beforeFabVoxNum * 3]), &(fabVoxOrder[beforeFabVoxNum * 3]), voxSet, voxGrid, layerPathFileName);
			}
			else
				fprintf(fp, "NULL\n");
		}*/

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Search voxels in the next front
		switch (nStrategy) {
		case 0: {
			_searchVoxelsInNextLayerByFlooding(voxSet, nDir, voxGrid, currentFront, nextFront, layerCounter, true);
		}break;
		case 1: {
			_searchVoxelsInNextLayerByFlooding(voxSet, nDir, voxGrid, currentFront, nextFront, layerCounter, false);
		}break;
		case 2: {
			_searchVoxelsInNextLayerByConvexFront(voxSet, nDir, voxGrid, currentFront, currentConvexFront, nextFront, layerCounter, distTolerance);
			//printf("layerCounter=%d\n",layerCounter);
			if (currentConvexFront != NULL) {
				//----------------------------------------------------------------------------------------------------------------------
				// As the platform mesh has been included in previous convex-front, it becomes redundancy here and can be neglected.
				//	"nextConvexFront=_constructNewConvexFront(voxSet, nextFront, currentConvexFront, platformMesh);"	
				nextConvexFront = _constructNewConvexFront(voxSet, nextFront, currentConvexFront, NULL);

				_freeMemoryConvexHull(currentConvexFront);
				currentConvexFront = nextConvexFront;
			}
			else {
				//---------------------------------------------------------------------------------------------------
				// To avoid input co-planar points, the first convex-front must be formed by two layers
				for (i = 0; i<nextFront->GetSize(); i++) { currentFront->Add(nextFront->GetAt(i)); }
				currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);
			}
		}break;
		}

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 3: Switch the current-front and the next-front
		GLKArray *tempArray = currentFront;
		currentFront = nextFront;     nextFront = tempArray;
		//---------------------------------------------------------------------------------------------
		nextFront->RemoveAll();

		layerCounter++;
	}
	/*fprintf(fp, "TOTAL_LAYER_NUM: %d\n", layerCounter - 1);
	fclose(fp);*/

	//---------------------------------------------------------------------------------------------------------------
	//	Step 4: Output file for the information of fabrication
	//OutputFabricationInfoFile(voxSet, fabVoxOrder, fabOrients, fabVoxLayerIndex, fabVoxNum, filename);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 5: Free the memory
	free(voxGrid);
	delete currentFront;	delete nextFront;
	//---------------------------------------------------------------------------------------------------------------
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);
	//---------------------------------------------------------------------------------------------------------------
	free(fabOrients);		free(fabVoxOrder);		free(fabVoxLayerIndex);

	printf("\n------------------------------------------------------------------\n");
	printf("%d voxels can be fabricated in %d curved-layers\n", fabVoxNum, (int)layerCounter);
	if (fabVoxNum != voxSet->nodeNum) {
		printf("\n================================================\n");
		printf("Warning: some voxels cannot be fabricated!\n");
		printf("================================================\n\n");
	}

	return fabVoxNum;
}

int VOXSetOperation::ComputeAdditiveManufacturingOrderwithShadowPrevention(VOXELSET *voxSet, short nDir, int baseTolerance, QuadTrglMesh *platformMesh)
{
	unsigned int *voxGrid;	int i, temp;     FILE *fp = NULL;
	GLKArray *currentFront, *nextFront;		UINT layerCounter;
	int fabVoxNum;
	CONVEXHULLSET *currentConvexFront, *nextConvexFront;
	double distTolerance = 0.86602540378*(voxSet->width);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of arrays and grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	
	// 0 value is specified for those places with no voxel
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]); 
	for (i = 0; i<voxSet->nodeNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res, voxSet->nodeArray[i].posIndex[0], voxSet->nodeArray[i].posIndex[1], voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}
	//---------------------------------------------------------------------------------------------------------------
	temp = voxSet->m_res[(nDir + 1) % 3] * voxSet->m_res[(nDir + 2) % 3];
	currentFront = new GLKArray(temp, temp, 0);  nextFront = new GLKArray(temp, temp, 0);
	GLKArray *shadowSafeVoxel = new GLKArray(temp, temp, 0);

	//---------------------------------------------------------------------------------------------------------------
	currentConvexFront = nextConvexFront = NULL;
	if (platformMesh != NULL) currentConvexFront = _constructNewConvexFront(voxSet, currentFront, NULL, platformMesh);

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Find the seed region of flooding - the first layer of materials
	UINT lowest = voxSet->m_res[nDir];
	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir]<lowest) lowest = voxSet->nodeArray[i].posIndex[nDir];
		voxSet->nodeArray[i].layerIndex = 0;  // stands for the voxels that have not been processed
	}
	std::cout << "Find the lowest layer is " << lowest << std::endl;

	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].posIndex[nDir] <= lowest + baseTolerance) {
			currentFront->Add(&(voxSet->nodeArray[i]));
			voxSet->nodeArray[i].layerIndex = 1;
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 3: Progressively flooding the regions into the whole model
	layerCounter = 1; int last_shadow_count = 0;
	while (currentFront->GetSize() > 0) {
		printf("Layer %d: the num of voxels is %d\n", (int)layerCounter, currentFront->GetSize());
		
		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 1: Search voxels in the next convex front and set them as processed.
		int i, j, num;	double pnt[3];

		//-------------------------------------------------------------------------------------------------------------------
		//	Rules for the voxels to be accumulated:
		//		Rule 1) - they must be the neighbors of the current voxel-front
		//		Rule 2) - these voxels must be outside the current convex-front to be collision-free
		
		for (i = 0; i<currentFront->GetSize(); i++) {

			VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));

			for (j = 0; j<neighborNum; j++) {
				VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid, (int)(currentNode->posIndex[0]) + neighborDelta[j][0],
					(int)(currentNode->posIndex[1]) + neighborDelta[j][1], (int)(currentNode->posIndex[2]) + neighborDelta[j][2]);

				if (neighborNode == NULL || neighborNode->layerIndex != 0) 	continue;
				if (currentConvexFront != NULL) {
					for (int k = 0; k < 3; k++) pnt[k] = voxSet->origin[k] + ((float)(neighborNode->posIndex[k]) + 0.5f)*(voxSet->width);
					if (_compDistanceToConvexFront(pnt, currentConvexFront) > 2 * distTolerance) continue;
				}
				nextFront->Add(neighborNode);	neighborNode->layerIndex = layerCounter + 1;
			}
		}


		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 2: Check if shadow voxel will apper in this step. If so, detect safe node.
		
		//GLKArray *shadowVoxelCandidate = new GLKArray(temp, temp, 0);
		int shadowPointNum = checkifVoxelSetgetShadowed(voxSet, nextFront, currentConvexFront, NULL, distTolerance);
		
		if (shadowPointNum > last_shadow_count) {
			//-----------------------------------------------------------------------------------------------------------
			//  Incrementally check the safe node and give it back to nextFront.
			bool incrementallyCheck = false;
			if (incrementallyCheck == true) {
				std::cout << "Shadow happen, shadowPointNum = " << shadowPointNum << ", progressively check the voxel in nextFront set!" << std::endl;
				CONVEXHULLSET *shadowConvexFront;
				for (int i = 0; i < nextFront->GetSize(); i++) {
					bool shadowHappen = false;
					shadowSafeVoxel->Add(nextFront->GetAt(i));
					int shadowPointNum_thisVoxel = checkifVoxelSetgetShadowed(voxSet, shadowSafeVoxel, currentConvexFront, NULL, distTolerance);
					if (shadowPointNum_thisVoxel > 0) {
						shadowSafeVoxel->RemoveAt(shadowSafeVoxel->GetSize());
						//shadowHappen = true;
						((VOXELSETNode *)nextFront->GetAt(i))->layerIndex = 0;
					}
					//std::cout << nextFront->GetSize() << " | " << i << " | " << shadowHappen << std::endl;
				}

				GLKArray *tempArrayThis = nextFront;
				nextFront = shadowSafeVoxel;     shadowSafeVoxel = tempArrayThis;
				shadowSafeVoxel->RemoveAll();

				shadowPointNum = checkifVoxelSetgetShadowed(voxSet, nextFront, currentConvexFront, NULL, distTolerance);
				std::cout << "After checking, shadowPointNum = " << shadowPointNum << ", progressively check the voxel in nextFront set!" << std::endl;
			}
			else {
				//-------------------------------------
				//  Use BSTree to generate safe node set
				BSTree *tree = new BSTree;
				tree->BSTreeFromVoxelGrid(voxSet, nextFront, shadowSafeVoxel, currentConvexFront);
				tree->GetSafeVoxel(nextFront);
				
				delete tree;
				shadowPointNum = checkifVoxelSetgetShadowed(voxSet, nextFront, currentConvexFront, NULL, distTolerance);
				std::cout << "After checking, shadowPointNum = " << shadowPointNum << ", progressively check the voxel in nextFront set!" << std::endl;
			}
		}

		nextConvexFront = _constructNewConvexFront(voxSet, nextFront, currentConvexFront, NULL);
		_freeMemoryConvexHull(currentConvexFront);
		currentConvexFront = nextConvexFront;

		//-----------------------------------------------------------------------------------------------------------
		//  Sub-Step 3: Switch the current-front and the next-front
		GLKArray *tempArray = currentFront;
		currentFront = nextFront;     nextFront = tempArray;
		nextFront->RemoveAll();
		layerCounter++;
		last_shadow_count = shadowPointNum;
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 4: Free the memory
	free(voxGrid);
	delete currentFront;	delete nextFront;
	//---------------------------------------------------------------------------------------------------------------
	if (currentConvexFront != NULL) _freeMemoryConvexHull(currentConvexFront);

	return fabVoxNum;
}

int VOXSetOperation::checkifVoxelSetgetShadowed(VOXELSET *voxSet, GLKArray *checkVoxelSet,
	CONVEXHULLSET *currentConvexFront, QuadTrglMesh *platformMesh, double tolerance) {

	double pnt[3] = { 0.0 };
	int shadowVoxelNum = 0;

	CONVEXHULLSET *checkVoxelSetConvexFront = _constructNewConvexFront(voxSet, checkVoxelSet, currentConvexFront, NULL);
	for (int ii = 0; ii<voxSet->nodeNum; ii++) {
		VOXELSETNode *currentNode = &voxSet->nodeArray[ii];
		//if voxel is not being processed
		if (currentNode->layerIndex != 0) continue;

		for (int jj = 0; jj < 3; jj++)
			pnt[jj] = voxSet->origin[jj] + ((float)(currentNode->posIndex[jj]) + 0.5f)*(voxSet->width);

		if (_isPntInsideConvexHull(checkVoxelSetConvexFront, pnt)) {
			if (_compDistanceToConvexFront(pnt, checkVoxelSetConvexFront) > 2 * tolerance) {
				shadowVoxelNum++;
			}
		}
	}

	_freeMemoryConvexHull(checkVoxelSetConvexFront);
	return shadowVoxelNum;
}

//int VOXSetOperation::AddingSupportingStructure(VOXELSET *voxSet)
//{
//	int newVoxelNum = 0, voxelNum;
//	unsigned int *voxGrid;		int i, temp, xIndex, yIndex, zIndex;
//	VOXELSETNode *currentNode, *neighborNode;
//	const int accumNeighborNum = 5;
//	const int accumConventionalNeighborDelta[][3] = { { 0, -1, 0 },{ -1, -1, 0 },{ 1, -1, 0 },{ 0, -1, -1 },{ 0, -1, 1 } };
//
//	//---------------------------------------------------------------------------------------------------------------
//	//	Step 1: Initialization (malloc the memory of grids)
//	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
//	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]); // 0 value is specified for those places with no voxel
//	voxelNum = voxSet->nodeNum;
//	for (i = 0; i<voxelNum; i++) {
//		voxGrid[_ijkToIndex(voxSet->m_res, voxSet->nodeArray[i].posIndex[0], voxSet->nodeArray[i].posIndex[1], voxSet->nodeArray[i].posIndex[2])] = i + 1;
//	}
//
//	//---------------------------------------------------------------------------------------------------------------
//	//	Step 2: Top-down sweeping (along y-axis) to add supporting structures
//	for (yIndex = voxSet->m_res[1] - 1; yIndex > 0; yIndex--) {
//		for (xIndex = 0; xIndex < voxSet->m_res[0]; xIndex++) {
//			for (zIndex = 0; zIndex < voxSet->m_res[2]; zIndex++) {
//				currentNode = _getNodeByIndex(voxSet, voxGrid, xIndex, yIndex, zIndex);
//
//				bool bWithSupport = false;
//				for (i = 0; i < accumNeighborNum; i++) {
//					neighborNode = _getNodeByIndex(voxSet, voxGrid,
//						xIndex + accumConventionalNeighborDelta[j][0], yIndex + accumConventionalNeighborDelta[j][1], zIndex + accumConventionalNeighborDelta[j][2]);
//					if (neighborNode == NULL || neighborNode->layerIndex != 0) // There is no voxel or the voxel has been processed.
//						continue;
//					bWithSupport = true;
//				}
//
//				if (!bWithSupport) {	// Adding support-structure below
//				}
//			}
//		}
//	}
//
//	//---------------------------------------------------------------------------------------------------------------
//	//	Step X: Free the memory
//	free(voxGrid);
//
//
//	return newVoxelNum;
//}

//void VOXSetOperation::seperateVoxelSetHalfbyPCA(VOXELSET *voxSet, GLKArray *processVoxelSet, std::vector<int> &left_index, std::vector<int> &right_index)
//{
//
//	left_index.clear(); // (pos[0],pos[1],pos[2]),(...),(...)
//	right_index.clear();
//
//	Eigen::MatrixXd voxelPoints(processVoxelSet->GetSize(),3);
//	for (int i = 0; i < processVoxelSet->GetSize(); i++) {
//		VOXELSETNode *currentNode = (VOXELSETNode *)processVoxelSet->GetAt(i);
//		voxelPoints(i, 0) = voxSet->origin[0] + ((float)(currentNode->posIndex[0]) + 0.5f)*(voxSet->width);
//		voxelPoints(i, 1) = voxSet->origin[1] + ((float)(currentNode->posIndex[1]) + 0.5f)*(voxSet->width);
//		voxelPoints(i, 2) = voxSet->origin[2] + ((float)(currentNode->posIndex[2]) + 0.5f)*(voxSet->width);
//	}
//
//	Eigen::Vector3d centroid;
//	Eigen::Matrix3d covariance;
//	Eigen::MatrixXd eigenvectors;
//	Eigen::VectorXd eigenvalues;
//
//	centroid.setZero();
//	for (int i = 0; i < voxelPoints.rows(); i++) { for (int j = 0; j < 3; j++)  centroid(j) += voxelPoints(i, j); }
//	centroid /= voxelPoints.rows();
//
//	covariance.setZero();
//
//	for (unsigned int i = 0; i < voxelPoints.rows(); ++i) {
//		Eigen::Vector3d pt;
//		for (int j = 0; j < 3; j++)	pt[j] = voxelPoints(i, j) - centroid[j];
//
//		covariance(1, 1) += pt.y() * pt.y(); //the non X parts
//		covariance(1, 2) += pt.y() * pt.z();
//		covariance(2, 2) += pt.z() * pt.z();
//
//		pt *= pt.x();
//		covariance(0, 0) += pt.x(); //the X related parts
//		covariance(0, 1) += pt.y();
//		covariance(0, 2) += pt.z();
//	}
//
//	covariance(1, 0) = covariance(0, 1); covariance(2, 0) = covariance(0, 2); covariance(2, 1) = covariance(1, 2);
//	/* normalize  */
//	covariance /= voxelPoints.rows();
//	
//    // Calculate eigenvectors
//	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance, Eigen::ComputeEigenvectors);
//	Eigen::Matrix3d eigenvectorsMatrix = eigenSolver.eigenvectors();
//	eigenvectorsMatrix.col(2) = eigenvectorsMatrix.col(0).cross(eigenvectorsMatrix.col(1));
//
//
//	/* Create the inverse of the transformation matrix and reproject the point cloud, saving it in a different array
//	* if reprojectInput is false or in the same array otherwise */
//	Eigen::Matrix4d transformationMatrixInv = Eigen::Matrix4d::Identity();
//	transformationMatrixInv.block<3, 3>(0, 0) = eigenvectorsMatrix.transpose();
//	transformationMatrixInv.block<3, 1>(0, 3) = -1.0F * (transformationMatrixInv.block<3, 3>(0, 0) * centroid);
//	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
//	transformationMatrix.block<3, 3>(0, 0) = eigenvectorsMatrix;
//	transformationMatrix.block<3, 1>(0, 3) = centroid;
//	Eigen::MatrixXd reproject_points;
//	reproject_points.resize(voxelPoints.rows(), 3);
//	for (int i = 0; i < voxelPoints.rows(); i++) {
//		double x = voxelPoints(i, 0);
//		double y = voxelPoints(i, 1);
//		double z = voxelPoints(i, 2);
//
//		double r_x = transformationMatrixInv(0, 0) * x + transformationMatrixInv(0, 1) * y +
//			transformationMatrixInv(0, 2) * z + transformationMatrixInv(0, 3);
//		double r_y = transformationMatrixInv(1, 0) * x + transformationMatrixInv(1, 1) * y +
//			transformationMatrixInv(1, 2) * z + transformationMatrixInv(1, 3);
//		double r_z = transformationMatrixInv(2, 0) * x + transformationMatrixInv(2, 1) * y +
//			transformationMatrixInv(2, 2) * z + transformationMatrixInv(2, 3);
//
//		reproject_points.row(i) = Eigen::RowVector3d(r_x, r_y, r_z);
//	}
//	//
//	//    // Calculate minimum and maximum points of the bounding box and the center
//	Eigen::Vector3d min, max;
//	min[0] = DBL_MAX;
//	min[1] = DBL_MAX;
//	min[2] = DBL_MAX;
//
//	max[0] = -DBL_MAX;
//	max[1] = -DBL_MAX;
//	max[2] = -DBL_MAX;
//
//	for (int i = 0; i < reproject_points.rows(); i++) {
//
//		double x = reproject_points(i, 0);
//		double y = reproject_points(i, 1);
//		double z = reproject_points(i, 2);
//		// Min/max on x
//		if (x < min[0]) min[0] = x;
//		if (x > max[0]) max[0] = x;
//
//		// Min/max on y
//		if (y < min[1]) min[1] = y;
//		if (y > max[1]) max[1] = y;
//
//		// Min/max on z
//		if (z < min[2]) min[2] = z;
//		if (z > max[2]) max[2] = z;
//	}
//
//	Eigen::Vector3d center = (min + max) / 2;
//
//	for (int i = 0; i < reproject_points.rows(); i++) {
//		double z = reproject_points(i, 2);
//		VOXELSETNode *currentNode = (VOXELSETNode *)processVoxelSet->GetAt(i);
//		if (z <= center(2)) {
//			for (int j = 0; i < 3; j++)
//				left_index.push_back(currentNode->posIndex[j]);
//		}		
//		else {
//			for (int j = 0; i < 3; j++)
//				right_index.push_back(currentNode->posIndex[j]);
//		}
//	}
//}

int VOXSetOperation::GetLayerNumber(VOXELSET *voxSet)
{
	int i, layernum = 0, num = voxSet->nodeNum;

	for (i = 0; i<num; i++) {
		if ((int)(voxSet->nodeArray[i].layerIndex)>layernum) layernum = (int)(voxSet->nodeArray[i].layerIndex);
	}

	return layernum;
}

void VOXSetOperation::FreeVoxelSet(VOXELSET *voxSet)
{
	if (voxSet->nodeNum != 0) {
		free(voxSet->nodeArray);
	}
	free(voxSet);
}

VOXELSET* VOXSetOperation::ConstructVoxelSetFromBSPSolid(BSPTREE *bspTree, float boundingBox[],
	float width, bool bClippingByPlatform)
{
	VOXELSET *voxSet;	float pnt[3];	bool *bGrid;	int index, solidVoxelNum = 0;

	//-------------------------------------------------------------------------------------------------
	//	Step 1: Initialization for voxel-set (origin, width and resolutions are determined)
	voxSet = (VOXELSET*)malloc(sizeof(VOXELSET));
	voxSet->origin[0] = boundingBox[0];	voxSet->origin[1] = boundingBox[2];	voxSet->origin[2] = boundingBox[4];
	voxSet->width = width;
	voxSet->m_res[0] = (UINT)((boundingBox[1] - boundingBox[0]) / width) + 1;
	voxSet->m_res[1] = (UINT)((boundingBox[3] - boundingBox[2]) / width) + 1;
	voxSet->m_res[2] = (UINT)((boundingBox[5] - boundingBox[4]) / width) + 1;
	if (bClippingByPlatform) {
		voxSet->origin[1] = 0.0f;		voxSet->m_res[1] = (UINT)((boundingBox[3] - 0.0f) / width) + 1;
	}
	printf("Resolution of voxels: (%d, %d, %d)\n", voxSet->m_res[0], voxSet->m_res[1], voxSet->m_res[2]);

	long time = clock();

	//determine voxelArray. fill all flag as false
	voxSet->voxelArray = (VOXEL*)malloc(sizeof(VOXEL)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));
	for (int ii = 0; ii < (voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]); ii++) voxSet->voxelArray[ii].isFill = false;

	//-------------------------------------------------------------------------------------------------
	//	Step 2: determine the bool set of voxels
	bGrid = (bool*)malloc(sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));
	memset(bGrid, 0, sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));
	for (UINT k = 0; k<voxSet->m_res[2]; k++) {
		pnt[2] = voxSet->origin[2] + ((float)k + 0.5f)*(voxSet->width);
		for (UINT j = 0; j<voxSet->m_res[1]; j++) {
			pnt[1] = voxSet->origin[1] + ((float)j + 0.5f)*(voxSet->width);
			for (UINT i = 0; i<voxSet->m_res[0]; i++) {
				pnt[0] = voxSet->origin[0] + ((float)i + 0.5f)*(voxSet->width);
				if (BSPSolidOperation::IsPntInsideBSPSolid(bspTree, pnt)) {	// the posint is inside the solid
					index = _ijkToIndex(voxSet->m_res, i, j, k);
					bGrid[index] = true;	solidVoxelNum++;
				}
			}
		}
	}
	printf("solidVoxelNum=%d \n", solidVoxelNum);

	//-------------------------------------------------------------------------------------------------
	//	Step 3: generating voxelNode for the voxel set
	voxSet->nodeNum = solidVoxelNum;
	voxSet->nodeArray = (VOXELSETNode*)malloc(sizeof(VOXELSETNode)*solidVoxelNum);
	if (voxSet->nodeArray == NULL) {
		printf("Warning: resolution is too high to construct the voxel-set!\n");
		free(bGrid);	free(voxSet);	return NULL;
	}
	//-------------------------------------------------------------------------------------------------
	solidVoxelNum = 0;
	for (UINT k = 0; k<voxSet->m_res[2]; k++) {
		for (UINT j = 0; j<voxSet->m_res[1]; j++) {
			for (UINT i = 0; i<voxSet->m_res[0]; i++) {
				index = _ijkToIndex(voxSet->m_res, i, j, k);
				if (bGrid[index]) {
					voxSet->voxelArray[index].isFill = true;
					voxSet->voxelArray[index].voxelPos[0] = voxSet->origin[0] + ((float)i + 0.5f)*(voxSet->width);
					voxSet->voxelArray[index].voxelPos[1] = voxSet->origin[1] + ((float)j + 0.5f)*(voxSet->width);
					voxSet->voxelArray[index].voxelPos[2] = voxSet->origin[2] + ((float)k + 0.5f)*(voxSet->width);

					voxSet->nodeArray[solidVoxelNum].materialIndex = 1;
					voxSet->nodeArray[solidVoxelNum].layerIndex = 1;
					voxSet->nodeArray[solidVoxelNum].posIndex[0] = i;
					voxSet->nodeArray[solidVoxelNum].posIndex[1] = j;
					voxSet->nodeArray[solidVoxelNum].posIndex[2] = k;
					voxSet->nodeArray[solidVoxelNum].layerIndex = j + 1;
					solidVoxelNum++;

					//-------------------------------------------------------------------------------------------------
					//	Build the connection between nodeArray and voxelArray
					voxSet->voxelArray[index].nodeArrayIndex = solidVoxelNum;
				}
			}
		}
	}
	printf("--------------------------------------------------------------\n");
	printf("xRes=%d yRes=%d zRes=%d\nvoxel-width=%f\n%d voxels have been generated in %ld (ms)\n",
		voxSet->m_res[0], voxSet->m_res[1], voxSet->m_res[2], voxSet->width,
		solidVoxelNum, clock() - time);

	//-------------------------------------------------------------------------------------------------
	//	Step 4: free the memory
	free(bGrid);

	return voxSet;
}

VOXELSET* VOXSetOperation::ConstructVoxelSetFromBSPSolid(BSPTREE *bspTree, float boundingBox[], int res)
{
	VOXELSET *voxSet;	float pnt[3], ww;	bool *bGrid;	int index, solidVoxelNum = 0;

	//-------------------------------------------------------------------------------------------------
	//	Step 1: Initialization for voxel-set (origin, width and resolutions are determined)
	voxSet = (VOXELSET*)malloc(sizeof(VOXELSET));
	voxSet->origin[0] = boundingBox[0];	voxSet->origin[1] = boundingBox[2];	voxSet->origin[2] = boundingBox[4];
	ww = (boundingBox[1] - boundingBox[0]) / (float)res;	voxSet->width = ww;
	ww = (boundingBox[3] - boundingBox[2]) / (float)res;	if (ww<(voxSet->width)) voxSet->width = ww;
	ww = (boundingBox[5] - boundingBox[4]) / (float)res;	if (ww<(voxSet->width)) voxSet->width = ww;
	ww = voxSet->width;
	voxSet->m_res[0] = (UINT)((boundingBox[1] - boundingBox[0]) / ww) + 1;
	voxSet->m_res[1] = (UINT)((boundingBox[3] - boundingBox[2]) / ww) + 1;
	voxSet->m_res[2] = (UINT)((boundingBox[5] - boundingBox[4]) / ww) + 1;

	long time = clock();
	//-------------------------------------------------------------------------------------------------
	//	Step 2: determine the bool set of voxels
	bGrid = (bool*)malloc(sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));
	memset(bGrid, 0, sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));
	for (UINT k = 0; k<voxSet->m_res[2]; k++) {
		pnt[2] = voxSet->origin[2] + ((float)k + 0.5f)*(voxSet->width);
		for (UINT j = 0; j<voxSet->m_res[1]; j++) {
			pnt[1] = voxSet->origin[1] + ((float)j + 0.5f)*(voxSet->width);
			for (UINT i = 0; i<voxSet->m_res[0]; i++) {
				pnt[0] = voxSet->origin[0] + ((float)i + 0.5f)*(voxSet->width);
				if (BSPSolidOperation::IsPntInsideBSPSolid(bspTree, pnt)) {	// the posint is inside the solid
					index = _ijkToIndex(voxSet->m_res, i, j, k);
					bGrid[index] = true;	solidVoxelNum++;
				}
			}
		}
	}
	printf("solidVoxelNum=%d \n", solidVoxelNum);

	//-------------------------------------------------------------------------------------------------
	//	Step 3: generating voxelNode for the voxel set
	voxSet->nodeNum = solidVoxelNum;
	voxSet->nodeArray = (VOXELSETNode*)malloc(sizeof(VOXELSETNode)*solidVoxelNum);
	if (voxSet->nodeArray == NULL) {
		printf("Warning: resolution is too high to construct the voxel-set!\n");
		free(bGrid);	free(voxSet);	return NULL;
	}
	//-------------------------------------------------------------------------------------------------
	solidVoxelNum = 0;
	for (UINT k = 0; k<voxSet->m_res[2]; k++) {
		for (UINT j = 0; j<voxSet->m_res[1]; j++) {
			for (UINT i = 0; i<voxSet->m_res[0]; i++) {
				index = _ijkToIndex(voxSet->m_res, i, j, k);
				if (bGrid[index]) {
					voxSet->nodeArray[solidVoxelNum].materialIndex = 1;
					voxSet->nodeArray[solidVoxelNum].layerIndex = 1;
					voxSet->nodeArray[solidVoxelNum].posIndex[0] = i;
					voxSet->nodeArray[solidVoxelNum].posIndex[1] = j;
					voxSet->nodeArray[solidVoxelNum].posIndex[2] = k;
					voxSet->nodeArray[solidVoxelNum].layerIndex = j + 1;
					solidVoxelNum++;
				}
			}
		}
	}
	printf("--------------------------------------------------------------\n");
	printf("xRes=%d yRes=%d zRes=%d\nvoxel-width=%f\n%d voxels have been generated in %ld (ms)\n",
		voxSet->m_res[0], voxSet->m_res[1], voxSet->m_res[2], voxSet->width,
		solidVoxelNum, clock() - time);

	//-------------------------------------------------------------------------------------------------
	//	Step 4: free the memory
	free(bGrid);

	return voxSet;
}


int VOXSetOperation::AddingSupportingStructure(VOXELSET *voxSet)
{
	int newVoxelNum = 0, voxelNum;
	unsigned int *voxGrid;		int i, temp, xIndex, yIndex, zIndex;
	VOXELSETNode *currentNode, *neighborNode;
	const int accumNeighborNum = 5;
	const int accumConventionalNeighborDelta[][3] = { { 0, -1, 0 },{ -1, -1, 0 },{ 1, -1, 0 },{ 0, -1, -1 },{ 0, -1, 1 } };

	//---------------------------------------------------------------------------------------------------------------
	//	Step 1: Initialization (malloc the memory of grids)
	voxGrid = (unsigned int *)malloc(sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	memset(voxGrid, 0, sizeof(unsigned int)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]); // 0 value is specified for those places with no voxel
	voxelNum = voxSet->nodeNum;
	for (i = 0; i<voxelNum; i++) {
		voxGrid[_ijkToIndex(voxSet->m_res, voxSet->nodeArray[i].posIndex[0], voxSet->nodeArray[i].posIndex[1], voxSet->nodeArray[i].posIndex[2])] = i + 1;
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step 2: Top-down sweeping (along y-axis) to add supporting structures
	for (yIndex = voxSet->m_res[1] - 1; yIndex > 0; yIndex--) {
		for (xIndex = 0; xIndex < voxSet->m_res[0]; xIndex++) {
			for (zIndex = 0; zIndex < voxSet->m_res[2]; zIndex++) {
				currentNode = _getNodeByIndex(voxSet, voxGrid, xIndex, yIndex, zIndex);

				bool bWithSupport = false;
				for (i = 0; i < accumNeighborNum; i++) {
					neighborNode = _getNodeByIndex(voxSet, voxGrid,
						xIndex + accumConventionalNeighborDelta[i][0], yIndex + accumConventionalNeighborDelta[i][1], zIndex + accumConventionalNeighborDelta[i][2]);
					if (neighborNode == NULL || neighborNode->layerIndex != 0) // There is no voxel or the voxel has been processed.
						continue;
					bWithSupport = true;
				}

				if (!bWithSupport) {	// Adding support-structure below
				}
			}
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	//	Step X: Free the memory
	free(voxGrid);


	return newVoxelNum;
}

bool VOXSetOperation::InputFabricationInfoFile(UINT *&fabVoxOrder, float *&fabOrients, UINT *&fabVoxLayerIndex, int &fabVoxNum,
	float origin[], UINT res[], float &width, char *filename)
{
	FILE *fp;

	fp = fopen(filename, "rb");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - AMR File Export!\n");
		printf("===============================================\n");
		return false;
	}


	fread(&fabVoxNum, sizeof(int), 1, fp);
	fread(origin, sizeof(float), 3, fp);	// the position of origin
	fread(res, sizeof(UINT), 3, fp);		// the resolutions of voxel-set in different directions
	fread(&width, sizeof(float), 1, fp);	// the width of voxels

	fabVoxOrder = (UINT*)malloc(sizeof(UINT)*fabVoxNum * 3);
	fabOrients = (float*)malloc(sizeof(float)*fabVoxNum * 3);
	fabVoxLayerIndex = (UINT*)malloc(sizeof(UINT)*fabVoxNum);
	//-----------------------------------------------------------------------------------------------------
	fread(fabVoxOrder, sizeof(UINT), fabVoxNum * 3, fp);
	fread(fabOrients, sizeof(float), fabVoxNum * 3, fp);
	fread(fabVoxLayerIndex, sizeof(UINT), fabVoxNum, fp);

	fclose(fp);

	return true;
}

bool VOXSetOperation::OutputFabricationInfoFile(VOXELSET *voxSet, UINT *fabVoxOrder, float *fabOrients, UINT *fabVoxLayerIndex, int fabVoxNum, char *filename)
{
	FILE *fp;

	fp = fopen(filename, "wb");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - AMR File Export!\n");
		printf("===============================================\n");
		return false;
	}

	fwrite(&fabVoxNum, sizeof(int), 1, fp);
	fwrite(voxSet->origin, sizeof(float), 3, fp);		// the position of origin
	fwrite(voxSet->m_res, sizeof(UINT), 3, fp);		// the resolutions of voxel-set in different directions
	fwrite(&(voxSet->width), sizeof(float), 1, fp);	// the width of voxels

	fwrite(fabVoxOrder, sizeof(UINT), fabVoxNum * 3, fp);
	fwrite(fabOrients, sizeof(float), fabVoxNum * 3, fp);
	fwrite(fabVoxLayerIndex, sizeof(UINT), fabVoxNum, fp);

	fclose(fp);

	return true;
}

bool VOXSetOperation::_outputLayerPathFile(int currentLayerIndex, int fabVoxNum, float *fabOrients, UINT *fabVoxOrder, VOXELSET *voxSet, unsigned int *voxGrid, char *layerPathFileName)
{
	FILE *fp;   int i;     float xx, yy, zz, rx, ry, rz;

	fp = fopen(layerPathFileName, "w");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - Layer Path File Export (*.lph)!\n");
		printf("===============================================\n");
		return false;
	}
	fprintf(fp, "LAYER_INDEX: %d \n", currentLayerIndex);

	for (i = 0; i<fabVoxNum; i++) {
		if (_isBoundaryVoxel(voxGrid, voxSet->m_res, fabVoxOrder[i * 3], fabVoxOrder[i * 3 + 1], fabVoxOrder[i * 3 + 2]))
			fprintf(fp, "B");
		else
			fprintf(fp, "I");

		xx = voxSet->origin[0] + ((float)(fabVoxOrder[i * 3]) + 0.5f)*voxSet->width;
		yy = voxSet->origin[1] + ((float)(fabVoxOrder[i * 3 + 1]) + 0.5f)*voxSet->width;
		zz = voxSet->origin[2] + ((float)(fabVoxOrder[i * 3 + 2]) + 0.5f)*voxSet->width;
		_mapCoordIntoRoboCoord(xx, yy, zz, rx, ry, rz);
		fprintf(fp, " %f %f %f", rx, ry, rz);

		xx = fabOrients[i * 3];        yy = fabOrients[i * 3 + 1];        zz = fabOrients[i * 3 + 2];
		_mapCoordIntoRoboCoord(xx, yy, zz, rx, ry, rz);
		fprintf(fp, " %f %f %f\n", rx, ry, rz);
	}

	fclose(fp);

	return true;
}

bool VOXSetOperation::_outputConvexFrontFile(CONVEXHULLSET *pConvexFront, char *convexFileName)
{
	FILE *fp;   int i;     float rx, ry, rz;

	fp = fopen(convexFileName, "w");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - Convex Front File Export (*.cov)!\n");
		printf("===============================================\n");
		return false;
	}
	fprintf(fp, "# VERTEX_NUM: %d  FACE_NUM: %d\n", pConvexFront->vertNum, pConvexFront->faceNum);

	for (i = 0; i<pConvexFront->vertNum; i++) {
		//fprintf(fp,"v %f %f %f\n",pConvexFront->vertPos[i*3],pConvexFront->vertPos[i*3+1],pConvexFront->vertPos[i*3+2]);
		_mapCoordIntoRoboCoord(pConvexFront->vertPos[i * 3], pConvexFront->vertPos[i * 3 + 1], pConvexFront->vertPos[i * 3 + 2], rx, ry, rz);
		fprintf(fp, "v %f %f %f\n", rx, ry, rz);
	}
	fprintf(fp, "\n");

	for (i = 0; i<pConvexFront->faceNum; i++) {
		fprintf(fp, "f %d %d %d\n", pConvexFront->faceTable[i * 3], pConvexFront->faceTable[i * 3 + 1], pConvexFront->faceTable[i * 3 + 2]);
	}

	fclose(fp);

	return true;
}

void VOXSetOperation::_mapCoordIntoRoboCoord(float xx, float yy, float zz, float &outputX, float &outputY, float &outputZ)
{
	outputX = zz; outputY = xx; outputZ = yy;
}

void VOXSetOperation::_searchVoxelsInNextLayerByConvexFront(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
	GLKArray *currentFront, CONVEXHULLSET *currentConvexFront,
	GLKArray *nextFront, int layerCounter, double distTolerance)
{
	int i, j, num;	double pnt[3];

	//-------------------------------------------------------------------------------------------------------------------
	//	Rules for the voxels to be accumulated:
	//
	//		Rule 1) - they must be the neighbors of the current voxel-front
	//		Rule 2) - these voxles must be outside the current convex-front to be collision-free
	//		
	num = currentFront->GetSize();
	for (i = 0; i<num; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));

		for (j = 0; j<neighborNum; j++) {
			VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + neighborDelta[j][0],
				(int)(currentNode->posIndex[1]) + neighborDelta[j][1],
				(int)(currentNode->posIndex[2]) + neighborDelta[j][2]);
			if (neighborNode == NULL || neighborNode->layerIndex != 0) // There is no voxel or the voxel has been processed.
				continue;

			if (currentConvexFront != NULL) {
				pnt[0] = voxSet->origin[0] + ((float)(neighborNode->posIndex[0]) + 0.5f)*(voxSet->width);
				pnt[1] = voxSet->origin[1] + ((float)(neighborNode->posIndex[1]) + 0.5f)*(voxSet->width);
				pnt[2] = voxSet->origin[2] + ((float)(neighborNode->posIndex[2]) + 0.5f)*(voxSet->width);
				if (_isPntInsideConvexHull(currentConvexFront, pnt)) {  
					// Checking if the rule no.2 is not obeyed.													
					if (_compDistanceToConvexFront(pnt,currentConvexFront)>distTolerance) //Chengkai double the tolerance here!
					continue;
				}
			}
			nextFront->Add(neighborNode);	neighborNode->layerIndex = layerCounter + 1;
		}
	}
}

int VOXSetOperation::_countNumOfCriticalVoxel(VOXELSET *voxSet, CONVEXHULLSET *pCurrentConvexFront)
{
	int num = 0;
	int i;  double pnt[3];

	for (i = 0; i<voxSet->nodeNum; i++) {
		if (voxSet->nodeArray[i].layerIndex == 0) {
			pnt[0] = voxSet->origin[0] + ((float)(voxSet->nodeArray[i].posIndex[0]) + 0.5f)*(voxSet->width);
			pnt[1] = voxSet->origin[1] + ((float)(voxSet->nodeArray[i].posIndex[1]) + 0.5f)*(voxSet->width);
			pnt[2] = voxSet->origin[2] + ((float)(voxSet->nodeArray[i].posIndex[2]) + 0.5f)*(voxSet->width);

			if (_isPntInsideConvexHull(pCurrentConvexFront, pnt)) num++;
		}
	}

	return num;
}

void VOXSetOperation::_searchVoxelsOnBoundary(VOXELSET *voxSet, unsigned int *voxGrid, GLKArray *resBndVoxSet)
{
	const int bndNeighborNum = 6;
	const int bndNeighborDelta[][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,-1,0 },{ 0,0,1 },{ 0,0,-1 } };

	int index = 0;
	int i, j, num;
	num = voxSet->nodeNum;
	for (i = 0; i < num; i++) {
		VOXELSETNode *currentNode = &(voxSet->nodeArray[i]);
		if (currentNode->layerIndex != 0) continue;

		bool bBoundaryVox = false;
		for (j = 0; j < bndNeighborNum; j++) {
			VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + bndNeighborDelta[j][0],
				(int)(currentNode->posIndex[1]) + bndNeighborDelta[j][1],
				(int)(currentNode->posIndex[2]) + bndNeighborDelta[j][2]);
			if (neighborNode == NULL || neighborNode->layerIndex != 0) { // There is no voxel or the voxel has been processed.
				bBoundaryVox = true;	break;
			}
		}

		if (bBoundaryVox) { 
			//std::cout << "the size of resBndVoxSet is " << resBndVoxSet->GetSize() << std::endl;
			resBndVoxSet->Add(currentNode); }
	}
}

void VOXSetOperation::_searchVoxelsInNextLayerByFlooding(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
	GLKArray *currentFront, GLKArray *nextFront, int layerCounter, bool bConventional)
{
	//	const int accumNeighborNum=5;	// This is the most safe strategy - only face-neighbors are accumulated
	//	const int accumNeighborDelta[][3]={	{-1,0,0},{1,0,0}, {0,1,0}, {0,0,-1},{0,0,1}  };		
	int accumNeighborNum = 9;	// This one have more DOF - face/edge neighbors are all accumulated
	const int accumNeighborDelta[][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,0,-1 },{ 0,0,1 },{ -1,1,0 },{ 1,1,0 },{ 0,1,-1 },{ 0,1,1 } };
	const int accumConventionalNeighborDelta[][3] = { { 0,1,0 },{ -1,1,0 },{ 1,1,0 },{ 0,1,-1 },{ 0,1,1 } };

	int i, j, num;	VOXELSETNode *neighborNode;

	if (bConventional) accumNeighborNum = 5;

	num = currentFront->GetSize();
	for (i = 0; i<num; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));

		for (j = 0; j<accumNeighborNum; j++) {
			if (bConventional)
				neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + accumConventionalNeighborDelta[j][0],
					(int)(currentNode->posIndex[1]) + accumConventionalNeighborDelta[j][1],
					(int)(currentNode->posIndex[2]) + accumConventionalNeighborDelta[j][2]);
			else
				neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + accumNeighborDelta[j][0],
					(int)(currentNode->posIndex[1]) + accumNeighborDelta[j][1],
					(int)(currentNode->posIndex[2]) + accumNeighborDelta[j][2]);
			if (neighborNode == NULL || neighborNode->layerIndex != 0) // There is no voxel or the voxel has been processed.
				continue;
			nextFront->Add(neighborNode);	neighborNode->layerIndex = layerCounter + 1;
		}
	}
}

void VOXSetOperation::_searchVoxelsInNextLayerByStressFieldGuideFlooding(VOXELSET *voxSet, short nDir, unsigned int *voxGrid,
	GLKArray *currentFront, GLKArray *nextFront, int layerCounter, int stressFieldCounter, bool bConventional)
{
	//	const int accumNeighborNum=5;	// This is the most safe strategy - only face-neighbors are accumulated
	//	const int accumNeighborDelta[][3]={	{-1,0,0},{1,0,0}, {0,1,0}, {0,0,-1},{0,0,1}  };		
	int accumNeighborNum = 9;	// This one have more DOF - face/edge neighbors are all accumulated
	const int accumNeighborDelta[][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,0,-1 },{ 0,0,1 },{ -1,1,0 },{ 1,1,0 },{ 0,1,-1 },{ 0,1,1 } };
	const int accumConventionalNeighborDelta[][3] = { { 0,1,0 },{ -1,1,0 },{ 1,1,0 },{ 0,1,-1 },{ 0,1,1 } };

	int i, j, num;	VOXELSETNode *neighborNode;

	if (bConventional) accumNeighborNum = 5;

	num = currentFront->GetSize();
	for (i = 0; i < num; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));

		for (j = 0; j < accumNeighborNum; j++) {
			if (bConventional)
				neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + accumConventionalNeighborDelta[j][0],
					(int)(currentNode->posIndex[1]) + accumConventionalNeighborDelta[j][1],
					(int)(currentNode->posIndex[2]) + accumConventionalNeighborDelta[j][2]);
			else
				neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + accumNeighborDelta[j][0],
					(int)(currentNode->posIndex[1]) + accumNeighborDelta[j][1],
					(int)(currentNode->posIndex[2]) + accumNeighborDelta[j][2]);
			if (neighborNode == NULL || neighborNode->layerIndex != 0) // There is no voxel or the voxel has been processed.
				continue;
			if (neighborNode->stressFieldLayerIndex > stressFieldCounter) continue;
			nextFront->Add(neighborNode);	neighborNode->layerIndex = layerCounter + 1;
		}
	}
}

CONVEXHULLSET* VOXSetOperation::_constructNewConvexFront(VOXELSET *voxSet, GLKArray *newVoxFront,
	CONVEXHULLSET *currentConvexFront, QuadTrglMesh *platformMesh)
{
	facetT *facet;		vertexT *vertex, **vertexp;
	int i, index, pntNum, num, stIndex;			float pos[3];
	double vec[3][3], dir[3], v1[3], v2[3];
	CONVEXHULLSET *newConvexFront = NULL;

	//-------------------------------------------------------------------------------------
	//	Step 1: initialization
	pntNum = newVoxFront->GetSize();
	if (currentConvexFront != NULL) { pntNum += currentConvexFront->vertNum; }
	if (platformMesh != NULL) { pntNum += platformMesh->GetNodeNumber(); }
	double *pntArray = (double*)malloc(sizeof(double) * 3 * pntNum);
	num = newVoxFront->GetSize();
	for (i = 0; i<num; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(newVoxFront->GetAt(i));
		pntArray[i * 3] = voxSet->origin[0] + ((float)(currentNode->posIndex[0]) + 0.5f)*(voxSet->width);
		pntArray[i * 3 + 1] = voxSet->origin[1] + ((float)(currentNode->posIndex[1]) + 0.5f)*(voxSet->width);
		pntArray[i * 3 + 2] = voxSet->origin[2] + ((float)(currentNode->posIndex[2]) + 0.5f)*(voxSet->width);
	}
	stIndex = num;
	if (currentConvexFront != NULL) {
		for (i = 0; i<currentConvexFront->vertNum; i++) {
			pntArray[stIndex * 3] = currentConvexFront->vertPos[i * 3];
			pntArray[stIndex * 3 + 1] = currentConvexFront->vertPos[i * 3 + 1];
			pntArray[stIndex * 3 + 2] = currentConvexFront->vertPos[i * 3 + 2];
			stIndex++;
		}
	}
	if (platformMesh != NULL) {
		for (i = 0; i<platformMesh->GetNodeNumber(); i++) {
			platformMesh->GetNodePos(i + 1, pos);
			pntArray[stIndex * 3] = pos[0];
			pntArray[stIndex * 3 + 1] = pos[1];
			pntArray[stIndex * 3 + 2] = pos[2];
			stIndex++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	//printf("Convex-Hull: %d faces with %d vertices\n",faceNum,nodeNum);
	if (faceNum>0 && nodeNum>0) {
		newConvexFront = _mallocMemoryConvexHull(faceNum, nodeNum);
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		newConvexFront->vertPos[index * 3] = vertex->point[0];
		newConvexFront->vertPos[index * 3 + 1] = vertex->point[1];
		newConvexFront->vertPos[index * 3 + 2] = vertex->point[2];
		index++;
		}
			//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			newConvexFront->normalVec[index * 3] = facet->normal[0];
		newConvexFront->normalVec[index * 3 + 1] = facet->normal[1];
		newConvexFront->normalVec[index * 3 + 2] = facet->normal[2];
		newConvexFront->offset[index] = facet->offset;
		//	It has been verified all normal[] vectors generated by qhull library are pointing outwards and are unit-vectors 
		//		(verified by the function -- QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)  ).

		int i = 0;
		FOREACHvertex_(facet->vertices) {
			newConvexFront->faceTable[index * 3 + i] = vertex->id + 1;
			SET(vec[i],vertex->point);
			i++;
			if (i >= 3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		if (DOT(dir,facet->normal)<0) {
			unsigned int temp = newConvexFront->faceTable[index * 3];
			newConvexFront->faceTable[index * 3] = newConvexFront->faceTable[index * 3 + 2];
			newConvexFront->faceTable[index * 3 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);

	return newConvexFront;
}

void VOXSetOperation::_motionPlanningInLayerPrimary(VOXELSET *voxSet, short nPrintDir, GLKArray *currentFront, UINT currentLayerIndex,
	float *fabOrients, UINT *fabVoxOrder, UINT *fabVoxLayerIndex, int &fabVoxNum)
{
	int i, num;

	num = currentFront->GetSize();
	for (i = 0; i<num; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));

		fabOrients[fabVoxNum * 3] = fabOrients[fabVoxNum * 3 + 1] = fabOrients[fabVoxNum * 3 + 2] = 0.0;	fabOrients[fabVoxNum * 3 + nPrintDir] = 1.0f;

		fabVoxOrder[fabVoxNum * 3] = currentNode->posIndex[0];
		fabVoxOrder[fabVoxNum * 3 + 1] = currentNode->posIndex[1];
		fabVoxOrder[fabVoxNum * 3 + 2] = currentNode->posIndex[2];

		fabVoxLayerIndex[fabVoxNum] = currentLayerIndex;
		fabVoxNum++;
	}
}

void VOXSetOperation::_motionPlanningInLayer(VOXELSET *voxSet, unsigned int *voxGrid, short nPrintDir, GLKArray *currentFront, UINT currentLayerIndex,
	CONVEXHULLSET *pConvexHull, float *fabOrients, UINT *fabVoxOrder, UINT *fabVoxLayerIndex, int &fabVoxNum)
{
	//	_motionPlanningInLayerPrimary(voxSet,nPrintDir,currentFront,currentLayerIndex,fabOrients,fabVoxOrder,fabVoxLayerIndex,fabVoxNum);	return;

	int i, j, num;
	bool *bVoxFlag;		GLKArray *regionOfFab, *boundaryOfFab, *interiorOfFab;
	float voxOrt[3], orient[3];	UINT voxPos[3];		int weight1, weight2, st, size, bndSt;

	//---------------------------------------------------------------------------------------------------------
	//	Step 1: Preparation
	bVoxFlag = (bool *)malloc(sizeof(bool)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	memset(bVoxFlag, 0, sizeof(bool)*voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2]);
	//---------------------------------------------------------------------------------------------------------
	num = currentFront->GetSize();
	regionOfFab = new GLKArray(num, num, 0);
	boundaryOfFab = new GLKArray(num, num, 0);
	interiorOfFab = new GLKArray(num, num, 0);
	//---------------------------------------------------------------------------------------------------------
	for (i = 0; i<currentFront->GetSize(); i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));
		currentNode->attachedPtr = NULL;
	}
	//printf("Num of Voxels in CurrentFront = %d\n",currentFront->GetSize());

	//---------------------------------------------------------------------------------------------------------
	//	Step 2: Processing the voxels the regions by flooding
	int seedSTIndex = 0;
	while (true) {
		//-----------------------------------------------------------------------------------------------------
		//	Sub-step 1: find the seed of flooding
		VOXELSETNode *seedNode = NULL;
		for (i = seedSTIndex; i<currentFront->GetSize(); i++) {
			VOXELSETNode *currentNode = (VOXELSETNode *)(currentFront->GetAt(i));
			currentNode->attachedPtr = NULL;
			if (bVoxFlag[_ijkToIndex(voxSet->m_res, currentNode->posIndex[0], currentNode->posIndex[1], currentNode->posIndex[2])]) continue;
			seedNode = currentNode;	seedSTIndex = i;	break;
		}
		if (seedNode == NULL) break;

		//-----------------------------------------------------------------------------------------------------
		//	Sub-step 2: flooding for searching regions
		regionOfFab->RemoveAll();
		_floodingRegionInCurvedLayer(voxSet, voxGrid, bVoxFlag, seedNode, currentLayerIndex, currentFront->GetSize(), regionOfFab);

		//-----------------------------------------------------------------------------------------------------
		//	Sub-step 3: separating the boundary and the interior regions
		boundaryOfFab->RemoveAll();		interiorOfFab->RemoveAll();
		for (i = 0; i<regionOfFab->GetSize(); i++) {
			VOXELSETNode *currentNode = (VOXELSETNode *)(regionOfFab->GetAt(i));
			if (_isBoundaryVoxel(voxGrid, voxSet->m_res, currentNode->posIndex[0], currentNode->posIndex[1], currentNode->posIndex[2]))
			{
				boundaryOfFab->Add(currentNode);
			}
			else
			{
				interiorOfFab->Add(currentNode);
			}
		}

		//-----------------------------------------------------------------------------------------------------
		//	Sub-step 4: processing the boundary of a region on the curved layer		
		//-----------------------------------------------------------------------------------------------------
		//printf("size of boundary: %d\n",size);
		if (boundaryOfFab->GetSize()>5) {
			_boundaryVoxelPlanning(voxSet, voxGrid, currentLayerIndex, boundaryOfFab);
		}
		//-----------------------------------------------------------------------------------------------------
		bndSt = fabVoxNum;   size = boundaryOfFab->GetSize();
		for (i = 0; i<boundaryOfFab->GetSize(); i++) {
			VOXELSETNode *currentNode = (VOXELSETNode *)(boundaryOfFab->GetAt(i));
			if (pConvexHull == NULL)
				_determineOrientByReliableMaterialAccumulation(voxSet, voxGrid, nPrintDir, currentLayerIndex, currentNode, orient);
			else {
				float materialAccumPrefOrient[3];
				_determineOrientByClosestPntOnConvexFront(voxSet, currentNode, pConvexHull, orient);
				//				_determineOrientByReliableMaterialAccumulation(voxSet,voxGrid,nPrintDir,currentLayerIndex,currentNode,materialAccumPrefOrient);
				//				_tuningOrient(orient,materialAccumPrefOrient); // This function has a bug - may generate signular normal and need to be fixed (Charlie: Jan 20, 2017).
			}
			fabOrients[fabVoxNum * 3] = orient[0];  fabOrients[fabVoxNum * 3 + 1] = orient[1];  fabOrients[fabVoxNum * 3 + 2] = orient[2];
			fabVoxOrder[fabVoxNum * 3] = currentNode->posIndex[0];  fabVoxOrder[fabVoxNum * 3 + 1] = currentNode->posIndex[1];  fabVoxOrder[fabVoxNum * 3 + 2] = currentNode->posIndex[2];

			fabVoxLayerIndex[fabVoxNum] = currentLayerIndex;
			fabVoxNum++;
		}

		//-----------------------------------------------------------------------------------------------------
		//	Sub-step 5: processing the interior of a region on the curved layer
		//-----------------------------------------------------------------------------------------------------
		st = fabVoxNum;   size = interiorOfFab->GetSize();
		for (i = 0; i<size; i++) {
			VOXELSETNode *currentNode = (VOXELSETNode *)(interiorOfFab->GetAt(i));
			if (pConvexHull == NULL)
				_determineOrientByReliableMaterialAccumulation(voxSet, voxGrid, nPrintDir, currentLayerIndex, currentNode, orient);
			else {
				float materialAccumPrefOrient[3];
				_determineOrientByClosestPntOnConvexFront(voxSet, currentNode, pConvexHull, orient);
				//				_determineOrientByReliableMaterialAccumulation(voxSet,voxGrid,nPrintDir,currentLayerIndex,currentNode,materialAccumPrefOrient);
				//				_tuningOrient(orient,materialAccumPrefOrient); // This function has a bug - may generate signular normal and need to be fixed (Charlie: Jan 20, 2017).
			}
			fabOrients[fabVoxNum * 3] = orient[0];  fabOrients[fabVoxNum * 3 + 1] = orient[1];  fabOrients[fabVoxNum * 3 + 2] = orient[2];
			fabVoxOrder[fabVoxNum * 3] = currentNode->posIndex[0];  fabVoxOrder[fabVoxNum * 3 + 1] = currentNode->posIndex[1];  fabVoxOrder[fabVoxNum * 3 + 2] = currentNode->posIndex[2];

			fabVoxLayerIndex[fabVoxNum] = currentLayerIndex;
			fabVoxNum++;
		}
		//-----------------------------------------------------------------------------------------------------
		//	The order of fabrication is determined by sorted locations (Tricks have been added to generate zig-zag pattern for interior voxels - by Charlie on January 20, 2017))
		if (currentLayerIndex == 1) {
			st = bndSt;
			size = boundaryOfFab->GetSize() + interiorOfFab->GetSize();	// to sort both boundary and interior voxels
		}
		for (i = st; i<st + size; i++) {
			if (currentLayerIndex % 2 == 0)
			{
				if (fabVoxOrder[i * 3 + (nPrintDir + 2) % 3] % 2 == 0)
					weight1 = fabVoxOrder[i * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
					+ fabVoxOrder[i * 3 + (nPrintDir + 2) % 3] * (voxSet->m_res[(nPrintDir + 1) % 3])
					+ fabVoxOrder[i * 3 + (nPrintDir + 1) % 3];
				else
					weight1 = fabVoxOrder[i * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
					+ fabVoxOrder[i * 3 + (nPrintDir + 2) % 3] * (voxSet->m_res[(nPrintDir + 1) % 3])
					+ (voxSet->m_res[(nPrintDir + 1) % 3] - fabVoxOrder[i * 3 + (nPrintDir + 1) % 3]);
			}
			else {
				if (fabVoxOrder[i * 3 + (nPrintDir + 1) % 3] % 2 == 0)
					weight1 = fabVoxOrder[i * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
					+ fabVoxOrder[i * 3 + (nPrintDir + 1) % 3] * (voxSet->m_res[(nPrintDir + 2) % 3])
					+ fabVoxOrder[i * 3 + (nPrintDir + 2) % 3];
				else
					weight1 = fabVoxOrder[i * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
					+ fabVoxOrder[i * 3 + (nPrintDir + 1) % 3] * (voxSet->m_res[(nPrintDir + 2) % 3])
					+ (voxSet->m_res[(nPrintDir + 2) % 3] - fabVoxOrder[i * 3 + (nPrintDir + 2) % 3]);
			}
			for (j = i + 1; j<st + size; j++) {
				if (currentLayerIndex % 2 == 0)
				{
					if (fabVoxOrder[j * 3 + (nPrintDir + 2) % 3] % 2 == 0)
						weight2 = fabVoxOrder[j * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
						+ fabVoxOrder[j * 3 + (nPrintDir + 2) % 3] * (voxSet->m_res[(nPrintDir + 1) % 3])
						+ fabVoxOrder[j * 3 + (nPrintDir + 1) % 3];
					else
						weight2 = fabVoxOrder[j * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
						+ fabVoxOrder[j * 3 + (nPrintDir + 2) % 3] * (voxSet->m_res[(nPrintDir + 1) % 3])
						+ (voxSet->m_res[(nPrintDir + 1) % 3] - fabVoxOrder[j * 3 + (nPrintDir + 1) % 3]);
				}
				else {
					if (fabVoxOrder[j * 3 + (nPrintDir + 1) % 3] % 2 == 0)
						weight2 = fabVoxOrder[j * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
						+ fabVoxOrder[j * 3 + (nPrintDir + 1) % 3] * (voxSet->m_res[(nPrintDir + 2) % 3])
						+ fabVoxOrder[j * 3 + (nPrintDir + 2) % 3];
					else
						weight2 = fabVoxOrder[j * 3 + nPrintDir] * (voxSet->m_res[(nPrintDir + 1) % 3]) * (voxSet->m_res[(nPrintDir + 2) % 3])
						+ fabVoxOrder[j * 3 + (nPrintDir + 1) % 3] * (voxSet->m_res[(nPrintDir + 2) % 3])
						+ (voxSet->m_res[(nPrintDir + 2) % 3] - fabVoxOrder[j * 3 + (nPrintDir + 2) % 3]);
				}

				if (weight1>weight2) {	// switch the locations of fabs
					voxOrt[0] = fabOrients[j * 3];		voxOrt[1] = fabOrients[j * 3 + 1];		voxOrt[2] = fabOrients[j * 3 + 2];
					fabOrients[j * 3] = fabOrients[i * 3];	fabOrients[j * 3 + 1] = fabOrients[i * 3 + 1];	fabOrients[j * 3 + 2] = fabOrients[i * 3 + 2];
					fabOrients[i * 3] = voxOrt[0];		fabOrients[i * 3 + 1] = voxOrt[1];		fabOrients[i * 3 + 2] = voxOrt[2];

					voxPos[0] = fabVoxOrder[j * 3];		voxPos[1] = fabVoxOrder[j * 3 + 1];		voxPos[2] = fabVoxOrder[j * 3 + 2];
					fabVoxOrder[j * 3] = fabVoxOrder[i * 3];	fabVoxOrder[j * 3 + 1] = fabVoxOrder[i * 3 + 1];	fabVoxOrder[j * 3 + 2] = fabVoxOrder[i * 3 + 2];
					fabVoxOrder[i * 3] = voxPos[0];		fabVoxOrder[i * 3 + 1] = voxPos[1];		fabVoxOrder[i * 3 + 2] = voxPos[2];
					weight1 = weight2;
				}
			}
		}
	}

	//---------------------------------------------------------------------------------------------------------
	//	Step 3: Free the memory
	free(bVoxFlag);
	//---------------------------------------------------------------------------------------------------------
	delete regionOfFab;		delete boundaryOfFab;	delete interiorOfFab;
}

void VOXSetOperation::_tuningOrient(float orient[], float materialAccuPreferredDir[])
{
	double rotAxis[3], vecToBeRotated[3];		GLKGeometry geo;	double angle;

	CROSS_PRODUCT(orient, materialAccuPreferredDir, rotAxis);
	if (!(geo.Normalize(rotAxis))) return;		// Two orientations are the same

	angle = ROTATE_TO_DEGREE(acos(DOT_PRODUCT(orient, materialAccuPreferredDir)));
	//if (angle<0.0) printf("ERROR ");	// for debug purpose
	if (angle>MAXALLOWED_TUNINGANGLE_ON_NOZZLE) angle = MAXALLOWED_TUNINGANGLE_ON_NOZZLE;

	SET(vecToBeRotated, orient);
	geo.RotateAroundVector(vecToBeRotated, rotAxis, angle);
	SET(orient, vecToBeRotated);
}

void VOXSetOperation::_findVoxelsOnConvexFront(GLKArray *inputVoxSet, CONVEXHULLSET *pCurrentConvexFront, GLKArray *resVoxSet, double thresholdDist, VOXELSET *voxSet)
{
	int i;  double pnt[3];

	resVoxSet->RemoveAll();
	for (i = 0; i<inputVoxSet->GetSize(); i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(inputVoxSet->GetAt(i));
		pnt[0] = voxSet->origin[0] + ((float)(currentNode->posIndex[0]) + 0.5f)*(voxSet->width);
		pnt[1] = voxSet->origin[1] + ((float)(currentNode->posIndex[1]) + 0.5f)*(voxSet->width);
		pnt[2] = voxSet->origin[2] + ((float)(currentNode->posIndex[2]) + 0.5f)*(voxSet->width);

		double dist = _compDistanceToConvexFront(pnt, pCurrentConvexFront);
		if (dist<thresholdDist) resVoxSet->Add(currentNode);
	}
}

double VOXSetOperation::_compDistanceToConvexFront(double pos[], CONVEXHULLSET *pConvexHull)
{
	double pc[3], dist, minDist;    int i, pIndex;

	//----------------------------------------------------------------------------------------------------------------------------------------
	//	Search the closest plane in the convex hull
	minDist = 1.0e+5;
	for (i = 0; i<pConvexHull->faceNum; i++) {
		pIndex = pConvexHull->faceTable[i * 3] - 1;
		pc[0] = pConvexHull->vertPos[pIndex * 3];		pc[1] = pConvexHull->vertPos[pIndex * 3 + 1];		pc[2] = pConvexHull->vertPos[pIndex * 3 + 2];

		dist = fabs((pos[0] - pc[0])*pConvexHull->normalVec[i * 3] + (pos[1] - pc[1])*pConvexHull->normalVec[i * 3 + 1] + (pos[2] - pc[2])*pConvexHull->normalVec[i * 3 + 2]);
		if (dist<minDist) { minDist = dist; }
	}

	return minDist;
}

void VOXSetOperation::_determineOrientByClosestPntOnConvexFront(VOXELSET *voxSet, VOXELSETNode *currentNode, CONVEXHULLSET *pConvexHull, float orient[])
{
	double pos[3], pc[3], ww, dist, minDist;
	int i, pIndex, nearestIndex = 0;
	double eps = 1.0e-5;

	ww = voxSet->width;
	pos[0] = voxSet->origin[0] + ((float)(currentNode->posIndex[0]) + 0.5)*ww;
	pos[1] = voxSet->origin[1] + ((float)(currentNode->posIndex[1]) + 0.5)*ww;
	pos[2] = voxSet->origin[2] + ((float)(currentNode->posIndex[2]) + 0.5)*ww;

	//	bool bDebug = false;
	//	if ((currentNode->posIndex[0] == 12) && (currentNode->posIndex[1] == 96) && (currentNode->posIndex[2] == 30)) {
	//		bDebug = true;	printf("**************************************\n");
	//	}

	//----------------------------------------------------------------------------------------------------------------------------------------
	//	Search the closest plane in the convex hull
	minDist = 1.0e+5;
	for (i = 0; i<pConvexHull->faceNum; i++) {
		pIndex = pConvexHull->faceTable[i * 3] - 1;
		pc[0] = pConvexHull->vertPos[pIndex * 3];		pc[1] = pConvexHull->vertPos[pIndex * 3 + 1];		pc[2] = pConvexHull->vertPos[pIndex * 3 + 2];
		/*
		double p1[3],p2[3],p3[3];
		pIndex=pConvexHull->faceTable[i*3]-1;
		p1[0]=pConvexHull->vertPos[pIndex*3];		p1[1]=pConvexHull->vertPos[pIndex*3+1];		p1[2]=pConvexHull->vertPos[pIndex*3+2];

		pIndex=pConvexHull->faceTable[i*3+1]-1;
		p2[0]=pConvexHull->vertPos[pIndex*3];		p2[1]=pConvexHull->vertPos[pIndex*3+1];     p2[2]=pConvexHull->vertPos[pIndex*3+2];

		pIndex=pConvexHull->faceTable[i*3+2]-1;
		p3[0]=pConvexHull->vertPos[pIndex*3];		p3[1]=pConvexHull->vertPos[pIndex*3+1];     p3[2]=pConvexHull->vertPos[pIndex*3+2];
		*/
		//        pc[0]=(p1[0]+p2[0]+p3[0])/3.0;        pc[1]=(p1[1]+p2[1]+p3[1])/3.0;        pc[2]=(p1[2]+p2[2]+p3[2])/3.0;
		//        if (fabs(p1[0]-p2[0])<eps && fabs(p1[1]-p2[1])<eps && fabs(p1[2]-p2[2])<eps) printf("********************************************************* Degenerate found\n");
		//        if (fabs(p3[0]-p2[0])<eps && fabs(p3[1]-p2[1])<eps && fabs(p3[2]-p2[2])<eps) printf("********************************************************* Degenerate found\n");
		//        if (fabs(p1[0]-p3[0])<eps && fabs(p1[1]-p3[1])<eps && fabs(p1[2]-p3[2])<eps) printf("********************************************************* Degenerate found\n");

		dist = fabs((pos[0] - pc[0])*pConvexHull->normalVec[i * 3] + (pos[1] - pc[1])*pConvexHull->normalVec[i * 3 + 1] + (pos[2] - pc[2])*pConvexHull->normalVec[i * 3 + 2]);
		//		if (bDebug) {printf("dist=%f normal=(%f,%f,%f)\n", dist, pConvexHull->normalVec[i * 3], pConvexHull->normalVec[i * 3 + 1], pConvexHull->normalVec[i * 3 + 2]);}
		if (dist<minDist) {
			minDist = dist; nearestIndex = i;
			//			if (bDebug) {printf("minD=%f normal=(%f,%f,%f)\n", minDist, pConvexHull->normalVec[i*3], pConvexHull->normalVec[i*3+1], pConvexHull->normalVec[i*3+2]);	}
		}
	}

	orient[0] = pConvexHull->normalVec[nearestIndex * 3];
	orient[1] = pConvexHull->normalVec[nearestIndex * 3 + 1];
	orient[2] = pConvexHull->normalVec[nearestIndex * 3 + 2];
}

void VOXSetOperation::_determineOrientByReliableMaterialAccumulation(VOXELSET *voxSet, unsigned int *voxGrid,
	short nPrintDir, UINT currentLayerIndex,
	VOXELSETNode *currentNode, float orient[])
{
	float nx, ny, nz, dd;
	bool bFaceNeighbor = false;

	orient[0] = orient[1] = orient[2] = 0.0f;
	for (int ii = 0; ii<faceNeighborNum; ii++) {
		VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid,
			(int)(currentNode->posIndex[0]) + faceNeighborDelta[ii][0],
			(int)(currentNode->posIndex[1]) + faceNeighborDelta[ii][1],
			(int)(currentNode->posIndex[2]) + faceNeighborDelta[ii][2]);
		if (neighborNode == NULL) continue;	 // There is not voxel
		if (neighborNode->layerIndex >= currentLayerIndex || neighborNode->layerIndex == 0) continue;	// this is a voxel has not been accumulated yet

		nx = -(float)(faceNeighborDelta[ii][0]);
		ny = -(float)(faceNeighborDelta[ii][1]);
		nz = -(float)(faceNeighborDelta[ii][2]);
		orient[0] += nx;	orient[1] += ny;	orient[2] += nz;		bFaceNeighbor = true;
	}
	dd = orient[0] * orient[0] + orient[1] * orient[1] + orient[2] * orient[2];

	if ((!bFaceNeighbor) || (dd<1.0e-5)) {
		orient[0] = orient[1] = orient[2] = 0.0f;
		for (int ii = 0; ii<neighborNum; ii++) {
			VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + neighborDelta[ii][0],
				(int)(currentNode->posIndex[1]) + neighborDelta[ii][1],
				(int)(currentNode->posIndex[2]) + neighborDelta[ii][2]);
			if (neighborNode == NULL) continue;	 // There is not voxel
			if (neighborNode->layerIndex >= currentLayerIndex || neighborNode->layerIndex == 0) continue;	// this is a voxel has not been accumulated yet

			nx = -(float)(neighborDelta[ii][0]) / neighborDist[ii];
			ny = -(float)(neighborDelta[ii][1]) / neighborDist[ii];
			nz = -(float)(neighborDelta[ii][2]) / neighborDist[ii];
			orient[0] += nx;	orient[1] += ny;	orient[2] += nz;
		}
		dd = orient[0] * orient[0] + orient[1] * orient[1] + orient[2] * orient[2];
	}

	if (dd<1.0e-5) {
		orient[0] = orient[1] = orient[2] = 0.0f;		orient[nPrintDir] = 1.0f;
	}
	else {
		dd = sqrt(dd);	orient[0] = orient[0] / dd;		orient[1] = orient[1] / dd;		orient[2] = orient[2] / dd;
	}
}

void VOXSetOperation::_boundaryVoxelPlanning(VOXELSET *voxSet, unsigned int *voxGrid, UINT currentLayerIndex, GLKArray *boundaryOfFab)
{
	int i, size;
	GLKGraph graph;		GLKObList nodeVisitList;	GLKPOSITION Pos;

	//---------------------------------------------------------------------------------------------------------
	//	Construct all graph nodes
	size = boundaryOfFab->GetSize();
	for (i = 0; i<size; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(boundaryOfFab->GetAt(i));
		GLKGraphNode *graphNode = new GLKGraphNode;
		currentNode->attachedPtr = graphNode;		graphNode->attachedObj = currentNode;
		graph.AddNode(graphNode);
	}

	//---------------------------------------------------------------------------------------------------------
	//	Construct all graph edges between neighboring voxels - avoid redundancy
	for (i = 0; i<size; i++) {
		VOXELSETNode *currentNode = (VOXELSETNode *)(boundaryOfFab->GetAt(i));
		GLKGraphNode *currentGraphNode = (GLKGraphNode *)(currentNode->attachedPtr);
		for (int ii = 0; ii<neighborNum; ii++) {
			VOXELSETNode *neighborNode = _getNodeByIndex(voxSet, voxGrid,
				(int)(currentNode->posIndex[0]) + neighborDelta[ii][0],
				(int)(currentNode->posIndex[1]) + neighborDelta[ii][1],
				(int)(currentNode->posIndex[2]) + neighborDelta[ii][2]);
			if (neighborNode == NULL || neighborNode->layerIndex != currentLayerIndex) // There is no voxel or a voxel not in the same layer.
				continue;
			if (neighborNode->attachedPtr == NULL) continue;	// Not a voxel of this boundary
			GLKGraphNode *neighborGraphNode = (GLKGraphNode *)(neighborNode->attachedPtr);

			bool bEdgeExisting = false;	GLKPOSITION PosEdge;
			for (PosEdge = neighborGraphNode->edgeList.GetHeadPosition(); PosEdge != NULL;) {
				GLKGraphEdge *edge = (GLKGraphEdge *)(neighborGraphNode->edgeList.GetNext(PosEdge));
				GLKGraphNode *otherGraphNode;
				if (edge->startNode == neighborGraphNode) otherGraphNode = edge->endNode; else otherGraphNode = edge->startNode;
				if (otherGraphNode == currentGraphNode) { bEdgeExisting = true; break; }
			}
			if (bEdgeExisting) continue;

			GLKGraphEdge *graphEdge = new GLKGraphEdge;
			graphEdge->startNode = currentGraphNode;		currentGraphNode->edgeList.AddTail(graphEdge);
			graphEdge->endNode = neighborGraphNode;		neighborGraphNode->edgeList.AddTail(graphEdge);
			graphEdge->m_weight = neighborDist[ii];
			graph.AddEdge(graphEdge);
		}
	}

	//---------------------------------------------------------------------------------------------------------
	//	Using TSP algorithm to determine the tool path
	graph.ApproximateTravelingSalesmanProblemTour(&nodeVisitList);
	//---------------------------------------------------------------------------------------------------------
	for (i = 0; i<size; i++) { ((VOXELSETNode *)(boundaryOfFab->GetAt(i)))->attachedPtr = NULL; }
	boundaryOfFab->RemoveAll();
	for (Pos = nodeVisitList.GetHeadPosition(); Pos != NULL;) {
		GLKGraphNode *graphNode = (GLKGraphNode *)(nodeVisitList.GetNext(Pos));
		VOXELSETNode *currentNode = (VOXELSETNode *)(graphNode->attachedObj);
		boundaryOfFab->Add(currentNode);
	}

	/*
	GLKGraphNode *node_a;	GLKGraphNode *node_b;	GLKGraphNode *node_c;	GLKGraphNode *node_d;	GLKGraphNode *node_e;
	GLKGraphEdge *edgeAB;	GLKGraphEdge *edgeAC;	GLKGraphEdge *edgeBC;	GLKGraphEdge *edgeBD;	GLKGraphEdge *edgeCE;

	node_a=new GLKGraphNode;	AddNode(node_a);	node_a->m_height=1;
	edgeAB=new GLKGraphEdge;	edgeAB->startNode=node_a;	edgeAB->endNode=node_b; edgeAB->m_weight=10.0;	AddEdge(edgeAB);
	*/
}

void VOXSetOperation::_floodingRegionInCurvedLayer(VOXELSET *voxSet, unsigned int *voxGrid, bool *bVoxFlag,
	VOXELSETNode *seedNode, UINT currentLayerIndex, int voxelNumInLayer,
	GLKArray *regionOfFab)
{
	//--------------------------------------------------------------------------------------------------------------------------
	//  The implementation based on front propagation can overcome the limit of stack size
	VOXELSETNode *currentNode, *neighborNode;
	GLKArray *currentFront, *nextFront, *tempPtr;
	currentFront = new GLKArray(voxelNumInLayer, voxelNumInLayer, 0);
	nextFront = new GLKArray(voxelNumInLayer, voxelNumInLayer, 0);

	regionOfFab->Add(seedNode);
	bVoxFlag[_ijkToIndex(voxSet->m_res, seedNode->posIndex[0], seedNode->posIndex[1], seedNode->posIndex[2])] = true;

	currentFront->Add(seedNode);
	while (currentFront->GetSize()>0) {
		for (int jj = 0; jj<currentFront->GetSize(); jj++) {
			currentNode = (VOXELSETNode *)(currentFront->GetAt(jj));

			for (int ii = 0; ii<neighborNum; ii++) {
				neighborNode = _getNodeByIndex(voxSet, voxGrid,
					(int)(currentNode->posIndex[0]) + neighborDelta[ii][0],
					(int)(currentNode->posIndex[1]) + neighborDelta[ii][1],
					(int)(currentNode->posIndex[2]) + neighborDelta[ii][2]);
				if (neighborNode == NULL || neighborNode->layerIndex != currentLayerIndex) // There is no voxel or a voxel not in the same layer.
					continue;
				if (bVoxFlag[_ijkToIndex(voxSet->m_res, neighborNode->posIndex[0], neighborNode->posIndex[1], neighborNode->posIndex[2])]) continue; // a voxel has been processed
				regionOfFab->Add(neighborNode);
				bVoxFlag[_ijkToIndex(voxSet->m_res, neighborNode->posIndex[0], neighborNode->posIndex[1], neighborNode->posIndex[2])] = true;
				nextFront->Add(neighborNode);
			}
		}

		tempPtr = currentFront;   currentFront = nextFront;     nextFront = tempPtr;  nextFront->RemoveAll();
	}

	delete currentFront;    delete nextFront;
}

bool VOXSetOperation::_isBoundaryVoxel(unsigned int *voxGrid, UINT res[], int i, int j, int k)
{
	for (int ii = 0; ii<faceNeighborNum; ii++) {
		if (!(_isVoxelExisting(voxGrid, res,
			i + faceNeighborDelta[ii][0], j + faceNeighborDelta[ii][1], k + faceNeighborDelta[ii][2])))
			return true;
	}

	return false;
}

bool VOXSetOperation::_isVoxelExisting(unsigned int *voxGrid, UINT res[], int ii, int jj, int kk)
{
	if ((ii<0) || (ii >= (int)(res[0]))
		|| (jj<0) || (jj >= (int)(res[1]))
		|| (kk<0) || (kk >= (int)(res[2]))) return false;

	int index = _ijkToIndex(res, ii, jj, kk);
	if (voxGrid[index] == 0) return false;

	return true;
}

VOXELSETNode* VOXSetOperation::_getNodeByIndex(VOXELSET *voxSet, unsigned int *voxGrid, int ii, int jj, int kk)
{
	if ((ii<0) || (ii >= (int)(voxSet->m_res[0]))
		|| (jj<0) || (jj >= (int)(voxSet->m_res[1]))
		|| (kk<0) || (kk >= (int)(voxSet->m_res[2]))) return NULL;

	int index = _ijkToIndex(voxSet->m_res, ii, jj, kk);
	if (voxGrid[index] == 0) return NULL;

	return (&(voxSet->nodeArray[(int)(voxGrid[index]) - 1]));
}

void VOXSetOperation::_ijkToPosition(VOXELSET *voxSet, int i, int j, int k, float pos[])
{
	pos[0] = voxSet->origin[0] + ((float)i + 0.5f)*(voxSet->width);
	pos[1] = voxSet->origin[1] + ((float)j + 0.5f)*(voxSet->width);
	pos[2] = voxSet->origin[2] + ((float)k + 0.5f)*(voxSet->width);
}

void VOXSetOperation::_indexToIJK(UINT res[], int index, int &i, int &j, int &k)
{
	k = index / (res[1] * res[0]);
	index = index % (res[1] * res[0]);

	j = index / (res[0]);

	i = index % (res[0]);
}

int VOXSetOperation::_ijkToIndex(UINT res[], int i, int j, int k)
{
	if (i < 0 || j < 0 || k < 0) return -1;
	if (i >= res[0] || j >= res[1] || k >= res[2]) return -2;
	return (k*res[1] * res[0] + j*res[0] + i);
}

CONVEXHULLSET* VOXSetOperation::_mallocMemoryConvexHull(int faceNum, int vertNum)
{
	CONVEXHULLSET* pConvexHull;

	pConvexHull = (CONVEXHULLSET*)malloc(sizeof(CONVEXHULLSET));
	pConvexHull->faceNum = faceNum;
	pConvexHull->normalVec = (double*)malloc(sizeof(double) * 3 * faceNum);
	pConvexHull->offset = (double*)malloc(sizeof(double)*faceNum);

	pConvexHull->faceTable = (unsigned int*)malloc(sizeof(unsigned int) * 3 * faceNum);

	pConvexHull->vertNum = vertNum;
	pConvexHull->vertPos = (double*)malloc(sizeof(double) * 3 * vertNum);

	return pConvexHull;
}

void VOXSetOperation::_freeMemoryConvexHull(CONVEXHULLSET *&pConvexHull)
{
	free((pConvexHull->normalVec));
	free((pConvexHull->offset));
	free((pConvexHull->faceTable));
	free((pConvexHull->vertPos));
	free(pConvexHull);

	pConvexHull = NULL;
}

bool VOXSetOperation::_isPntInsideConvexHull(CONVEXHULLSET *pConvexHull, double pnt[])
{
	double normVec[3], offValue;

	for (int i = 0; i<pConvexHull->faceNum; i++) {
		normVec[0] = pConvexHull->normalVec[i * 3];
		normVec[1] = pConvexHull->normalVec[i * 3 + 1];
		normVec[2] = pConvexHull->normalVec[i * 3 + 2];
		offValue = pConvexHull->offset[i];
		if ((DOT(pnt, normVec) + offValue) >= 0.0) return false;
	}

	return true;
}

void VOXSetOperation::voxelVisualization(PolygenMesh* voxelMesh, VOXELSET *voxSet)
{
	QMeshPatch *voxelModel = new QMeshPatch;
	voxelModel->SetIndexNo(voxelMesh->GetMeshList().GetCount()); //index begin from 0
	voxelMesh->meshList.AddTail(voxelModel);

	int fabVoxNum = 0;

	for (UINT k = 0; k < voxSet->m_res[2]; k++) {
		for (UINT j = 0; j < voxSet->m_res[1]; j++) {
			for (UINT i = 0; i < voxSet->m_res[0]; i++) {
				int index = _ijkToIndex(voxSet->m_res, i, j, k);
				if (voxSet->voxelArray[index].isFill == true) {
					fabVoxNum++;

					QMeshNode* Node = new QMeshNode;
					VOXEL thisVoxel = voxSet->voxelArray[index];
					Node->SetCoord3D(thisVoxel.voxelPos[0], thisVoxel.voxelPos[1], thisVoxel.voxelPos[2]);
					//std::cout << voxNode.posIndex[0] << std::endl;
					Node->SetMeshPatchPtr(voxelModel);
					Node->SetIndexNo(voxelModel->GetNodeList().GetCount() + 1);
					voxelModel->GetNodeList().AddTail(Node);

					//detect neighboor
					if (_isVoxelExisting(voxSet, i + 1, j, k) == false) { Node->voxelFlags[0] = true; Node->isBoundaryVoxelNode = true; }
					if (_isVoxelExisting(voxSet, i - 1, j, k) == false) { Node->voxelFlags[1] = true; Node->isBoundaryVoxelNode = true; }
					if (_isVoxelExisting(voxSet, i, j + 1, k) == false) { Node->voxelFlags[2] = true; Node->isBoundaryVoxelNode = true; }
					if (_isVoxelExisting(voxSet, i, j - 1, k) == false) { Node->voxelFlags[3] = true; Node->isBoundaryVoxelNode = true; }
					if (_isVoxelExisting(voxSet, i, j, k + 1) == false) { Node->voxelFlags[4] = true; Node->isBoundaryVoxelNode = true; }
					if (_isVoxelExisting(voxSet, i, j, k - 1) == false) { Node->voxelFlags[5] = true; Node->isBoundaryVoxelNode = true; }
				}
			}
		}
	}
}

void VOXSetOperation::voxelVisualization_layerUpdate(PolygenMesh* voxelMesh, VOXELSET *voxSet)
{
	QMeshPatch *voxelModel = (QMeshPatch*)voxelMesh->GetMeshList().GetHead();

	int *fillVoxelIndex;
	fillVoxelIndex = (int*)malloc(sizeof(int) * 3* (voxelModel->GetNodeNumber()));

	int nodeIndex = 0;
	for (UINT k = 0; k < voxSet->m_res[2]; k++) {
		for (UINT j = 0; j < voxSet->m_res[1]; j++) {
			for (UINT i = 0; i < voxSet->m_res[0]; i++) {
				int index = _ijkToIndex(voxSet->m_res, i, j, k);
				if (voxSet->voxelArray[index].isFill == true) {
					voxSet->voxelArray[index].isLayerDraw = true;
					fillVoxelIndex[nodeIndex * 3] = i;
					fillVoxelIndex[nodeIndex * 3 + 1] = j;
					fillVoxelIndex[nodeIndex * 3 + 2] = k;
					nodeIndex++;
				}
			}
		}
	}
	
	nodeIndex = 0;
	for (GLKPOSITION Pos = voxelModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelModel->GetNodeList().GetNext(Pos);
		for (int i = 0; i < 6; i++) Node->voxelFlags[i] = false;
		if (Node->isVoxelDraw == false) {
			int index = _ijkToIndex(voxSet->m_res, fillVoxelIndex[nodeIndex * 3],
				fillVoxelIndex[nodeIndex * 3 + 1], fillVoxelIndex[nodeIndex * 3 + 2]);
			voxSet->voxelArray[index].isLayerDraw = false;
		}
		nodeIndex++;
	}

	nodeIndex = 0;
	for (GLKPOSITION Pos = voxelModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelModel->GetNodeList().GetNext(Pos);
		for (int i = 0; i < 6; i++) Node->voxelFlags[i] = false;
		Node->isBoundaryVoxelNode = false;

		if (Node->isVoxelDraw == true) {
			int i = fillVoxelIndex[nodeIndex * 3];
			int j = fillVoxelIndex[nodeIndex * 3 + 1];
			int k = fillVoxelIndex[nodeIndex * 3 + 2];
			//if (_isVoxelExisting_layerupdate(voxSet, i + 1, j, k) == false) Node->voxelFlags[0] = true;
			//if (_isVoxelExisting_layerupdate(voxSet, i - 1, j, k) == false) Node->voxelFlags[1] = true;
			//if (_isVoxelExisting_layerupdate(voxSet, i, j + 1, k) == false) Node->voxelFlags[2] = true;
			//if (_isVoxelExisting_layerupdate(voxSet, i, j - 1, k) == false) Node->voxelFlags[3] = true;
			//if (_isVoxelExisting_layerupdate(voxSet, i, j, k + 1) == false) Node->voxelFlags[4] = true;
			//if (_isVoxelExisting_layerupdate(voxSet, i, j, k - 1) == false) Node->voxelFlags[5] = true;
			if (_isVoxelExisting_layerupdate(voxSet, i + 1, j, k) == false) { Node->voxelFlags[0] = true; Node->isBoundaryVoxelNode = true; }
			if (_isVoxelExisting_layerupdate(voxSet, i - 1, j, k) == false) { Node->voxelFlags[1] = true; Node->isBoundaryVoxelNode = true; }
			if (_isVoxelExisting_layerupdate(voxSet, i, j + 1, k) == false) { Node->voxelFlags[2] = true; Node->isBoundaryVoxelNode = true; }
			if (_isVoxelExisting_layerupdate(voxSet, i, j - 1, k) == false) { Node->voxelFlags[3] = true; Node->isBoundaryVoxelNode = true; }
			if (_isVoxelExisting_layerupdate(voxSet, i, j, k + 1) == false) { Node->voxelFlags[4] = true; Node->isBoundaryVoxelNode = true; }
			if (_isVoxelExisting_layerupdate(voxSet, i, j, k - 1) == false) { Node->voxelFlags[5] = true; Node->isBoundaryVoxelNode = true; }
		}
		nodeIndex++;
	}
	free(fillVoxelIndex);
}

void VOXSetOperation::voxelVisualizationwithPlatform(PolygenMesh* voxelMesh, VOXELSET *voxSet, QMeshPatch *platform)
{
	voxelVisualization(voxelMesh, voxSet);

	QMeshPatch *voxelModel = (QMeshPatch *)voxelMesh->GetMeshList().GetHead();
	//add platform node into voxelModel
	for (GLKPOSITION Pos = platform->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)platform->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp[0], pp[1], pp[2]);
		QMeshNode *platformNode = new QMeshNode;
		platformNode->SetCoord3D(pp[0],pp[1],pp[2]);
		platformNode->SetMeshPatchPtr(voxelModel);
		platformNode->SetIndexNo(voxelModel->GetNodeList().GetCount() + 1);
		voxelModel->GetNodeList().AddTail(platformNode);
		platformNode->isPlatformNode = true;
	}
}

bool VOXSetOperation::_isVoxelExisting(VOXELSET *voxSet, int ii, int jj, int kk)
{
	if ((ii<0) || (ii >= (int)(voxSet->m_res[0]))
		|| (jj<0) || (jj >= (int)(voxSet->m_res[1]))
		|| (kk<0) || (kk >= (int)(voxSet->m_res[2]))) return false;

	int index = _ijkToIndex(voxSet->m_res, ii, jj, kk);
	if (voxSet->voxelArray[index].isFill == false) return false;

	return true;
}

bool VOXSetOperation::_isVoxelExisting_layerupdate(VOXELSET *voxSet, int ii, int jj, int kk)
{
	if ((ii<0) || (ii >= (int)(voxSet->m_res[0]))
		|| (jj<0) || (jj >= (int)(voxSet->m_res[1]))
		|| (kk<0) || (kk >= (int)(voxSet->m_res[2]))) return false;

	int index = _ijkToIndex(voxSet->m_res, ii, jj, kk);
	if (voxSet->voxelArray[index].isLayerDraw == false) return false;

	return true;
}

void VOXSetOperation::layerInformationUpdate(PolygenMesh* voxelMesh, VOXELSET *voxSet)
{
	QMeshPatch* voxelModel = (QMeshPatch*)voxelMesh->GetMeshList().GetHead();
	/*int voxelMeshNodeNum = 0;
	for (GLKPOSITION Pos = voxelModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelModel->GetNodeList().GetNext(Pos);
		if (!Node->isPlatformNode) voxelMeshNodeNum++;
	}*/
	
	int index = 0;
	for (GLKPOSITION Pos = voxelModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelModel->GetNodeList().GetNext(Pos);
		if (!Node->isPlatformNode) {
			Node->voxelLayerIndex = voxSet->nodeArray[index].layerIndex;
			index++;
		}
	}
	if (index != voxSet->nodeNum) { std::cout << "Error, element number is not equal!" << std::endl; return; }
}

void VOXSetOperation::expandingVoxelSet(VOXELSET *voxSet) {
	bool *bGrid; bool*neighborGrid;
	const int bndNeighborDelta[6][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,-1,0 },{ 0,0,1 },{ 0,0,-1 } };

	bGrid = (bool*)malloc(sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));
	memset(bGrid, 0, sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));

	neighborGrid = (bool*)malloc(sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));
	memset(neighborGrid, 0, sizeof(bool)*(voxSet->m_res[0])*(voxSet->m_res[1])*(voxSet->m_res[2]));

	for (UINT k = 0; k < voxSet->m_res[2]; k++) {
		for (UINT j = 0; j < voxSet->m_res[1]; j++) {
			for (UINT i = 0; i < voxSet->m_res[0]; i++) {
				int index = _ijkToIndex(voxSet->m_res, i, j, k);
				if (voxSet->voxelArray[index].isFill == true) {
					bGrid[index] = true;
					for (int ii = 0; ii < 6; ii++) {
						int voxelIndex = _ijkToIndex(voxSet->m_res, i + bndNeighborDelta[ii][0]
							, j + bndNeighborDelta[ii][1], k + bndNeighborDelta[ii][2]);
						if (voxelIndex < 0) continue;
						else neighborGrid[voxelIndex] = true;
					}
				}
			}
		}
	}

	int solidVoxelNum = 0;
	for (UINT k = 0; k < voxSet->m_res[2]; k++) {
		for (UINT j = 0; j < voxSet->m_res[1]; j++) {
			for (UINT i = 0; i < voxSet->m_res[0]; i++) {
				int index = _ijkToIndex(voxSet->m_res, i, j, k);
				if (bGrid[index] || neighborGrid[index]) {
					solidVoxelNum++;
				}
			}
		}
	}

	voxSet->nodeNum = solidVoxelNum;
	free(voxSet->nodeArray);
	voxSet->nodeArray = (VOXELSETNode*)malloc(sizeof(VOXELSETNode)*solidVoxelNum);

	solidVoxelNum = 0;
	for (UINT k = 0; k<voxSet->m_res[2]; k++) {
		for (UINT j = 0; j<voxSet->m_res[1]; j++) {
			for (UINT i = 0; i<voxSet->m_res[0]; i++) {
				int index = _ijkToIndex(voxSet->m_res, i, j, k);

				if (bGrid[index] || neighborGrid[index]) {
					voxSet->voxelArray[index].isFill = true;
					voxSet->voxelArray[index].voxelPos[0] = voxSet->origin[0] + ((float)i + 0.5f)*(voxSet->width);
					voxSet->voxelArray[index].voxelPos[1] = voxSet->origin[1] + ((float)j + 0.5f)*(voxSet->width);
					voxSet->voxelArray[index].voxelPos[2] = voxSet->origin[2] + ((float)k + 0.5f)*(voxSet->width);

					voxSet->nodeArray[solidVoxelNum].materialIndex = 1;
					voxSet->nodeArray[solidVoxelNum].layerIndex = 1;
					voxSet->nodeArray[solidVoxelNum].posIndex[0] = i;
					voxSet->nodeArray[solidVoxelNum].posIndex[1] = j;
					voxSet->nodeArray[solidVoxelNum].posIndex[2] = k;
					voxSet->nodeArray[solidVoxelNum].layerIndex = j + 1;
					solidVoxelNum++;

					//-------------------------------------------------------------------------------------------------
					//	Build the connection between nodeArray and voxelArray
					voxSet->voxelArray[index].nodeArrayIndex = solidVoxelNum;
				}
			}
		}
	}

	//-------------------------------------------------------------------------------------------------
	//	Step 4: free the memory
	free(bGrid);
	free(neighborGrid);
}

void VOXSetOperation::transformVoxeltoMeshField(VOXELSET *voxSet, QMeshPatch *patch) {
	double bbox[6];
	patch->ComputeBoundingBox(bbox);

	double pp[3],voxPP[3];
	int index[3], voxelIndex, neighborIndex;
	const int bndNeighborDelta[6][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,-1,0 },{ 0,0,1 },{ 0,0,-1 } };
	const int accumNeighborDelta[][3] = { { -1,0,0 },{ 1,0,0 },{ 0,1,0 },{ 0,0,-1 },{ 0,0,1 },{ -1,1,0 },{ 1,1,0 },{ 0,1,-1 },{ 0,1,1 } };

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) index[i] = floor((pp[i] - voxSet->origin[i]) / double(voxSet->width));
		voxelIndex = _ijkToIndex(voxSet->m_res, index[0], index[1], index[2]);
		/*std::cout << pp[0] << ", " << pp[1] << ", " << pp[2] << std::endl;
		std::cout << voxSet->voxelArray[voxelIndex].voxelPos[0] << ", " 
			<< voxSet->voxelArray[voxelIndex].voxelPos[1]<< ", " << 
			voxSet->voxelArray[voxelIndex].voxelPos[2]<< std::endl<< std::endl;*/
		if (voxSet->voxelArray[voxelIndex].isFill == false) {
			std::cout << "Seems no voxel is connected with this node, you should spend more voxel layer!" << std::endl;
			return;
		}

		int finalVoxelIndex = voxelIndex;
		bool findConnectVoxel = false;
		VOXEL thisvoxel = voxSet->voxelArray[voxelIndex];

		for (int i = 0; i < 3; i++) voxPP[i] = thisvoxel.voxelPos[i];
		double voxDistance = DISTANCE(pp, voxPP);
		if (sqrt(voxDistance) > voxSet->width / 2) {
			for (int j = 0; j < 9; j++) {

				neighborIndex = _ijkToIndex(voxSet->m_res, index[0] + accumNeighborDelta[j][0]
					, index[1] + accumNeighborDelta[j][1], index[2] + accumNeighborDelta[j][2]);
				if (neighborIndex < 0) continue;
				else if(voxSet->voxelArray[neighborIndex].isFill == false) continue;

				VOXEL neighborVoxel = voxSet->voxelArray[neighborIndex];
				for (int i = 0; i < 3; i++) voxPP[i] = neighborVoxel.voxelPos[i];
				double neighvoxDistance = DISTANCE(pp, voxPP);
				if (voxDistance > neighvoxDistance) {
					std::cout << "Neighbor voxel is more close! update the voxel" << std::endl;
					voxDistance = neighvoxDistance;
					finalVoxelIndex = neighborIndex;
				}
			}
		}
		//Node->guideFieldValue = voxSet->nodeArray[(voxSet->voxelArray[voxelIndex].nodeArrayIndex)].layerIndex;
		Node->guideFieldValue = voxSet->nodeArray[(voxSet->voxelArray[finalVoxelIndex].nodeArrayIndex)].layerIndex;
		std::cout << Node->guideFieldValue << std::endl;
	}
}

void VOXSetOperation::getVoxelPosbyIndex(VOXELSET *voxSet, VOXELSETNode *Node, double x, double y, double z)
{
	x = voxSet->origin[0] + ((float)(Node->posIndex[0]) + 0.5f)*(voxSet->width);
	y = voxSet->origin[1] + ((float)(Node->posIndex[1]) + 0.5f)*(voxSet->width);
	z = voxSet->origin[2] + ((float)(Node->posIndex[2]) + 0.5f)*(voxSet->width);
}

void VOXSetOperation::transformMeshFieldValuetoVoxel(VOXELSET *voxSet, QMeshPatch *tetPatch, QMeshPatch *voxelPatch) {

	/*build Hashing table - used to find the nearest node in tetPatch with voxSet*/

	//---Step1: compute bound and expand with ratio
	double expandRatio = 1.2, density = 10;
	double hBound[6] = { -1.0e+10,1.0e+10,-1.0e+10,1.0e+10,-1.0e+10,1.0e+10 }; //xmax,xmin,ymax,ymin,zmax,zmin
	double pp[3];
	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *tetNode = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
		tetNode->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) {
			if (hBound[2 * i] < pp[i]) hBound[2 * i] = pp[i];
			if (hBound[2 * i + 1] > pp[i]) hBound[2 * i + 1] = pp[i];}
	}
	double boxsize[3] = { hBound[0] - hBound[1], hBound[2] - hBound[3] ,hBound[4] - hBound[5] };
	
	for (int i = 0; i < 3; i++) { //expand the boundary
		hBound[2 * i] = boxsize[i] / 2 + expandRatio*boxsize[i];
		hBound[2 * i + 1] = boxsize[i] / 2 - expandRatio*boxsize[i];
	}

	//---Step2: detect average edge distance and build the hash grid
	double averLength = 0.0;
	for (GLKPOSITION Pos = tetPatch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *thisEdge = (QMeshEdge*)tetPatch->GetEdgeList().GetNext(Pos);
		averLength += thisEdge->CalLength();}
	averLength /= tetPatch->GetEdgeNumber();

	int hashDim[3] = { (int)ceil(boxsize[0] * density / averLength),
		(int)ceil(boxsize[1] * density / averLength), (int)ceil(boxsize[2] * density / averLength) };

	double res[3];
	for (int i = 0; i < 3; i++) res[i] = (hBound[2 * i] - hBound[2 * i + 1]) / hashDim[i];
	std::cout << std::endl << "Hash grid dimension = " << hashDim[0] << ", "<< hashDim[1] << ", " << hashDim[2] <<
		". Res = "<< res[0] << ", " << res[1] << ", " << res[2] << std::endl;

	std::vector<std::vector<int>> hashTable(hashDim[0] * hashDim[1] * hashDim[2]);

	/*Using Node to build the hash table*/
	//int* nodeHashIndex; //this install the hash box index of each node, x -> y -> z
	//nodeHashIndex = new int[3 * tetPatch->GetNodeNumber()];
	//int nodeIndex = 0, tableIndex;
	//for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode *tetNode = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
	//	tetNode->GetCoord3D(pp[0], pp[1], pp[2]);
	//	
	//	//build node -> hashbox index
	//	for (int i = 0; i < 3; i++)
	//		nodeHashIndex[nodeIndex * 3 + i] = floor((pp[i] - hBound[2 * i + 1]) / res);
	//
	//	//build hashbox -> nodeindex
	//	for (int i = 0; i < 3; i++)
	//		if (nodeHashIndex[nodeIndex * 3 + i] == hashDim[i] ||
	//			nodeHashIndex[nodeIndex * 3 + i] < 0) std::cout << "ERROR! Out of Range!" << std::endl;
	//	tableIndex = nodeHashIndex[nodeIndex * 3] + nodeHashIndex[nodeIndex * 3 + 1] * hashDim[0] +
	//		nodeHashIndex[nodeIndex * 3 + 2] * hashDim[0] * hashDim[1];
	//	hashTable[tableIndex].push_back(nodeIndex);
	//	
	//	nodeIndex++;
	//}
	//std::cout << "HashTable is bulit." << std::endl;

	/*Using Tetrahedral to build the hash table*/
	int tetIndex = 0;
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *thisTet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		thisTet->SetIndexNo(tetIndex);

		//building tetrahedral bounding box
		double tetBound[6] = { -1.0e+10,1.0e+10,-1.0e+10,1.0e+10,-1.0e+10,1.0e+10 }; //xmax,xmin,ymax,ymin,zmax,zmin
		for (int j = 0; j < 4; j++) {
			thisTet->GetNodeRecordPtr(j + 1)->GetCoord3D(pp[0], pp[1], pp[2]);

			for (int i = 0; i < 3; i++) {
				if (tetBound[2 * i] < pp[i]) tetBound[2 * i] = pp[i];
				if (tetBound[2 * i + 1] > pp[i]) tetBound[2 * i + 1] = pp[i];
			}
		}

		int tetHashIndex[6];
		for (int i = 0; i < 3; i++) {
			tetHashIndex[2 * i] = (int)floor((tetBound[2 * i] - hBound[2 * i + 1]) / res[i]);
			tetHashIndex[2 * i + 1] = (int)floor((tetBound[2 * i + 1] - hBound[2 * i + 1]) / res[i]);
		}

		for (int i = 0; i < 3; i++) {
			if (tetHashIndex[2 * i] == hashDim[i] || tetHashIndex[2 * i] < 0) 
				std::cout << "ERROR! Out of Range!" << std::endl;
			if (tetHashIndex[2 * i + 1] == hashDim[i] || tetHashIndex[2 * i + 1] < 0) 
				std::cout << "ERROR! Out of Range!" << std::endl;
		}
			
		for (int i = tetHashIndex[1]; i < tetHashIndex[0] + 1; i++) {
			for (int j = tetHashIndex[3]; j < tetHashIndex[2] + 1; j++) {
				for (int k = tetHashIndex[5]; k < tetHashIndex[4] + 1; k++) {
					int tableIndex = i + j * hashDim[0] + k * hashDim[0] * hashDim[1];

					//In this code, we only install the tetrahedral index of every hash box, 
					//however not save the hash box index for each tetrahedral here. (Guoxin 10/18/2019)
					hashTable[tableIndex].push_back(tetIndex); 
				}
			}
		}

		tetIndex++;
	}

	/*build PQP model, this is used to check those voxel outside the volume mesh*/
	PQP_Model *pqpModel = new PQP_Model();
	pqpModel->BeginModel();
	int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];

	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

		//only build pqp for boundary mesh
		if (Face->GetLeftTetra() != nullptr && Face->GetRightTetra() != nullptr) continue;

		Face->boundIndex = index;
		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);
		
		pqpModel->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel->EndModel();

	////testing PQP speed
	//PQP_REAL p[3];
	//for (int i = 0; i < voxSet->nodeNum; i++) {
	//	thisVoxel = voxSet->voxelArray[i];

	//	if (thisVoxel.isFill) {
	//		PQP_DistanceResult dres;	dres.last_tri = pqpModel->last_tri;
	//		for (int j = 0; j < 3; j++) p[j] = thisVoxel.voxelPos[j];
	//		PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);
	//		float closestPt[3];	// closest point
	//		closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];
	//		int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero
	//		float minDist = dres.Distance();	//	minimal distance
	//	}
	//}

	/*Searching algorithm*/
	std::cout << "----Begin hash grid searching for each voxel----" << std::endl;

	//VOXEL thisVoxel;
	int voxelIndex = 0;
	int htNumSum = 0;

	int Core = 12; int voxelNum = voxSet->m_res[0] * voxSet->m_res[1] * voxSet->m_res[2];
	int EachCore = (int)floor(voxelNum / Core) + 1;

#pragma omp parallel   
	{
	#pragma omp for  
		for (int omptime = 0; omptime < Core; omptime++) {
			/*int min = EachCore*omptime;
			int max = EachCore*(omptime + 1); if (max > voxelNum) max = voxelNum;*/
			//std::cout << min << "," << max << std::endl;


			//for (int i = 0; i < voxSet->nodeNum; i++) {
			//for (int i = min; i < max; i++) {
			for (int i = 0; i < voxelNum; i++) {
				if (i % Core != omptime) continue;

				if (!voxSet->voxelArray[i].isFill) continue;
				//thisVoxel.inside = false;

				Eigen::Vector3d voxelPos;
				for (int j = 0; j < 3; j++) voxelPos(j) = voxSet->voxelArray[i].voxelPos[j];

				/*Direct search method*/
				/*for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra *Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

				for (int k = 0; k < 4; k++)
				Tet->GetNodeRecordPtr(k + 1)->GetCoord3D(tetPos(0, k), tetPos(1, k), tetPos(2, k));
				if (detectVoxelInsideTetrahedral(voxelPos, tetPos)) {
				std::cout << "T1 No. " << voxelIndex << " Voxel. Traversing, finding tetrahedral index = " << Tet->GetIndexNo() << std::endl;
				thisVoxel.inside = true;
				break;
				}
				}
				if (thisVoxel.inside == false) {
				if (detectVoxelOutsideMesh(voxelPos, pqpModel, tetPatch))
				std::cout << "T2 No. " << voxelIndex << " Voxel. Traversing, outside mesh. Voxel coordinate: "
				<< voxelPos(0) << "," << voxelPos(1) << "," << voxelPos(2) << std::endl;
				else std::cout << "T3 ERROR! No. " << voxelIndex << " Voxel. Inside mesh however cannot find tetrahedral!" << std::endl;
				}*/

				/*finding relevent hashbox and get the closest point - voxel*/
				int hashIndex[3];
				for (int j = 0; j < 3; j++) hashIndex[j] = (int)floor((voxelPos(j) - hBound[2 * j + 1]) / res[j]);
				for (int j = 0; j < 3; j++) { if (hashIndex[j] == hashDim[j] || hashIndex[j] < 0) std::cout << "ERROR! Out of Range!" << std::endl; }

				int voxelHashIndex = hashIndex[0] + hashIndex[1] * hashDim[0] + hashIndex[2] * hashDim[0] * hashDim[1];
				int htEleNum = hashTable[voxelHashIndex].size(); //hash table element number
																 /*if the hash grid of this voxel contains no tetrahedral, we first check if it is outside the mesh*/
				htNumSum += htEleNum;
				if (htEleNum == 0) {
					voxSet->voxelArray[i].fieldValue = computeVoxelValuebyNearestTriangle(voxelPos, pqpModel, tetPatch);

					//if (directSearchMethodfromVoxeltoTet(tetPatch, voxelPos)) std::cout << "Although Hashing Table contains no tetgen, The neighbor is still find by direct searching!" << std::endl;
					/*else if (detectVoxelOutsideMesh(voxelPos, pqpModel, tetPatch))
						std::cout << "ERROR, voxel is inside the tet mesh, the hash searching should not give back NULL! Voxel index - " << voxelIndex << std::endl;*/
					//else std::cout << "This voxel is outside the tet mesh! Voxel index - " << voxelIndex << std::endl;
				}
				else {
					bool findTet = false;

					for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
						QMeshTetra *Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

						for (int j = 0; j < htEleNum; j++) {
							if (Tet->GetIndexNo() == hashTable[voxelHashIndex][j]) {

								Eigen::MatrixXd tetPos(3, 4);
								for (int k = 0; k < 4; k++) Tet->GetNodeRecordPtr(k + 1)->GetCoord3D(tetPos(0, k), tetPos(1, k), tetPos(2, k));
								if (detectVoxelInsideTetrahedral(voxelPos, tetPos)) {
									Eigen::Vector4d tetValue;
									for (int k = 0; k < 4; k++) tetValue(k) = Tet->GetNodeRecordPtr(k + 1)->guideFieldValue;

									voxSet->voxelArray[i].fieldValue = computeVoxelValuebyTetrahedral(voxelPos, tetPos, tetValue);
									findTet = true; break;
									//std::cout << "SUCCESS!, this voxel is inside the tet mesh, Voxel index - " << voxelIndex << ". Tetrahedral index - " << Tet->GetIndexNo() << std::endl;
								}

							}
						}

						if (findTet) break;
					}
					if (!findTet) {

						voxSet->voxelArray[i].fieldValue = computeVoxelValuebyNearestTriangle(voxelPos, pqpModel, tetPatch);
						//std::cout << "Tetrahedral not found by Hashing Table! Voxel index - " << voxelIndex << std::endl;
						//if (directSearchMethodfromVoxeltoTet(tetPatch, voxelPos)) std::cout << "Althouth neighbor tetgens do not contain this node, the neighbor is find by direct searching!" << std::endl;
						/*else if (detectVoxelOutsideMesh(voxelPos, pqpModel, tetPatch))
							std::cout << "ERROR, this voxel is inside the tet mesh however cannot find related tetrahedral! Voxel index - " << voxelIndex << std::endl;*/
						//else std::cout << "This voxel is outside the tet mesh! Voxel index - " << voxelIndex << std::endl;
					}
				}

				/*finding relevant hash box and get the closest point - voxel*/
				/*if (!detectVoxelOutsideMesh(voxelPos, pqpModel, tetPatch)) {
					if (directSearchMethodfromVoxeltoTet(tetPatch, voxelPos) != -1) std::cout << "ERROR! The neighbor is find!" << std::endl;
				}
				else {
					int hashIndex[3];
					for (int j = 0; j < 3; j++) hashIndex[j] = (int)floor((voxelPos(j) - hBound[2 * j + 1]) / res[j]);
					for (int j = 0; j < 3; j++) { if (hashIndex[j] == hashDim[j] || hashIndex[j] < 0) std::cout << "ERROR! Out of Range!" << std::endl; }
					int voxelHashIndex = hashIndex[0] + hashIndex[1] * hashDim[0] + hashIndex[2] * hashDim[0] * hashDim[1];
					int htEleNum = hashTable[voxelHashIndex].size(); //hash table element number
					if (htEleNum == 0) {
						std::cout << "ERROR, voxel is inside the tet mesh, the hash searching should not give back NULL! Voxel index - " << voxelIndex << std::endl;
						if (directSearchMethodfromVoxeltoTet(tetPatch, voxelPos) != -1) std::cout << "However, The neighbor is find by direct search!" << std::endl;
					}
					bool findTet = false
					for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
						QMeshTetra *Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
						for (int j = 0; j < htEleNum; j++) {
							if (Tet->GetIndexNo() == hashTable[voxelHashIndex][j]) {
								Eigen::MatrixXd tetPos(3, 4);
								for (int k = 0; k < 4; k++) Tet->GetNodeRecordPtr(k + 1)->GetCoord3D(tetPos(0, k), tetPos(1, k), tetPos(2, k));
								if (detectVoxelInsideTetrahedral(voxelPos, tetPos)) {
									//std::cout << "SUCCESS!, this voxel is inside mesh and find correspond tet, Voxel index - " << voxelIndex << ". Tetrahedral index - " << Tet->GetIndexNo() << std::endl;
									findTet = true; break;
								}
							}
						}
						if (findTet) break;
						if (!findTet) {}
							//std::cout << "ERROR, Tetrahedral not found by Hashing Table! Voxel index - " << voxelIndex << std::endl;
					}
				}*/

				/*Always update this - although not working for the parallel computing case*/
				voxelIndex++;
			}
		}
	}

	std::cout << std::endl << "----Finish searching!----Average searching tet number for voxel is "<< htNumSum / voxelIndex << std::endl;

	//transfer Voxel to Patch for Visualization
	Eigen::VectorXd fValue(voxSet->nodeNum);
	voxelIndex = 0;
	for (int i = 0; i < voxelNum; i++) {
		if (!voxSet->voxelArray[i].isFill) continue;

		fValue(voxelIndex) = voxSet->voxelArray[i].fieldValue;
		voxelIndex++;
	}
	//std::cout << fValue << std::endl;

	int layerNum = 50;
	//average the 
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < voxSet->nodeNum; i++) { if (minPhi > fValue(i)) minPhi = fValue(i); if (maxPhi < fValue(i)) maxPhi = fValue(i);}
	double range = maxPhi - minPhi;

	for (int i = 0; i<voxSet->nodeNum; i++) fValue(i) = 1 - (fValue(i) - minPhi) / range;

	// std sort
	Eigen::VectorXi ind;
	Eigen::VectorXd sorted_vec;
	ind = Eigen::VectorXi::LinSpaced(fValue.size(), 0, fValue.size() - 1); //[0 1 2 3 ... N-1]
	auto rule = [fValue](int i, int j)->bool {
		return fValue(i)<fValue(j); //sorting rules
	};
	std::sort(ind.data(), ind.data() + ind.size(), rule);
	sorted_vec.resize(fValue.size());

	int searching = floor(fValue.size() / layerNum);
	for (int i = 0; i < fValue.size(); i++) sorted_vec(i) = fValue(ind(i));

	Eigen::VectorXd isoCurveValue(layerNum);
	for (int i = 0; i < layerNum; i++) 	isoCurveValue(i) = sorted_vec(searching * i + 1);

	voxelIndex = 0;
	for (GLKPOSITION Pos = voxelPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *voxelNode = (QMeshNode*)voxelPatch->GetNodeList().GetNext(Pos);
		
		if(voxelNode->isPlatformNode) continue;
		int layerIndex = 0; 
		double value = fValue(voxelIndex);
		for (layerIndex = 0; layerIndex < layerNum; layerIndex++) {
			if (value < isoCurveValue(layerIndex)) {
				break;
			}
		}

		voxelNode->voxelLayerIndex = layerNum - layerIndex;
		voxelIndex++;
	}
}

bool VOXSetOperation::directSearchMethodfromVoxeltoTet(QMeshPatch *tetPatch, Eigen::Vector3d &voxelPos) {
	bool inside = false;
	Eigen::MatrixXd tetPos(3, 4);

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		for (int k = 0; k < 4; k++)
			Tet->GetNodeRecordPtr(k + 1)->GetCoord3D(tetPos(0, k), tetPos(1, k), tetPos(2, k));

		if (detectVoxelInsideTetrahedral(voxelPos, tetPos)) {
			//std::cout << "T1 No. " << voxelIndex << " Voxel. Traversing, finding tetrahedral index = " << Tet->GetIndexNo() << std::endl;
			inside = true;
			break;
		}
	}
	
	return inside; //find -> true, otherwise -> false
}

double VOXSetOperation::ScTP(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c) {
	return a.dot(b.cross(c));
}

bool VOXSetOperation::detectVoxelInsideTetrahedral(Eigen::Vector3d &nodePos, Eigen::MatrixXd &tetPos) {

	/* https://www.cdsimpson.net/2014/10/barycentric-coordinates.html */

	Eigen::Vector3d vap = nodePos - tetPos.col(0);
	Eigen::Vector3d vbp = nodePos - tetPos.col(1);

	Eigen::Vector3d vab = tetPos.col(1) - tetPos.col(0);
	Eigen::Vector3d vac = tetPos.col(2) - tetPos.col(0);
	Eigen::Vector3d vad = tetPos.col(3) - tetPos.col(0);

	Eigen::Vector3d vbc = tetPos.col(2) - tetPos.col(1);
	Eigen::Vector3d vbd = tetPos.col(3) - tetPos.col(1);

	// ScTP computes the scalar triple product
	double va6 = ScTP(vbp, vbd, vbc);
	double vb6 = ScTP(vap, vac, vad);
	double vc6 = ScTP(vap, vad, vab);
	double vd6 = ScTP(vap, vab, vac);
	double v6 = 1 / ScTP(vab, vac, vad);

	double baryCent[4] = { va6*v6, vb6*v6, vc6*v6, vd6*v6 };

	double sum = baryCent[0] + baryCent[1] + baryCent[2] + baryCent[3];
	if (abs(sum-1) > 0.00001) std::cout << "Compute baryCent ERROR!" << std::endl;

	if(baryCent[0] >=0 && baryCent[0] <= 1 && baryCent[1] >= 0 && baryCent[1] <= 1 
		&& baryCent[2] >= 0 && baryCent[2] <= 1 && baryCent[3] >= 0 && baryCent[3] <= 1) return true;
	else return false;
}

double VOXSetOperation::computeVoxelValuebyTetrahedral(Eigen::Vector3d &nodePos, Eigen::MatrixXd &tetPos, Eigen::Vector4d &tetValue) {
	double fieldValue = 0;

	Eigen::Vector3d vap = nodePos - tetPos.col(0);
	Eigen::Vector3d vbp = nodePos - tetPos.col(1);

	Eigen::Vector3d vab = tetPos.col(1) - tetPos.col(0);
	Eigen::Vector3d vac = tetPos.col(2) - tetPos.col(0);
	Eigen::Vector3d vad = tetPos.col(3) - tetPos.col(0);

	Eigen::Vector3d vbc = tetPos.col(2) - tetPos.col(1);
	Eigen::Vector3d vbd = tetPos.col(3) - tetPos.col(1);

	// ScTP computes the scalar triple product
	double va6 = ScTP(vbp, vbd, vbc);
	double vb6 = ScTP(vap, vac, vad);
	double vc6 = ScTP(vap, vad, vab);
	double vd6 = ScTP(vap, vab, vac);
	double v6 = 1 / ScTP(vab, vac, vad);

	double baryCent[4] = { va6*v6, vb6*v6, vc6*v6, vd6*v6 };

	for (int i = 0; i < 4; i++) fieldValue += baryCent[i] * tetValue(i);
	//std::cout << fieldValue << std::endl;
	return fieldValue;
}

bool VOXSetOperation::detectVoxelOutsideMesh(Eigen::Vector3d &nodePos, PQP_Model *pqpModel, QMeshPatch *tetPatch) {

	PQP_DistanceResult dres;	dres.last_tri = pqpModel->last_tri;
	double p[3];
	for (int j = 0; j < 3; j++) p[j] = nodePos(j);
	PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);
	//float closestPt[3];	// closest point
	//closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];
	//float minDist = dres.Distance();	//	minimal distance

	int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero
	//std::cout << minTriId << std::endl;

	double v[3], D, pp1[3], pp[3];
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *thisFace = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

		if (thisFace->boundIndex == minTriId) {

			if (thisFace->GetLeftTetra() != nullptr && thisFace->GetRightTetra() != nullptr)
				std::cout << "This is not a boundary face, ERROR!" << std::endl;

			thisFace->GetPlaneEquation(v[0], v[1], v[2], D);
			thisFace->CalCenterPos(pp1[0], pp1[1], pp1[2]);
			for (int i = 0; i < 3; i++) pp[i] = nodePos(i) - pp1[i];
			double dProduct = DOT(pp, v);

			if (dProduct > 0) return true; //outside the mesh
			else return false; //inside the mesh
		}
	}

}

double VOXSetOperation::computeVoxelValuebyNearestTriangle(Eigen::Vector3d &nodePos, PQP_Model *pqpModel, QMeshPatch *tetPatch) {

	GLKGeometry geo; 	double fieldValue = 0;
	PQP_DistanceResult dres;	dres.last_tri = pqpModel->last_tri;
	double p[3];
	for (int j = 0; j < 3; j++) p[j] = nodePos(j);
	PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);
	double closestPt[3];	// closest point on the surface
	closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];
	//float minDist = dres.Distance();	//	minimal distance

	int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero

	double v[3], D, pp1[3], pp[3];
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *thisFace = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

		if (thisFace->boundIndex == minTriId) {
			//First check the distance of clostPt and thisFace
			double p1[3], p2[3], p3[3];
			thisFace->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
			thisFace->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
			thisFace->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

			if (geo.Distance_to_Triangle(closestPt, p1, p2, p3) > 0.001) std::cout << "ERROR, this node is not located in the plane!" << std::endl;
			//std::cout << geo.Distance_to_Triangle(closestPt, p1, p2, p3) <<std::endl;

			//begin compute barycentric-coordinates of triangles
			double area[4] = { geo.SpatialTriangleArea(closestPt,p2,p3), geo.SpatialTriangleArea(closestPt,p1,p3), 
				geo.SpatialTriangleArea(closestPt,p2,p1), geo.SpatialTriangleArea(p1,p2,p3) };
			double baryCent[3] = { area[0] / area[3], area[1] / area[3] , area[2] / area[3] };

			if (baryCent[0] >= 0 && baryCent[0] <= 1 && baryCent[1] >= 0 && baryCent[1] <= 1 && baryCent[2] >= 0 && baryCent[2] <= 1) {
				for (int i = 0; i < 3; i++) fieldValue += baryCent[i] * thisFace->GetNodeRecordPtr(i)->guideFieldValue;
			}
			break;
		}
	}
	if (abs(fieldValue) < 0.00001) std::cout << "ERROR compute fieldValue for this node" << std::endl;
	//std::cout << fieldValue << std::endl;
	return fieldValue;
}