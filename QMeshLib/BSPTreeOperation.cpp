#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h> 

#include "PMBody.h"
#include "BSPTreeOperation.h"

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

#define MIN(a,b)	((a)<(b))?(a):(b)
#define MAX(a,b)	((a)>(b))?(a):(b)

#define MAXNUM_OF_BSPNODE_ONTREE		50000000	// Specifying the maximal num of nodes,
                                                    // where different number should be use with different PCs according to the memory
#define MAXLEVEL_OF_STACK				1000        // Specifying the maximal level of stack for tree-traversal

#define CROSS_PRODUCT(vec1,vec2,resVec)		{	resVec[0]=vec1[1]*vec2[2]-vec1[2]*vec2[1];	\
												resVec[1]=vec1[2]*vec2[0]-vec1[0]*vec2[2];	\
												resVec[2]=vec1[0]*vec2[1]-vec1[1]*vec2[0];	}
#define DOT_PRODUCT(vec1,vec2)				(vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2])
#define EPS						1.0e-5f
#define EPS_DOUBLEPRECISION		1.0e-8f

#define LP_FEASIBLEREGIONCHECK_WITH_FEASIBLEPNT				true
//#define BY_USING_CONVENTIAL_LP_FOR_FEASIBLEREGIONCHECK		true

//#define FAST_BSPTREE_CONSTRUCTION		true	//  The fast construction may lead to "void" region inside the solid
//#define GLOBALTRGL_COUNT_LEVEL		15      //  This macro is for the function of statistics
#ifdef GLOBALTRGL_COUNT_LEVEL
int globalTotalTrglNum;
#endif

typedef struct bspTreeCollapseHashNode {
	UINT leftID,rightID;
	UINT data;
}BSPTREEHashNode;

long totalTimeForFeasibilityCheck;
int totalNumberOfLP_Taken;
int totalNumberOfSimp_Projection;

BSPSolidOperation::BSPSolidOperation(void)
{
}

BSPSolidOperation::~BSPSolidOperation(void)
{
}

void BSPSolidOperation::_bspTreeFreeMemory(BSPTREE *&ptrBSPTree)
{
	if (ptrBSPTree == NULL) return;
	//if (bCUDADeviceMemory) {
	//	//        if (ptrBSPTree->planeArraySize!=0) {cudaFree(ptrBSPTree->planeArray); }
	//	//        if (ptrBSPTree->nodeArray!=0) {cudaFree(ptrBSPTree->nodeArray); }
	//}
	else {
		if (ptrBSPTree->planeArraySize != 0) { free(ptrBSPTree->planeArray); }
		if (ptrBSPTree->nodeArray != 0) { free(ptrBSPTree->nodeArray); }
	}
	free(ptrBSPTree);	ptrBSPTree = NULL;
}

void BSPSolidOperation::BooleanOperation(BSPTREE *treeAptr, float solidABndBox[], 
										 BSPTREE *treeBptr, float solidBBndBox[], short nOperationType,
										 BSPTREE *&treeResPtr, float resBndBox[])
{
	float computingBndBox[6];

	_booleanOperationOnBoundingBox(solidABndBox, solidBBndBox, nOperationType, resBndBox);
	_booleanOperationOnBoundingBox(solidABndBox, solidBBndBox, 0, computingBndBox);

	UINT stIndex=_booleanOperationByNaiveMerge(treeAptr, treeBptr, nOperationType, treeResPtr); 	//_expandMergedTree(treeResPtr);
	_bspTreeFreeMemory(treeAptr);		_bspTreeFreeMemory(treeBptr);	
	printf("Feasibility check stating from %d ... ...\n",(int)stIndex);
	_reduceMergedTree(computingBndBox,treeResPtr,stIndex);

	//_booleanOperationByTreeTravel(computingBndBox,treeAptr, treeBptr, nOperationType, treeResPtr);
	//_bspTreeFreeMemory(treeAptr);		_bspTreeFreeMemory(treeBptr);	
}

void BSPSolidOperation::BSPTreeCollapse(BSPTREE *treePtr)
{
	UINT *newIDArray;	UINT newIDCounter;
	int i,j;
	GLKArray **bspHashingArray;

	//--------------------------------------------------------------------------------------------------------
	//	Step 1: initialization
	newIDArray=(UINT*)malloc(sizeof(UINT)*(treePtr->nodeArraySize));
	memset(newIDArray,0,sizeof(UINT)*(treePtr->nodeArraySize));
	newIDArray[0]=1;	// root node
	newIDArray[1]=2;	// solid leaf-node
	newIDArray[2]=3;	// empty left-node 	
	newIDCounter=3;
	//--------------------------------------------------------------------------------------------------------
	//	Hashing table	
	int planeNum=treePtr->planeArraySize;
	bspHashingArray=(GLKArray **)new long[planeNum];
	for(i=0;i<planeNum;i++) bspHashingArray[i]=new GLKArray(10,10,GLKARRAY_VOIDPTR_TYPE);

	//--------------------------------------------------------------------------------------------------------
	//	Step 2: assign new ID for the tree node
	_bspSubTreeCollapse(treePtr,treePtr->nodeArray[0].uLeftChildID,bspHashingArray,newIDArray,newIDCounter);	
	_bspSubTreeCollapse(treePtr,treePtr->nodeArray[0].uRightChildID,bspHashingArray,newIDArray,newIDCounter);

	//--------------------------------------------------------------------------------------------------------
	//	Step 3: collapse the memory of the BSP tree (note that: in some case, the value of newIDArray[...] could be zero)
	//--------------------------------------------------------------------------------------------------------
	//int size=0;	printf("\n");
	//for(i=0;i<(int)(treePtr->nodeArraySize);i++) {
	//	if (newIDArray[i]!=0) size++; else printf("%d ",i+1);
	//}printf("\n");
	//printf("Collapse BSP-Tree from %d nodes into %d nodes!\n",(int)(treePtr->nodeArraySize),size);
	printf("Collapse BSP-Tree from %d nodes into %d nodes!\n",(int)(treePtr->nodeArraySize),newIDCounter);
	//--------------------------------------------------------------------------------------------------------
	BSPTREEArrayNode *oldNodeArray=treePtr->nodeArray;
	int oldArraySize=treePtr->nodeArraySize;
	//--------------------------------------------------------------------------------------------------------
	BSPTREEArrayNode *newNodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*newIDCounter);
	memset(newNodeArray,0,sizeof(BSPTREEArrayNode)*newIDCounter);
	newNodeArray[0].planeID=oldNodeArray[0].planeID;
	newNodeArray[0].uLeftChildID=newIDArray[oldNodeArray[0].uLeftChildID-1];
	newNodeArray[0].uRightChildID=newIDArray[oldNodeArray[0].uRightChildID-1];
	newNodeArray[1].planeID=oldNodeArray[1].planeID;	newNodeArray[2].planeID=oldNodeArray[2].planeID;
	for(i=3;i<oldArraySize;i++) {
		int id=newIDArray[i];
		if (newNodeArray[id-1].planeID!=0) continue;		// to by-pass the existing node
		if (newIDArray[oldNodeArray[i].uLeftChildID-1]		// to by-pass the node with equal left-child and right-child
			==newIDArray[oldNodeArray[i].uRightChildID-1]) continue;

		newNodeArray[id-1].planeID=oldNodeArray[i].planeID;
		newNodeArray[id-1].uLeftChildID=newIDArray[oldNodeArray[i].uLeftChildID-1];
		newNodeArray[id-1].uRightChildID=newIDArray[oldNodeArray[i].uRightChildID-1];
	}
	free(oldNodeArray);
	treePtr->nodeArray=newNodeArray;
	treePtr->nodeArraySize=newIDCounter;

	//--------------------------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int maxSize=0;
	for(i=0;i<planeNum;i++) {
		int size=bspHashingArray[i]->GetSize();
		for(j=0;j<size;j++) {
			BSPTREEHashNode *hashNode=(BSPTREEHashNode *)(bspHashingArray[i]->GetAt(j));
			free(hashNode);	
		}
		maxSize=MAX(size,maxSize);
		delete (GLKArray*)(bspHashingArray[i]);
	}
	delete []bspHashingArray;
	//--------------------------------------------------------------------------------------------------------
	free(newIDArray);
	printf("Max-Size of Hashing Table: %d\n",maxSize);

	printf("****************************************\n");
}

void BSPSolidOperation::BSPTreeConstructionFromBRep(QuadTrglMesh *mesh, BSPTREE *&treePtr, float boundingBox[],
													int maxLevelAllowed, bool bOrthogonalClipping)
{
	TRGLForBSP *trglArray;
	float p1[3],p2[3],p3[3],p4[3],lastClippingPlane[4];	
	int i,num,trglNum,planeNum;
	UINT vi1,vi2,vi3,vi4;
	BSPTREEArrayPlane	*bspPlaneArray;			UINT bspPlaneArraySize;
	BSPTREEArrayNode	*bspTreeNodeArray;		UINT bspTreeNodeArraySize;

	treePtr=(BSPTREE*)malloc(sizeof(BSPTREE));
	num=mesh->GetFaceNumber();		planeNum=num;
	bspTreeNodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*MAXNUM_OF_BSPNODE_ONTREE/2);	bspTreeNodeArraySize=0;
	if (bspTreeNodeArray==NULL) {
		printf("Err! Not enough memory!\n");exit(1);
	}
	bspPlaneArray=(BSPTREEArrayPlane*)malloc(sizeof(BSPTREEArrayPlane)*MAXNUM_OF_BSPNODE_ONTREE/2);		bspPlaneArraySize=0;
	if (bspPlaneArray==NULL) {
		printf("Err! Not enough memory!\n");exit(1);
	}

#ifdef GLOBALTRGL_COUNT_LEVEL
	globalTotalTrglNum=0;
#endif

	//------------------------------------------------------------------------------------------------
	//	Step 1: converting mesh into a triangle soup and fill the plane array of BSPTREE
	trglNum=num;
	for(i=0;i<num;i++) {if (mesh->IsQuadFace(i+1)) trglNum++;}
	//------------------------------------------------------------------------------------------------
	trglArray=(TRGLForBSP *)malloc(sizeof(TRGLForBSP)*trglNum);	trglNum=0;
	for(i=0;i<num;i++) {
		if (mesh->IsQuadFace(i+1)) {
			mesh->GetFaceNodes(i+1,vi1,vi2,vi3,vi4);
			mesh->GetNodePos(vi1,p1);	mesh->GetNodePos(vi2,p2);	mesh->GetNodePos(vi3,p3);	mesh->GetNodePos(vi4,p4);

			trglArray[trglNum].verPnts[0][0]=p1[0];	trglArray[trglNum].verPnts[0][1]=p1[1];	trglArray[trglNum].verPnts[0][2]=p1[2];
			trglArray[trglNum].verPnts[1][0]=p2[0];	trglArray[trglNum].verPnts[1][1]=p2[1];	trglArray[trglNum].verPnts[1][2]=p2[2];
			trglArray[trglNum].verPnts[2][0]=p3[0];	trglArray[trglNum].verPnts[2][1]=p3[1];	trglArray[trglNum].verPnts[2][2]=p3[2];
			_compPlaneEquation(p1,p2,p3,bspPlaneArray[bspPlaneArraySize].abcd[0],bspPlaneArray[bspPlaneArraySize].abcd[1],
				bspPlaneArray[bspPlaneArraySize].abcd[2],bspPlaneArray[bspPlaneArraySize].abcd[3]);
			bspPlaneArraySize++;	trglArray[trglNum].planeIndex=i+1;		trglNum++;

			trglArray[trglNum].verPnts[0][0]=p1[0];	trglArray[trglNum].verPnts[0][1]=p1[1];	trglArray[trglNum].verPnts[0][2]=p1[2];
			trglArray[trglNum].verPnts[1][0]=p3[0];	trglArray[trglNum].verPnts[1][1]=p3[1];	trglArray[trglNum].verPnts[1][2]=p3[2];
			trglArray[trglNum].verPnts[2][0]=p4[0];	trglArray[trglNum].verPnts[2][1]=p4[1];	trglArray[trglNum].verPnts[2][2]=p4[2];
									trglArray[trglNum].planeIndex=i+1;		trglNum++;
		}
		else {
			mesh->GetFaceNodes(i+1,vi1,vi2,vi3,vi4);

			mesh->GetNodePos(vi1,trglArray[trglNum].verPnts[0]);
			mesh->GetNodePos(vi2,trglArray[trglNum].verPnts[1]);
			mesh->GetNodePos(vi3,trglArray[trglNum].verPnts[2]);
			_compPlaneEquation(trglArray[trglNum].verPnts[0],trglArray[trglNum].verPnts[1],trglArray[trglNum].verPnts[2],
				bspPlaneArray[bspPlaneArraySize].abcd[0],bspPlaneArray[bspPlaneArraySize].abcd[1],
				bspPlaneArray[bspPlaneArraySize].abcd[2],bspPlaneArray[bspPlaneArraySize].abcd[3]);
			bspPlaneArraySize++;	trglArray[trglNum].planeIndex=i+1;		trglNum++;
		}
	}

	//------------------------------------------------------------------------------------------------
	//	Step 2: generating and slighly enlarging the bounding box
	float dx,dy,dz,dd,bndBox[6];
	//------------------------------------------------------------------------------------------------
	mesh->CompBoundingBox(bndBox);
	dx=bndBox[1]-bndBox[0];	dy=bndBox[3]-bndBox[2];	dz=bndBox[5]-bndBox[4];
	dd=dx*0.01;
	//dx += dd*2.0;	dy += dd*2.0;	dz += dd*2.0;
	dx += dd*5.0;	dy += dd*5.0;	dz += dd*5.0;

	boundingBox[0]=(float)(bndBox[0]-dd);	boundingBox[1]=(float)(bndBox[1]+dd);
	boundingBox[2]=(float)(bndBox[2]-dd);	boundingBox[3]=(float)(bndBox[3]+dd);
	boundingBox[4]=(float)(bndBox[4]-dd);	boundingBox[5]=(float)(bndBox[5]+dd);

	//------------------------------------------------------------------------------------------------
	//	Step 3: splitting based BSP-solid construction
	bspTreeNodeArraySize=3;	
	bspTreeNodeArray[1].MakeSolidNode();	bspTreeNodeArray[2].MakeEmptyNode();
	bspTreeNodeArray[0].planeID=1;
	bspTreeNodeArray[0].uLeftChildID=bspTreeNodeArray[0].uRightChildID=3;	// empty leaf-node
	lastClippingPlane[0]=lastClippingPlane[1]=lastClippingPlane[2]=lastClippingPlane[3]=0.0;	
	lastClippingPlane[0]=1.0;
	_constructBSPNode(&(bspTreeNodeArray[0]),bspTreeNodeArray,bspTreeNodeArraySize,0,lastClippingPlane,trglNum,trglArray,
		bspPlaneArray,bspPlaneArraySize,boundingBox,maxLevelAllowed,bOrthogonalClipping);

	//------------------------------------------------------------------------------------------------
	//	Step 4: fill the planeArray and the nodeArray of BSPTREE
	treePtr->planeArraySize=bspPlaneArraySize;
	if (bspPlaneArraySize>0) {
		treePtr->planeArray=(BSPTREEArrayPlane*)malloc(sizeof(BSPTREEArrayPlane)*bspPlaneArraySize);
		memcpy(treePtr->planeArray,bspPlaneArray,sizeof(BSPTREEArrayPlane)*bspPlaneArraySize);
	}
	treePtr->nodeArraySize=bspTreeNodeArraySize;
	if (bspTreeNodeArraySize>0) {
		treePtr->nodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*bspTreeNodeArraySize);
		memcpy(treePtr->nodeArray,bspTreeNodeArray,sizeof(BSPTREEArrayNode)*bspTreeNodeArraySize);
	}

	//------------------------------------------------------------------------------------------------
	//	Step 5: free the memory 
	//		Note that: the memory of "trglArray" has been released inside the function "_constructBSPNode(...)"
	free(bspPlaneArray);	free(bspTreeNodeArray);

	//------------------------------------------------------------------------------------------------
	//	Step 6: code for generating the statistics
#ifdef GLOBALTRGL_COUNT_LEVEL
	printf("--------------------------------------------------\n");
	printf("At level %d, the total triangle number is %d with %f (MB) memory consumption!\n",
		(int)GLOBALTRGL_COUNT_LEVEL,globalTotalTrglNum,
		(float)(sizeof(TRGLForBSP)*globalTotalTrglNum)/(1024.0f*1024.0f));
#endif
}

void BSPSolidOperation::BSPSolidTranslation(BSPTREE *treePtr, float boundingBox[], float tx, float ty, float tz)
{
	UINT i;

	boundingBox[0]+=tx;	boundingBox[1]+=tx;
	boundingBox[2]+=ty;	boundingBox[3]+=ty;
	boundingBox[4]+=tz;	boundingBox[5]+=tz;

	for(i=0;i<treePtr->planeArraySize;i++) {
		treePtr->planeArray[i].abcd[3]=treePtr->planeArray[i].abcd[3]
			-tx*treePtr->planeArray[i].abcd[0]
			-ty*treePtr->planeArray[i].abcd[1]
			-tz*treePtr->planeArray[i].abcd[2];
	}
}

void BSPSolidOperation::BSPSolidFlipping(short nDir, BSPTREE *treePtr, float boundingBox[])
{
	float minValue,maxValue;	UINT i;

	if (nDir<0 || nDir>2) return;

	minValue=MIN(-boundingBox[nDir*2],-boundingBox[nDir*2+1]);
	maxValue=MAX(-boundingBox[nDir*2],-boundingBox[nDir*2+1]);
	boundingBox[nDir*2]=minValue;	boundingBox[nDir*2+1]=maxValue;

	for(i=0;i<treePtr->planeArraySize;i++) {
		treePtr->planeArray[i].abcd[nDir]=-(treePtr->planeArray[i].abcd[nDir]);
	}
}

void BSPSolidOperation::BSPSolidScaling(BSPTREE *treePtr, float boundingBox[], float ratio)
{
	UINT i; 

	for(i=0;i<6;i++) boundingBox[i]=ratio*boundingBox[i];
	for(i=0;i<treePtr->planeArraySize;i++) {
		treePtr->planeArray[i].abcd[3]=ratio*treePtr->planeArray[i].abcd[3];
	}
}

void BSPSolidOperation::BSPSolidRotation(BSPTREE *treePtr, float boundingBox[], float axisVec[], float angle)
{
	UINT i;		float aa,bb,cc;

	//-------------------------------------------------------------
	// Build rotation matrix
	float rMat[3][3];
	double dd,ax,ay,az,cs,sn;
	cs=cos(DEGREE_TO_ROTATE((double)angle));	sn=sin(DEGREE_TO_ROTATE((double)angle));
	dd=sqrt(((double)(axisVec[0]))*((double)(axisVec[0]))+((double)(axisVec[1]))*((double)(axisVec[1]))+((double)(axisVec[2]))*((double)(axisVec[2])));
	ax=((double)(axisVec[0]))/dd;	ay=((double)(axisVec[1]))/dd;	az=((double)(axisVec[2]))/dd;
	rMat[0][0]=(float)(cs+(1.0-cs)*ax*ax);		rMat[0][1]=(float)((1.0-cs)*ax*ay-sn*az);	rMat[0][2]=(float)((1.0-cs)*ax*az+sn*ay);
	rMat[1][0]=(float)((1.0-cs)*ax*ay+sn*az);	rMat[1][1]=(float)(cs+(1.0-cs)*ay*ay);		rMat[1][2]=(float)((1.0-cs)*ay*az-sn*ax);
	rMat[2][0]=(float)((1.0-cs)*ax*az-sn*ay);	rMat[2][1]=(float)((1.0-cs)*ay*az+sn*ax);	rMat[2][2]=(float)(cs+(1.0-cs)*az*az);

	//-------------------------------------------------------------
	// Rotate the boundingBox
	float pnts[8][3];
	//-------------------------------------------------------------
	pnts[0][0]=boundingBox[0];	pnts[0][1]=boundingBox[2];	pnts[0][2]=boundingBox[4];
	pnts[1][0]=boundingBox[0];	pnts[1][1]=boundingBox[2];	pnts[1][2]=boundingBox[5];
	pnts[2][0]=boundingBox[0];	pnts[2][1]=boundingBox[3];	pnts[2][2]=boundingBox[4];
	pnts[3][0]=boundingBox[0];	pnts[3][1]=boundingBox[3];	pnts[3][2]=boundingBox[5];
	pnts[4][0]=boundingBox[1];	pnts[4][1]=boundingBox[2];	pnts[4][2]=boundingBox[4];
	pnts[5][0]=boundingBox[1];	pnts[5][1]=boundingBox[2];	pnts[5][2]=boundingBox[5];
	pnts[6][0]=boundingBox[1];	pnts[6][1]=boundingBox[3];	pnts[6][2]=boundingBox[4];
	pnts[7][0]=boundingBox[1];	pnts[7][1]=boundingBox[3];	pnts[7][2]=boundingBox[5];
	//-------------------------------------------------------------
	for(i=0;i<8;i++) {
		aa=pnts[i][0];		bb=pnts[i][1];		cc=pnts[i][2];
		pnts[i][0]=rMat[0][0]*aa+rMat[0][1]*bb+rMat[0][2]*cc;
		pnts[i][1]=rMat[1][0]*aa+rMat[1][1]*bb+rMat[1][2]*cc;
		pnts[i][2]=rMat[2][0]*aa+rMat[2][1]*bb+rMat[2][2]*cc;
	}
	//-------------------------------------------------------------
	boundingBox[0]=boundingBox[1]=pnts[0][0];
	boundingBox[2]=boundingBox[3]=pnts[0][1];
	boundingBox[4]=boundingBox[5]=pnts[0][2];
	for(i=1;i<8;i++) {
		boundingBox[0]=MIN(boundingBox[0],pnts[i][0]);		boundingBox[1]=MAX(boundingBox[1],pnts[i][0]);
		boundingBox[2]=MIN(boundingBox[2],pnts[i][1]);		boundingBox[3]=MAX(boundingBox[3],pnts[i][1]);
		boundingBox[4]=MIN(boundingBox[4],pnts[i][2]);		boundingBox[5]=MAX(boundingBox[5],pnts[i][2]);
	}

	//-------------------------------------------------------------
	// Rotate the planes
	for(i=0;i<treePtr->planeArraySize;i++) {
		aa=treePtr->planeArray[i].abcd[0];		bb=treePtr->planeArray[i].abcd[1];		cc=treePtr->planeArray[i].abcd[2];

		treePtr->planeArray[i].abcd[0]=rMat[0][0]*aa+rMat[0][1]*bb+rMat[0][2]*cc;
		treePtr->planeArray[i].abcd[1]=rMat[1][0]*aa+rMat[1][1]*bb+rMat[1][2]*cc;
		treePtr->planeArray[i].abcd[2]=rMat[2][0]*aa+rMat[2][1]*bb+rMat[2][2]*cc;
	}
}

void BSPSolidOperation::StatisticsOfBSPTree(BSPTREE *treePtr)
{
	int maxLevel=CountLevelNumber(treePtr);
	int i,totalNum,*nodeNumInLevels;

	int leafNodeNum=_countLeafNodeNumberInDifferentLevels(treePtr);
	int simpTreeNodeNum=treePtr->nodeArraySize+leafNodeNum-2;

	printf("\n\n");
	printf("**************************************************\n");
	printf("*           Statistics of BSP tree               *\n");
	printf("**************************************************\n");
	printf("A BSP solid with %d nodes and %d clipping-planes has been constructed\n",(int)(treePtr->nodeArraySize),(int)(treePtr->planeArraySize));
	printf("Total memory usage is %f (MB) with %f (MB) in Simple-Tree-Rep (with %d nodes)\n",
		(float)((sizeof(BSPTREEArrayPlane)*(treePtr->planeArraySize))+(sizeof(BSPTREEArrayNode)*(treePtr->nodeArraySize)))/(1024.0f*1024.0f),
		(float)((sizeof(BSPTREEArrayNode)+sizeof(float)*3)*simpTreeNodeNum)/(1024.0f*1024.0f),simpTreeNodeNum
		);
	printf("Max Level on BSP-tree is: %d\n",maxLevel);
	printf("--------------------------------------------------\n");
	nodeNumInLevels=new int[maxLevel+1];	for(i=0;i<=maxLevel;i++) nodeNumInLevels[i]=0;
	_countNodeNumberInDifferentLevels(treePtr,nodeNumInLevels,1,0);
	totalNum=0;
	for(i=0;i<=maxLevel;i++) {printf("Level %d node number: %d\n",i,nodeNumInLevels[i]); totalNum+=nodeNumInLevels[i];}
	printf("--------------------------------------------------\n");
	printf("Total node number of the BSP-tree is: %d\n",totalNum);
	printf("Total non-leaf node number of the BSP-tree is: %d\n",totalNum-leafNodeNum);

	delete []nodeNumInLevels;
}

void BSPSolidOperation::BSPFileImport(BSPTREE *&treePtr, char *filename, float boundingBox[], bool bWithPrint)
{
	FILE *fp;	UINT planeArraySize,nodeArraySize;

	fp = fopen(filename, "rb");
    if(!fp) {
		printf("===============================================\n");
	    printf("Can not open the data file - BSP File Export!\n");
		printf("===============================================\n");
	}

	fread(boundingBox,sizeof(float),6,fp);
	fread(&planeArraySize,sizeof(UINT),1,fp);
	fread(&nodeArraySize,sizeof(UINT),1,fp);

	if (planeArraySize==0 || nodeArraySize==0) {treePtr=NULL; return;}

	treePtr=(BSPTREE*)malloc(sizeof(BSPTREE));
	treePtr->planeArraySize=planeArraySize;
	treePtr->nodeArraySize=nodeArraySize;
	treePtr->planeArray=(BSPTREEArrayPlane*)malloc(sizeof(BSPTREEArrayPlane)*planeArraySize);
	treePtr->nodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*nodeArraySize);
	fread(treePtr->planeArray,sizeof(BSPTREEArrayPlane),planeArraySize,fp);
	fread(treePtr->nodeArray,sizeof(BSPTREEArrayNode),nodeArraySize,fp);

	fclose(fp);

	if (bWithPrint) {
		printf("-----------------------------------------------------\n");
		printf("The BSP file has been successfully Loaded!\n");
		printf("Plane number: %d\n",planeArraySize);
		printf("Node number: %d\n",nodeArraySize);
	}
}

void BSPSolidOperation::BSPFileExport(BSPTREE *treePtr, char *filename, float boundingBox[])
{
	FILE *fp;

	fp = fopen(filename, "wb");
    if(!fp) {
		printf("===============================================\n");
	    printf("Can not open the data file - BSP File Export!\n");
		printf("===============================================\n");
	    return;
	}

	fwrite(boundingBox,sizeof(float),6,fp);

	fwrite(&(treePtr->planeArraySize),sizeof(UINT),1,fp);
	fwrite(&(treePtr->nodeArraySize),sizeof(UINT),1,fp);

	fwrite(treePtr->planeArray,sizeof(BSPTREEArrayPlane),treePtr->planeArraySize,fp);
	fwrite(treePtr->nodeArray,sizeof(BSPTREEArrayNode),treePtr->nodeArraySize,fp);

	fclose(fp);

	printf("-----------------------------------------------------\n");
	printf("The BSP file has been successfully saved!\n");
}

void BSPSolidOperation::BPTFileImport(BSPTREE *&treePtr, char *filename, float boundingBox[])
{
	FILE *fp;

	treePtr = NULL;
	fp = fopen(filename, "r");
    if(!fp) {
	    printf("===============================================");
	    printf("Can not open the data file - BSP File Import!");
	    printf("===============================================");
	    return;
	}

	int nFlag;
	fscanf(fp,"%d ",&nFlag);
	if (nFlag==0) {
		GLKArray *bspPlaneArray,*bspTreeNodeArray;
		treePtr=(BSPTREE*)malloc(sizeof(BSPTREE));
		bspPlaneArray=new GLKArray(200000,200000,GLKARRAY_FLOAT_TYPE);
		bspTreeNodeArray=new GLKArray(200000,200000,GLKARRAY_VOIDPTR_TYPE);

		BSPTREEArrayNode *bspNode=(BSPTREEArrayNode *)malloc(sizeof(BSPTREEArrayNode));	
		bspNode->uLeftChildID=bspNode->uRightChildID=0;
		bspTreeNodeArray->Add(bspNode);
		//------------------------------------------------------------------------------------------------
		BSPTREEArrayNode *solidLeaf,*emptyLeaf;
		solidLeaf=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode));
		solidLeaf->uLeftChildID=solidLeaf->uRightChildID=0;				solidLeaf->planeID=1;
		bspTreeNodeArray->Add(solidLeaf);
		//------------------------------------------------------------------------------------------------
		emptyLeaf=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode));		
		emptyLeaf->uLeftChildID=emptyLeaf->uRightChildID=0;				emptyLeaf->planeID=0;
		bspTreeNodeArray->Add(emptyLeaf);
		//------------------------------------------------------------------------------------------------
		_bptNodeImport(bspNode, bspPlaneArray, bspTreeNodeArray, fp);

		//------------------------------------------------------------------------------------------------
		//	fill the planeArray and the nodeArray of BSPTREE
		int i,size;
		size=bspPlaneArray->GetSize()/4;
		treePtr->planeArraySize=(UINT)size;
		if (size>0) {
			treePtr->planeArray=(BSPTREEArrayPlane*)malloc(sizeof(BSPTREEArrayPlane)*size);
			for(i=0;i<size;i++) {
				treePtr->planeArray[i].abcd[0]=bspPlaneArray->GetFloatAt(i*4);
				treePtr->planeArray[i].abcd[1]=bspPlaneArray->GetFloatAt(i*4+1);
				treePtr->planeArray[i].abcd[2]=bspPlaneArray->GetFloatAt(i*4+2);
				treePtr->planeArray[i].abcd[3]=bspPlaneArray->GetFloatAt(i*4+3);
			}
		}
		//------------------------------------------------------------------------------------------------
		size=bspTreeNodeArray->GetSize();
		treePtr->nodeArraySize=size;
		if (size>0) {
			treePtr->nodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*size);
			for(i=0;i<size;i++) {
				BSPTREEArrayNode* bspNode=(BSPTREEArrayNode*)(bspTreeNodeArray->GetAt(i));
				treePtr->nodeArray[i].planeID=bspNode->planeID;
				treePtr->nodeArray[i].uLeftChildID=bspNode->uLeftChildID;
				treePtr->nodeArray[i].uRightChildID=bspNode->uRightChildID;
				free(bspNode);
			}
		}
		
		delete bspPlaneArray;	delete bspTreeNodeArray;
	}

	fscanf(fp, "%f %f %f %f %f %f ",&(boundingBox[1]),&(boundingBox[0]),
			&(boundingBox[3]),&(boundingBox[2]),&(boundingBox[5]),&(boundingBox[4]));

	fclose(fp);

	printf("-----------------------------------------------------\n");
	printf("The BSP file has been successfully Loaded!\n");
	printf("Plane number: %d\n",treePtr->planeArraySize);
	printf("Node number: %d\n",treePtr->nodeArraySize);
}

void BSPSolidOperation::BPTFileExport(BSPTREE *treePtr, char *filename, float boundingBox[])
{
	FILE *fp;

	fp = fopen(filename, "w");
    if(!fp) {
	    printf("===============================================");
	    printf(" Can not open the data file - BSP File Export! ");
	    printf("===============================================");
	    return;
	}

	if (treePtr!=NULL && treePtr->nodeArraySize>0) {
		fprintf(fp,"0 ");
		_bptNodeExport(treePtr, 1, fp);
	}

	fprintf(fp, "%f %f %f %f %f %f ",(boundingBox[1]),(boundingBox[0]),
			(boundingBox[3]),(boundingBox[2]),(boundingBox[5]),(boundingBox[4]));

	fclose(fp);

	printf("-----------------------------------------------------\n");
	printf("The BSP file has been successfully saved!\n");
}

int BSPSolidOperation::CountLevelNumber(BSPTREE *treePtr, int currentNodeID, int nodeLevel)
{
	BSPTREEArrayNode *currentNode;
	int leftNum,rightNum;

	currentNode=&(treePtr->nodeArray[currentNodeID-1]);
	leftNum=rightNum=nodeLevel;
	if (currentNode->uLeftChildID>0) leftNum=CountLevelNumber(treePtr,currentNode->uLeftChildID,nodeLevel+1);
	if (currentNode->uRightChildID>0) rightNum=CountLevelNumber(treePtr,currentNode->uRightChildID,nodeLevel+1);

	return MAX(leftNum,rightNum);
}

bool BSPSolidOperation::IsPntInsideBSPSolid(BSPTREE *treePtr, float pnt[])
{
	UINT currentNodeID=1;
	int id;		float dd;

	do{
		BSPTREEArrayNode *currentNode=&(treePtr->nodeArray[currentNodeID-1]);

		if (currentNode->IsLeafNode()) {return (currentNode->IsSolidNode());}

		id=currentNode->planeID-1;
		dd=treePtr->planeArray[id].abcd[0]*pnt[0] 
			+treePtr->planeArray[id].abcd[1]*pnt[1]
			+treePtr->planeArray[id].abcd[2]*pnt[2]
			+treePtr->planeArray[id].abcd[3];
		if (dd>=0.0f) {	// keep visiting the left sub-tree
			currentNodeID=currentNode->uLeftChildID;
		}
		else {	// keep visiting the right sub-tree
			currentNodeID=currentNode->uRightChildID;
		}
	}while(currentNodeID!=0);

	return false;
}


int BSPSolidOperation::_countLeafNodeNumberInDifferentLevels(BSPTREE *treePtr, int currentNodeID)
{
	BSPTREEArrayNode *currentNode;
	int leftNum,rightNum;

	currentNode=&(treePtr->nodeArray[currentNodeID-1]);
	if (currentNode->IsLeafNode()) return 1;
	leftNum=_countLeafNodeNumberInDifferentLevels(treePtr,currentNode->uLeftChildID);
	rightNum=_countLeafNodeNumberInDifferentLevels(treePtr,currentNode->uRightChildID);
	
	return (leftNum+rightNum);
}

void BSPSolidOperation::_countNodeNumberInDifferentLevels(BSPTREE *treePtr, int *nodeNumInLevels, 
														  int currentNodeID, int currentLevel)
{
	BSPTREEArrayNode *currentNode=&(treePtr->nodeArray[currentNodeID-1]);
//	if (currentNode->IsLeaveNode()) return;

	(nodeNumInLevels[currentLevel])++;

	if (currentNode->uLeftChildID>0) 
		_countNodeNumberInDifferentLevels(treePtr,nodeNumInLevels,currentNode->uLeftChildID,currentLevel+1);
	if (currentNode->uRightChildID>0) 
		_countNodeNumberInDifferentLevels(treePtr,nodeNumInLevels,currentNode->uRightChildID,currentLevel+1);
}

void BSPSolidOperation::_constructBSPNode(BSPTREEArrayNode *treeNodePtr, BSPTREEArrayNode *bspTreeNodeArray, UINT &bspTreeNodeArraySize, 
										  int level, float lastClippingPlane[], 
										  int trglNum, TRGLForBSP *trglArray, BSPTREEArrayPlane *bspPlaneArray, UINT &bspPlaneArraySize,
										  float boundingBox[], int maxLevelAllowed, bool bOrthogonalClipping)
{
	TrglForBSP *trgl1,*trgl2,*trgl3;	bool trgl1Above,trgl2Above,trgl3Above;
	GLKArray *lowerFacePtr,*upperFacePtr;
	bool *trglArrayRemainedFlag,*trglArrayAboveFlag;
	float aa,bb,cc,dd,dist,cp[3],np[3];
	float posArea,negArea;
	int i,st;	short nState;
	float eps=1.0e-5;
	float lowerBndBox[6],upperBndBox[6];
	bool orthogonalCutting=false;
	bool bClippingByTrglFace=false;
	const int levelThreshold=15;
	const int trglNumThreshold=30;

#ifdef GLOBALTRGL_COUNT_LEVEL
	if (level==GLOBALTRGL_COUNT_LEVEL) {globalTotalTrglNum+=trglNum;}
#endif

	//------------------------------------------------------------------------------------------------
	//	Step 1: determine the cutting plane
	aa=bb=cc=dd=0.0;	aa=1.0;
	if (bOrthogonalClipping && (level<levelThreshold) && (trglNum>trglNumThreshold)) {
#ifdef FAST_BSPTREE_CONSTRUCTION
		cp[0]=(boundingBox[0]+boundingBox[1])*0.5; 
		cp[1]=(boundingBox[2]+boundingBox[3])*0.5; 
		cp[2]=(boundingBox[4]+boundingBox[5])*0.5;
		np[0]=np[1]=np[2]=0.0; np[level%3]=1.0;
		aa=np[0]; bb=np[1]; cc=np[2]; dd=-(np[0]*cp[0]+np[1]*cp[1]+np[2]*cp[2]);
		bspPlaneArray->Add(aa);		bspPlaneArray->Add(bb);		bspPlaneArray->Add(cc);		bspPlaneArray->Add(dd);
		treeNode->planeID=(UINT)(bspPlaneArray->GetSize()/4);
		orthogonalCutting=true;
		if ((fabs(boundingBox[1]-boundingBox[0])<1.0e-4) 
			|| (fabs(boundingBox[3]-boundingBox[2])<1.0e-4) 
			|| (fabs(boundingBox[5]-boundingBox[4])<1.0e-4)) orthogonalCutting=false;
		else {
			for(i=0;i<6;i++) {lowerBndBox[i]=upperBndBox[i]=boundingBox[i];}
			lowerBndBox[(level%3)*2+1]=cp[level%3];	
			upperBndBox[(level%3)*2+0]=cp[level%3];
		}
#else
		float bndBox[6];
		_compBoundingBox(trglNum, trglArray, bndBox);
		cp[0]=(bndBox[0]+bndBox[1])*0.5; 
		cp[1]=(bndBox[2]+bndBox[3])*0.5; 
		cp[2]=(bndBox[4]+bndBox[5])*0.5;
		np[0]=np[1]=np[2]=0.0; np[level%3]=1.0;
		aa=np[0]; bb=np[1]; cc=np[2]; dd=-(np[0]*cp[0]+np[1]*cp[1]+np[2]*cp[2]);
		bspPlaneArray[bspPlaneArraySize].abcd[0]=aa;	bspPlaneArray[bspPlaneArraySize].abcd[1]=bb;	
		bspPlaneArray[bspPlaneArraySize].abcd[2]=cc;	bspPlaneArray[bspPlaneArraySize].abcd[3]=dd;	
		bspPlaneArraySize++;
		treeNodePtr->planeID=bspPlaneArraySize;
		orthogonalCutting=true;
		if ((fabs(bndBox[1]-bndBox[0])<1.0e-4) 
			|| (fabs(bndBox[3]-bndBox[2])<1.0e-4) 
			|| (fabs(bndBox[5]-bndBox[4])<1.0e-4)) orthogonalCutting=false;
		else {
			for(i=0;i<6;i++) {lowerBndBox[i]=upperBndBox[i]=boundingBox[i];}
			lowerBndBox[(level%3)*2+1]=cp[level%3];	
			upperBndBox[(level%3)*2+0]=cp[level%3];
		}
#endif
	}
	st=-1;
	if (!orthogonalCutting)	{
		aa=bb=cc=dd=0.0;	aa=1.0;		
		//--------------------------------------------------------------------------------------------
		//	Search the clipping plane which is most perpendicular to last clipping plane
		float dot,minDot=1.0e+10f;
		for(i=0;i<trglNum;i++) {
			int id=trglArray[i].planeIndex-1;
			aa=bspPlaneArray[id].abcd[0];
			bb=bspPlaneArray[id].abcd[1];
			cc=bspPlaneArray[id].abcd[2];
			dot=fabs(lastClippingPlane[0]*aa+lastClippingPlane[1]*bb+lastClippingPlane[2]*cc);
			if (dot<minDot) {minDot=dot; st=i;}
		}
		if (st>=0) {
			treeNodePtr->planeID=trglArray[st].planeIndex;
			int id=trglArray[st].planeIndex-1;
			aa=bspPlaneArray[id].abcd[0];	bb=bspPlaneArray[id].abcd[1];
			cc=bspPlaneArray[id].abcd[2];	dd=bspPlaneArray[id].abcd[3];
			bClippingByTrglFace=true;
		}
		else {
			bspPlaneArray[bspPlaneArraySize].abcd[0]=aa;	bspPlaneArray[bspPlaneArraySize].abcd[1]=bb;	
			bspPlaneArray[bspPlaneArraySize].abcd[2]=cc;	bspPlaneArray[bspPlaneArraySize].abcd[3]=dd;	
			bspPlaneArraySize++;
			treeNodePtr->planeID=bspPlaneArraySize;
		}
	}

	//------------------------------------------------------------------------------------------------
	//	Step 2: splitting the triangles into two groups
	posArea=negArea=0.0;
	trglArrayRemainedFlag=(bool*)malloc(sizeof(bool)*trglNum);
	trglArrayAboveFlag=(bool*)malloc(sizeof(bool)*trglNum);
	lowerFacePtr=new GLKArray(2000,2000,GLKARRAY_VOIDPTR_TYPE);		upperFacePtr=new GLKArray(2000,2000,GLKARRAY_VOIDPTR_TYPE);
	for(i=0;i<trglNum;i++) {
		trglArrayRemainedFlag[i]=false;

		//--------------------------------------------------------------------------------------------
		//	filtering unnecessary triangles
		if (i==st) {
			posArea+=_compArea(trglArray[i].verPnts[0],trglArray[i].verPnts[1],trglArray[i].verPnts[2]);
			continue;
		}
		dist=fabs(trglArray[i].verPnts[0][0]-trglArray[i].verPnts[1][0])
			+fabs(trglArray[i].verPnts[0][1]-trglArray[i].verPnts[1][1]) 
			+fabs(trglArray[i].verPnts[0][2]-trglArray[i].verPnts[1][2]);
		if (dist<eps) continue;
		dist=fabs(trglArray[i].verPnts[2][0]-trglArray[i].verPnts[1][0])
			+fabs(trglArray[i].verPnts[2][1]-trglArray[i].verPnts[1][1]) 
			+fabs(trglArray[i].verPnts[2][2]-trglArray[i].verPnts[1][2]);
		if (dist<eps) continue;
		dist=fabs(trglArray[i].verPnts[0][0]-trglArray[i].verPnts[2][0])
			+fabs(trglArray[i].verPnts[0][1]-trglArray[i].verPnts[2][1])
			+fabs(trglArray[i].verPnts[0][2]-trglArray[i].verPnts[2][2]);
		if (dist<eps) continue;

		if (_splittingTrgl(&(trglArray[i]),bspPlaneArray,aa,bb,cc,dd,nState,trgl1,trgl1Above,trgl2,trgl2Above,trgl3,trgl3Above)) {
			if (trgl1!=NULL) {if (trgl1Above) upperFacePtr->Add(trgl1); else lowerFacePtr->Add(trgl1);}
			if (trgl2!=NULL) {if (trgl2Above) upperFacePtr->Add(trgl2); else lowerFacePtr->Add(trgl2);}
			if (trgl3!=NULL) {if (trgl3Above) upperFacePtr->Add(trgl3); else lowerFacePtr->Add(trgl3);}
		}
		else { 
			trglArrayRemainedFlag[i]=true;
			if (nState==0) {
				posArea+=_compArea(trglArray[i].verPnts[0],trglArray[i].verPnts[1],trglArray[i].verPnts[2]);
				trglArrayRemainedFlag[i]=false;
			} // on-the-plane (in the same orientation)
			else if (nState==1) {trglArrayRemainedFlag[i]=true; trglArrayAboveFlag[i]=false;}	 // below
			else if (nState==2) {trglArrayRemainedFlag[i]=true; trglArrayAboveFlag[i]=true;} 	 // above
			else {	//nState==3									 // on-the-plane (in the opporsite orientation)
				negArea+=_compArea(trglArray[i].verPnts[0],trglArray[i].verPnts[1],trglArray[i].verPnts[2]);
				trglArrayRemainedFlag[i]=false;
			}
		}
	}

	//------------------------------------------------------------------------------------------------
	//	Step 3: create new array and free the memory
	int lowerTrglNum,upperTrglNum;	TrglForBSP *lowerTrglArray=NULL,*upperTrglArray=NULL;
	lowerTrglNum=upperTrglNum=0;
	for(i=0;i<trglNum;i++) {
		if (trglArrayRemainedFlag[i]) {
			if (trglArrayAboveFlag[i]) upperTrglNum++; else lowerTrglNum++;
		}
	}
	upperTrglNum+=upperFacePtr->GetSize();	lowerTrglNum+=lowerFacePtr->GetSize();
	if ((!bOrthogonalClipping) && ((negArea-posArea)>0.5*posArea)) {
		int temp=upperTrglNum;	upperTrglNum=lowerTrglNum;	lowerTrglNum=temp;
		if (lowerTrglNum>0) {
			lowerTrglArray=(TrglForBSP*)malloc(sizeof(TrglForBSP)*lowerTrglNum);
			int index=0;
			for(i=0;i<trglNum;i++) {
				if (trglArrayRemainedFlag[i]) {
					if (trglArrayAboveFlag[i]) {lowerTrglArray[index]=trglArray[i];	index++;}
				}
			}
			for(i=0;i<upperFacePtr->GetSize();i++) {TrglForBSP* trgl=(TrglForBSP*)(upperFacePtr->GetAt(i)); lowerTrglArray[index]= (*trgl);	index++; delete trgl;}
		}
		if (upperTrglNum>0) {
			upperTrglArray=(TrglForBSP*)malloc(sizeof(TrglForBSP)*upperTrglNum);
			int index=0;
			for(i=0;i<trglNum;i++) {
				if (trglArrayRemainedFlag[i]) {
					if (!(trglArrayAboveFlag[i])) {upperTrglArray[index]=trglArray[i];	index++;}
				}
			}
			for(i=0;i<lowerFacePtr->GetSize();i++) {TrglForBSP* trgl=(TrglForBSP*)(lowerFacePtr->GetAt(i)); upperTrglArray[index]= (*trgl);	index++; delete trgl;}
		}
		//----------------------------------------------------------------------------------------------
		//	inverse the clipping plane and add a new element in bspPlaneArray
		aa=-aa;	bb=-bb;	cc=-cc;	dd=-dd;
		bspPlaneArray[bspPlaneArraySize].abcd[0]=aa;	bspPlaneArray[bspPlaneArraySize].abcd[1]=bb;	
		bspPlaneArray[bspPlaneArraySize].abcd[2]=cc;	bspPlaneArray[bspPlaneArraySize].abcd[3]=dd;	
		bspPlaneArraySize++;
		treeNodePtr->planeID=bspPlaneArraySize;
	}
	else 
	{
		if (lowerTrglNum>0) {
			lowerTrglArray=(TrglForBSP*)malloc(sizeof(TrglForBSP)*lowerTrglNum);
			int index=0;
			for(i=0;i<trglNum;i++) {
				if (trglArrayRemainedFlag[i]) {
					if (!(trglArrayAboveFlag[i])) {lowerTrglArray[index]=trglArray[i];	index++;}
				}
			}
			for(i=0;i<lowerFacePtr->GetSize();i++) {TrglForBSP* trgl=(TrglForBSP*)(lowerFacePtr->GetAt(i)); lowerTrglArray[index]= (*trgl);	index++; delete trgl;}
		}
		if (upperTrglNum>0) {
			upperTrglArray=(TrglForBSP*)malloc(sizeof(TrglForBSP)*upperTrglNum);
			int index=0;
			for(i=0;i<trglNum;i++) {
				if (trglArrayRemainedFlag[i]) {
					if (trglArrayAboveFlag[i]) {upperTrglArray[index]=trglArray[i];	index++;}
				}
			}
			for(i=0;i<upperFacePtr->GetSize();i++) {TrglForBSP* trgl=(TrglForBSP*)(upperFacePtr->GetAt(i)); upperTrglArray[index]=(*trgl);	index++; delete trgl;}
		}
	}
	//------------------------------------------------------------------------------------------------
	free(trglArray);		
	free(trglArrayRemainedFlag);		
	free(trglArrayAboveFlag);
	delete lowerFacePtr;	
	delete upperFacePtr;

	//------------------------------------------------------------------------------------------------
	//	Special Processing for truncating a solid (i.e., obtain a BSP solid with less details)
	//------------------------------------------------------------------------------------------------
	//	This following implementation is based on a heuristic, which may not correctly capture the interial volume of solids
	if (maxLevelAllowed>0 && level>=maxLevelAllowed) {
		printf("Limited Max Level (%d) is reached!\n",maxLevelAllowed);
		treeNodePtr->uLeftChildID=3;
		treeNodePtr->uRightChildID=2;
		return;
	}

	//------------------------------------------------------------------------------------------------
	//	Step 4: construct the children BSP-tree node
	BSPTREEArrayNode *leftChild,*rightChild;
	float abcd[4];	abcd[0]=aa;	abcd[1]=bb;	abcd[2]=cc;	abcd[3]=dd;
	//------------------------------------------------------------------------------------------------
	if (upperTrglNum>0) {
		leftChild=&(bspTreeNodeArray[bspTreeNodeArraySize]);
		leftChild->uLeftChildID=leftChild->uRightChildID=0;
		bspTreeNodeArraySize++;		treeNodePtr->uLeftChildID=bspTreeNodeArraySize;
		_constructBSPNode(leftChild,bspTreeNodeArray,bspTreeNodeArraySize,level+1,abcd,
			upperTrglNum,upperTrglArray,bspPlaneArray,bspPlaneArraySize,upperBndBox,maxLevelAllowed,bOrthogonalClipping);
	}
	else {
		//--------------------------------------------------------------------------------------------
		//	This following implementation is based on a heuristic, which may not correctly capture the interial volume of solids
		treeNodePtr->uLeftChildID=3;	// empty leaf-node
	}
	//------------------------------------------------------------------------------------------------
	if (lowerTrglNum>0) {
		rightChild=&(bspTreeNodeArray[bspTreeNodeArraySize]);	
		rightChild->uLeftChildID=rightChild->uRightChildID=0;
		bspTreeNodeArraySize++;		treeNodePtr->uRightChildID=bspTreeNodeArraySize;
		_constructBSPNode(rightChild,bspTreeNodeArray,bspTreeNodeArraySize,level+1,abcd,
			lowerTrglNum,lowerTrglArray,bspPlaneArray,bspPlaneArraySize,lowerBndBox,maxLevelAllowed,bOrthogonalClipping);
	}
	else {
		//--------------------------------------------------------------------------------------------
		//	This following implementation is based on a heuristic, which may not correctly capture the interial volume of solids
		if (bClippingByTrglFace)	
			treeNodePtr->uRightChildID=2;	// solid leaf-node 
		else 
			treeNodePtr->uRightChildID=3;	// empty leaf-node
	}
}

bool BSPSolidOperation::_splittingTrgl(TrglForBSP *trgl, BSPTREEArrayPlane *bspPlaneArray, 
									   float aa, float bb, float cc, float dd, short &nStatus,
									   TrglForBSP *&trgl1, bool &trgl1Above, 					//	nStatus -	0 (on the plane) 
									   TrglForBSP *&trgl2, bool &trgl2Above, 					//				1 (below the plane)	
									   TrglForBSP *&trgl3, bool &trgl3Above)					//				2 (above the plane)
{
	const float eps=1.0e-5f;
	bool bAboveFound,bBelowFound;	int i;	float dist;	short nVerStatus[3];

	bAboveFound=bBelowFound=false;	
	trgl1=trgl2=trgl3=NULL;		trgl1Above=trgl2Above=trgl3Above=false;

	//--------------------------------------------------------------------------------------------------------
	//	Detect the position of vertices relative to the plane
	for(i=0;i<3;i++) {
		dist=trgl->verPnts[i][0]*aa+trgl->verPnts[i][1]*bb+trgl->verPnts[i][2]*cc+dd;
		if (dist>eps) {bAboveFound=true; nVerStatus[i]=2;}
		else if (dist<-eps) {bBelowFound=true; nVerStatus[i]=1;}
		else {nVerStatus[i]=0;}
	}
	//--------------------------------------------------------------------------------------------------------
	//	If the triangle has no intersection with the plane, detect its relative position to the plane
	if (!(bAboveFound && bBelowFound)) {
		if ((!bAboveFound) && (!bBelowFound)) nStatus=0;
		else if (bBelowFound) nStatus=1;
		else nStatus=2;
		if (nStatus==0) {
			int id=trgl->planeIndex-1;
			float sign=bspPlaneArray[id].abcd[0]*aa
				+bspPlaneArray[id].abcd[1]*bb
				+bspPlaneArray[id].abcd[2]*cc;
			if (sign<0.0f) nStatus=3;
		}
		return false;
	}

	//--------------------------------------------------------------------------------------------------------
	//	Create new triangles
	float d1,d2,alpha,interPnts[2][3];
	int interEdgeIndex[2],interPntNum=0;
	for(i=0;i<3;i++) {
		if (nVerStatus[i]==0 || nVerStatus[(i+1)%3]==0) continue;
		if (nVerStatus[i]==nVerStatus[(i+1)%3]) continue;
		interEdgeIndex[interPntNum]=i;
		d1=trgl->verPnts[i][0]*aa+trgl->verPnts[i][1]*bb+trgl->verPnts[i][2]*cc+dd;
		d2=trgl->verPnts[(i+1)%3][0]*aa+trgl->verPnts[(i+1)%3][1]*bb+trgl->verPnts[(i+1)%3][2]*cc+dd;
		alpha=fabs(d1)+fabs(d2);	if (alpha<eps) alpha=eps;
		alpha=fabs(d1)/alpha;
		interPnts[interPntNum][0]=(1.0-alpha)*trgl->verPnts[i][0]+alpha*trgl->verPnts[(i+1)%3][0];
		interPnts[interPntNum][1]=(1.0-alpha)*trgl->verPnts[i][1]+alpha*trgl->verPnts[(i+1)%3][1];
		interPnts[interPntNum][2]=(1.0-alpha)*trgl->verPnts[i][2]+alpha*trgl->verPnts[(i+1)%3][2];
		interPntNum++;		if (interPntNum==2) break;
	}
	if (interPntNum==1) {	// Two triangles are created
		i=interEdgeIndex[0];
		trgl1=new TrglForBSP;
		trgl1->verPnts[0][0]=interPnts[0][0];			trgl1->verPnts[0][1]=interPnts[0][1];			trgl1->verPnts[0][2]=interPnts[0][2];
		trgl1->verPnts[1][0]=trgl->verPnts[(i+1)%3][0];	trgl1->verPnts[1][1]=trgl->verPnts[(i+1)%3][1];	trgl1->verPnts[1][2]=trgl->verPnts[(i+1)%3][2];
		trgl1->verPnts[2][0]=trgl->verPnts[(i+2)%3][0];	trgl1->verPnts[2][1]=trgl->verPnts[(i+2)%3][1];	trgl1->verPnts[2][2]=trgl->verPnts[(i+2)%3][2];
		trgl1->planeIndex=trgl->planeIndex;
		if (nVerStatus[(i+1)%3]==2) trgl1Above=true; else trgl1Above=false;

		trgl2=new TrglForBSP;
		trgl2->verPnts[0][0]=interPnts[0][0];			trgl2->verPnts[0][1]=interPnts[0][1];			trgl2->verPnts[0][2]=interPnts[0][2];
		trgl2->verPnts[1][0]=trgl->verPnts[(i+2)%3][0];	trgl2->verPnts[1][1]=trgl->verPnts[(i+2)%3][1];	trgl2->verPnts[1][2]=trgl->verPnts[(i+2)%3][2];
		trgl2->verPnts[2][0]=trgl->verPnts[i][0];		trgl2->verPnts[2][1]=trgl->verPnts[i][1];		trgl2->verPnts[2][2]=trgl->verPnts[i][2];	
		trgl2->planeIndex=trgl->planeIndex;
		if (nVerStatus[i]==2) trgl2Above=true; else trgl2Above=false;
	}
	else if (interPntNum==2) {	// Three triangles are created
		int j;
		if ((interEdgeIndex[0]+1)%3==interEdgeIndex[1]) 
			{i=interEdgeIndex[0];	j=0;}
		else
			{i=interEdgeIndex[1];	j=1;}

		trgl1=new TrglForBSP;
		trgl1->verPnts[0][0]=interPnts[j][0];			trgl1->verPnts[0][1]=interPnts[j][1];			trgl1->verPnts[0][2]=interPnts[j][2];
		trgl1->verPnts[1][0]=trgl->verPnts[(i+1)%3][0];	trgl1->verPnts[1][1]=trgl->verPnts[(i+1)%3][1];	trgl1->verPnts[1][2]=trgl->verPnts[(i+1)%3][2];
		trgl1->verPnts[2][0]=interPnts[(j+1)%2][0];		trgl1->verPnts[2][1]=interPnts[(j+1)%2][1];		trgl1->verPnts[2][2]=interPnts[(j+1)%2][2];
		trgl1->planeIndex=trgl->planeIndex;
		if (nVerStatus[(i+1)%3]==2) trgl1Above=true; else trgl1Above=false;

		d1=(interPnts[j][0]-trgl->verPnts[(i+2)%3][0])*(interPnts[j][0]-trgl->verPnts[(i+2)%3][0])
			+(interPnts[j][1]-trgl->verPnts[(i+2)%3][1])*(interPnts[j][1]-trgl->verPnts[(i+2)%3][1])
			+(interPnts[j][2]-trgl->verPnts[(i+2)%3][2])*(interPnts[j][2]-trgl->verPnts[(i+2)%3][2]);
		d2=(interPnts[(j+1)%2][0]-trgl->verPnts[i][0])*(interPnts[(j+1)%2][0]-trgl->verPnts[i][0])
			+(interPnts[(j+1)%2][1]-trgl->verPnts[i][1])*(interPnts[(j+1)%2][1]-trgl->verPnts[i][1])
			+(interPnts[(j+1)%2][2]-trgl->verPnts[i][2])*(interPnts[(j+1)%2][2]-trgl->verPnts[i][2]);
		if (d1<d2) {
			trgl2=new TrglForBSP;
			trgl2->verPnts[0][0]=interPnts[j][0];			trgl2->verPnts[0][1]=interPnts[j][1];			trgl2->verPnts[0][2]=interPnts[j][2];
			trgl2->verPnts[1][0]=interPnts[(j+1)%2][0];		trgl2->verPnts[1][1]=interPnts[(j+1)%2][1];		trgl2->verPnts[1][2]=interPnts[(j+1)%2][2];
			trgl2->verPnts[2][0]=trgl->verPnts[(i+2)%3][0];	trgl2->verPnts[2][1]=trgl->verPnts[(i+2)%3][1];	trgl2->verPnts[2][2]=trgl->verPnts[(i+2)%3][2];
			trgl2->planeIndex=trgl->planeIndex;
			if (nVerStatus[(i+2)%3]==2) trgl2Above=true; else trgl2Above=false;

			trgl3=new TrglForBSP;
			trgl3->verPnts[0][0]=interPnts[j][0];			trgl3->verPnts[0][1]=interPnts[j][1];			trgl3->verPnts[0][2]=interPnts[j][2];
			trgl3->verPnts[1][0]=trgl->verPnts[(i+2)%3][0];	trgl3->verPnts[1][1]=trgl->verPnts[(i+2)%3][1];	trgl3->verPnts[1][2]=trgl->verPnts[(i+2)%3][2];
			trgl3->verPnts[2][0]=trgl->verPnts[i][0];		trgl3->verPnts[2][1]=trgl->verPnts[i][1];		trgl3->verPnts[2][2]=trgl->verPnts[i][2];
			trgl3->planeIndex=trgl->planeIndex;
			if (nVerStatus[(i+2)%3]==2) trgl3Above=true; else trgl3Above=false;
		}
		else {
			trgl2=new TrglForBSP;
			trgl2->verPnts[0][0]=interPnts[j][0];		trgl2->verPnts[0][1]=interPnts[j][1];		trgl2->verPnts[0][2]=interPnts[j][2];
			trgl2->verPnts[1][0]=interPnts[(j+1)%2][0];	trgl2->verPnts[1][1]=interPnts[(j+1)%2][1];	trgl2->verPnts[1][2]=interPnts[(j+1)%2][2];
			trgl2->verPnts[2][0]=trgl->verPnts[i][0];	trgl2->verPnts[2][1]=trgl->verPnts[i][1];	trgl2->verPnts[2][2]=trgl->verPnts[i][2];
			trgl2->planeIndex=trgl->planeIndex;
			if (nVerStatus[i]==2) trgl2Above=true; else trgl2Above=false;

			trgl3=new TrglForBSP;
			trgl3->verPnts[0][0]=trgl->verPnts[i][0];		trgl3->verPnts[0][1]=trgl->verPnts[i][1];		trgl3->verPnts[0][2]=trgl->verPnts[i][2];
			trgl3->verPnts[1][0]=interPnts[(j+1)%2][0];		trgl3->verPnts[1][1]=interPnts[(j+1)%2][1];		trgl3->verPnts[1][2]=interPnts[(j+1)%2][2];
			trgl3->verPnts[2][0]=trgl->verPnts[(i+2)%3][0];	trgl3->verPnts[2][1]=trgl->verPnts[(i+2)%3][1];	trgl3->verPnts[2][2]=trgl->verPnts[(i+2)%3][2];
			trgl3->planeIndex=trgl->planeIndex;
			if (nVerStatus[i]==2) trgl3Above=true; else trgl3Above=false;
		}
	}
	else {
		printf("Warning: no intersection point is found (trgl Splitting)!\n");
	}

	return true;
}

UINT BSPSolidOperation::_bspSubTreeCollapse(BSPTREE *treePtr, UINT currentNodeID, 
											GLKArray **bspHashingArray, UINT *newIDArray, UINT &newIDCounter)
{
	if (currentNodeID==0) return 0;
	BSPTREEArrayNode *currentNode=&(treePtr->nodeArray[currentNodeID-1]);

	//----------------------------------------------------------------------------------------------------------------
	//	For the cases that the newID of the current node has been assigned
	if (newIDArray[currentNodeID-1]!=0) return newIDArray[currentNodeID-1];		

	UINT leftChildID=_bspSubTreeCollapse(treePtr,currentNode->uLeftChildID,bspHashingArray,newIDArray,newIDCounter);
	UINT rightChildID=_bspSubTreeCollapse(treePtr,currentNode->uRightChildID,bspHashingArray,newIDArray,newIDCounter);
	if (leftChildID==rightChildID) {
		newIDArray[currentNodeID-1]=leftChildID;
//		if (newIDArray[currentNode->uLeftChildID-1]==newIDArray[currentNode->uRightChildID-1]) 
//			printf("ERROR! %d %d (%d-%d) -- the value: %d\n",currentNode->uLeftChildID,currentNode->uRightChildID,leftChildID,rightChildID,newIDArray[currentNode->uLeftChildID-1]);
		return leftChildID;
	}

	int indexInHashingTable;
	if (_isSameSubTreeExist(bspHashingArray,currentNode->planeID,leftChildID,rightChildID,indexInHashingTable)) {
		BSPTREEHashNode *hashNode=(BSPTREEHashNode*)(bspHashingArray[currentNode->planeID-1]->GetAt(indexInHashingTable));
		newIDArray[currentNodeID-1]=hashNode->data;
		return (hashNode->data);
	}

	newIDCounter++;
	newIDArray[currentNodeID-1]=newIDCounter;

	//------------------------------------------------------------------------------------------------------------------
	//	Insert new hashing element into the hashing table
	BSPTREEHashNode *newHashNode=(BSPTREEHashNode *)malloc(sizeof(BSPTREEHashNode));
	newHashNode->leftID=leftChildID;	newHashNode->rightID=rightChildID;
	newHashNode->data=newIDCounter;
	bspHashingArray[currentNode->planeID-1]->Add(newHashNode);

	return newIDCounter;
}

bool BSPSolidOperation::_isSameSubTreeExist(GLKArray **bspHashingArray, 
											UINT planeID, UINT leftChildID, UINT rightChildID, int &indexInHashingTable)
{
	int size=bspHashingArray[planeID-1]->GetSize();
	for(indexInHashingTable=0;indexInHashingTable<size;indexInHashingTable++) {
		BSPTREEHashNode *hashNode=(BSPTREEHashNode*)(bspHashingArray[planeID-1]->GetAt(indexInHashingTable));
		if (hashNode->leftID==leftChildID && hashNode->rightID==rightChildID) return true;
	}

	return false;
}

void BSPSolidOperation::_compBoundingBox(int trglNum, TrglForBSP *trglArray, float bndBox[])
{
	int i,j;
	bndBox[0]=bndBox[2]=bndBox[4]=1.0e+9f;	bndBox[1]=bndBox[3]=bndBox[5]=-1.0e+9f;

	for(i=0;i<trglNum;i++) {
		for(j=0;j<3;j++) {
			bndBox[0]=MIN(bndBox[0],trglArray[i].verPnts[j][0]);
			bndBox[1]=MAX(bndBox[1],trglArray[i].verPnts[j][0]);
			bndBox[2]=MIN(bndBox[2],trglArray[i].verPnts[j][1]);
			bndBox[3]=MAX(bndBox[3],trglArray[i].verPnts[j][1]);
			bndBox[4]=MIN(bndBox[4],trglArray[i].verPnts[j][2]);
			bndBox[5]=MAX(bndBox[5],trglArray[i].verPnts[j][2]);
		}
	}
}

bool BSPSolidOperation::_compPlaneEquation(float p1[], float p2[], float p3[], 
										  float &aa, float &bb, float &cc, float &dd)
{
	float nv[3],cp[3],v1[3],v2[3];

	aa=bb=cc=dd=0.0;	aa=1.0;

//	cp[0]=(p1[0]+p2[0]+p3[0])/3.0;	cp[1]=(p1[1]+p2[1]+p3[1])/3.0;	cp[2]=(p1[2]+p2[2]+p3[2])/3.0;
	cp[0]=p1[0];	cp[1]=p1[1];	cp[2]=p1[2];
	v1[0]=p2[0]-cp[0];	v1[1]=p2[1]-cp[1];	v1[2]=p2[2]-cp[2];
	v2[0]=p3[0]-cp[0];	v2[1]=p3[1]-cp[1];	v2[2]=p3[2]-cp[2];

//	nv[0]=v1[1]*v2[2]-v1[2]*v2[1];	nv[1]=v1[2]*v2[0]-v1[0]*v2[2];	nv[2]=v1[0]*v2[1]-v1[1]*v2[0];
//	dd=nv[0]*nv[0]+nv[1]*nv[1]+nv[2]*nv[2];
	CROSS_PRODUCT(v1,v2,nv);
	dd=DOT_PRODUCT(nv,nv);
	if (dd<1.0e-20) return false;

	dd=sqrt(dd);	nv[0]=nv[0]/dd;	nv[1]=nv[1]/dd;	nv[2]=nv[2]/dd;

	aa=nv[0];	bb=nv[1];	cc=nv[2];	dd=-(cp[0]*nv[0]+cp[1]*nv[1]+cp[2]*nv[2]);

	return true;
}

float BSPSolidOperation::_compArea(float p1[], float p2[], float p3[])
{
	float x1,y1,z1,x2,y2,z2;
	float ii,jj,kk;
	float area;

	x1=p1[0]-p3[0];	y1=p1[1]-p3[1];	z1=p1[2]-p3[2];
	x2=p2[0]-p3[0];	y2=p2[1]-p3[1];	z2=p2[2]-p3[2];

	ii=y1*z2-z1*y2;
	jj=x2*z1-x1*z2;
	kk=x1*y2-x2*y1;

	area=sqrt(ii*ii+jj*jj+kk*kk)/2.0;

	return area;
}

void BSPSolidOperation::_bptNodeExport(BSPTREE *treePtr, int currentNodeID, FILE *fp)
{
	int leftChildID,rightChildID;
	int id=treePtr->nodeArray[currentNodeID-1].planeID-1;

	fprintf(fp,"%f %f %f %f\n",(float)(treePtr->planeArray[id].abcd[0]),
		(float)(treePtr->planeArray[id].abcd[1]),
		(float)(treePtr->planeArray[id].abcd[2]),
		(float)(treePtr->planeArray[id].abcd[3]));

	leftChildID=treePtr->nodeArray[currentNodeID-1].uLeftChildID;
	rightChildID=treePtr->nodeArray[currentNodeID-1].uRightChildID;

	if (treePtr->nodeArray[leftChildID-1].IsLeafNode()) {
		if (treePtr->nodeArray[leftChildID-1].IsSolidNode())
			fprintf(fp,"1 1 \n");
		else
			fprintf(fp,"1 0 \n");
	}
	else {
		fprintf(fp,"0 ");
		_bptNodeExport(treePtr,leftChildID,fp);
	}

	if (treePtr->nodeArray[rightChildID-1].IsLeafNode()) {
		if (treePtr->nodeArray[rightChildID-1].IsSolidNode())
			fprintf(fp,"1 1 \n");
		else
			fprintf(fp,"1 0 \n");
	}
	else {
		fprintf(fp,"0 ");
		_bptNodeExport(treePtr,rightChildID,fp);
	}
}

void BSPSolidOperation::_bptNodeImport(BSPTREEArrayNode *currentNode, GLKArray *bspPlaneArray, GLKArray *bspTreeNodeArray, FILE *fp)
{
	int nFlag;	float abcd[4];

	fscanf(fp,"%f %f %f %f\n",&(abcd[0]),&(abcd[1]),&(abcd[2]),&(abcd[3]));
	bspPlaneArray->Add(abcd[0]);	bspPlaneArray->Add(abcd[1]);
	bspPlaneArray->Add(abcd[2]);	bspPlaneArray->Add(abcd[3]);
	currentNode->planeID=(UINT)(bspPlaneArray->GetSize()/4);

	fscanf(fp,"%d ",&nFlag);
	if (nFlag==0) {	// NOT a leaf node
		BSPTREEArrayNode *leftChildNode=(BSPTREEArrayNode *)malloc(sizeof(BSPTREEArrayNode));	
		leftChildNode->uLeftChildID=leftChildNode->uRightChildID=0;
		bspTreeNodeArray->Add(leftChildNode);
		currentNode->uLeftChildID=(UINT)(bspTreeNodeArray->GetSize());

		_bptNodeImport(leftChildNode,bspPlaneArray,bspTreeNodeArray,fp);
	}
	else { // Is a leaf node
		fscanf(fp,"%d\n",&nFlag);
		currentNode->uLeftChildID=3;
		if (nFlag==1)	// solid
			currentNode->uLeftChildID=2;
	}

	fscanf(fp,"%d ",&nFlag);
	if (nFlag==0) {	// NOT a leaf node
		BSPTREEArrayNode *rightChildNode=(BSPTREEArrayNode *)malloc(sizeof(BSPTREEArrayNode));	
		rightChildNode->uLeftChildID=rightChildNode->uRightChildID=0;
		bspTreeNodeArray->Add(rightChildNode);
		currentNode->uRightChildID=(UINT)(bspTreeNodeArray->GetSize());

		_bptNodeImport(rightChildNode,bspPlaneArray,bspTreeNodeArray,fp);
	}
	else { // is a leaf node
		fscanf(fp,"%d\n",&nFlag);
		currentNode->uRightChildID=3;
		if (nFlag==1)	// solid
			currentNode->uRightChildID=2;
	}
}

void BSPSolidOperation::_booleanOperationOnBoundingBox(float solidABndBox[], float solidBBndBox[], short nOperationType, float resBndBox[])
{
	switch(nOperationType){
	case 0:{	// union
		resBndBox[0]=MIN(solidABndBox[0],solidBBndBox[0]); 	resBndBox[1]=MAX(solidABndBox[1],solidBBndBox[1]);		// x-min and x-max
		resBndBox[2]=MIN(solidABndBox[2],solidBBndBox[2]);	resBndBox[3]=MAX(solidABndBox[3],solidBBndBox[3]);		// y-min and y-max
		resBndBox[4]=MIN(solidABndBox[4],solidBBndBox[4]);	resBndBox[5]=MAX(solidABndBox[5],solidBBndBox[5]);		// z-min and z-max
		   }break;
	case 1:{	// intersection
		resBndBox[0]=MAX(solidABndBox[0],solidBBndBox[0]); 	resBndBox[1]=MIN(solidABndBox[1],solidBBndBox[1]);		// x-min and x-max
		resBndBox[2]=MAX(solidABndBox[2],solidBBndBox[2]);	resBndBox[3]=MIN(solidABndBox[3],solidBBndBox[3]);		// y-min and y-max
		resBndBox[4]=MAX(solidABndBox[4],solidBBndBox[4]);	resBndBox[5]=MIN(solidABndBox[5],solidBBndBox[5]);		// z-min and z-max
		   }break;
	case 2:{	// subtraction
		resBndBox[0]=solidABndBox[0]; 	resBndBox[1]=solidABndBox[1];		// x-min and x-max
		resBndBox[2]=solidABndBox[2];	resBndBox[3]=solidABndBox[3];		// y-min and y-max
		resBndBox[4]=solidABndBox[4];	resBndBox[5]=solidABndBox[5];		// z-min and z-max
		   }break;
	case 3:{	// inversed-subtraction
		resBndBox[0]=solidBBndBox[0]; 	resBndBox[1]=solidBBndBox[1];		// x-min and x-max
		resBndBox[2]=solidBBndBox[2];	resBndBox[3]=solidBBndBox[3];		// y-min and y-max
		resBndBox[4]=solidBBndBox[4];	resBndBox[5]=solidBBndBox[5];		// z-min and z-max
		   }break;
	}
}

UINT BSPSolidOperation::_booleanOperationByNaiveMerge(BSPTREE *treeAptr,
													  BSPTREE *treeBptr,
													  short nOperationType, BSPTREE *&treeResPtr)
{
	UINT i,range;

	//--------------------------------------------------------------------------------------------------
	//	Step 1: preparation
	//--------------------------------------------------------------------------------------------------
	//	Swap A and B (for inversed subtraction), and computethe complementary set of B (for subtraction operation)
	if (nOperationType==3) /* swap A & B solids */ {BSPTREE *tempTree=treeBptr; treeBptr=treeAptr; treeAptr=tempTree; nOperationType=2;}
	UINT planeSizeA,planeSizeB,nodeSizeA,nodeSizeB;
	planeSizeA=treeAptr->planeArraySize;	nodeSizeA=treeAptr->nodeArraySize;
	planeSizeB=treeBptr->planeArraySize;	nodeSizeB=treeBptr->nodeArraySize;
	if (nOperationType==2) {	// converting subtraction into the intersection of complementary set
		for(i=3;i<nodeSizeB;i++) {
			if (treeBptr->nodeArray[i].uLeftChildID==2) 
				{	treeBptr->nodeArray[i].uLeftChildID=3;	}
			else if (treeBptr->nodeArray[i].uLeftChildID==3) 
				{	treeBptr->nodeArray[i].uLeftChildID=2;	}
			if (treeBptr->nodeArray[i].uRightChildID==2) 
				{	treeBptr->nodeArray[i].uRightChildID=3;	}
			else if (treeBptr->nodeArray[i].uRightChildID==3) 
				{	treeBptr->nodeArray[i].uRightChildID=2;	}
		}
		nOperationType=1;
	}
	//--------------------------------------------------------------------------------------------------
	//	Swap A and B (for realizing an efficient Boolean operation)
	if (CountLevelNumber(treeBptr)<CountLevelNumber(treeAptr)) {
		BSPTREE *tempTree=treeBptr; treeBptr=treeAptr; treeAptr=tempTree;
		planeSizeA=treeAptr->planeArraySize;	nodeSizeA=treeAptr->nodeArraySize;
		planeSizeB=treeBptr->planeArraySize;	nodeSizeB=treeBptr->nodeArraySize;
	}
	//--------------------------------------------------------------------------------------------------
	//	Memory allocation
	treeResPtr=(BSPTREE*)malloc(sizeof(BSPTREE));
	treeResPtr->planeArraySize=planeSizeA+planeSizeB;
	treeResPtr->planeArray=(BSPTREEArrayPlane*)malloc(sizeof(BSPTREEArrayPlane)*(planeSizeA+planeSizeB));
	UINT newSize=nodeSizeA+(nodeSizeB-2);
	treeResPtr->nodeArraySize=newSize;
	treeResPtr->nodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*newSize);
	range=nodeSizeA;
//	CountLevelNumber(treeAptr)+CountLevelNumber(treeBptr);

	//--------------------------------------------------------------------------------------------------
	//	Step 2: copy information of treeA and treeB into the new tree
	memcpy(&(treeResPtr->planeArray[0]),treeAptr->planeArray,sizeof(BSPTREEArrayPlane)*planeSizeA);
	memcpy(&(treeResPtr->planeArray[planeSizeA]),treeBptr->planeArray,sizeof(BSPTREEArrayPlane)*planeSizeB);
	memcpy(&(treeResPtr->nodeArray[0]),treeAptr->nodeArray,sizeof(BSPTREEArrayNode)*nodeSizeA);
	treeResPtr->nodeArray[nodeSizeA]=treeBptr->nodeArray[0];
	memcpy(&(treeResPtr->nodeArray[nodeSizeA+1]),&(treeBptr->nodeArray[3]),sizeof(BSPTREEArrayNode)*(nodeSizeB-3));
	//--------------------------------------------------------------------------------------------------
	for(i=nodeSizeA;i<newSize;i++) {
		treeResPtr->nodeArray[i].planeID+=planeSizeA;
		if (treeResPtr->nodeArray[i].uLeftChildID!=2 && treeResPtr->nodeArray[i].uLeftChildID!=3) {
			treeResPtr->nodeArray[i].uLeftChildID+=(nodeSizeA-2);
		}
		if (treeResPtr->nodeArray[i].uRightChildID!=2 && treeResPtr->nodeArray[i].uRightChildID!=3) {
			treeResPtr->nodeArray[i].uRightChildID+=(nodeSizeA-2);
		}
	}

	//--------------------------------------------------------------------------------------------------
	//	Step 3: naive merging of two trees by changing the pointers (in terms of index)
	if (nOperationType==0) {	// union
		for(i=3;i<nodeSizeA;i++) {
			if (treeResPtr->nodeArray[i].uLeftChildID==3) treeResPtr->nodeArray[i].uLeftChildID=nodeSizeA+1;
			if (treeResPtr->nodeArray[i].uRightChildID==3) treeResPtr->nodeArray[i].uRightChildID=nodeSizeA+1;
		}
	}
	else {	// intersection (and also subtraction, which is converted into intersection of complementary set)
		for(i=3;i<nodeSizeA;i++) {
			if (treeResPtr->nodeArray[i].uLeftChildID==2) treeResPtr->nodeArray[i].uLeftChildID=nodeSizeA+1;
			if (treeResPtr->nodeArray[i].uRightChildID==2) treeResPtr->nodeArray[i].uRightChildID=nodeSizeA+1;
		}
	}

	return range;
}

UINT BSPSolidOperation::_reduceMergedTreeSubRoutine(UINT indexInMergedTree, BSPTREE *treePtr, UINT &currentNum, BSPTREEArrayNode* newNodeArray,
													UINT stackSize, BSPTREETravelStackNode *stackForTraversal, float computingBndBox[], float feaPnt[],
													UINT stIndexForFeasibleCheck)
{
	UINT currentIndex;				bool bLeftFeasible,bRightFeasible;//,bStPntAbove,bEdPntAbove,bIntersect;	
	UINT leftNodeID,rightNodeID;	float leftFeaPnt[6],rightFeaPnt[6];//,resPnt[3],stPnt[3],edPnt[3];

	if (treePtr->nodeArray[indexInMergedTree-1].IsLeafNode()) {return indexInMergedTree;}

	if (stackSize>MAXLEVEL_OF_STACK) {
		printf("Err: beyond the maximal allowed level of tree/stack!\n");
		return 3;		// Beyond the capability of MAXLEVEL_OF_STACK, simply return an empty node.
	}
	if (currentNum==MAXNUM_OF_BSPNODE_ONTREE) {
		printf("Err: running out of memory on BSP-solid!\n");
		return 3;		// Beyond the capability of memory, simply return an empty node.
	}

	//----------------------------------------------------------------------------------------------------------------------------
	//	Step 0: initialization
	bLeftFeasible=bRightFeasible=true;
	leftFeaPnt[0]=rightFeaPnt[0]=feaPnt[0];		leftFeaPnt[1]=rightFeaPnt[1]=feaPnt[1];		leftFeaPnt[2]=rightFeaPnt[2]=feaPnt[2];
	leftFeaPnt[3]=rightFeaPnt[3]=feaPnt[3];		leftFeaPnt[4]=rightFeaPnt[4]=feaPnt[4];		leftFeaPnt[5]=rightFeaPnt[5]=feaPnt[5];

	//----------------------------------------------------------------------------------------------------------------------------
	//	Step 1: check the feasibility of sub-regions
	//----------------------------------------------------------------------------------------------------------------------------
	long time=clock();
#ifdef BY_USING_CONVENTIAL_LP_FOR_FEASIBLEREGIONCHECK
	if (indexInMergedTree>=stIndexForFeasibleCheck) {
		stackForTraversal[stackSize].uTreeNodeID=indexInMergedTree;		stackForTraversal[stackSize].bIsOnLeftOrRightChild=true;
		bLeftFeasible=_detectFeasibleRegion(computingBndBox,treePtr,stackForTraversal,stackSize+1,leftFeaPnt);		
		stackForTraversal[stackSize].uTreeNodeID=indexInMergedTree;		stackForTraversal[stackSize].bIsOnLeftOrRightChild=false;
		bRightFeasible=_detectFeasibleRegion(computingBndBox,treePtr,stackForTraversal,stackSize+1,rightFeaPnt);
	}
#else
	stackForTraversal[stackSize].uTreeNodeID=indexInMergedTree;		stackForTraversal[stackSize].bIsOnLeftOrRightChild=true;
	bLeftFeasible=_recomputeFeasibleByLPwithTreePrior(computingBndBox,treePtr,stackForTraversal,stackSize+1,leftFeaPnt);
//	bLeftFeasible=_recomputeFeasiblePntByLinearProgramming(computingBndBox,treePtr,stackForTraversal,stackSize+1,leftFeaPnt);
	stackForTraversal[stackSize].uTreeNodeID=indexInMergedTree;		stackForTraversal[stackSize].bIsOnLeftOrRightChild=false;
	bRightFeasible=_recomputeFeasibleByLPwithTreePrior(computingBndBox,treePtr,stackForTraversal,stackSize+1,rightFeaPnt);
//	bRightFeasible=_recomputeFeasiblePntByLinearProgramming(computingBndBox,treePtr,stackForTraversal,stackSize+1,rightFeaPnt);
#endif
	totalTimeForFeasibilityCheck += (clock()-time);

	//----------------------------------------------------------------------------------------------------------------------------
	//	Step 2: obtain the node indices of sub-regions
	if ((!bRightFeasible) && (!bLeftFeasible)) return 3;	// return an empty node
	if (!bRightFeasible) {	// right-child is infeasible, return the ID of left-child
		leftNodeID=_reduceMergedTreeSubRoutine(treePtr->nodeArray[indexInMergedTree-1].uLeftChildID,
							treePtr,currentNum,newNodeArray,stackSize,stackForTraversal,computingBndBox,leftFeaPnt,stIndexForFeasibleCheck);
		return leftNodeID;
	}
	if (!bLeftFeasible) {	// left-child is infeasible, return the ID of right-child
		rightNodeID=_reduceMergedTreeSubRoutine(treePtr->nodeArray[indexInMergedTree-1].uRightChildID,
							treePtr,currentNum,newNodeArray,stackSize,stackForTraversal,computingBndBox,rightFeaPnt,stIndexForFeasibleCheck);
		return rightNodeID;
	}
	//----------------------------------------------------------------------------------------------------------------------------
	stackForTraversal[stackSize].uTreeNodeID=indexInMergedTree;		stackForTraversal[stackSize].bIsOnLeftOrRightChild=true;
	leftNodeID=_reduceMergedTreeSubRoutine(treePtr->nodeArray[indexInMergedTree-1].uLeftChildID,
						treePtr,currentNum,newNodeArray,stackSize+1,stackForTraversal,computingBndBox,leftFeaPnt,stIndexForFeasibleCheck);
	stackForTraversal[stackSize].uTreeNodeID=indexInMergedTree;		stackForTraversal[stackSize].bIsOnLeftOrRightChild=false;
	rightNodeID=_reduceMergedTreeSubRoutine(treePtr->nodeArray[indexInMergedTree-1].uRightChildID,
						treePtr,currentNum,newNodeArray,stackSize+1,stackForTraversal,computingBndBox,rightFeaPnt,stIndexForFeasibleCheck);
	if (leftNodeID==rightNodeID) return leftNodeID;
	//----------------------------------------------------------------------------------------------------------------------------
	currentNum++;	currentIndex=currentNum;
	newNodeArray[currentIndex-1].planeID=treePtr->nodeArray[indexInMergedTree-1].planeID;
	newNodeArray[currentIndex-1].uLeftChildID=leftNodeID;	newNodeArray[currentIndex-1].uRightChildID=rightNodeID;

	return currentIndex;
}

void BSPSolidOperation::_reduceMergedTree(float computingBndBox[], BSPTREE *treePtr, UINT stIndexForFeasibleCheck)
{
	BSPTREETravelStackNode stackForTraversal[MAXLEVEL_OF_STACK];
	UINT i,currentNum,planeNum;			BSPTREEArrayNode *newNodeArray;		
	UINT *newPlaneIndexArray;
	long time=clock();

	//--------------------------------------------------------------------------------------------------
	//	Step 1: information preparation
	//--------------------------------------------------------------------------------------------------
	newNodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*MAXNUM_OF_BSPNODE_ONTREE);
	//--------------------------------------------------------------------------------------------------
	printf("The buffer memory allocation takes %ld (ms)\n",clock()-time);	time=clock();
	totalTimeForFeasibilityCheck=0;		totalNumberOfLP_Taken=0;	totalNumberOfSimp_Projection=0;

	//--------------------------------------------------------------------------------------------------
	//	Step 2: expanding the merged tree by only recording the feasible nodes of the tree
	//--------------------------------------------------------------------------------------------------
	float feaPnt[6];
	feaPnt[0]=feaPnt[1]=feaPnt[2]=feaPnt[3]=feaPnt[4]=feaPnt[5]=0.0f;
	//--------------------------------------------------------------------------------------------------
	currentNum=3;	
	newNodeArray[1].MakeSolidNode();	newNodeArray[2].MakeEmptyNode();
	newNodeArray[0].planeID=treePtr->nodeArray[0].planeID;
	//--------------------------------------------------------------------------------------------------
	stackForTraversal[0].uTreeNodeID=1;		stackForTraversal[0].bIsOnLeftOrRightChild=true;
	newNodeArray[0].uLeftChildID=_reduceMergedTreeSubRoutine(treePtr->nodeArray[0].uLeftChildID,
		treePtr,currentNum,newNodeArray,1,stackForTraversal,computingBndBox,feaPnt,stIndexForFeasibleCheck);
	//--------------------------------------------------------------------------------------------------
	stackForTraversal[0].uTreeNodeID=1;		stackForTraversal[0].bIsOnLeftOrRightChild=false;
	newNodeArray[0].uRightChildID=_reduceMergedTreeSubRoutine(treePtr->nodeArray[0].uRightChildID,
		treePtr,currentNum,newNodeArray,1,stackForTraversal,computingBndBox,feaPnt,stIndexForFeasibleCheck);
	//--------------------------------------------------------------------------------------------------
	printf("The expanding with LP takes %ld (ms)\n",clock()-time);	time=clock();
	printf("\n----------------------------------------------------\n");
	printf("Total time for feasibility check is %ld (ms)\n",totalTimeForFeasibilityCheck);
	printf("%d times of Linear-Programming are taken \n(%d times are simply projection)\n",totalNumberOfLP_Taken,totalNumberOfSimp_Projection);
	printf("----------------------------------------------------\n\n");

	//--------------------------------------------------------------------------------------------------
	//	Step 3: memory management of nodeArray
	//--------------------------------------------------------------------------------------------------
	free(treePtr->nodeArray);	
	//--------------------------------------------------------------------------------------------------
	treePtr->nodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*currentNum);
	memcpy(treePtr->nodeArray,newNodeArray,sizeof(BSPTREEArrayNode)*currentNum);
	treePtr->nodeArraySize=currentNum;
	free(newNodeArray);

	//--------------------------------------------------------------------------------------------------
	//	Step 4: reduction of unused planes in the planeArray
	//--------------------------------------------------------------------------------------------------
	planeNum=treePtr->planeArraySize;
	newPlaneIndexArray=(UINT*)malloc(sizeof(UINT)*planeNum);	memset(newPlaneIndexArray,0,sizeof(UINT)*planeNum);
	//--------------------------------------------------------------------------------------------------
	newPlaneIndexArray[treePtr->nodeArray[0].planeID-1]=1;	// the following loop must bypass the solid/empty nodes
	for(i=3;i<treePtr->nodeArraySize;i++) {	newPlaneIndexArray[treePtr->nodeArray[i].planeID-1]=1;	}
	//--------------------------------------------------------------------------------------------------
	UINT newPlaneNum=0;
	for(i=0;i<planeNum;i++) {
		if (newPlaneIndexArray[i]==0) continue;
		newPlaneNum++;	newPlaneIndexArray[i]=newPlaneNum;	
	}
	//--------------------------------------------------------------------------------------------------
	BSPTREEArrayPlane *newPlaneArray=(BSPTREEArrayPlane*)malloc(sizeof(BSPTREEArrayPlane)*newPlaneNum);
	for(i=0;i<planeNum;i++) {if (newPlaneIndexArray[i]!=0) newPlaneArray[newPlaneIndexArray[i]-1]=treePtr->planeArray[i];	}
	//--------------------------------------------------------------------------------------------------
	treePtr->nodeArray[0].planeID=newPlaneIndexArray[treePtr->nodeArray[0].planeID-1];	// the following loop must bypass the solid/empty nodes
	for(i=3;i<treePtr->nodeArraySize;i++) {	treePtr->nodeArray[i].planeID=newPlaneIndexArray[treePtr->nodeArray[i].planeID-1];	}
	//--------------------------------------------------------------------------------------------------
	free(treePtr->planeArray);			free(newPlaneIndexArray);
	treePtr->planeArray=newPlaneArray;	treePtr->planeArraySize=newPlaneNum;
	//--------------------------------------------------------------------------------------------------
	printf("The management of memory takes %ld (ms)\n",clock()-time);

	printf("The tree has been reduced into %d nodes with %d planes.\n",(int)(treePtr->nodeArraySize),(int)(treePtr->planeArraySize));
}

UINT BSPSolidOperation::_expandMergedTreeSubRoutine(UINT indexInMergedTree, BSPTREE *treePtr, UINT &currentNum, BSPTREEArrayNode* newNodeArray)
{
	UINT currentIndex;

	if (treePtr->nodeArray[indexInMergedTree-1].IsLeafNode()) {return indexInMergedTree;}

	if (currentNum==MAXNUM_OF_BSPNODE_ONTREE) {
		printf("Err: running out of memory on BSP-solid!\n");
		return 3;		// Beyond the capability of memory, simply return an empty node.
	}

	currentNum++;	currentIndex=currentNum;
	newNodeArray[currentIndex-1].planeID=treePtr->nodeArray[indexInMergedTree-1].planeID;
	newNodeArray[currentIndex-1].uLeftChildID=_expandMergedTreeSubRoutine(treePtr->nodeArray[indexInMergedTree-1].uLeftChildID,treePtr,currentNum,newNodeArray);
	newNodeArray[currentIndex-1].uRightChildID=_expandMergedTreeSubRoutine(treePtr->nodeArray[indexInMergedTree-1].uRightChildID,treePtr,currentNum,newNodeArray);

	return currentIndex;
}

void BSPSolidOperation::_expandMergedTree(BSPTREE *treePtr)
{
	UINT currentNum;			BSPTREEArrayNode *newNodeArray;		

	//--------------------------------------------------------------------------------------------------
	//	Step 1: information preparation
	newNodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*MAXNUM_OF_BSPNODE_ONTREE);
	if (newNodeArray==NULL) {printf("Err! Not enough memory!\n");exit(1);}

	//--------------------------------------------------------------------------------------------------
	//	Step 2: expanding the merged tree
	currentNum=3;
	newNodeArray[1].MakeSolidNode();	newNodeArray[2].MakeEmptyNode();
	newNodeArray[0].planeID=treePtr->nodeArray[0].planeID;
	newNodeArray[0].uLeftChildID=_expandMergedTreeSubRoutine(treePtr->nodeArray[0].uLeftChildID,treePtr,currentNum,newNodeArray);
	newNodeArray[0].uRightChildID=_expandMergedTreeSubRoutine(treePtr->nodeArray[0].uRightChildID,treePtr,currentNum,newNodeArray);

	//--------------------------------------------------------------------------------------------------
	//	Step 3: memory management
	free(treePtr->nodeArray);	
	treePtr->nodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*currentNum);
	memcpy(treePtr->nodeArray,newNodeArray,sizeof(BSPTREEArrayNode)*currentNum);
	treePtr->nodeArraySize=currentNum;
	free(newNodeArray);

	printf("The tree has been expanded into %d nodes with %d planes.\n",(int)currentNum,(int)(treePtr->planeArraySize));
}

void BSPSolidOperation::_removeRedundancyOnTree(BSPTREE *treePtr)
{
	bool *bPlaneVisited,*bNodeVisited;
	UINT *newNodeIndexArray,*newPlaneIndexArray;
	BSPTREEArrayPlane* newPlaneArray;
	BSPTREEArrayNode* newNodeArray;	
	UINT i,newNodeNum,newPlaneNum;	
	UINT planeNum=treePtr->planeArraySize;
	UINT nodeNum=treePtr->nodeArraySize;

	//--------------------------------------------------------------------------------------------------------------------
	//	Step 1: initialization
	bPlaneVisited=(bool*)malloc(sizeof(bool)*planeNum);			memset(bPlaneVisited,0,sizeof(bool)*planeNum);
	bNodeVisited=(bool*)malloc(sizeof(bool)*nodeNum);			memset(bNodeVisited,0,sizeof(bool)*nodeNum);
	newPlaneIndexArray=(UINT*)malloc(sizeof(UINT)*planeNum);	memset(newPlaneIndexArray,0,sizeof(UINT)*planeNum);
	newNodeIndexArray=(UINT*)malloc(sizeof(UINT)*nodeNum);		memset(newNodeIndexArray,0,sizeof(UINT)*nodeNum);

	//--------------------------------------------------------------------------------------------------------------------
	//	Step 2: check whether the nodes and the planes have been visited
	bNodeVisited[1]=bNodeVisited[2]=true;
	_specifyVisitFlagOfTreeNodesPlanes(treePtr,1,bPlaneVisited,bNodeVisited);

	//--------------------------------------------------------------------------------------------------------------------
	//	Step 3: remove the unvisted treeNode and planeNode
	newNodeNum=newPlaneNum=0;
	for(i=0;i<planeNum;i++) if (bPlaneVisited[i]) {newPlaneNum++;	newPlaneIndexArray[i]=newPlaneNum;}
	for(i=0;i<nodeNum;i++) if (bNodeVisited[i]) {newNodeNum++;	newNodeIndexArray[i]=newNodeNum;}
	//--------------------------------------------------------------------------------------------------------------------
	newPlaneArray=(BSPTREEArrayPlane*)malloc(sizeof(BSPTREEArrayPlane)*newPlaneNum);
	for(i=0;i<planeNum;i++) {
		if (bPlaneVisited[i]) {newPlaneArray[newPlaneIndexArray[i]-1]=treePtr->planeArray[i];}
	}
	//--------------------------------------------------------------------------------------------------------------------
	newNodeArray=(BSPTREEArrayNode*)malloc(sizeof(BSPTREEArrayNode)*newNodeNum);
	for(i=0;i<nodeNum;i++) {
		if (bNodeVisited[i]) {
			UINT j=newNodeIndexArray[i]-1;
			if (treePtr->nodeArray[i].IsEmptyNode()) 
				{newNodeArray[j].MakeEmptyNode();}
			else if (treePtr->nodeArray[i].IsSolidNode()) 
				{newNodeArray[j].MakeSolidNode();}
			else {
				newNodeArray[j].planeID=newPlaneIndexArray[treePtr->nodeArray[i].planeID-1];
				newNodeArray[j].uLeftChildID=newNodeIndexArray[treePtr->nodeArray[i].uLeftChildID-1];
				newNodeArray[j].uRightChildID=newNodeIndexArray[treePtr->nodeArray[i].uRightChildID-1];
			}
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	free(treePtr->planeArray);	treePtr->planeArray=newPlaneArray;	treePtr->planeArraySize=newPlaneNum;
	free(treePtr->nodeArray);	treePtr->nodeArray=newNodeArray;	treePtr->nodeArraySize=newNodeNum;

	//--------------------------------------------------------------------------------------------------------------------
	//	Step 4: free the memory
	free(newPlaneIndexArray);	free(newNodeIndexArray);	free(bPlaneVisited);	free(bNodeVisited);

	printf("----------------------------------------------\n");
	printf("The redundency of tree has been removed: \nfrom %d planes into %d planes\nfrom %d nodes into %d nodes\n",
		planeNum,newPlaneNum,nodeNum,newNodeNum);
	printf("----------------------------------------------\n");
}

void BSPSolidOperation::_specifyVisitFlagOfTreeNodesPlanes(BSPTREE *treePtr, UINT currentNodeID, bool *bPlaneVisited, bool *bNodeVisited)
{
	UINT leftID,rightID;

	bNodeVisited[currentNodeID-1]=true;		bPlaneVisited[treePtr->nodeArray[currentNodeID-1].planeID-1]=true;

	leftID=treePtr->nodeArray[currentNodeID-1].uLeftChildID;
	if (!(treePtr->nodeArray[leftID-1].IsLeafNode())) _specifyVisitFlagOfTreeNodesPlanes(treePtr,leftID,bPlaneVisited,bNodeVisited);

	rightID=treePtr->nodeArray[currentNodeID-1].uRightChildID;
	if (!(treePtr->nodeArray[rightID-1].IsLeafNode())) _specifyVisitFlagOfTreeNodesPlanes(treePtr,rightID,bPlaneVisited,bNodeVisited);
}

bool BSPSolidOperation::_detectFeasibleRegion(float computingBndBox[], 
											  BSPTREE *trPtr, BSPTREETravelStackNode *stackForTreeTraversal, UINT stackSize,
											  float feaPnt[])
{
	bool bRes=true;
	UINT i,planeIndex;
	double lowerbound[3],upperbound[3],result[3],objectivevector[3];
	float dd;

#ifdef LP_FEASIBLEREGIONCHECK_WITH_FEASIBLEPNT
	for(i=0;i<stackSize;i++) {
		planeIndex=trPtr->nodeArray[stackForTreeTraversal[i].uTreeNodeID-1].planeID-1;
		dd=trPtr->planeArray[planeIndex].abcd[0]*feaPnt[0]
			+trPtr->planeArray[planeIndex].abcd[1]*feaPnt[1]
			+trPtr->planeArray[planeIndex].abcd[2]*feaPnt[2]
			+trPtr->planeArray[planeIndex].abcd[3];
		if ((stackForTreeTraversal[i].bIsOnLeftOrRightChild)) {	// inverse the constraint
			dd=-dd;
		}
		if (dd>0.0f) {bRes=false; break;}
	}
	if (bRes) return true;
#endif

	lowerbound[0]=computingBndBox[0];	upperbound[0]=computingBndBox[1];
	lowerbound[1]=computingBndBox[2];	upperbound[1]=computingBndBox[3];
	lowerbound[2]=computingBndBox[4];	upperbound[2]=computingBndBox[5];
	objectivevector[0]=objectivevector[1]=objectivevector[2]=0.0;	objectivevector[0]=1.0;

	double constraints[MAXLEVEL_OF_STACK*4];
	for(i=0;i<stackSize;i++) {
		planeIndex=trPtr->nodeArray[stackForTreeTraversal[i].uTreeNodeID-1].planeID-1;
		constraints[i*4]=trPtr->planeArray[planeIndex].abcd[0];
		constraints[i*4+1]=trPtr->planeArray[planeIndex].abcd[1];
		constraints[i*4+2]=trPtr->planeArray[planeIndex].abcd[2];
		constraints[i*4+3]=-(trPtr->planeArray[planeIndex].abcd[3]);
		if ((stackForTreeTraversal[i].bIsOnLeftOrRightChild)) {	// inverse the constraint
			constraints[i*4]=-constraints[i*4];			constraints[i*4+1]=-constraints[i*4+1];
			constraints[i*4+2]=-constraints[i*4+2];		constraints[i*4+3]=-constraints[i*4+3];
		}
	}
	bRes=_linearProgrammingBySeidelMethodFor3D(stackSize,constraints,objectivevector,lowerbound,upperbound,result);
	totalNumberOfLP_Taken++;

	feaPnt[0]=result[0];	feaPnt[1]=result[1];	feaPnt[2]=result[2];

	return bRes;
}

bool BSPSolidOperation::_detectOverlapping(float computingBndBox[], 
										   BSPTREE *treeAptr, BSPTREETravelStackNode *stackForTreeA, UINT stackSizeA,
										   BSPTREE *treeBptr, BSPTREETravelStackNode *stackForTreeB, UINT stackSizeB)
{
	bool bRes=true;
	unsigned int i,planeIndex;
	double lowerbound[3],upperbound[3],result[3],objectivevector[3];

	lowerbound[0]=computingBndBox[0];	upperbound[0]=computingBndBox[1];
	lowerbound[1]=computingBndBox[2];	upperbound[1]=computingBndBox[3];
	lowerbound[2]=computingBndBox[4];	upperbound[2]=computingBndBox[5];
	objectivevector[0]=objectivevector[1]=objectivevector[2]=0.0;	objectivevector[0]=1.0;

	unsigned int constraintsnum=stackSizeA+stackSizeB;
	double *constraints=new double[constraintsnum*4];
	for(i=0;i<stackSizeA;i++) {
		planeIndex=treeAptr->nodeArray[stackForTreeA[i].uTreeNodeID-1].planeID-1;
		constraints[i*4]=treeAptr->planeArray[planeIndex].abcd[0];
		constraints[i*4+1]=treeAptr->planeArray[planeIndex].abcd[1];
		constraints[i*4+2]=treeAptr->planeArray[planeIndex].abcd[2];
		constraints[i*4+3]=-(treeAptr->planeArray[planeIndex].abcd[3]);
		if ((stackForTreeA[i].bIsOnLeftOrRightChild)) {	// inverse the constraint
			constraints[i*4]=-constraints[i*4];			constraints[i*4+1]=-constraints[i*4+1];
			constraints[i*4+2]=-constraints[i*4+2];		constraints[i*4+3]=-constraints[i*4+3];
		}
	}
	for(i=0;i<stackSizeB;i++) {
		planeIndex=treeBptr->nodeArray[stackForTreeB[i].uTreeNodeID-1].planeID-1;
		constraints[(i+stackSizeA)*4]=treeBptr->planeArray[planeIndex].abcd[0];
		constraints[(i+stackSizeA)*4+1]=treeBptr->planeArray[planeIndex].abcd[1];
		constraints[(i+stackSizeA)*4+2]=treeBptr->planeArray[planeIndex].abcd[2];
		constraints[(i+stackSizeA)*4+3]=-(treeBptr->planeArray[planeIndex].abcd[3]);
		if ((stackForTreeB[i].bIsOnLeftOrRightChild)) {	// inverse the constraint
			constraints[(i+stackSizeA)*4]=-constraints[(i+stackSizeA)*4];		constraints[(i+stackSizeA)*4+1]=-constraints[(i+stackSizeA)*4+1];
			constraints[(i+stackSizeA)*4+2]=-constraints[(i+stackSizeA)*4+2];	constraints[(i+stackSizeA)*4+3]=-constraints[(i+stackSizeA)*4+3];
		}
	}
	bRes=_linearProgrammingBySeidelMethod(3,constraintsnum,constraints,objectivevector,lowerbound,upperbound,result);
//	bRes=_linearProgrammingBySeidelMethodFor3D(constraintsnum,constraints,objectivevector,lowerbound,upperbound,result);
	delete []constraints;

	//-------------------------------------------------------------------------------------------------------------
	//	For debug purpose
/*	constraintsnum=3;
	double *constraints=new double[constraintsnum*4];
	int i=0;
	constraints[i*4]=1.0;	constraints[i*4+1]=0.0;		constraints[i*4+2]=0.0;		constraints[i*4+3]=0.0;		i++;
	constraints[i*4]=0.0;	constraints[i*4+1]=1.0;		constraints[i*4+2]=0.0;		constraints[i*4+3]=0.0;		i++;
	constraints[i*4]=0.0;	constraints[i*4+1]=0.0;		constraints[i*4+2]=-1.0;	constraints[i*4+3]=0.0;		i++;
	bRes=_linearProgrammingBySeidelMethod(3,constraintsnum,constraints,objectivevector,lowerbound,upperbound,result);
	delete []constraints;*/
/*	constraintsnum=3;
	double *constraints=new double[constraintsnum*2];
	int i=0;
	constraints[i*2]=-1.0;	constraints[i*2+1]=10.0;		i++;
	constraints[i*2]=-1.0;	constraints[i*2+1]=12.0;		i++;
	constraints[i*2]=-1.0;	constraints[i*2+1]=-1.0;		i++;
	bRes=_linearProgrammingBySeidelMethod(1,constraintsnum,constraints,objectivevector,lowerbound,upperbound,result);
	delete []constraints;*/

	return bRes;
}

bool BSPSolidOperation::_linearProgrammingBySeidelMethodFor3D(unsigned int constraintsnum, double constraints[],
															  double objectivevector[], double lowerbound[], 
															  double upperbound[], double result[])
{
	const unsigned int dimension = 3;
	double Apri[(MAXLEVEL_OF_STACK+2)*3];

	for(unsigned int i=0; i<dimension; i++) {
		result[i] = (objectivevector[i]>=0) ?(upperbound[i]):(lowerbound[i]);
	}

	unsigned int countA = constraintsnum;
	bool booleanresult = true;
	while(countA!=0) {
		double testing3 = 0.0;
		unsigned int k;
		for(unsigned int i=0; i<dimension; i++)
		{
			testing3 += result[i]*constraints[(countA-1)*(dimension+1)+i];
		}
		if(testing3-constraints[countA*(dimension+1)-1]>10.0*EPS_DOUBLEPRECISION) {
//			k = PickMaxOrMinAbsValue(dimension, &constraints[(countA-1)*(dimension+1)], true);	// ??? the following is what I guessed.
			//k = MAX(dimension, (fabs(constraints[(countA-1)*(dimension+1)])) );
			k=0;
			for(unsigned int temp=1; temp<dimension; temp++) {
				if (fabs(constraints[(countA-1)*(dimension+1)+temp])>fabs(constraints[(countA-1)*(dimension+1)+k])) k=temp;
			}
		}
		else
		{
			countA--;
			continue;
		}
		
		if(fabs(constraints[(countA-1)*(dimension+1)+k])<EPS_DOUBLEPRECISION)
		{
			booleanresult = false;
			break;
		}

		for(unsigned int i=0; i<constraintsnum-countA; i++)
		{
			unsigned int j;
			double factor = constraints[(countA+i)*(dimension+1)+k]/constraints[(countA-1)*(dimension+1)+k];
			for(j=0; j<k; j++)
			{
				Apri[i*dimension+j] = constraints[(countA+i)*(dimension+1)+j]-
					factor*constraints[(countA-1)*(dimension+1)+j];
			}
			for(j=k+1; j<dimension+1; j++)
			{
				Apri[i*dimension+j-1] = constraints[(countA+i)*(dimension+1)+j]-
					factor*constraints[(countA-1)*(dimension+1)+j];
			}
		}

		double cpri[2];
		double factorforc = objectivevector[k]/constraints[(countA-1)*(dimension+1)+k];
		for(unsigned int i=0; i<k; i++)
		{
			cpri[i] = objectivevector[i] - factorforc*constraints[(countA-1)*(dimension+1)+i];
		}
		for(unsigned int i=k+1; i<dimension; i++)
		{
			cpri[i-1] = objectivevector[i] - factorforc*constraints[(countA-1)*(dimension+1)+i];
		}

		double factorforextratwocons = 1.0/constraints[(countA-1)*(dimension+1)+k];
		for(unsigned int i=0; i<k; i++)
		{
			Apri[(constraintsnum-countA)*dimension+i] = -factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
			Apri[(constraintsnum-countA+1)*dimension+i] = factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
		}
		for(unsigned int i=k+1; i<dimension; i++)
		{
			Apri[(constraintsnum-countA)*dimension+i-1] = -factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
			Apri[(constraintsnum-countA+1)*dimension+i-1] = factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
		}
		Apri[(constraintsnum-countA+1)*dimension-1] = upperbound[k]-factorforextratwocons*constraints[(countA-1)*(dimension+1)+dimension];
		Apri[(constraintsnum-countA+2)*dimension-1] = -lowerbound[k]+factorforextratwocons*constraints[(countA-1)*(dimension+1)+dimension];

		double upri[2],lpri[2];
		for(unsigned int i=0; i<k; i++)
		{
			upri[i] = upperbound[i];
			lpri[i] = lowerbound[i];
		}
		for(unsigned int i=k+1; i<dimension; i++)
		{
			upri[i-1] = upperbound[i];
			lpri[i-1] = lowerbound[i];
		}

		double resultpri[2];
		bool localresult = _linearProgrammingBySeidelMethod(dimension-1, constraintsnum-countA+2, Apri, cpri, lpri, upri, resultpri);
		if(!localresult)
		{
			booleanresult = false;
			break;
		}

		for(unsigned int i=0; i<k; i++)
			result[i] = resultpri[i];
		result[k] = 0.0;
		for(unsigned int i=k+1; i<dimension; i++)
			result[i] = resultpri[i-1];
		double temnumber = 0.0;
		for(unsigned int i=0; i<dimension; i++)
		{
			temnumber += constraints[(countA-1)*(dimension+1)+i]*result[i];
		}
		result[k] = factorforextratwocons*(constraints[(countA)*(dimension+1)-1]-temnumber);

		countA--;
	}

	if(booleanresult)
		return true;
	else
		return false;
}

bool BSPSolidOperation::_linearProgrammingBySeidelMethod(unsigned int dimension, unsigned int constraintsnum, double constraints[],
														 double objectivevector[], double lowerbound[], double upperbound[], double result[])
{
	if(dimension==1)
	{
		double high = upperbound[0];
		double low = lowerbound[0];
		double z = 1.0e+20;
		for(unsigned int i=0; i<constraintsnum; i++)
		{
			if(constraints[i*2]>EPS_DOUBLEPRECISION)
			{
				double testing1 = constraints[i*2+1]/constraints[i*2];
				if(testing1< high)
					high = testing1;
			}
			else if(constraints[i*2]<-EPS_DOUBLEPRECISION)
			{
				double testing2 = constraints[i*2+1]/constraints[i*2];
				if(testing2>low)
					low = testing2;
			}
			else
			{
				if(constraints[i*2+1]<z)
					z = constraints[2*i+1];
			}
		}

		if(z<-EPS_DOUBLEPRECISION || high-low<-EPS_DOUBLEPRECISION ) {
			/*std::cout<<" infeasible region!"<<std::endl;*/
			return false;
		}

		if(objectivevector[0]>0) result[0] = high; else result[0] = low;
		return true;
	}

	for(unsigned int i=0; i<dimension; i++) {
		result[i] = (objectivevector[i]>=0) ?(upperbound[i]):(lowerbound[i]);
	}

	unsigned int countA = constraintsnum;
	bool booleanresult = true;
	while(countA!=0) {
		double testing3 = 0.0;
		unsigned int k;
		for(unsigned int i=0; i<dimension; i++)
		{
			testing3 += result[i]*constraints[(countA-1)*(dimension+1)+i];
		}
		if(testing3-constraints[countA*(dimension+1)-1]>10.0*EPS_DOUBLEPRECISION) {
//			k = PickMaxOrMinAbsValue(dimension, &constraints[(countA-1)*(dimension+1)], true);	// ??? the following is what I guessed.
			//k = MAX(dimension, (fabs(constraints[(countA-1)*(dimension+1)])) );
			k=0;
			for(unsigned int temp=1; temp<dimension; temp++) {
				if (fabs(constraints[(countA-1)*(dimension+1)+temp])>fabs(constraints[(countA-1)*(dimension+1)+k])) k=temp;
			}
		}
		else
		{
			countA--;
			continue;
		}
		
		if(fabs(constraints[(countA-1)*(dimension+1)+k])<EPS_DOUBLEPRECISION)
		{
			booleanresult = false;
			break;
		}

		double *Apri = new double[dimension*(constraintsnum-countA+2)];
		for(unsigned int i=0; i<constraintsnum-countA; i++)
		{
			unsigned int j;
			double factor = constraints[(countA+i)*(dimension+1)+k]/constraints[(countA-1)*(dimension+1)+k];
			for(j=0; j<k; j++)
			{
				Apri[i*dimension+j] = constraints[(countA+i)*(dimension+1)+j]-
					factor*constraints[(countA-1)*(dimension+1)+j];
			}
			for(j=k+1; j<dimension+1; j++)
			{
				Apri[i*dimension+j-1] = constraints[(countA+i)*(dimension+1)+j]-
					factor*constraints[(countA-1)*(dimension+1)+j];
			}
		}

		double *cpri = new double[dimension-1];
		double factorforc = objectivevector[k]/constraints[(countA-1)*(dimension+1)+k];
		for(unsigned int i=0; i<k; i++)
		{
			cpri[i] = objectivevector[i] - factorforc*constraints[(countA-1)*(dimension+1)+i];
		}
		for(unsigned int i=k+1; i<dimension; i++)
		{
			cpri[i-1] = objectivevector[i] - factorforc*constraints[(countA-1)*(dimension+1)+i];
		}

		double factorforextratwocons = 1.0/constraints[(countA-1)*(dimension+1)+k];
		for(unsigned int i=0; i<k; i++)
		{
			Apri[(constraintsnum-countA)*dimension+i] = -factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
			Apri[(constraintsnum-countA+1)*dimension+i] = factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
		}
		for(unsigned int i=k+1; i<dimension; i++)
		{
			Apri[(constraintsnum-countA)*dimension+i-1] = -factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
			Apri[(constraintsnum-countA+1)*dimension+i-1] = factorforextratwocons*constraints[(countA-1)*(dimension+1)+i];
		}
		Apri[(constraintsnum-countA+1)*dimension-1] = upperbound[k]-factorforextratwocons*constraints[(countA-1)*(dimension+1)+dimension];
		Apri[(constraintsnum-countA+2)*dimension-1] = -lowerbound[k]+factorforextratwocons*constraints[(countA-1)*(dimension+1)+dimension];

		double *upri = new double[dimension-1];
		double *lpri = new double[dimension-1];
		for(unsigned int i=0; i<k; i++)
		{
			upri[i] = upperbound[i];
			lpri[i] = lowerbound[i];
		}
		for(unsigned int i=k+1; i<dimension; i++)
		{
			upri[i-1] = upperbound[i];
			lpri[i-1] = lowerbound[i];
		}

		double *resultpri = new double[dimension-1];
		bool localresult = _linearProgrammingBySeidelMethod(dimension-1, constraintsnum-countA+2, Apri, cpri, lpri, upri, resultpri);
		if(!localresult)
		{
			booleanresult = false;
			delete Apri;
			delete cpri;
			delete upri;
			delete lpri;
			delete resultpri;
			break;
		}

		for(unsigned int i=0; i<k; i++)
			result[i] = resultpri[i];
		result[k] = 0.0;
		for(unsigned int i=k+1; i<dimension; i++)
			result[i] = resultpri[i-1];
		double temnumber = 0.0;
		for(unsigned int i=0; i<dimension; i++)
		{
			temnumber += constraints[(countA-1)*(dimension+1)+i]*result[i];
		}
		result[k] = factorforextratwocons*(constraints[(countA)*(dimension+1)-1]-temnumber);

		countA--;
		delete Apri;
		delete cpri;
		delete upri;
		delete lpri;
		delete resultpri;
	}

	if(booleanresult)
		return true;
	else
		return false;
}

bool BSPSolidOperation::_clippingSegmentByPlane(float stPnt[], float edPnt[], float abcd[], float &resAlpha, bool &bStPntAbove, bool &bEdPntAbove)
{
	float d1,d2,alpha;

	d1=stPnt[0]*abcd[0]+stPnt[1]*abcd[1]+stPnt[2]*abcd[2]+abcd[3];
	if (d1>=0.0f) bStPntAbove=true; else bStPntAbove=false;
	d2=edPnt[0]*abcd[0]+edPnt[1]*abcd[1]+edPnt[2]*abcd[2]+abcd[3];
	if (d2>=0.0f) bEdPntAbove=true; else bEdPntAbove=false;
	if (bStPntAbove==bEdPntAbove) return false;

	alpha=fabs(d1)+fabs(d2);	if (alpha<EPS) alpha=EPS;
	alpha=fabs(d1)/alpha;
	resAlpha=alpha;

	return true;
}

bool BSPSolidOperation::_clippingPolygonByPlane(float *pnts, int pntsNum, float abcd[], float stPnt[], float edPnt[])
{
	int i,j,stIndex,edIndex;
	float d1=0.0f,d2,alpha;		bool bLastSign,bCurrentSign;

	stIndex=edIndex=-1;		
	for(i=0;i<=pntsNum;i++) {
		j=i%pntsNum;
		
		d2=pnts[j*3]*abcd[0]+pnts[j*3+1]*abcd[1]+pnts[j*3+2]*abcd[2]+abcd[3];
		if (d2>0.0f) bCurrentSign=true; else bCurrentSign=false;

		if (i==0) {bLastSign=bCurrentSign; d1=d2; continue;	}

		if (bLastSign!=bCurrentSign) {
			if (bLastSign) {	// above
				edIndex=j;
				alpha=fabs(d1)+fabs(d2);	
				if (alpha<EPS) alpha=0.5f; else alpha=fabs(d1)/alpha;
				edPnt[0]=(1.0-alpha)*pnts[((j+(pntsNum-1))%pntsNum)*3]+alpha*pnts[j*3];
				edPnt[1]=(1.0-alpha)*pnts[((j+(pntsNum-1))%pntsNum)*3+1]+alpha*pnts[j*3+1];
				edPnt[2]=(1.0-alpha)*pnts[((j+(pntsNum-1))%pntsNum)*3+2]+alpha*pnts[j*3+2];
			}	
			else {				// below
				stIndex=j;
				alpha=fabs(d1)+fabs(d2);	
				if (alpha<EPS) alpha=0.5f; else alpha=fabs(d1)/alpha;
				stPnt[0]=(1.0-alpha)*pnts[((j+(pntsNum-1))%pntsNum)*3]+alpha*pnts[j*3];
				stPnt[1]=(1.0-alpha)*pnts[((j+(pntsNum-1))%pntsNum)*3+1]+alpha*pnts[j*3+1];
				stPnt[2]=(1.0-alpha)*pnts[((j+(pntsNum-1))%pntsNum)*3+2]+alpha*pnts[j*3+2];
			}
		}
		bLastSign=bCurrentSign;		d1=d2;
	}
	if (stIndex<0 || edIndex<0) return false;

	return true;
}

void BSPSolidOperation::_determineHalfSpaceByIndex(BSPTREETravelStackNode *stackForTreeTraversal,
												   BSPTREE *trPtr, UINT index, float abcd[])
{
	UINT planeID=trPtr->nodeArray[stackForTreeTraversal[index].uTreeNodeID-1].planeID;

	if (stackForTreeTraversal[index].bIsOnLeftOrRightChild) {
		abcd[0]=trPtr->planeArray[planeID-1].abcd[0];
		abcd[1]=trPtr->planeArray[planeID-1].abcd[1];
		abcd[2]=trPtr->planeArray[planeID-1].abcd[2];
		abcd[3]=trPtr->planeArray[planeID-1].abcd[3];
	}
	else {
		abcd[0]=-(trPtr->planeArray[planeID-1].abcd[0]);
		abcd[1]=-(trPtr->planeArray[planeID-1].abcd[1]);
		abcd[2]=-(trPtr->planeArray[planeID-1].abcd[2]);
		abcd[3]=-(trPtr->planeArray[planeID-1].abcd[3]);
	}
}

bool BSPSolidOperation::_recomputeFeasibleByLPwithTreePrior(float computingBndBox[], BSPTREE *trPtr,		//	If NOT feasible, the reture will be FALSE	
															BSPTREETravelStackNode *stackForTreeTraversal, UINT stackSize, float feaPnt[])	//	The last plane in stack is the newly added one.
{
	float pnts[12],abcd[4],maxValue,stPnt[3],edPnt[3],pntVec[3],lowerBnd,upperBnd,alpha,stDot,edDot,px,py,pz,sign;	
	UINT i,j,nAxis,id;				float *pp;

	_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,stackSize-1,abcd);
	if (abcd[0]*feaPnt[0]+abcd[1]*feaPnt[1]+abcd[2]*feaPnt[2]+abcd[3]>=EPS) return true;

	totalNumberOfLP_Taken++;

	//---------------------------------------------------------------------------------------------------------------------------------
	//	Determine the polygon for computing the feasible point
	nAxis=0;	maxValue=fabs(abcd[0]);
	if (fabs(abcd[1])>maxValue) {maxValue=fabs(abcd[1]); nAxis=1;}
	if (fabs(abcd[2])>maxValue) {nAxis=2;}
	//---------------------------------------------------------------------------------------------------------------------------------
	//	(xmin,xmax,ymin,ymax,zmin,zmax) - computingBndBox[0,1,2,3,4,5]
	//---------------------------------------------------------------------------------------------------------------------------------
	pp=&(pnts[0]);
	pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)];		// (a+1)-min
	pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)];		// (a+2)-min	
	pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
	//---------------------------------------------------------------------------------------------------------------------------------
	pp=&(pnts[3]);
	pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)+1];		// (a+1)-max
	pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)];		// (a+2)-min
	pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
	//---------------------------------------------------------------------------------------------------------------------------------
	pp=&(pnts[6]);
	pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)+1];		// (a+1)-max 
	pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)+1];		// (a+2)-max
	pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
	//---------------------------------------------------------------------------------------------------------------------------------
	pp=&(pnts[9]);
	pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)];		// (a+1)-min
	pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)+1];		// (a+2)-max
	pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
	//---------------------------------------------------------------------------------------------------------------------------------
	//	determine (px,py,pz) by projection
	float newPnt[3];
	alpha=-(feaPnt[0]*abcd[0]+feaPnt[1]*abcd[1]+feaPnt[2]*abcd[2]+abcd[3])/(abcd[0]*abcd[0]+abcd[1]*abcd[1]+abcd[2]*abcd[2]);
	newPnt[(nAxis+1)%3]=feaPnt[(nAxis+1)%3]+alpha*abcd[(nAxis+1)%3];
	newPnt[(nAxis+2)%3]=feaPnt[(nAxis+2)%3]+alpha*abcd[(nAxis+2)%3];
	newPnt[nAxis]=-(abcd[(nAxis+1)%3]*newPnt[(nAxis+1)%3]+abcd[(nAxis+2)%3]*newPnt[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
	px=newPnt[0];	py=newPnt[1];	pz=newPnt[2];
	//---------------------------------------------------------------------------------------------------------------------------------
	//	random picking a point
//	px=pnts[0];		py=pnts[1];		pz=pnts[2];	
	if (stackSize==1) {feaPnt[0]=px;	feaPnt[1]=py;	feaPnt[2]=pz;	return true;}

	//---------------------------------------------------------------------------------------------------------------------------------
	//	Linear Programming in 2D to find the feasible point on the plane
	//---------------------------------------------------------------------------------------------------------------------------------
	//	determining the order of processing (for increasing the numerical robustness)	
//	float tempWeight,weights[MAXLEVEL_OF_STACK];	UINT tempIndex;
	UINT indexMap[MAXLEVEL_OF_STACK];
	for(i=0;i<stackSize-1;i++) {
//		id=trPtr->nodeArray[stackForTreeTraversal[i].uTreeNodeID-1].planeID-1;
//		weights[i]=abcd[0]*(trPtr->planeArray[id].abcd[0])+abcd[1]*(trPtr->planeArray[id].abcd[1])+abcd[2]*(trPtr->planeArray[id].abcd[2]);
		indexMap[i]=stackSize-2-i;
	}
/*	for(i=0;i<stackSize-1;i++) {
		for(j=0;j<i;j++) {
			if (weights[i]<weights[j]) 
				{tempWeight=weights[i];	weights[i]=weights[j];	weights[j]=tempWeight;	tempIndex=indexMap[i];	indexMap[i]=indexMap[j];	indexMap[j]=tempIndex;}
		}
	}*/
	//---------------------------------------------------------------------------------------------------------------------------------
	bool bIntersected,bStPntAbove,bEdPntAbove;			bool bSimpleProjection=true;
	for(i=0;i<stackSize-1;i++) {
		_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,indexMap[i],abcd);
		if (abcd[0]*px+abcd[1]*py+abcd[2]*pz+abcd[3]>=0) continue;

		bSimpleProjection=false;
		if (_clippingPolygonByPlane(pnts,4,abcd,stPnt,edPnt)) {
			//-----------------------------------------------------------------------------------------
			//	subsub-step 1: find the first feasible region in LP on the projection plane
			lowerBnd=0.0;	upperBnd=1.0;	
			pntVec[0]=edPnt[0]-stPnt[0];	pntVec[1]=edPnt[1]-stPnt[1];	pntVec[2]=edPnt[2]-stPnt[2];

			//-----------------------------------------------------------------------------------------
			//	subsub-step 2: incrementally check and update the feasible region in 1D
			for(j=0;j<i;j++) {
				_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,indexMap[j],abcd);
				bIntersected=_clippingSegmentByPlane(stPnt,edPnt,abcd,alpha,bStPntAbove,bEdPntAbove);
				if (!bIntersected) {
					if (bStPntAbove) continue;
					return false;
				}
				if (bStPntAbove) upperBnd=MIN(upperBnd,alpha); else lowerBnd=MAX(lowerBnd,alpha);
				if (lowerBnd>=upperBnd-EPS) return false;
			}

			//-----------------------------------------------------------------------------------------
			//	subsub-step 3: determine the feasible point in 2D
			alpha=(lowerBnd+upperBnd)*0.5f;
			px=stPnt[0]+pntVec[0]*alpha;	py=stPnt[1]+pntVec[1]*alpha;	pz=stPnt[2]+pntVec[2]*alpha;
		}
		else {
			return false;
		}
	}
	if (bSimpleProjection) totalNumberOfSimp_Projection++;

	feaPnt[0]=px;	feaPnt[1]=py;	feaPnt[2]=pz;
	return true;
}

bool BSPSolidOperation::_recomputeFeasiblePntByLinearProgramming(float computingBndBox[], BSPTREE *trPtr,		//	If NOT feasible, the reture will be FALSE	
																 BSPTREETravelStackNode *stackForTreeTraversal, 
																 UINT stackSize, float feaPnt[])				//	The last plane in stack is the newly added one.
{
	float pnts[12],abcd[4],maxValue,stPnt[3],edPnt[3],pntVec[3],lowerBnd,upperBnd,alpha,stDot,edDot,px,py,pz,sign;	
	UINT i,j,nAxis,infeasibleIndex=0;				float *pp;

    px=feaPnt[0];	py=feaPnt[1];	pz=feaPnt[2];

	bool bFeasible=true;	
	for(i=0;i<stackSize;i++) {
		_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,stackSize-1-i,abcd);
		if (abcd[0]*px+abcd[1]*py+abcd[2]*pz+abcd[3]<0) {infeasibleIndex=i; bFeasible=false; break;}
	}
	if (bFeasible) return true;

	//-------------------------------------------------------------------------------------------------------------
	//	Using linear programming to find the feasible point
	totalNumberOfLP_Taken++;
	while(!bFeasible){
		//---------------------------------------------------------------------------------------------------------------------------------
		//	Determine the polygon for computing the feasible point
		_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,stackSize-1-infeasibleIndex,abcd);
		nAxis=0;	maxValue=fabs(abcd[0]);
		if (fabs(abcd[1])>maxValue) {maxValue=fabs(abcd[1]); nAxis=1;}
		if (fabs(abcd[2])>maxValue) {nAxis=2;}
		//---------------------------------------------------------------------------------------------------------------------------------
		//	(xmin,xmax,ymin,ymax,zmin,zmax) - computingBndBox[0,1,2,3,4,5]
		//---------------------------------------------------------------------------------------------------------------------------------
		pp=&(pnts[0]);
		pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)];		// (a+1)-min
		pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)];		// (a+2)-min	
		pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
		//---------------------------------------------------------------------------------------------------------------------------------
		pp=&(pnts[3]);
		pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)+1];		// (a+1)-max
		pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)];		// (a+2)-min
		pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
		//---------------------------------------------------------------------------------------------------------------------------------
		pp=&(pnts[6]);
		pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)+1];		// (a+1)-max 
		pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)+1];		// (a+2)-max
		pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];
		//---------------------------------------------------------------------------------------------------------------------------------
		pp=&(pnts[9]);
		pp[(nAxis+1)%3]=computingBndBox[2*((nAxis+1)%3)];		// (a+1)-min
		pp[(nAxis+2)%3]=computingBndBox[2*((nAxis+2)%3)+1];		// (a+2)-max
		pp[nAxis]=-(abcd[(nAxis+1)%3]*pp[(nAxis+1)%3]+abcd[(nAxis+2)%3]*pp[(nAxis+2)%3]+abcd[3])/abcd[nAxis];

		//---------------------------------------------------------------------------------------------------------------------------------
		//	Linear Programming in 2D to find the feasible point on the plane
		px=pnts[0];		py=pnts[1];		pz=pnts[2];
		for(i=0;i<infeasibleIndex;i++) {
			_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,stackSize-1-i,abcd);
			if (abcd[0]*px+abcd[1]*py+abcd[2]*pz+abcd[3]>=0) continue;

			//---------------------------------------------------------------------------------------------------------------------------------
			if (_clippingPolygonByPlane(pnts,4,abcd,stPnt,edPnt)) {
				//-----------------------------------------------------------------------------------------
				//	subsub-step 1: find the first feasible region in LP on the projection plane
				lowerBnd=0.0;	upperBnd=1.0;	
				pntVec[0]=edPnt[0]-stPnt[0];	pntVec[1]=edPnt[1]-stPnt[1];	pntVec[2]=edPnt[2]-stPnt[2];

				//-----------------------------------------------------------------------------------------
				//	subsub-step 2: incrementally check and update the feasible region in 1D
				for(j=0;j<i;j++) {
					_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,stackSize-1-j,abcd);
					stDot=abcd[0]*stPnt[0]+abcd[1]*stPnt[1]+abcd[2]*stPnt[2];
					edDot=abcd[0]*edPnt[0]+abcd[1]*edPnt[1]+abcd[2]*edPnt[2];

					if (fabs(edDot-stDot)<EPS) { // the segment stPnt-edPnt is parallel to the plane abcd[...]
						if (abcd[0]*stPnt[0]+abcd[1]*stPnt[1]+abcd[2]*stPnt[2]+abcd[3]<0)
							return false;	// An infeasible region as: the half-space does NOT contains the segment
						continue;			// A possible feasible region as: the half-space contains the segment
					}
					alpha=(-abcd[3]-stDot)/(edDot-stDot);

					sign=pntVec[0]*abcd[0]+pntVec[1]*abcd[1]+pntVec[2]*abcd[2];
					if (sign>=0.0f) 
						lowerBnd=MAX(lowerBnd,alpha);	// lowerBnd may need to be updated
					else
						upperBnd=MIN(upperBnd,alpha);	// upperBnd may need to be updated
					if (lowerBnd>=upperBnd) break;
				}

				//-----------------------------------------------------------------------------------------
				//	subsub-step 3: determine the feasible point in 2D
				if (lowerBnd>=upperBnd) return false;	// This is an infeasible region.
				alpha=(lowerBnd+upperBnd)*0.5f;
				px=stPnt[0]+pntVec[0]*alpha;	py=stPnt[1]+pntVec[1]*alpha;	pz=stPnt[2]+pntVec[2]*alpha;
			}
			else {
				return false;
			}
		}

		bFeasible=true;	
		for(i=infeasibleIndex+1;i<stackSize;i++) {
			_determineHalfSpaceByIndex(stackForTreeTraversal,trPtr,stackSize-1-i,abcd);
			if (abcd[0]*px+abcd[1]*py+abcd[2]*pz+abcd[3]<0) {infeasibleIndex=i; bFeasible=false; break;}
		}
	}

	feaPnt[0]=px;	feaPnt[1]=py;	feaPnt[2]=pz;
	return true;
}
