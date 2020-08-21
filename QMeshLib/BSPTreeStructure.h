#ifndef _CCL_BSPTREE_STRUCT
#define _CCL_BSPTREE_STRUCT

#define UINT	unsigned int

#define THREADS_PER_BLOCK			256
#define BLOCKS_PER_GRID				32

typedef struct bspTreeArrayNode {
	bspTreeArrayNode(void) { uLeftChildID = 0; uRightChildID = 0; planeID = 0; };
	bool IsLeafNode() { return (uLeftChildID == 0 && uRightChildID == 0); };
	bool IsSolidNode() { return (uLeftChildID == 0 && uRightChildID == 0 && planeID != 0); };
	bool IsEmptyNode() { return (uLeftChildID == 0 && uRightChildID == 0 && planeID == 0); };

	void MakeEmptyNode() { uLeftChildID = 0; uRightChildID = 0; planeID = 0; };
	void MakeSolidNode() { uLeftChildID = 0; uRightChildID = 0; planeID = 1; };
	// For an empty leaf-node: (uLeftChildID==uRightChildID==0) and (planeID==0)
	// For a solid leaf-node: (uLeftChildID==uRightChildID==0) && (planeID!=0)

	UINT planeID;
	UINT uLeftChildID, uRightChildID;
	//	bool bSolid;	// true - this is a solid cell and its left/right child must be NULL
	// false - if (uLeftChildID==0 && uRightChildID==0), this is an empty cell; otherwise, this is a fuzzy cell.
}BSPTREEArrayNode;

#define CUDA_IS_LEAFNODE(nodePtr)		((nodePtr)->uLeftChildID==0 && (nodePtr)->uRightChildID==0)
#define CUDA_IS_SOLIDNODE(nodePtr)		((nodePtr)->uLeftChildID==0 && (nodePtr)->uRightChildID==0 && (nodePtr)->planeID!=0)
#define CUDA_IS_EMPTYNODE(nodePtr)		((nodePtr)->uLeftChildID==0 && (nodePtr)->uRightChildID==0 && (nodePtr)->planeID==0)

typedef struct bspTreeArrayPlane {
	float abcd[4];
}BSPTREEArrayPlane;

typedef struct bspTree {
	BSPTREEArrayPlane* planeArray;		UINT planeArraySize;
	BSPTREEArrayNode* nodeArray;		UINT nodeArraySize;
}BSPTREE;

typedef struct bspTreeTravelStackNode {
	UINT uTreeNodeID;
	bool bIsOnLeftOrRightChild;
}BSPTREETravelStackNode;

typedef struct TrglForBSP {
	float verPnts[3][3];
	UINT planeIndex;
}TRGLForBSP;

#endif
