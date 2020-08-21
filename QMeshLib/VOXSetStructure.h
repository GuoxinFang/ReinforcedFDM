#ifndef _CCL_VOXSET_STRUCT
#define _CCL_VOXSET_STRUCT

#define UINT	unsigned int
#define UCHAR	unsigned char

/*Only solid voxel will be installed here, however the position is not installed*/
typedef struct voxelSetNode {
	UINT posIndex[3];			//	position of a voxel
	unsigned short layerIndex;	//	curved layer index of a voxel
	unsigned short stressFieldLayerIndex; //	layer index generate from stress field
	UCHAR materialIndex;		//	material index of a voxel
	void *attachedPtr;
}VOXELSETNode;

/*The number of VoxelArray is the full rank number!*/
typedef struct VOXEL {
	bool isFill = false;
	bool isLayerDraw = false;
	//unsigned short vindex;
	double voxelPos[3]; //this install the center of single voxel
	int nodeArrayIndex = -1;
	//bool inside; //this voxel is inside the tetrahedral mesh
	double fieldValue = 0.0;
}VOXEL;

typedef struct voxelSet {
	int nodeNum;			VOXELSETNode* nodeArray;
	VOXEL* voxelArray;
	float origin[3], width;	// origin of voxel-set and dimension of each voxel	
	UINT m_res[3];			// Resolution of voxel set in different directions
}VOXELSET;

#endif