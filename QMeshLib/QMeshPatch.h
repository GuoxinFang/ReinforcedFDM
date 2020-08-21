// QMeshPatch.h: interface for the QMeshPatch class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _QMESHPATCH
#define _QMESHPATCH

#include "../GLKLib/GLKObList.h"
#include "QMeshTetra.h"
#include "QMeshFace.h"
#include "QMeshEdge.h"
#include "QMeshNode.h"

class QMeshTetra;
class QMeshFace;
class QMeshEdge;
class QMeshNode;


class QMeshPatch : public GLKObject
{
public:
	QMeshPatch();
	virtual ~QMeshPatch();

public:
	void ClearAll();
	void initializeListIndex();

	bool GetAttribFlag(const int whichBit);
	void SetAttribFlag(const int whichBit, const bool toBe = true);

	int GetIndexNo();		//from 1 to n
	void SetIndexNo(const int _index = 1);

	bool drawDeformationField = false;
	bool drawStressField = false;
	bool drawgeoField = false;
	bool drawPrincipleStressField = false;
	bool drawThisIsoLayer = false;
	double minStressValue = 0, maxStressValue = 0;
	double minPrincipleStressValue = 0, maxPrincipleStressValue = 0;
	double drawValue[2] = { 0 };

	int GetTetraNumber();
	QMeshTetra* GetTetraRecordPtr(int No);	//from 1 to n
	GLKObList& GetTetraList();

	int GetFaceNumber();
	QMeshFace* GetFaceRecordPtr(int No);	//from 1 to n
	GLKObList& GetFaceList();

	int GetEdgeNumber();
	QMeshEdge* GetEdgeRecordPtr(int No);	//from 1 to n
	GLKObList& GetEdgeList();

	int GetNodeNumber();
	QMeshNode* GetNodeRecordPtr(int No);	//from 1 to n
	GLKObList& GetNodeList();

	bool inputOBJFile(char* filename, bool bOBTFile = false);
	bool inputOFFFile(char* filename, bool bOBTFile = false);

	bool inputTETFile(char* filename, bool bOBTFile = false);
	bool inputMFile(char* filename);
	bool inputPLY2File(char* filename);
	bool inputOFFFile(char* filename);

	void outputOBJFile(char* filename, bool bOBTFile = false);
	void outputTrglOBJFile(char* filename);

	void InverseOrientation();
	void ComputeBoundingBox(double &xmin, double &ymin, double &zmin, double &xmax, double &ymax, double &zmax);
	void ComputeBoundingBox(double boundingBox[]);

	void constructionFromVerFaceTable(int nodeNum, float *nodeTable, int faceNum, unsigned int* faceTable);

	GLKObList& GetAttrib_EdgeList();

	QMeshPatch *CopyMesh();
	void deleteTetra(QMeshTetra *tetra);
	void deleteFace(QMeshFace *face);

	//for volume mesh
	bool isVolume = false;
	bool spaceComp = false;

	//--SIGGRAPH: for iso surface generation
	double isoSurfaceValue;

	bool drawScalarFieldGradient = false;
	bool drawVectorField = false;

	bool includeSupportRegion = false;

	bool isSupportConvexHull = false;
	//--SIGGRAPH: for tool-path generation
	//bool drawgeoField = false;
	int isoNum;
	bool isToolPathPatch = false;
	bool connectWithBoundary = false;
	bool toolPathDraw;

	bool sparseVectorDraw = false;

	bool isSupport = false;

	bool isSmallPatch = false;
	bool stressFieldToolpathControl;
	std::string layerName;

	bool drawLayerThickness = false;

	int boundNodeNum;
	int splitIndex;

	int layerToolpathType;

	bool isoSurfaceGuideFieldComputed = false;
	bool checkCollision = false;

	bool isInnerSlcingLayer = false;

private:
	int indexno;			// start from 1 to n  

	bool flags[8];			// bit 0 -- TRUE for displaying the valence on nodes
							//			FALSE for NOT displaying the valence on nodes
							// bit 1 -- TRUE for displaying the tensile energy on edges
							//			FALSE for NOT displaying the tensile energy on edges
							// bit 2 -- TRUE for the 2D pattern has been determined
							//			FALSE for the 2D pattern has NOT been determined

	GLKObList tetraList;	// a list of mesh's tetras (QMeshTetra)
	GLKObList faceList;		// a list of mesh's faces (QMeshFace)
	GLKObList edgeList;		// a list of mesh's edges (QMeshEdge)
	GLKObList nodeList;		// a list of mesh's nodes (QMeshNode)

	GLKObList Attrib_EdgeList;
};

#endif
