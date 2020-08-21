#pragma once
#include "../Library/Eigen/Eigen"
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\PolygenMesh.h"

class QMeshPatch;
class PolygenMesh;

typedef struct qHullSet {
	int faceNum;	double *normalVec;	double *offset;
	int vertNum;	double *vertPos;
	unsigned int *faceTable;	//	Note that: the index starts from '1'
}QHULLSET;

class FabricationProcess {

public:
	FabricationProcess(PolygenMesh *printingLayer, PolygenMesh *platform, PolygenMesh *tetModel){
		isoSurfaceSet = printingLayer; 
		printingPlatform = platform;
		tetrahedralModel = tetModel;
	};
	~FabricationProcess() { };

	bool systemBuild = false;

	/* -- 3D rotation function 2020-04-27 -- */
	double Yshift = 30.0; // 30  is used for Yoga model
	//double Yshift = 10.0; // 30  is used for Yoga model

	void runBestFabricationDirectionDetection(bool compute, double _theta, double _beta, bool inverse); // 3D
	void runBestFabricationDirectionDetection(bool compute, int rotateDegree); // 2D

private:
	void _initializeSystem_RotationSystem();
	bool _rotateLayerSetbyMatrix(
		Eigen::Matrix3d& rotationMatrix, bool updateVisual, std::vector<Eigen::MatrixXd>& layerCoord);
	QHULLSET* _constructNewConvexFront(
		QMeshPatch* curveLayer, QHULLSET* currentConvexFront, PolygenMesh* platformMesh, Eigen::MatrixXd& layerCoord);
	
	double _compShadowVolume(QMeshPatch* Layer, QHULLSET* currentConvexFront, Eigen::MatrixXd& layerCoord);

	/* -- 3D rotation function for 2.5D slicing before 2020-04-27 -- */
	QHULLSET* _constructNewConvexFront(
		QMeshPatch* curveLayer, QHULLSET* currentConvexFront, PolygenMesh* platformMesh);
	double compShadowVolume(QMeshPatch* Layer, QHULLSET* currentConvexFront);
	void initializeSystem_RotationSystem();


public:





	bool runFabricationCollisionDetection(QMeshPatch* printNozzle);
	bool checkSingleNodeCollision_withinItsLayer(QMeshPatch* printNozzle, QMeshPatch* layer, QMeshNode* checkNode);

	//checkSingleBelow = false means only check single layer for support point
	void runSupportRegionCompute(bool checkSingleBelow);
	void computeSupportNode(PolygenMesh *supportRegion);

	void computeSupportingConvexHull(PolygenMesh *supportRegion);
	void outputConvexHullandTetrahedralMesh(PolygenMesh *supportRegion);

	QHULLSET* computeNozzleConvexHull(QMeshPatch* nozzleMesh);

	void support_tet_link_topology(QMeshPatch* initTet, QMeshPatch* supportTet);
	void support_surface_delete_nonimportant_region(
		PolygenMesh *supportRegion, PolygenMesh *supportIsoSurface);
	QMeshPatch* supportPatch;

	double supportNodeBoxSize = 1.5;

	PolygenMesh* isoSurfaceSet;

private:
	PolygenMesh* tetrahedralModel;
	PolygenMesh* printingPlatform;

	int CPUCoreNum = 12;
	double supportNodeGap = 5;

	bool rotateLayerandCenter_checkOrder(int degree);
	bool intersetTriangle(Eigen::Vector3d& orig, Eigen::Vector3d& dir, 
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& insertP);
	bool intersetTriangle_supportStructure(Eigen::Vector3d& orig, Eigen::Vector3d& endNode,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2);

	static QHULLSET* _mallocMemoryConvexHull(int faceNum, int vertNum);
	
	void _freeMemoryConvexHull(QHULLSET *&pConvexHull);
	bool _isPntInsideConvexHull(QHULLSET *pConvexHull, double pnt[]);
	double _compDistanceToConvexFront(double pos[], QHULLSET *pConvexHull);
	void _drawConvexHull(bool order);
	void _drawConvexHull(QHULLSET *currentConvexFront, PolygenMesh *supportRegion);

	void buildSupportingNode(
		QMeshPatch *supportStructure, Eigen::Vector3d& supportNode, Eigen::Vector3d& orig, QMeshNode* connectedNode);

	bool compareVector(Eigen::Vector4i& a, Eigen::Vector4i& b);

	double minZBox, maxZBox;
};