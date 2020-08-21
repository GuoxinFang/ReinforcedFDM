#pragma once
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\PolygenMesh.h"

class isoSurface
{

public:
	isoSurface(QMeshPatch* Patch);
	~isoSurface();
	
	void generateIsoSurface(PolygenMesh* isoSurface, int layerNum);
	void generateIsoSurface_fromVoxelLayer(PolygenMesh* isoSurface, int layerNum);
	void generateIsoSurface_supportStructure(PolygenMesh* isoSurface, int layerNum);

	void splitSupportandModelSurface(PolygenMesh* isoSurface_support);

	bool checkIsoSurfaceDistance(PolygenMesh* isoSurface);
	bool checkIsoSurfaceDistanceAndBuildNewLayer_supportStructure(PolygenMesh* isoSurface);

	void computeLayerThicknessForNodeAndDraw(PolygenMesh* isoSurface, bool isSupportLayer);

	void deleteCloseIsoSurface(PolygenMesh* isoSurface);
	void deleteCloseIsoSurface_supportLayer(PolygenMesh* isoSurface);

	void smoothingIsoSurface(PolygenMesh* isoSurface);

	void planeCutSurfaceMesh(QMeshPatch* surfaceMesh, QMeshFace* cutPlane);
	bool planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh, QMeshFace* cutPlane);

	double maxLayerThickness;
	double minLayerThickness;

private:
	QMeshPatch* tetMesh; // mesh used for slicing
	QMeshPatch* generatesingleIsoSurface(
		double isoValue, PolygenMesh* isoSurface, bool support);
	bool checkTwoLayerDistance(QMeshPatch* bLayer, QMeshPatch* tLayer);
	bool checkTwoLayerDistance_supportStructureLayerSubstraction(QMeshPatch* bLayer, QMeshPatch* tLayer);

	void deleteNodeinPatch(QMeshNode* thisNode, QMeshPatch* layer);
	void reconstructSurface(QMeshPatch* layer);

	void splitSingleLayer(QMeshPatch* initLayer, QMeshPatch* splitLayer, bool isSupport);

};