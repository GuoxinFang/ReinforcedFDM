#pragma once
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\PolygenMesh.h"

class FabricationDirectionDetection
{
public:
	FabricationDirectionDetection(
		PolygenMesh* initMesh, PolygenMesh* platformMesh, PolygenMesh* isoSurfaceMeshSet);
	~FabricationDirectionDetection() {};

	void runFabDir2DCaseDetection(bool compute, double installedAngle);

	void convexHullGenerator(PolygenMesh* polygenMesh);

private:
	QMeshPatch* tetMesh;
	QMeshPatch* platform;
	PolygenMesh* isoSurfaceSet;
	PolygenMesh* platformMeshInstall; // for debug usage

	double offsetY = 10.0;

	void _initializeSystemIndex();

	double _compShadowVolume_OpenMP(int degree);
	bool _rotateLayerSetbyMatrixandDetectOrder(
		int degree, std::vector<Eigen::MatrixXd>& layerCoord, bool updateVisual);
	void _constructConvexHullfromPatchSet(Eigen::MatrixXd layerCoord, QMeshPatch* convexHull, bool isFirstLayer);
	void _convexHullGenerator(Eigen::MatrixXd& pointSet, QMeshPatch* convexHull);
	double _compShadowVolume(
		QMeshPatch* layer, QMeshPatch* convexHull, Eigen::MatrixXd& layerCoord);
	double _detectPntInsideConvexHullandMinDist(QMeshPatch* convexHull, double pnt[3]);

	long timeConvexHull = 0;

	//-----------------------------------------------------------------------
	//  Function when the rotation degree is determined, direct work on mesh.
	bool _finalizePrintingDirandRotate(int degree);
	void _rotateMeshandUpdateVectorField(double theta);
	void _computeLayerSetCenterandMove();
	bool _detectLayerOrderFlip();
};

