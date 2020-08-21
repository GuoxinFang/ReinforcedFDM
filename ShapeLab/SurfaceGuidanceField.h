#pragma once
#include "..\QMeshLib\PolygenMesh.h"

class SurfaceGuidanceField
{
public:
	SurfaceGuidanceField() { };
	~SurfaceGuidanceField() { };

	void runIsoLayerVectorFielCompute(QMeshPatch* isoSurface, int layerIndex);

private:
	bool _initializeIsoSurface(QMeshPatch* isoSurface);
	bool _SurfaceVectorFieldOrientationDetection(QMeshPatch* isoSurface);
	void _projectVectortoSurface(QMeshFace* Face);

	void _surfaceVectorFieldFulfill(QMeshPatch* isoSurface, int iter, int type);
	void _surfaceVectorFieldFillNIERegion(QMeshPatch* isoSurface);

};
