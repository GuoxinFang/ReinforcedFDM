#pragma once
#include "..\QMeshLib\PolygenMesh.h"

class QMeshPatch;
class PolygenMesh;
class GLKMatrix;

class meshOperator
{
public:
	meshOperator() {};
	~meshOperator() {};

	void compTetMeshVolumeMatrix(QMeshPatch* tetMesh);

};