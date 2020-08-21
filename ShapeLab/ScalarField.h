#pragma once
#include "..\QMeshLib\PolygenMesh.h"

class ScalarField
{
public:
	ScalarField(QMeshPatch* mesh, bool support) { tetMesh = mesh; };
	~ScalarField() {};

	void compScalarField_initMesh();
	void compScalarField_supportStructure();

private:
	QMeshPatch* tetMesh;
	void _ScalarFieldNormalize_supportStructure(bool support, int initNodeNum);
};
