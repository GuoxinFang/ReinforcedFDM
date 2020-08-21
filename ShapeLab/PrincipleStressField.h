#pragma once
#include "..\QMeshLib\PolygenMesh.h"

class PrincipleStressField
{
public:
	PrincipleStressField(QMeshPatch *mesh);
	~PrincipleStressField();

	QMeshPatch* tetMesh;

	void InputFEMResult(std::string filename);
	void ComputeElementPrincipleStress();
	void DetermineCriticalTensileandCompressRegion(double rangeT, double rangeC);

private:

	int selectedInfluenceNum;
	int compressEleNum, tensileEleNum;

	int compCeleNum; // compress element used in computation.
	int compTeleNum; // tensile element used in computation.

	void _selectTensileandCompressiveRegion(double rangeT, double rangeC);
};
