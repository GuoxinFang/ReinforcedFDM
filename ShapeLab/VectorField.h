#pragma once
#include "..\QMeshLib\PolygenMesh.h"

class VectorField
{
public:
	VectorField(QMeshPatch* mesh, bool support) { tetMesh = mesh; };
	~VectorField() {};
	QMeshPatch* tetMesh;

	void initMeshVectorFieldCompute();

private:
	int selectedTetraNum = 0;
	int compRegionNum = 0; int tensileRegionNum = 0;

	void _initializeIndex();
	void _criticalRegionVectorFieldInitialGuess();
	void _detectSmallCriticalRegionandClear();

	void _criticalRegionOrientationDetectionbyFlooding();
	void _smoothVectorFieldCriticalRegion();

	Eigen::SimplicialLDLT <Eigen::SparseMatrix<double>> vectorFieldSolver;
	void _optOrientationBetweenRegions();
	void _buildVectorFieldCompSystem(
		int NIETetNum, int NIEFaceNum, Eigen::SparseMatrix<double>& A);
	void _changeCriticalRegionFieldDir(Eigen::VectorXi& orderiter);
	double _iterFillNIEandComputeEnergy(
		int NIETetNum, int NIEFaceNum, Eigen::SparseMatrix<double>& A);



	void _fillNIERegionandSmooth(int iterTime, bool globalSmooth);

	void _detectNeighborTetrabyNode(std::vector< QMeshTetra* >& TetraSet, 
		QMeshTetra* Tetra, bool checkProcess, int ele_type, bool face_node);

	void _singleRegionOrientationEstimation(int type, int index);
	void _compVectorFieldwithLocalLaplacian(int eleType, QMeshTetra* Tetra);

};