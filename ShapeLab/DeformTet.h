#pragma once
#include "..\eigen3\Eigen\Eigen"

class QMeshPatch;
class GLKMatrix;

/////////////////////////Guoxin Fang/////////////////////////
//The DeformTet is writen for topology optimization project//
/////////////////////////////////////////////////////////////

class DeformTet
{
public:
	DeformTet();
	~DeformTet();

	void PreProcess();
	bool RunIteration(int loop = 1);

	//Read mesh from platform
	void SetMesh(QMeshPatch *mesh) { TetMesh = mesh; }
	void moveHandleRegion();

	double proposalYoungs = 1.0;
	double proposalPossion = 1.0;


private:

	void ClearAll();
	void Initialization();
	void FillMatrixA();
	void FactorizeMatrixA();
	void FillVectorB();
	bool Solve();
	void LocalProjection();
	bool UpdateVertex();
	void ShifttoInitialPos();
	void ComputeLocalGoalAndInverse();

	int vertNum, eleNum;

	//Define all matrix used in computings
	Eigen::SparseMatrix<double> matA;
	Eigen::SparseMatrix<double> matAT;
	Eigen::SimplicialLDLT <Eigen::SparseMatrix<double>> Solver;
	//SparseQR <Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> Solver;
	//SparseLU<SparseMatrix<double>> Solver;
	std::vector<Eigen::MatrixXd> LocalCoord, InverseP, LocalGoal;
	std::vector<Eigen::VectorXd> VectorXPosition, VectorBSide, VectorXPosition_Last;

private:
	QMeshPatch *TetMesh;
};

