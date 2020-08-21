#include "stdafx.h"
#include "SORODeform.h"
#include <QCoreApplication>

#include "..\GLKLib\GLKObList.h"
#include "..\GLKLib\GLKGeometry.h"
#include "..\GLKLib\GLKMatrixLib.h"
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\QMeshNode.h"
#include "..\QMeshLib\QMeshTetra.h"

#include <omp.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define VELE 4

SORODeform::SORODeform()
{

}

SORODeform::~SORODeform()
{
	ClearAll();
}

void SORODeform::ClearAll()
{
	/*if (FactorizedA) { LinearSolver::DeleteF(FactorizedA); FactorizedA = 0; }
	if (VectorX) { delete VectorX; VectorX = 0; }
	if (VectorB) { delete VectorB; VectorB = 0; }
	if (matA) delete matA;
	if (LocalCoord) delete[] LocalCoord;
	if (LocalGoal) delete[] LocalGoal;
	if (InverseP) delete[] InverseP;*/
}

void SORODeform::Initialization()
{
	ClearAll();

	//set node and face number
	vertNum = 0;
	for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode *node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
		if (node->isFixed == false && node->isHandle == false)
			node->SetIndexNo(vertNum++);
		else node->SetIndexNo(-1);

		double pp[3] = { 0 };
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		node->SetCoord3D_last(pp[0], pp[1], pp[2]);
	}

	eleNum = 0;
	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		if (Tet->IsFixed()) Tet->SetIndexNo(-1);
		else Tet->SetIndexNo(eleNum++);
	}

	//initialize all computing matrix
	matA.resize(VELE * eleNum, vertNum);
	matAT.resize(vertNum, VELE * eleNum);

	LocalCoord.resize(eleNum);
	InverseP.resize(eleNum);
	LocalGoal.resize(eleNum);
	VectorXPosition.resize(3);
	VectorXPosition_Last.resize(3);
	VectorBSide.resize(3);

	for (int i = 0; i < 3; i++) {
		VectorXPosition[i] = Eigen::VectorXd::Zero(vertNum);
		VectorXPosition_Last[i] = Eigen::VectorXd::Zero(vertNum);
		VectorBSide[i] = Eigen::VectorXd::Zero(eleNum * VELE);
	}

	//precomputing
	//ComputeLocalGoalAndInverse();
}

void SORODeform::ComputeLocalGoalAndInverse()
{
	//For the deformation usage, every tetra should remain its initial shape
	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		int fdx = Tet->GetIndexNo();
		if (fdx < 0) continue;
		Tet->selected = false;
		for (int i = 0; i < VELE; i++) {
			QMeshNode* Tet_Node = Tet->GetNodeRecordPtr(i + 1);
			if (Tet_Node->selected == true) {
				Tet->selected = true;
				break;
			}
		}

		QMeshNode *nodes[VELE];
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, VELE);
		double center[3] = { 0 };

		for (int i = 0; i < VELE; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= VELE;
		for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		//This is for selected reigon size
		if (Tet->selected == true) {
			for (int i = 0; i < VELE; i++) {
				for (int j = 0; j < 3; j++) { P(j, i) *= pow(1.5, 1.0 / 3); }
			}
		}

		//solving pseudoInverse with GLKMATRIX Lib, Eigen may occur error here
		LocalGoal[fdx] = P;

		GLKMatrix GLKP(3, VELE), GLKInverseP(3, VELE);
		InverseP[fdx] = Eigen::MatrixXd::Zero(3, 4);

		for (int i = 0; i < 3; i++) { for (int j = 0; j < 4; j++)  GLKP(i, j) = P(i, j); }

		GLKMatrix TP(VELE, 3), GLKATA(3, 3);
		GLKMatrixLib::Transpose(GLKP, 3, VELE, TP);
		GLKMatrixLib::Mul(GLKP, TP, 3, VELE, 3, GLKATA);

		if (!GLKMatrixLib::Inverse(GLKATA, 3)) {
			printf("ERROR in finding Inverse!\n");
			getchar();
		}
		GLKMatrixLib::Mul(GLKATA, GLKP, 3, 3, VELE, GLKInverseP);

		for (int i = 0; i < 3; i++) { for (int j = 0; j < 4; j++) InverseP[fdx](i, j) = GLKInverseP(i, j); }
	}
	printf("Finish Compute local inverse!\n");
}

void SORODeform::FillMatrixA()
{
	//give memory to sparse matrix, to accerate the insert speed
	matA.reserve(VectorXi::Constant(VELE * eleNum, 1000));

	float c1 = -1.0 / VELE, c2 = 1 + c1;

	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->GetIndexNo() < 0) continue;

		int fdx = Tetra->GetIndexNo() * VELE;

		int vdxArr[VELE];
		for (int i = 0; i < VELE; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		for (int i = 0; i < VELE; i++) {
			if (vdxArr[i] < 0) continue;
			for (int j = 0; j < VELE; j++) {
				if (vdxArr[j] < 0) continue;
				if (i == j) matA.insert(fdx + j, vdxArr[i]) = c2;
				else matA.insert(fdx + j, vdxArr[i]) = c1;
			}
		}
	}

	matA.makeCompressed();
}

void SORODeform::FactorizeMatrixA()
{
	Eigen::SparseMatrix<double> matATA(vertNum, vertNum);

	matAT = matA.transpose();
	matATA = matAT*matA;

	Solver.compute(matATA);
	printf("end factorize materix A\n");
}

void SORODeform::FillVectorB()
{
	double c1 = -1.0 / VELE, c2 = 1 + c1;

	int Core = 12;
	int EachCore = eleNum / Core + 1;
	//int n = 0;

#pragma omp parallel   
	{
#pragma omp for  
		for (int omptime = 0; omptime < Core; omptime++) {
			//int BeginTetraIndex = EachCore * omptime + 1;
			//				if (TetraIndex > TetraNumSum) break;
			for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra *Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);

				if (Tetra->GetIndexNo() < omptime*EachCore) continue;
				else if (Tetra->GetIndexNo() >(1 + omptime)*EachCore) break;

				if (Tetra->GetIndexNo() != -1) {
					int fdx = Tetra->GetIndexNo();
					//double center[3]; face->CalCenterPos(center[0], center[1], center[2]);
					double center[3] = { 0 };
					for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) center[j] += LocalCoord[fdx](j, i);
					for (int i = 0; i < 3; i++) center[i] /= VELE;

					for (int i = 0; i < VELE; i++) {
						for (int j = 0; j < 3; j++) {
							VectorBSide[j](fdx*VELE + i) = LocalCoord[fdx](j, i) - center[j];
							//VectorBSide[j](fdx*VELE + i) = LocalCoord[fdx](j, i)* cc2 / c2;

							//if (abs(VectorBSide[j](fdx*VELE + i)) < 0.0001) VectorBSide[j](fdx*VELE + i) = 0.0;
							//cout << VectorBSide[j](fdx*VELE + i) <<endl;
						}
						for (int k = 0; k < VELE; k++) {
							QMeshNode *node = Tetra->GetNodeRecordPtr(k + 1);
							if (node->GetIndexNo() < 0) {
								//printf("This happen once\n\n");
								double p[3]; node->GetCoord3D(p[0], p[1], p[2]);
								if (i == k) for (int l = 0; l < 3; l++) VectorBSide[l](fdx *VELE + i) -= p[l] * c2;
								else for (int l = 0; l < 3; l++) VectorBSide[l](fdx *VELE + i) -= p[l] * c1;
							}
						}
					}
				}
			}
		}
	}
}

bool SORODeform::Solve()
{
#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < 3; i++) {
			Eigen::VectorXd ATB = matAT*VectorBSide[i];
			VectorXPosition[i] = Solver.solve(ATB);
		}
	}
	return true;
}

void SORODeform::LocalProjection()
{
	int Core = 12;
	int EachCore = eleNum / Core + 1;

#pragma omp parallel   
	{
#pragma omp for  
		for (int omptime = 0; omptime < Core; omptime++) {
			//int BeginTetraIndex = EachCore * omptime + 1;
			//				if (TetraIndex > TetraNumSum) break;
			for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra *Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);

				if (Tetra->GetIndexNo() < omptime*EachCore) continue;
				else if (Tetra->GetIndexNo() > (1 + omptime)*EachCore) break;

				int fdx = Tetra->GetIndexNo(); if (fdx < 0) continue;


				double center[3] = { 0 };
				Tetra->CalCenterPos(center[0], center[1], center[2]);

				//This tP should be put ahead to see if it works for the energy function.
				Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(VELE, 3); QMeshNode *nodes[VELE];
				for (int i = 0; i < VELE; i++) {
					nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
					nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
				}
				for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

				Eigen::Matrix3d T = Matrix3d::Zero(3, 3),
					T_transpose = Matrix3d::Zero(3, 3), R = Matrix3d::Zero(3, 3);
				T = InverseP[fdx] * tP;
				T_transpose = T.transpose();

				/////////////////Eigen SVD decomposition/////////////////////////

				JacobiSVD<Eigen::MatrixXd> svd(T_transpose, ComputeThinU | ComputeThinV);
				Matrix3d V = svd.matrixV(), U = svd.matrixU();
				R = U * V.transpose();

				LocalCoord[fdx] = R * LocalGoal[fdx];
			}
		}
	}
}

bool SORODeform::UpdateVertex()
{
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		int idx = node->GetIndexNo();
		if (idx > -1)
			node->SetCoord3D(VectorXPosition[0](idx), VectorXPosition[1](idx), VectorXPosition[2](idx));
	}
	return true;
}

void SORODeform::PreProcess()
{
	Initialization();
	MoveHandleRegion();
	ComputeLocalGoalAndInverse();

	FillMatrixA();
	FactorizeMatrixA();
	cout << "finish preprocess the system" << endl;
}

bool SORODeform::Run(int loop)
{
	for (int i = 0; i < loop; i++) {
		LocalProjection();
		FillVectorB();
		Solve();
		if (!UpdateVertex()) return false;
		//ShifttoInitialPos();
	}
	return true;
}

void SORODeform::ShifttoInitialPos() {
	double CoordLast[3] = { 0 }, CoordCurrent[3] = { 0 }, shift[3] = { 0 };
	int nodeNum = 0;
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		if (node->selected == true) continue;
		node->GetCoord3D_last(CoordLast[0], CoordLast[1], CoordLast[2]);
		node->GetCoord3D(CoordCurrent[0], CoordCurrent[1], CoordCurrent[2]);
		for (int i = 0; i < 3; i++) shift[i] += CoordCurrent[i] - CoordLast[i];
		nodeNum++;
	}
	for (int i = 0; i < 3; i++) shift[i] /= (double)nodeNum;
	//cout << nodeNum << " , " << shift[0] <<" , "<< shift[1] << " , " << shift[2] << endl;
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		node->GetCoord3D(CoordCurrent[0], CoordCurrent[1], CoordCurrent[2]);
		node->SetCoord3D(CoordCurrent[0] - shift[0], CoordCurrent[1] - shift[1], CoordCurrent[2] - shift[2]);
		node->CalNormal();
	}
}

void SORODeform::MoveHandleRegion() {
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		if (node->isHandle) {
			double pp[3];
			node->GetCoord3D_last(pp[0], pp[1], pp[2]);
			pp[1] += 1.0;
			node->SetCoord3D(pp[0], pp[1], pp[2]);
		}
	}
}