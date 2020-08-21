#include "stdafx.h"
#include "ScalarField.h"

using namespace std;
using namespace Eigen;

void ScalarField::compScalarField_initMesh() {

	// ---- method 1: apply laplacian to all the element (including constrained)

	Eigen::SparseMatrix<double> Parameter(3 * tetMesh->GetTetraNumber(), tetMesh->GetNodeNumber()); //A
	std::cout << 3 * tetMesh->GetTetraNumber() << "," << tetMesh->GetNodeNumber() << std::endl;

	Eigen::VectorXd guideField(tetMesh->GetNodeNumber()); //x

	Eigen::VectorXd b(3 * tetMesh->GetTetraNumber()); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		double weight = 1.0;

		if (Tetra->isTensileorCompressSelect && Tetra->sigma_max > 0) weight = 3.0;
		if (Tetra->isSupportBottomRegion)  weight = 10.0;
		if (Tetra->tetSupportElement == false) weight = 10.0;
		if (Tetra->collisionTetra) weight = 5.0;

		//if (Tetra->isTensileorCompressSelect) weight = 3.0;

		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 4; i++) {
				QMeshNode* Node = Tetra->GetNodeRecordPtr(i + 1);
				ParaTriplet.push_back(Eigen::Triplet<double>(
					Tetra->GetIndexNo() * 3 + j, Node->GetIndexNo(), -Tetra->VolumeMatrix(i, j) * weight)); // infill A
			}
		}

		for (int i = 0; i < 3; i++) b(3*Tetra->GetIndexNo() + i) = Tetra->vectorField(i) * weight; // infill B
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(tetMesh->GetNodeNumber(), tetMesh->GetNodeNumber());
	ATA = Parameter.transpose() * Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	Solver.compute(ATA);

	Eigen::VectorXd ATb(tetMesh->GetNodeNumber());
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);

	Eigen::VectorXd guideFieldNormalize(tetMesh->GetNodeNumber());
	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->scalarField = guideFieldNormalize(Node->GetIndexNo());
		Node->scalarField_init = guideField(Node->GetIndexNo());
	}
}

void ScalarField::compScalarField_supportStructure() {

	int index = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->tetSupportNode == false) Node->supportIndex = -1;
		else { Node->supportIndex = index; index++; }
	}
	int supportNodeNum = index;

	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		/*	if (Tetra->GetNodeRecordPtr(1)->supportIndex < 0 && Tetra->GetNodeRecordPtr(2)->supportIndex < 0 &&
				Tetra->GetNodeRecordPtr(3)->supportIndex < 0 && Tetra->GetNodeRecordPtr(4)->supportIndex < 0)*/
		if (Tetra->tetSupportElement == false) 	Tetra->supportIndex = -1;
		else { Tetra->supportIndex = index; index++; }
	}

	int supportEleNum = index;

	std::cout << " -- support element number = " << supportEleNum << ", "
		" support node number = " << supportNodeNum << std::endl;

	Eigen::SparseMatrix<double> Parameter(3 * supportEleNum, supportNodeNum); //A

	Eigen::VectorXd guideField(supportNodeNum); //x

	Eigen::VectorXd b(3 * supportEleNum); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (Tetra->supportIndex < 0) continue;

		double weight = 1.0;

		//bottom region should be preserved, yoga 3D model is not sutible for this!
		// if (Tetra->isSupportBottomRegion) weight = 5.0;

		if (Tetra->initialModelBoundyTet) weight = 3.0;


		for (int i = 0; i < 3; i++) b(Tetra->supportIndex * 3 + i) = Tetra->vectorField(i) * weight; // infill B

		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 4; i++) {

				QMeshNode* Node = Tetra->GetNodeRecordPtr(i + 1);
				int nodeIndex = Node->supportIndex;
				if (nodeIndex >= 0)
					ParaTriplet.push_back(Eigen::Triplet<double>(
						Tetra->supportIndex * 3 + j, nodeIndex, -Tetra->VolumeMatrix(i, j) * weight)); // infill A
				else {
					b(Tetra->supportIndex * 3 + j) -= Node->guideFieldValue * (-Tetra->VolumeMatrix(i, j)) * weight; //constrain
				}
			}
		}
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(supportNodeNum, supportNodeNum);
	ATA = Parameter.transpose() * Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	Solver.compute(ATA);

	Eigen::VectorXd ATb(supportNodeNum);
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb); // this vector is only for support region

	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->supportIndex < 0) continue;
		else Node->guideFieldValue = guideField(Node->supportIndex);
	}

	this->_ScalarFieldNormalize_supportStructure(true, tetMesh->GetNodeNumber() - supportNodeNum);

	std::cout << " -- finish compute scalar field by hard constrain method" << std::endl;

}

void ScalarField::_ScalarFieldNormalize_supportStructure(bool support, int initNodeNum) {

	Eigen::VectorXd guideField(tetMesh->GetNodeNumber());
	Eigen::VectorXd guideFieldNormalize(tetMesh->GetNodeNumber());

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		guideField(Node->GetIndexNo()) = Node->guideFieldValue;
	}

	// compute max and min phis 
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	//for (int i = 0; i < tetMesh->GetNodeNumber(); i++) {
	for (int i = 0; i < initNodeNum; i++) {

		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);

	}

	double range = maxPhi - minPhi;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->guideFieldValue = guideFieldNormalize(Node->GetIndexNo());
	}

	printf("finish normalize scalar field, notice the number for support region is out of [0,1] range \n");
}
