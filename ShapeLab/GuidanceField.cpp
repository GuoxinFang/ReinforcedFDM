#include "stdafx.h"
#include "GuidanceField.h"
#include <omp.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "GLKGeometry.h"

using namespace std;
using namespace Eigen;

#define TENSILE 0
#define COMPRESS 1

GuidanceField::GuidanceField(QMeshPatch *mesh) { tetMesh = mesh; }

GuidanceField::~GuidanceField() { ClearALL(); }

void GuidanceField::ClearALL() {}

void GuidanceField::runFieldComputing() {

	this->initialIndex();
	std::cout << "-- Initialize index" << std::endl;

	//this->compVectorField();
	this->compVectorFieldwithLocalLaplacian();
	std::cout << "-- Step 1: Finish compute the vecotr field." << std::endl << std::endl;

	this->compGuideField();
	std::cout << "-- Step 2: Finish compute the scalar field." << std::endl << std::endl;

	//this->visualGuideFieldGradient();
	std::cout << "-- Visualize gradient" << std::endl;

}

void GuidanceField::runFieldComputing_TenslieandCompressRegion() {

	this->initialIndex();
	std::cout << "-- Initialize index" << std::endl;

	this->VectorField_initialGuess_Tensile_Compress_Computing();
	this->clear_small_neighbor_region();
	this->compress_region_flooding_compute_orientation();
	this->smooth_compress_and_tensile_region_vector_field();



	this->_optimizeCriticalRegionVectorDir();

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (!Tetra->isTensileorCompressSelect)
			Tetra->vectorField = Eigen::Vector3d::Zero();
	}

	tetMesh->drawVectorField = true;

}

void GuidanceField::_optimizeCriticalRegionVectorDir() {

	int NIETetNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (!Tetra->isTensileorCompressSelect) {
			Tetra->vectorCompIndex = NIETetNum; NIETetNum++;
		}
	}

	int NIEFaceNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;
		else if (Face->GetLeftTetra()->isTensileorCompressSelect && 
			Face->GetRightTetra()->isTensileorCompressSelect) continue;
		
		NIEFaceNum++;
	}

	long time = clock();
	Eigen::SparseMatrix<double> A; // A matrix for vector field computing
	this->_buildVectorFieldCompSystem(NIETetNum, NIEFaceNum, A);
	printf(" TIMER -- Build the vector field compute system takes %ld ms.\n", clock() - time);

	time = clock();

	this->_fillBandCompute(NIETetNum, NIEFaceNum, A);
	double initEnergy = this->_compVectorLaplacianEnergyValue();
	std::cout << "init Energy = " << initEnergy << std::endl;

	int critRegNum = compRegionNum + tensileRegionNum;
	Eigen::VectorXd energyIter(critRegNum - 1);

	int maxIter = 10;
	for (int i = 0; i < maxIter; i++) {
		for (int iterTime = 0; iterTime < critRegNum - 1; iterTime++) {
			this->_changeRegionFieldDir(iterTime);
			this->_fillBandCompute(NIETetNum, NIEFaceNum, A);
			energyIter(iterTime) = this->_compVectorLaplacianEnergyValue();
			this->_changeRegionFieldDir(iterTime);
		}
		std::cout << " This iter, init energy = " << initEnergy 
			<< ", updated energy = " << energyIter << std::endl;

		int minIter;
		if (energyIter.minCoeff(&minIter) >= initEnergy) {
			std::cout << "find best direction! " << std::endl;  break;
		}
		else {
			this->_changeRegionFieldDir(minIter);
			this->_fillBandCompute(NIETetNum, NIEFaceNum, A);
			initEnergy = this->_compVectorLaplacianEnergyValue();
		}
	}

	printf(" TIMER -- find optimized field takes %ld ms.\n", clock() - time);

}

void GuidanceField::_buildVectorFieldCompSystem(
	int NIETetNum, int NIEFaceNum, Eigen::SparseMatrix<double>& A) {

	// First build the system of vector field optimization.

	//A
	A.resize(NIEFaceNum, NIETetNum);

	std::vector<Eigen::Triplet<double>> vFieldParaTriplet;

	std::cout << "the size of A is " << NIEFaceNum << " * " << NIETetNum << std::endl;

	int rowIndex = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);

		QMeshTetra* leftTet = Face->GetLeftTetra();
		QMeshTetra* rightTet = Face->GetRightTetra();

		if (leftTet == NULL || rightTet == NULL) continue;
		else if (leftTet->isTensileorCompressSelect && rightTet->isTensileorCompressSelect) continue;

		double area = Face->CalArea();
		if (leftTet->isTensileorCompressSelect == false)
			vFieldParaTriplet.push_back(Eigen::Triplet<double>(rowIndex, leftTet->vectorCompIndex, area)); // infill A

		if (rightTet->isTensileorCompressSelect == false)
			vFieldParaTriplet.push_back(Eigen::Triplet<double>(rowIndex, rightTet->vectorCompIndex, -area)); // infill A

		rowIndex++;
	}

	//---- Solve the linear system
	A.setFromTriplets(vFieldParaTriplet.begin(), vFieldParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(NIETetNum, NIETetNum);
	ATA = A.transpose() * A;

	long time = clock();

	vectorFieldSolver.compute(ATA);

	printf(" TIMER -- solve ATA takes %ld ms.\n", clock() - time);

}

void GuidanceField::_fillBandCompute(
	int NIETetNum, int NIEFaceNum, Eigen::SparseMatrix<double>& A) {

	//b
	std::vector<Eigen::VectorXd> b(3);
	for (int i = 0; i < 3; i++) b[i] = Eigen::VectorXd::Zero(NIEFaceNum);
	//x
	std::vector<Eigen::VectorXd> vectorField(3);
	for (int i = 0; i < 3; i++) vectorField[i] = Eigen::VectorXd::Zero(NIETetNum);

	int rowIndex = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);

		QMeshTetra* leftTet = Face->GetLeftTetra();
		QMeshTetra* rightTet = Face->GetRightTetra();

		if (leftTet == NULL || rightTet == NULL) continue;
		else if (leftTet->isTensileorCompressSelect && rightTet->isTensileorCompressSelect) continue;

		double area = Face->CalArea();
		if (leftTet->isTensileorCompressSelect)
		    for (int i = 0; i < 3; i++) b[i](rowIndex) -= area * leftTet->vectorField(i); // otherwise infill B

		if (rightTet->isTensileorCompressSelect)
			for (int i = 0; i < 3; i++) b[i](rowIndex) += area * rightTet->vectorField(i); // otherwise infill B	
		
		rowIndex++;
	}


	for (int i = 0; i < 3; i++) {
		Eigen::VectorXd ATb(NIETetNum);
		ATb = A.transpose() * b[i];
		vectorField[i] = vectorFieldSolver.solve(ATb);
	}

	//---- output the computed vector field and visulization
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect) continue;
		for (int i = 0; i < 3; i++)
			Tetra->vectorField(i) = vectorField[i](Tetra->vectorCompIndex);
		Tetra->vectorField = Tetra->vectorField.normalized();
	}

}

void GuidanceField::_changeRegionFieldDir(int regionIndex) {

	int detectRegion = 0;
	if (regionIndex < tensileRegionNum - 1) detectRegion = regionIndex + 2;
	else detectRegion = -(regionIndex - tensileRegionNum + 2);

	// std::cout << detectRegion << std::endl;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (Tetra->floodingRegionIndex == regionIndex)
			Tetra->vectorField = -Tetra->vectorField;
	}
}

void GuidanceField::runSupportTetFieldCompute() {

	// the index of tetMesh is already initialized
	// element flag - tetSupportElement, node flag - tetSupportNode

	// -- first give basement element with normal direction
	this->supportTetFieldCompute_set_bottom_constrain();

	// -- expand the connect tetrahedral with initial region
	//this->expandConnectRegion_withFieldConstrain();

	// -- use laplacian to fill the rest region - well protect the initial vector field
	this->supportTetFieldCompute_fill_NIE_by_laplacian(100);

	// -- compute scalar field
	//this->compGuideField();

	this->supportTetFieldCompute_compute_GuideField_support_by_hard_constrain();
	
	//this->supportTetFieldCompute_compute_GuideField_support_by_soft_constrain();

	//this->compGuideField();

	this->visualGuideFieldGradient(false);

}

void GuidanceField::runIsoLayerFielCompute(QMeshPatch* isoSurface, int layerIndex) {

	//only vector field will be computed on this stage.
	
	if (this->initialize_isoSurface(isoSurface) == false) {
		isoSurface->stressFieldToolpathControl = false; 

		Vector3d vectorDir = Eigen::Vector3d::Zero();
		if (layerIndex % 2 == 0) vectorDir << 1.0, 0.0, 1.0;
		else vectorDir << 1.0, 0.0, -1.0;
		vectorDir = vectorDir.normalized();

		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			Face->principleStressDir = vectorDir;
			Eigen::Vector3d faceNorm; double D;
			Face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
			faceNorm = faceNorm.normalized();
			double dotProduct = Face->principleStressDir.dot(faceNorm);
			Eigen::Vector3d planeNorm = Face->principleStressDir - dotProduct * faceNorm;
			Face->principleStressDir = planeNorm.normalized();
		}		
	}

	else {
		//isosurface contains control vector region
		isoSurface->stressFieldToolpathControl = true;
		this->fieldSmooth_isoSurface(isoSurface);
		this->fieldFulfillLaplacian_isoSurface(isoSurface);
		//this->scalarFieldCompute_isoSurface(isoSurface);		
	}

}


/* Guoxin: 04-26-2020 method -- result to an under-determined linear system*/
void GuidanceField::runFieldComputing_optVectorField_oneShoot() {

	/* ---- Initialize ---- */
	int innerFaceNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		if (Face->GetRightTetra() == NULL || Face->GetLeftTetra() == NULL) continue;
		else innerFaceNum++;
	}

	int criticalTetNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tet->vectorField = Eigen::Vector3d::Zero();
		if (Tet->isTensileorCompressSelect == true) criticalTetNum++;
	}

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tet->isTensileorCompressSelect == false) continue;
		for (int i = 0; i < 4; i++) {
			if (Tet->GetFaceRecordPtr(i + 1)->GetRightTetra() == NULL || Tet->GetFaceRecordPtr(i + 1)->GetLeftTetra() == NULL)
				continue;
		}
		Tet->selectedHandleTet = true; break;
	}
	int eqTetNum = 0; int selectedHandleTetNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tet->selectedHandleTet == false) { Tet->SetIndexNo(eqTetNum); eqTetNum++; }
		else { 
			Tet->SetIndexNo(-1); 
			selectedHandleTetNum++; 

			// -- Give initial value: 
			Eigen::Vector3d stressNorm = Tet->tau_max.normalized();
			Eigen::Vector3d tensile_Vector; tensile_Vector << 1, -stressNorm(0) / stressNorm(1), 0;
			Tet->vectorField = tensile_Vector.normalized();

		}
	}

	std::cout << "-- Initialize the system finished! " << std::endl;

	/* ---- build linear system ---- */

	int Arow = innerFaceNum + criticalTetNum - selectedHandleTetNum;
	int Acolumn = 3 * (tetMesh->GetTetraNumber() - selectedHandleTetNum);

	Eigen::SparseMatrix<double> Parameter(Arow, Acolumn); //A

	std::cout << " A matrix size = " << Arow << "," << Acolumn << std::endl;

	Eigen::VectorXd VectorField(Acolumn); //x
	Eigen::VectorXd b(Arow); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	std::cout << "-- Build the equation system finished! " << std::endl;

	/* ---- face equation ---- */

	int faceEqIndex = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);

		if (Face->GetRightTetra() == NULL || Face->GetLeftTetra() == NULL) continue;

		double weight = Face->CalArea();
		//std::cout << "FACE - " << weight << std::endl;

		int leftTetIndex = Face->GetLeftTetra()->GetIndexNo();
		int rightTetIndex = Face->GetRightTetra()->GetIndexNo();
		
		// ---- A*(v_left - v_right) = 0, infill A
		for (int i = 0; i < 3; i++) {
			if(leftTetIndex >= 0)
				ParaTriplet.push_back(Eigen::Triplet<double>(faceEqIndex, 3 * leftTetIndex + i, weight));
			if (rightTetIndex >= 0)
				ParaTriplet.push_back(Eigen::Triplet<double>(faceEqIndex, 3 * rightTetIndex + i, -weight));
		}

		// ---- if handle tetrahedral detected, fulfill the constrain in b
		if (leftTetIndex < 0) {
			for (int i = 0; i < 3; i++) b(faceEqIndex) -= weight* Face->GetLeftTetra()->vectorField(i);
		}
		if (rightTetIndex < 0) {
			for (int i = 0; i < 3; i++) b(faceEqIndex) -= (-weight) * Face->GetRightTetra()->vectorField(i);
		}

		faceEqIndex++;
	}

	/* ---- selected critical tetrahedral equation ---- */
	int tetEqIndex = faceEqIndex;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (Tet->isTensileorCompressSelect == false || Tet->selectedHandleTet == true) continue;

		double weight = Tet->CalVolume() * 100;
		//std::cout << " TET -- " << weight << std::endl;

		// ---- v_e * tau_max = 0, infill A
		for (int i = 0; i < 3; i++)
			ParaTriplet.push_back(Eigen::Triplet<double>(tetEqIndex, 3 * Tet->GetIndexNo() + i, weight * Tet->tau_max(i)));

		tetEqIndex++;
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	std::cout << "-- fill matrix finished! " << std::endl;

	Eigen::SparseMatrix<double> ATA(Acolumn, Acolumn);
	ATA = Parameter.transpose() * Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	//Eigen::SimplicialLDLT <Eigen::SparseMatrix<double>> Solver;
	//Solver.factorize(Parameter);
	Solver.compute(ATA);
	if (Solver.info() != Success)  // decomposition failed 
	{
		printf("compute parameter failed!\n"); return;
	}
	
	VectorField = Solver.solve(b);
	if (Solver.info() != Success)  // decomposition failed 
	{
		printf("compute b failed!\n"); return;
	}

	

	Solver.factorize(ATA);

	Eigen::VectorXd ATb(Acolumn);
	ATb = Parameter.transpose() * b;
	VectorField = Solver.solve(ATb);

	std::cout << "-- solve linear system finished! " << std::endl;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tet->GetIndexNo() >= 0) {
			for (int i = 0; i < 3; i++) Tet->vectorField(i) = VectorField(Tet->GetIndexNo() * 3 + i);
		}
		Tet->vectorField = Tet->vectorField.normalized();
	}
	std::cout << "-- FINISHED!!!!! Vector field is given! " << std::endl;

}

/* Guoxin: 04-27-2020 method */
void GuidanceField::runFieldComputing_optVectorField() {

	/* Initialize system and build initial value to critical region*/
	this->initialIndex();
	this->VectorField_initialGuess_Tensile_Compress_Computing();

	/* Segment to detect differnet critical region */
	this->vectorDirRange = 0.75;
	this->_vectorField_segmentCriticalRegion();
	this->_vectorField_processSegementRegion();

	tetMesh->drawVectorField = true;
}

void GuidanceField::_vectorField_segmentCriticalRegion() {

	/* flooding to detect region */
	int compRegionIndex = 0; int tensileRegionIndex = 0;
	bool stopIter; 
	do {
		compRegionIndex++; this->_region_detection_by_flooding(COMPRESS, -compRegionIndex);
	} while (_regionFloodingStop(COMPRESS) == false);

	do {
		tensileRegionIndex++; this->_region_detection_by_flooding(TENSILE, tensileRegionIndex);
	} while (_regionFloodingStop(TENSILE) == false);

	/* delete small region and update index */

	//delete critical region with element number less than minEleNum
	int flitedTensileRegionIndex = 0; 	int flitedCompressRegionIndex = 0;

	// -- TENSILE
	for (int i = 0; i < tensileRegionIndex; i++) {
		int regionTetNum = 0;
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->floodingRegionIndex == i + 1) regionTetNum++;
		}

		if (regionTetNum < minEleNum) {
			for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
				if (Tetra->floodingRegionIndex == i + 1) {
					Tetra->isTensileorCompressSelect = false; Tetra->floodingRegionIndex = 0;
					Tetra->vectorField = Eigen::Vector3d::Zero();
				}
			}
			std::cout << " ## the tensile region No." << i + 1 << " is small, DELETE!" << std::endl;
		}
		else {
			for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
				if (Tetra->floodingRegionIndex == i + 1)  Tetra->floodingRegionIndex = flitedTensileRegionIndex + 1;				
			}
			std::cout << " ## the tensile region No." << i + 1 << 
				" is large, updated index = " << flitedTensileRegionIndex + 1 << std::endl;
			flitedTensileRegionIndex++;
		}
	}

	// -- COMPRESS
	for (int i = 0; i < compRegionIndex; i++) {
		int regionTetNum = 0;
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->floodingRegionIndex == -(i + 1)) regionTetNum++;
		}

		if (regionTetNum < minEleNum) {
			for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
				if (Tetra->floodingRegionIndex == -(i + 1)) {
					Tetra->isTensileorCompressSelect = false; Tetra->floodingRegionIndex = 0;
					Tetra->vectorField = Eigen::Vector3d::Zero();
				}
			}
			std::cout << " ## the compress region No." << i + 1 << "is small, DELETE!" << std::endl;
		}
		else {
			for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
				if (Tetra->floodingRegionIndex == -(i + 1))  
					Tetra->floodingRegionIndex = -(flitedCompressRegionIndex + 1);
			}
			std::cout << " ## the compress region No." << -(i + 1) <<
				"is large, updated index = " << -(flitedCompressRegionIndex + 1) << std::endl;;


			flitedCompressRegionIndex++;
		}
	}
	
	std::cout << " -- #### final Compress region number = " << flitedCompressRegionIndex << std::endl;
	std::cout << " -- #### final Tensile region number = " << flitedTensileRegionIndex << std::endl;

	this->compRegionNum = flitedCompressRegionIndex;
	this->tensileRegionNum = flitedTensileRegionIndex;

}

void GuidanceField::_vectorField_processSegementRegion() {

	for (int i = 0; i < this->tensileRegionNum; i++) this->_smoothRegionVector(i + 1);
	for (int i = 0; i < this->compRegionNum; i++) this->_smoothRegionVector(-i - 1);

}

void GuidanceField::_smoothRegionVector(int regionIndex) {

	Eigen::Vector3d initDir = Eigen::Vector3d::Zero();

	int regionTetNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->floodingRegionIndex == regionIndex) {
			initDir = Tetra->vectorField; regionTetNum++;
		}	
	}
	/*if (regionTetNum < 50 ) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->floodingRegionIndex == regionIndex) {
				Tetra->isTensileorCompressSelect = false;
				Tetra->floodingRegionIndex = 0;
				Tetra->vectorField = Eigen::Vector3d::Zero();
			}
		}
		std::cout << "Index = " << regionIndex << " region is too small, tetrahedral number = " << regionTetNum << std::endl;
		return;
	}*/

	// build right direction
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->floodingRegionIndex == regionIndex) {
			if (Tetra->vectorField.dot(initDir) < 0) Tetra->vectorField = -Tetra->vectorField;
		}
	}

	// laplacian smoothness
	for (int lapIter = 0; lapIter < 3; lapIter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->floodingRegionIndex == regionIndex) {

				Eigen::Vector3d averageField = Eigen::Vector3d::Zero();
				averageField += Tetra->vectorField;
				for (int i = 0; i < 4; i++) {
					QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
					for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
						QMeshTetra* ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);
						if (ConnectTetra == Tetra || ConnectTetra->floodingRegionIndex != regionIndex) continue;
						averageField += ConnectTetra->vectorField;
					}
				}
				Tetra->vectorField = averageField.normalized();
			}
		}
	}

	std::cout << "Index = " << regionIndex << ", tetrahedral number = " << regionTetNum << std::endl;
}

void GuidanceField::_region_detection_by_flooding(int type, int index) {

	std::vector< QMeshTetra* > TetraSet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (Tetra->isTensileorCompressSelect == false || Tetra->flooding_Processed) continue;

		else if (type == COMPRESS && Tetra->sigma_max >= 0) continue;
		else if (type == TENSILE && Tetra->sigma_max < 0) continue;

		// initial tetra used in flooding
		TetraSet.push_back(Tetra); 
		Tetra->flooding_Processed = true; Tetra->floodingRegionIndex = index;

		//find fist ring neighbor
		this->_detectNeiborTetbyNode_vectorConstrain(TetraSet, Tetra, type);
		
		break;
	}

	std::cout << " initialize tetraset number = " << TetraSet.size()
		<< " ---- Region INDEX = " << index << " Begin flooding ... " <<  std::endl;

	int setNum;
	do {
		setNum = TetraSet.size();

		std::cout << "... Before ... " << TetraSet.size();

		/* #### This is important, used to forbid loop in one side #### Guoxin 2020-03-30 */
		std::vector< QMeshTetra* > TetraSetThisLoop(TetraSet.size());
		for (int i = 0; i < TetraSet.size(); i++) TetraSetThisLoop[i] = TetraSet[i];

		for (int i = 0; i < TetraSetThisLoop.size(); i++) 
			this->_detectNeiborTetbyNode_vectorConstrain(TetraSet, TetraSetThisLoop[i], type);

		for (int i = 0; i < TetraSet.size(); i++) {
			TetraSet[i]->floodingRegionIndex = index; // -- set region index		
		}

		std::cout<< " After ... " << TetraSet.size() << " ||  ";

	} while (setNum != TetraSet.size());

	std::cout << std::endl << " -- finish one single region orientation detection -- " << std::endl << std::endl;
}

bool GuidanceField::_regionFloodingStop(int type) {

	bool stopIter = true;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (type == COMPRESS) {
			if (Tetra->isTensileorCompressSelect && Tetra->sigma_max < 0 && Tetra->flooding_Processed == false) {
				stopIter = false; break;
			}
		}
		else if (type == TENSILE) {
			if (Tetra->isTensileorCompressSelect && Tetra->sigma_max >= 0.0 && Tetra->flooding_Processed == false) {
				stopIter = false; break;
			}
		}		
	}

	return stopIter;
}

void GuidanceField::_detectNeiborTetbyNode_vectorConstrain(
	std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, int ele_type) {

	/* Detect node-based one-ring neighbor, meanwhile should satisfied with vector normal constrain */
	for (int i = 0; i < 4; i++) {
		QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
		for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);

			if (ConnectTetra == Tetra || ConnectTetra->flooding_Processed || ConnectTetra->isTensileorCompressSelect == false) continue;
			
			if (ConnectTetra->sigma_max >= 0 && ele_type == COMPRESS) continue;
			if (ConnectTetra->sigma_max < 0 && ele_type == TENSILE) continue;

			// detect if this tetra is already inside the tetraset
			bool exist_in_set = false;
			for (int j = 0; j < TetraSet.size(); j++) {
				if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) { exist_in_set = true; break; }
			}
			if (exist_in_set) continue;

			// detect if the normal direction have large difference
			if (fabs(ConnectTetra->vectorField.dot(Tetra->vectorField)) < vectorDirRange) continue;

			//the connected tetra has not being processed before
			TetraSet.push_back(ConnectTetra);
			ConnectTetra->flooding_Processed = true;
		}
	}
}


void GuidanceField::runFieldComputing_DeleteRegion() {
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetNodeRecordPtr(i + 1)->selected) Tetra->isTensileorCompressSelect = false;
		}
		if (!Tetra->isTensileorCompressSelect)
			Tetra->vectorField = Eigen::Vector3d::Zero();
	}
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->selected = false;
	}
}

void GuidanceField::runFieldComputing_computeScalarField() {

	/* Optimization of region direction, automatically make field compatable*/

	//this->_fieldRegionDirOptimization();
	long time = clock();
	this->fill_vector_field_and_smooth(200, true);
	printf("-------------------------------------------------------------\n");
	printf("computing vector field taking %ld ms.\n", clock() - time);

	std::cout << "Energy value = " << this->_compVectorLaplacianEnergyValue() << std::endl;
	std::cout << "-- Step 2: Finish compute the vecotr field." << std::endl << std::endl;

	time = clock();
	this->compGuideField();
	printf("-------------------------------------------------------------\n");
	printf(" computing guide field taking %ld ms.\n", clock() - time);

	//this->visualGuideFieldGradient();

}

void GuidanceField::_fieldRegionDirOptimization() {

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->vectorField_initial = Tetra->vectorField;
	}

	std::cout << " ---- Begin region dir optimization, tensile Region # = " << tensileRegionNum <<
		", compress Region # = " << compRegionNum << std::endl << std::endl;

	/*Keep the first tensile region*/
	std::vector<bool> regionDir(tensileRegionNum + compRegionNum);



}

double GuidanceField::_compVectorLaplacianEnergyValue() {

	double lapEnergy = 0.0;

	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

		/*lapEnergy += Face->CalArea() *
			pow(1 - Face->GetLeftTetra()->vectorField.dot(Face->GetRightTetra()->vectorField), 3);*/

		lapEnergy += Face->CalArea() * 
			pow((Face->GetLeftTetra()->vectorField - Face->GetRightTetra()->vectorField).norm(), 4);

	}



	//for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
	//	QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
	//	//if (Tetra->isTensileorCompressSelect) continue;

	//	QMeshTetra* neighTetra;
	//	for (int i = 0; i < 4; i++) {
	//		QMeshFace* Face = Tetra->GetFaceRecordPtr(i + 1);
	//		if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

	//		if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
	//		else neighTetra = Face->GetLeftTetra();

	//		//Eigen::Vector3d vectorDif = Tetra->vectorField - neighTetra->vectorField;
	//		lapEnergy += 1 - Tetra->vectorField.dot(neighTetra->vectorField);
	//		//lapEnergy += vectorDif.norm();
	//	}
	//}
	return lapEnergy;
}

















void GuidanceField::runFieldComputing_FlipSelectedRegion() {

	QMeshTetra * startTetra = NULL;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (startTetra != NULL) break;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetNodeRecordPtr(i + 1)->selected && Tetra->isTensileorCompressSelect) {
				startTetra = Tetra; break;
			}
		}
	}

	std::vector< QMeshTetra* > TetraSet;
	TetraSet.push_back(startTetra);
	int type;
	if (startTetra->sigma_max > 0) type = TENSILE;
	else type = COMPRESS;

	int setNum;
	do {
		setNum = TetraSet.size();

		for (int i = 0; i < TetraSet.size(); i++) {
			this->detect_Neighbor_Tetrahedral_by_Node(TetraSet, TetraSet[i], false, type, false);}

	} while (setNum != TetraSet.size());

	for (int i = 0; i < TetraSet.size(); i++) {
		TetraSet[i]->vectorField = -TetraSet[i]->vectorField;	
	}

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->selected = false;
	}
}

void GuidanceField::runFieldComputing_VoxelGuide() {
	this->initialIndex();
	/*Compute the vector field based on existing scalar field gradient*/
	this->visualGuideFieldGradient(false);

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->vectorField = Tetra->scalarFieldGradient;
	}

	for (int iter = 0; iter < 50; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			int neighborCount = 0;

			QMeshTetra *neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element
			for (int i = 0; i < 4; i++) {
				QMeshFace * Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField;

				neighborCount++;
			}

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;
			//std::cout << neighTetra->GetIndexNo() << endl;

			Tetra->vectorField = averageField.normalized();

		}
	}

	this->compGuideField();

}

void GuidanceField::supportTetFieldCompute_set_bottom_constrain() {

	// select the bottom node
	double minY = 9999.999;
	double alpha = 2.0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp); if (pp[1] < minY) minY = pp[1];
	}
	std::cout << "the minY = " << minY << endl;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp);
		if (pp[1] < minY + alpha) Node->isSupportBottom = true;
	}

	//select the bottom element and set vector field as constrain
	Eigen::Vector3d averDir = Vector3d::Zero();
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		averDir += Tetra->vectorField;
	}

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->tetSupportElement == false) Tetra->isTensileorCompressSelect = true;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetNodeRecordPtr(i + 1)->isSupportBottom) {

				if (averDir.normalized()(1) > 0)
					Tetra->vectorField << 0.0, 1.0, 0.0;
				else Tetra->vectorField << 0.0, -1.0, 0.0;

				Tetra->isTensileorCompressSelect = true;
				Tetra->isSupportBottomRegion = true;
				break;
			}
		}
	}

	std::cout << std::endl <<  "Support Tet mesh field compute" <<std::endl<<
		" -- Set bottom tetrahedral vector field constrain finished!" << std::endl;
}

void GuidanceField::expandConnectRegion_withFieldConstrain() {
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		
		if (Tetra->tetSupportElement == true) continue;
		for (int i = 0; i < 4; i++) {
			QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
			if (thisFace->GetLeftTetra() == NULL || thisFace->GetRightTetra() == NULL) {
				std::cerr << "ERROR, the inner (original model) tet's face should not belongs to boundary." << std::endl;
				continue;
			}
			
			QMeshTetra* neighborTet = thisFace->GetLeftTetra(); 
			if (neighborTet == Tetra) neighborTet = thisFace->GetRightTetra();

			if (neighborTet->tetSupportElement == false) continue;
			else {
				neighborTet->vectorField = Tetra->vectorField;
				Tetra->isTensileorCompressSelect = true;
				neighborTet->initialModelBoundyTet = true;
			}
		}	
	}
}

void GuidanceField::supportTetFieldCompute_fill_NIE_by_laplacian(int itertime) {

	// Laplacian by tetrahedral face neighborhood 

	for (int iter = 0; iter < itertime; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			if (Tetra->tetSupportElement == false) continue;
			if (Tetra->isTensileorCompressSelect) continue;

			int neighborCount = 0;

			QMeshTetra *neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();
			averageField += Tetra->vectorField;

			//find neighbor tetrahedral element - by face
			for (int i = 0; i < 4; i++) {
				QMeshFace * Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField;
				neighborCount++;
			}
			Tetra->vectorField = averageField.normalized();
			/*std::vector< QMeshTetra* > TetraSet;
			TetraSet.push_back(Tetra);
			for (int i = 0; i < 4; i++) {
				QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
				for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
					QMeshTetra* ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);
					bool exist_in_set = false;
					for (int j = 0; j < TetraSet.size(); j++) {
						if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
							exist_in_set = true; break;
						}
					}
					if (exist_in_set) continue;
					else TetraSet.push_back(ConnectTetra);
				}
			}
					
			for (int j = 0; j < TetraSet.size(); j++) {
				Tetra->vectorField += TetraSet[j]->vectorField;
			}
			Tetra->vectorField = Tetra->vectorField.normalized();*/
			//std::cout << TetraSet.size() << "," << Tetra->vectorField << std::endl;

		}
		std::cout << " -- Finish compute vector field, ITER = " << iter <<  std::endl;
	}
}

void GuidanceField::initialIndex() {

	//----initial the edge and face index, start from 0

	int index = 0;
	for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index);
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index);
		index++;
	}

	//----build the index for tetrahedral element
	index = 0;
	int selectTensileTetraNum = 0;
	int selectCompressTetraNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect) {
			//Tensile region, only the selected will be add index
			if (Tetra->sigma_max > 0) {
				//Tetra->SetIndexNo(selectTensileTetraNum);
				selectTensileTetraNum++;
			}
			//Compress region, only the selected will be add index
			else {
				//Tetra->SetIndexNo(selectCompressTetraNum);
				selectCompressTetraNum++;
			}

			/*for (int i = 0; i < 4; i++) {
			Tetra->GetNodeRecordPtr(i + 1)->tetraSelect = true;
			}*/
		}

		Tetra->SetIndexNo(index); index++;
	}
	selectedTetraNum = selectTensileTetraNum + selectCompressTetraNum;

	nNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		//if (Node->isFixed == true && Node->tetraSelect == false) {
		//	Node->guideFieldValue = 0;
		//	Node->SetIndexNo(-1); continue;
		//}
		Node->SetIndexNo(nNum);
		nNum++;
	}
}

void GuidanceField::compVectorFieldwithLocalLaplacian() {

	//-------------------------------------
	//Step 1: compute the inital guess for tensile and compressed region
	this->VectorField_initialGuess_Tensile_Compress_Computing();

	//-------------------------------------
	//Step 2: smooth the initial guess for both tensile and compressed region

	/*--Stress region find orientation by flooding algorithm--*/

	// ---- Delete all the tetra element (both tensile and compress region) that not have enough neighbor ---- //
	this->clear_small_neighbor_region();

	//compressed region flooding (Note: based on our algorithm the tensile region should always not have normal issue)
	this->compress_region_flooding_compute_orientation();

	//laplacian smoothness for only tensile and compress region
	this->smooth_compress_and_tensile_region_vector_field();

	//-------------------------------------
	//Step 3: fill the rest unimportant region with laplacian

	//-------------------------------------
	//Step 4: smooth the whole vector field (normally should not be use!)

	this->fill_vector_field_and_smooth(200, true);

	tetMesh->drawVectorField = true;
}


bool GuidanceField::initialize_isoSurface(QMeshPatch* isoSurface) {

	//project the principle stress normal
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->principleStressDir.norm() < 0.01) Face->isNIEface = true;
		else {
			Eigen::Vector3d faceNorm; double D;
			Face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
			faceNorm = faceNorm.normalized();
			double dotProduct = Face->principleStressDir.dot(faceNorm);
			Eigen::Vector3d planeNorm = Face->principleStressDir - dotProduct * faceNorm;
			//Face->principleStressDir = planeNorm.normalized().cross(faceNorm);
			Face->principleStressDir = planeNorm.normalized();
		}
	}

	Eigen::Vector3d planeStressDir = Eigen::Vector3d::Zero();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->isNIEface == false) {
			planeStressDir = Face->principleStressDir; break;
		}
	}

	if (planeStressDir.norm() < 0.01) return false;
	else return true;
}

bool GuidanceField::fieldSmooth_isoSurface(QMeshPatch* isoSurface) {

	Eigen::Vector3d planeStressDir = Eigen::Vector3d::Zero();
	planeStressDir << 0.0, 1.0, 0.0; //-- yoga model?
	//planeStressDir << 1.0, 0.0, 1.0; //-- bridge model?
	//planeStressDir << 1.0, 0.0, 0.0; //-- suitable for topopt model!
	//planeStressDir << 1.0, 0.0, 1.0; //-- CSquare Model, after collisiondetection

	planeStressDir = planeStressDir.normalized();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->isNIEface) continue;
		if (Face->principleStressDir.dot(planeStressDir) < 0) {
			Face->principleStressDir = -Face->principleStressDir;
		}
	}

	return true;
}

void GuidanceField::fieldFulfillLaplacian_isoSurface(QMeshPatch* isoSurface) {

	fieldFulfillLaplacian_isoSurface(isoSurface, 15, 1); // smooth important region
	fieldFulfillLaplacian_isoSurface(isoSurface, 15, 2); // fullfill NIE region

	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		Eigen::Vector3d faceNorm; double D;
		Face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
		faceNorm = faceNorm.normalized();
		double dotProduct = Face->principleStressDir.dot(faceNorm);
		Eigen::Vector3d planeNorm = Face->principleStressDir - dotProduct * faceNorm;
		Face->principleStressDir = planeNorm.normalized();
	}

	fieldFulfillLaplacian_isoSurface(isoSurface, 10, 0); // smooth all region
	//fieldFulfillLaplacian_isoSurface(isoSurface, 100, 0); // csquare region

	//fulfill the rest region
	Eigen::Vector3d averageDir = Eigen::Vector3d::Zero();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		averageDir += Face->principleStressDir;
	}
	averageDir = averageDir.normalized();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->principleStressDir.norm() > 0.01) continue;
		else {
			Face->principleStressDir = averageDir;
			Eigen::Vector3d faceNorm; double D;
			Face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
			faceNorm = faceNorm.normalized();
			double dotProduct = Face->principleStressDir.dot(faceNorm);
			Eigen::Vector3d planeNorm = Face->principleStressDir - dotProduct * faceNorm;
			Face->principleStressDir = planeNorm.normalized();
		}
	}
}

void GuidanceField::fieldFulfillLaplacian_isoSurface(QMeshPatch* isoSurface, int iter, int type) {

	for (int iterTime = 0; iterTime < iter; iterTime++) {
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			Vector3d averageField = Face->principleStressDir;

			if (type == 1 && Face->isNIEface == true) continue;
			if (type == 2 && Face->isNIEface == false) continue;

			for (int i = 0; i < 3; i++) {
				QMeshEdge* Edge = Face->GetEdgeRecordPtr(i + 1);
				if (Edge->IsBoundaryEdge()) continue;
				else {
					if (Face == Edge->GetLeftFace()) averageField += Edge->GetRightFace()->principleStressDir;
					else  averageField += Edge->GetLeftFace()->principleStressDir;
				}
			}
			Face->principleStressDir = averageField.normalized();

		}
	}
}

void GuidanceField::scalarFieldCompute_isoSurface(QMeshPatch* isoSurface) {

	int index = 0;
	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index); index++;
	}

	// ---- method 1: apply laplacian to all the element (including constrained)

	Eigen::SparseMatrix<double> Parameter(3 * isoSurface->GetFaceNumber(), isoSurface->GetNodeNumber()); //A

	Eigen::VectorXd guideField(isoSurface->GetNodeNumber()); //x

	Eigen::VectorXd b(3 * isoSurface->GetFaceNumber()); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	index = 0;
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index); index++;
		double weight = 1.0;
		if (Face->isNIEface == false) weight = 3.0;
		double faceAera = Face->CalArea();

		Eigen::Matrix3d faceMatrixPara, faceNodeCoord;
		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetCoord3D(faceNodeCoord(i, 0), faceNodeCoord(i, 1), faceNodeCoord(i, 2));
		}
		faceMatrixPara.row(0) = (faceNodeCoord.row(2) - faceNodeCoord.row(1)) / (2 * faceAera);
		faceMatrixPara.row(1) = (faceNodeCoord.row(0) - faceNodeCoord.row(2)) / (2 * faceAera);
		faceMatrixPara.row(2) = (faceNodeCoord.row(1) - faceNodeCoord.row(0)) / (2 * faceAera);

		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 3; i++) {
				QMeshNode* Node = Face->GetNodeRecordPtr(i);
				ParaTriplet.push_back(Eigen::Triplet<double>(
					Face->GetIndexNo() * 3 + j, Node->GetIndexNo(), faceMatrixPara(i, j) * weight)); // infill A
			}
		}

		for (int i = 0; i < 3; i++) b(Face->GetIndexNo() * 3 + i) = Face->principleStressDir(i) * weight; // infill B
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(isoSurface->GetNodeNumber(), isoSurface->GetNodeNumber());
	ATA = Parameter.transpose() * Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	Solver.compute(ATA);

	Eigen::VectorXd ATb(isoSurface->GetNodeNumber());
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);

	Eigen::VectorXd guideFieldNormalize(isoSurface->GetNodeNumber());
	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < isoSurface->GetNodeNumber(); i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i < isoSurface->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
		//Node->guideFieldValue = guideField(Node->GetIndexNo());
		Node->guideFieldValue = guideFieldNormalize(Node->GetIndexNo());
		Node->zigzagValue = guideFieldNormalize(Node->GetIndexNo());

	}
	isoSurface->isoSurfaceGuideFieldComputed = true;
}

void GuidanceField::scalarFieldCompute_supportSurface(QMeshPatch* layer_support) {

	int layerIndex = stoi(layer_support->layerName.substr(0, layer_support->layerName.length() - 3));

	Eigen::Vector3d planeStressDir = Eigen::Vector3d::Zero();
	if (layerIndex % 2 == 0) planeStressDir << 1.0, 0.0, 1.0;
	else planeStressDir << 1.0, 0.0, -1.0;
	planeStressDir = planeStressDir.normalized();

	for (GLKPOSITION Pos = layer_support->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer_support->GetFaceList().GetNext(Pos);
		Face->principleStressDir = planeStressDir;

		Eigen::Vector3d faceNorm; double D;
		Face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
		faceNorm = faceNorm.normalized();
		double dotProduct = Face->principleStressDir.dot(faceNorm);
		Eigen::Vector3d planeNorm = Face->principleStressDir - dotProduct * faceNorm;
		Face->principleStressDir = planeNorm.normalized();

	}

	this->scalarFieldCompute_isoSurface(layer_support);
}


//ele_type = 0 for TENSIER, ele_type = 1 for COMPRESS
void GuidanceField::detect_Neighbor_Tetrahedral_by_Node(
	std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, bool checkProcess, int ele_type, bool face_node){
	
	//using node to find neighbor tetrahedral 
	if (face_node == false) {
		for (int i = 0; i < 4; i++) {
			QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
			for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra *ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);

				if (ConnectTetra == Tetra) continue;
				if (ConnectTetra->flooding_Processed && checkProcess) continue;

				if (!ConnectTetra->isTensileorCompressSelect) continue;

				if (ConnectTetra->sigma_max >= 0 && ele_type == COMPRESS) continue;
				if (ConnectTetra->sigma_max < 0 && ele_type == TENSILE) continue;


				bool exist_in_set = false;

				for (int j = 0; j < TetraSet.size(); j++) {
					if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
						exist_in_set = true; break;
					}
				}
				if (exist_in_set) continue;

				//the connected tetra has not being processed before
				TetraSet.push_back(ConnectTetra);
				//if (checkProcess) ConnectTetra->flooding_Processed = true;
			}
		}
	}
	

	//using face to find neighbor tetrahedral 
	else {
		for (int i = 0; i < 4; i++) {
			QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);

			if (thisFace->GetLeftTetra() == NULL || thisFace->GetRightTetra() == NULL) continue;

			QMeshTetra *ConnectTetra = thisFace->GetRightTetra();
			if (Tetra == ConnectTetra) ConnectTetra = thisFace->GetLeftTetra();

			if (ConnectTetra == Tetra) continue;

			if (!ConnectTetra->isTensileorCompressSelect) continue;

			if (ConnectTetra->sigma_max >= 0 && ele_type == COMPRESS) continue;
			if (ConnectTetra->sigma_max < 0 && ele_type == TENSILE) continue;

			if (ConnectTetra->flooding_Processed && checkProcess) continue;

			bool exist_in_set = false;

			for (int j = 0; j < TetraSet.size(); j++) {
				if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
					exist_in_set = true; break;
				}
			}
			if (exist_in_set) continue;

			//the connected tetra has not being processed before
			TetraSet.push_back(ConnectTetra);
			if (checkProcess) ConnectTetra->flooding_Processed = true;

		}
	}
	
}

// Compress (index > 0) / Tensile (index < 0)
void GuidanceField::orientation_Estimation_by_flooding(int type, int index) {
	
	std::vector< QMeshTetra* > TetraSet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (!Tetra->isTensileorCompressSelect || Tetra->flooding_Processed) continue;
		else if (type == COMPRESS && Tetra->sigma_max >= 0) continue;
		else if (type == TENSILE && Tetra->sigma_max < 0) continue;
		
		// initial tetra used in flooding
		TetraSet.push_back(Tetra); 
		Tetra->flooding_Processed = true;

		//find fist ring neighbor
		this->detect_Neighbor_Tetrahedral_by_Node(TetraSet, Tetra, true, type, false);

		for (int i = 0; i < TetraSet.size(); i++) {		
			if (Tetra->vectorField.dot(TetraSet[i]->vectorField) < 0)
				TetraSet[i]->vectorField = -TetraSet[i]->vectorField;
		}
		break;
	}

	std::cout << " (initialize) current tetraset number = " << TetraSet.size() 
		<< " ---- Region INDEX = "<< index << std::endl;
	
	int setNum;
	do {
		setNum = TetraSet.size();

		/*Eigen::Vector3d averageRegionField = Eigen::Vector3d::Zero();
		for (int j = 0; j < TetraSet.size(); j++) averageRegionField += TetraSet[j]->vectorField / TetraSet.size();
		averageRegionField = averageRegionField.normalized();

		std::cout << averageRegionField << std::endl << "--------------" << std::endl;*/

		std::cout << "Before tracking neighbor, tetraset size = " << TetraSet.size();

		/* #### This is important, used to forbid loop in one side #### Guoxin 2020-03-30 */
		std::vector< QMeshTetra* > TetraSetThisLoop(TetraSet.size());
		for (int i = 0; i < TetraSet.size(); i++) TetraSetThisLoop[i] = TetraSet[i];
		

		for (int i = 0; i < TetraSetThisLoop.size(); i++) {
			this->detect_Neighbor_Tetrahedral_by_Node(TetraSet, TetraSetThisLoop[i], true, type, false );
		}
		std::cout << ", After is = " << TetraSet.size() << std::endl;

		for (int i = 0; i < TetraSet.size(); i++) {

			if (TetraSet[i]->flooding_Processed == true) continue;

			//detect neighbor normal
			std::vector< QMeshTetra* > thisTetraNeighborSet;
			this->detect_Neighbor_Tetrahedral_by_Node(thisTetraNeighborSet, TetraSet[i], false, type, false);
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();
			for (int j = 0; j < thisTetraNeighborSet.size(); j++) {
				if (thisTetraNeighborSet[j]->flooding_Processed == true) {
					averageField += thisTetraNeighborSet[j]->vectorField;
				}
			}
			averageField = averageField.normalized();
		
			TetraSet[i]->flooding_Processed = true; // -- set processed flag
			TetraSet[i]->floodingRegionIndex = index; // -- set region index

			if (averageField.dot(TetraSet[i]->vectorField) < 0)
				TetraSet[i]->vectorField = -TetraSet[i]->vectorField;
		}


	} while (setNum != TetraSet.size());

	std::cout << " -- finish one single region orientation detection -- " << std::endl << std::endl;
}

void GuidanceField::compress_region_flooding_compute_orientation() {

	bool stopIter; int iter = 0;
	do {
		stopIter = true; iter++;
		this->orientation_Estimation_by_flooding(COMPRESS, -iter);

		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			// Tetra belongs to compressed region and not being processed
			if (Tetra->isTensileorCompressSelect && Tetra->sigma_max < 0 && !Tetra->flooding_Processed) {
				stopIter = false; break;
			}
		}
	} while (!stopIter);
	std::cout << "Compress region number = " << iter << std::endl;
	this->compRegionNum = iter;

	stopIter; iter = 0;
	do {
		stopIter = true; iter++;
		this->orientation_Estimation_by_flooding(TENSILE, iter);

		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			// Tetra belongs to compressed region and not being processed
			if (Tetra->isTensileorCompressSelect && Tetra->sigma_max > 0 && !Tetra->flooding_Processed) {
				stopIter = false; break;
			}
		}
	} while (!stopIter);
	std::cout << "Tensile region number = " << iter << std::endl;
	this->tensileRegionNum = iter;

}

void GuidanceField::smooth_compress_and_tensile_region_vector_field() {
	for (int iter = 0; iter < 10; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			if (Tetra->isTensileorCompressSelect) {
				if (Tetra->sigma_max > 0)
					this->compVectorFieldwithLocalLaplacian_neighborTetLaplacian(TENSILE, Tetra);
				else this->compVectorFieldwithLocalLaplacian_neighborTetLaplacian(COMPRESS, Tetra);
			}
		}
	}
}

void GuidanceField::fill_vector_field_and_smooth(int iterTime, bool globalSmooth) {

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect) continue;
		else Tetra->vectorField = Eigen::Vector3d::Zero();
	}

	for (int iter = 0; iter < iterTime; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->isTensileorCompressSelect) continue;

			int neighborCount = 0;

			QMeshTetra *neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element
			for (int i = 0; i < 4; i++) {
				QMeshFace * Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField;

				//std::cout << neighTetra->GetIndexNo() << endl;
				neighborCount++;
			}

			//if (neighborCount == 0) std::cout << "ERROR, neighborhood number = 0" << std::endl;

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;
			Tetra->vectorField = averageField.normalized();

			/*for (int i = 0; i < 3; i++) {
				if (Tetra->vectorField(i) != Tetra->vectorField(i)) {
					std::cout << "NAN!!!, iter " << iter << std::endl;
					std::cout << averageField << std::endl;
				}
			}*/
		}

	}

	if (globalSmooth == false) return;

	//---- smooth the whole vector field (normally should not be use!)

	//for 2D model

	//for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
	//	QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
	//	Tetra->vectorField(2) = 0;
	//	Tetra->vectorField = Tetra->vectorField.normalized();
	//}

	for (int iter = 0; iter < 20; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			int neighborCount = 0;

			QMeshTetra *neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element
			for (int i = 0; i < 4; i++) {
				QMeshFace * Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField;

				//std::cout << neighTetra->GetIndexNo() << endl;
				neighborCount++;
			}

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;
			Tetra->vectorField = averageField.normalized();
		}
	}
}

void GuidanceField::VectorField_initialGuess_Tensile_Compress_Computing() {

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);


		if (!Tetra->isTensileorCompressSelect) {
			Tetra->vectorField << 0.0, 0.0, 0.0; continue;
		}

		Eigen::Vector3d stressNorm = Tetra->tau_max.normalized();
		Eigen::Vector3d tensile_Vector;
		tensile_Vector << 1, -stressNorm(0) / stressNorm(1), 0;
		//tensile_Vector << 1/ stressNorm(0), 0, 1/ stressNorm(2);

		stressNorm << 0.0, 1.0, 0.0;

		/*--tensile tetrahedral initial guess--*/
		if (Tetra->sigma_max > 0) {

			//Method 1: for 2D case, compute the vector on XY plane
			if(tetMesh->spaceComp == false)
				Tetra->vectorField = tensile_Vector.normalized();

			//Method 2: for 3D case, using the second principle stress direction
			else
				Tetra->vectorField = Tetra->tau_min;
		}
		else {

			/*--compressed tetrahedral direction from principle stress--*/
			if (tetMesh->spaceComp == false)
				Tetra->vectorField = tensile_Vector.normalized();
			else
				Tetra->vectorField = Tetra->tau_min;
			//Tetra->vectorField = stressNorm;
		}
	}
}

void GuidanceField::clear_small_neighbor_region() {
	// ---- Delete all the tetra element (both tensile and compress region) that not have enough neighbor ---- //
	int smallRegionSize = 8;

	int stressEleNum = 0, removeEleNum_compress = 0;
	int tensileEleNum = 0, removeEleNum_tensile = 0;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->lessNeighborFlag = false;

		if (Tetra->isTensileorCompressSelect && Tetra->sigma_max < 0) {
			std::vector< QMeshTetra* > TetraSet; stressEleNum++;
			this->detect_Neighbor_Tetrahedral_by_Node(TetraSet, Tetra, false, COMPRESS, false);
			if (TetraSet.size() < smallRegionSize) Tetra->lessNeighborFlag = true;
		}

		if (Tetra->isTensileorCompressSelect && Tetra->sigma_max >= 0) {
			std::vector< QMeshTetra* > TetraSet; tensileEleNum++;
			this->detect_Neighbor_Tetrahedral_by_Node(TetraSet, Tetra, false, TENSILE, false);
			if (TetraSet.size() < smallRegionSize) Tetra->lessNeighborFlag = true;
		}
	}

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->lessNeighborFlag && Tetra->sigma_max < 0) { Tetra->isTensileorCompressSelect = false; removeEleNum_compress++; }
		if (Tetra->lessNeighborFlag && Tetra->sigma_max > 0) { Tetra->isTensileorCompressSelect = false; removeEleNum_tensile++; }
	}

	std::cout << "For COMPRESS region, move " << removeEleNum_compress << " elements out of " << stressEleNum << std::endl;
	std::cout << "For TENSILE region, move " << removeEleNum_tensile << " elements out of " << tensileEleNum << std::endl;

}


void GuidanceField::compVectorFieldwithLocalLaplacian_neighborTetLaplacian(int eleType, QMeshTetra* Tetra) {

	//we already make sure that all the elemnt have more than N neighbor before
	Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

	std::vector< QMeshTetra* > TetraSet; TetraSet.push_back(Tetra);

	detect_Neighbor_Tetrahedral_by_Node(TetraSet, Tetra, false, eleType, false);
	for (int i = 0; i < TetraSet.size(); i++) {
		averageField += TetraSet[i]->vectorField;
	}
	for (int i = 0; i < 3; i++) averageField(i) /= TetraSet.size();
	Tetra->vectorField = averageField.normalized();

	////find neighbor tetrahedral element - by face neighbor
	/*int neighborCount = 0;

	QMeshTetra *neighTetra;
	Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

	for (int i = 0; i < 4; i++) {
		QMeshFace * Face = Tetra->GetFaceRecordPtr(i + 1);
		if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

		if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
		else neighTetra = Face->GetLeftTetra();

		if (!neighTetra->isTensileorCompressSelect) continue;
		else if (eleType == TENSILE && neighTetra->principleStress < 0) continue;
		else if (eleType == COMPRESS && neighTetra->principleStress > 0) continue;

		averageField += neighTetra->vectorField;

		neighborCount++;
	}

	if (neighborCount == 0) return false;*/
}

void GuidanceField::compGuideField() {

	// ---- method 1: apply laplacian to all the element (including constrained)

	Eigen::SparseMatrix<double> Parameter(3 * tetMesh->GetTetraNumber(), tetMesh->GetNodeNumber()); //A
	std::cout << 3 * tetMesh->GetTetraNumber() << "," << tetMesh->GetNodeNumber() << std::endl;

	Eigen::VectorXd guideField(tetMesh->GetNodeNumber()); //x

	Eigen::VectorXd b(3 * tetMesh->GetTetraNumber()); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
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
					Tetra->GetIndexNo() * 3 + j, Node->GetIndexNo(), -Tetra->VolumeMatrix(i, j)*weight)); // infill A
			}
		}

		for (int i = 0; i < 3; i++) b(Tetra->GetIndexNo() * 3 + i) = Tetra->vectorField(i)*weight; // infill B
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(tetMesh->GetNodeNumber(), tetMesh->GetNodeNumber());
	ATA = Parameter.transpose()*Parameter;
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

	for (int i = 0; i<tetMesh->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		//Node->guideFieldValue = guideField(Node->GetIndexNo());
		Node->guideFieldValue = guideFieldNormalize(Node->GetIndexNo());
		Node->guideFieldValue_no_normalize = guideField(Node->GetIndexNo());
	}
}

void GuidanceField::supportTetFieldCompute_compute_GuideField_support_by_hard_constrain() {

	int index = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->tetSupportNode == false) Node->supportIndex = -1;
		else { Node->supportIndex = index; index++; }
	}
	int supportNodeNum = index;

	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

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
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

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
						Tetra->supportIndex * 3 + j, nodeIndex, -Tetra->VolumeMatrix(i, j)* weight)); // infill A
				else {
					b(Tetra->supportIndex * 3 + j) -= Node->guideFieldValue*(-Tetra->VolumeMatrix(i, j)) * weight; //constrain
				}
			}
		}
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(supportNodeNum, supportNodeNum);
	ATA = Parameter.transpose()*Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	Solver.compute(ATA);

	Eigen::VectorXd ATb(supportNodeNum);
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb); // this vector is only for support region

	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->supportIndex < 0) continue;
		else Node->guideFieldValue = guideField(Node->supportIndex);
	}

	this->supportTetFieldCompute_normalize_guideField(true, tetMesh->GetNodeNumber()-supportNodeNum);

	std::cout << " -- finish compute scalar field by hard constrain method" << std::endl;



	/*Eigen::VectorXd allGuideField(tetMesh->GetNodeNumber()); 
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->supportIndex < 0)
			allGuideField(index) = Node->guideFieldValue;
		else allGuideField(index) = guideField(Node->supportIndex);

		index++;
	}

	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->guideFieldValue = allGuideField(index); index++;

		if (Node->tetSupportNode)
			std::cout << Node->guideFieldValue - Node->guideFieldValue_no_normalize << std::endl;

	}*/

}

//false - normalize for all region, true - compute [0,1] for initial region
void GuidanceField::supportTetFieldCompute_normalize_guideField(bool support, int initNodeNum) {

	Eigen::VectorXd guideField(tetMesh->GetNodeNumber());
	Eigen::VectorXd guideFieldNormalize(tetMesh->GetNodeNumber());

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
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

	for (int i = 0; i<tetMesh->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->guideFieldValue = guideFieldNormalize(Node->GetIndexNo());
	}

	printf("finish normalize scalar field, notice the number for support region is out of [0,1] range \n");
}

void GuidanceField::supportTetFieldCompute_compute_GuideField_support_by_soft_constrain() {

	// 2020-03-09 set soft constrain
	int initNodeNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->tetSupportNode == false) initNodeNum++;
	}
	std::cout << "init Node Num = " << "," << initNodeNum;


	//index = 0;
	//for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
	//	QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

	//	if (Tetra->GetNodeRecordPtr(1)->supportIndex < 0 && Tetra->GetNodeRecordPtr(2)->supportIndex < 0 &&
	//		Tetra->GetNodeRecordPtr(3)->supportIndex < 0 && Tetra->GetNodeRecordPtr(4)->supportIndex < 0)
	//		Tetra->supportIndex = -1;
	//	else { Tetra->supportIndex = index; index++; }
	//}

	//int supportEleNum = index;

	//std::cout << supportEleNum << "," << supportNodeNum;

	Eigen::SparseMatrix<double> 
		Parameter(3 * tetMesh->GetTetraNumber() + initNodeNum, tetMesh->GetNodeNumber()); //A

	Eigen::VectorXd guideField(tetMesh->GetNodeNumber()); //x

	Eigen::VectorXd b(3 * tetMesh->GetTetraNumber() + initNodeNum); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	// vector field equation
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->tetSupportElement == false) continue;

		double weight = 1.0;

		if (Tetra->isSupportBottomRegion) weight = 10.0;
		//if (Tetra->tetSupportElement == false) weight = 5.0;
		for (int i = 0; i < 3; i++) b(Tetra->GetIndexNo() * 3 + i) = Tetra->vectorField(i) * weight; // infill B

		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 4; i++) {
				QMeshNode* Node = Tetra->GetNodeRecordPtr(i + 1);
				ParaTriplet.push_back(Eigen::Triplet<double>(
					Tetra->GetIndexNo() * 3 + j, Node->GetIndexNo(), -Tetra->VolumeMatrix(i, j) * weight)); // infill A
			}
		}
	}

	// scalar field constrain
	double init_field_weight = 10.0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->tetSupportNode) continue;

		b(tetMesh->GetTetraNumber() * 3 + Node->GetIndexNo()) = Node->guideFieldValue * init_field_weight; // infill B
		std::cout << Node->guideFieldValue << std::endl;

		ParaTriplet.push_back(Eigen::Triplet<double>(
			tetMesh->GetTetraNumber() * 3 + Node->GetIndexNo(), Node->GetIndexNo(), init_field_weight));
	}


	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(tetMesh->GetNodeNumber(), tetMesh->GetNodeNumber());
	ATA = Parameter.transpose()*Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	Solver.compute(ATA);

	Eigen::VectorXd ATb(tetMesh->GetNodeNumber());
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);


	Eigen::VectorXd guideFieldNormalize(tetMesh->GetNodeNumber());
	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	//for (int i = 0; i < tetMesh->GetNodeNumber(); i++) {
	for (int i = 0; i < 6000; i++) {

		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i<tetMesh->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		//Node->guideFieldValue = guideField(Node->GetIndexNo());
		Node->guideFieldValue = guideFieldNormalize(Node->GetIndexNo());
	}

	std::cout << " -- finish compute scalar field" << std::endl;

}


void GuidanceField::bary_tet(QMeshTetra* Tetra, Eigen::Vector4d& coordinate, Eigen::Vector3d& CenterP)
{

	Vector3d a, b, c, d, p;
	p = CenterP;

	Tetra->GetNodeRecordPtr(1)->GetCoord3D(a(0), a(1), a(2));
	Tetra->GetNodeRecordPtr(2)->GetCoord3D(b(0), b(1), b(2));
	Tetra->GetNodeRecordPtr(3)->GetCoord3D(c(0), c(1), c(2));
	Tetra->GetNodeRecordPtr(4)->GetCoord3D(d(0), d(1), d(2));

	Vector3d vap = p - a;
	Vector3d vbp = p - b;

	Vector3d vab = b - a;
	Vector3d vac = c - a;
	Vector3d vad = d - a;

	Vector3d vbc = c - b;
	Vector3d vbd = d - b;
	// ScTP computes the scalar triple product
	double va6 = (vbp.cross(vbd)).dot(vbc);
	double vb6 = (vap.cross(vac)).dot(vad);
	double vc6 = (vap.cross(vad)).dot(vab);
	double vd6 = (vap.cross(vab)).dot(vac);
	double v6 = 1 / ((vab.cross(vac)).dot(vad));

	coordinate(0) = va6*v6;
	coordinate(1) = vb6*v6;
	coordinate(2) = vc6*v6;
	coordinate(3) = vd6*v6;
}

void GuidanceField::bary_tet(QMeshTetra* Tetra, Eigen::Vector4d& coordinate)
{

	/*
	https://stackoverflow.com/questions/38545520/barycentric-coordinates-of-a-tetrahedron
	https://mathinsight.org/scalar_triple_product
	*/

	Vector3d a, b, c, d, p;
	Tetra->CalCenterPos(p(0),p(1),p(2));

	Tetra->GetNodeRecordPtr(1)->GetCoord3D(a(0), a(1), a(2));
	Tetra->GetNodeRecordPtr(2)->GetCoord3D(b(0), b(1), b(2));
	Tetra->GetNodeRecordPtr(3)->GetCoord3D(c(0), c(1), c(2));
	Tetra->GetNodeRecordPtr(4)->GetCoord3D(d(0), d(1), d(2));

	Vector3d vap = p - a;
	Vector3d vbp = p - b;

	Vector3d vab = b - a;
	Vector3d vac = c - a;
	Vector3d vad = d - a;

	Vector3d vbc = c - b;
	Vector3d vbd = d - b;
	// ScTP computes the scalar triple product
	double va6 = (vbp.cross(vbd)).dot(vbc);
	double vb6 = (vap.cross(vac)).dot(vad);
	double vc6 = (vap.cross(vad)).dot(vab);
	double vd6 = (vap.cross(vab)).dot(vac);
	double v6 = 1 / ((vab.cross(vac)).dot(vad));

	coordinate(0) = va6*v6;
	coordinate(1) = vb6*v6;
	coordinate(2) = vc6*v6;
	coordinate(3) = vd6*v6;
}

void GuidanceField::compVectorField() {

	// ---- method 1: apply laplacian to all the element (including constrained)

	//A
	Eigen::SparseMatrix<double> Parameter(selectedTetraNum + tetMesh->GetTetraNumber(), tetMesh->GetTetraNumber());

	//x
	std::vector<Eigen::VectorXd> vectorField(3);
	for (int i = 0; i < 3; i++) vectorField[i] = Eigen::VectorXd::Zero(tetMesh->GetTetraNumber());

	//b
	std::vector<Eigen::VectorXd> b(3);
	for (int i = 0; i < 3; i++) b[i] = Eigen::VectorXd::Zero(selectedTetraNum + tetMesh->GetTetraNumber());

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	//build the linear equation, start with constrained element
	int iter = 0;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (!Tetra->isTensileorCompressSelect) continue;

		int index = Tetra->GetIndexNo();
		ParaTriplet.push_back(Eigen::Triplet<double>(iter, index, 1)); // infill A

		Eigen::Vector3d stressNorm = Tetra->tau_max;
		stressNorm.normalized();

		Eigen::Vector3d fieldDir;
		Eigen::Vector3d printDir; printDir << 1, 0, 0;
		printDir.normalized();
		fieldDir = printDir - (stressNorm.dot(printDir)) * stressNorm;
		fieldDir.normalized();

		for (int i = 0; i < 3; i++) {
			if (Tetra->sigma_max > 0) {
				b[i](iter) = fieldDir(i); //tensile tetrahedral, fill B
			}
			else {
				stressNorm << 0, -1, 0;
				b[i](iter) = stressNorm(i); //compressed tetrahedral, fill B
			}
		}
		iter++;
	}

	//first apply local laplacian for the tensile region -- not finish yet!!!!
	//for (int iter = 0; iter < 10; iter++) {
	//	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
	//		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
	//		if (!Tetra->isTensileorCompressSelect || Tetra->principleStress < 0) continue;

	//		int neighborCount = 0;

	//		QMeshTetra *neighTetra;
	//		Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

	//		//find neighbor tetrahedral element
	//		for (int i = 0; i < 4; i++) {
	//			QMeshFace * Face = Tetra->GetFaceRecordPtr(i + 1);
	//			if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

	//			if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
	//			else neighTetra = Face->GetLeftTetra();

	//			if (!neighTetra->isTensileorCompressSelect || neighTetra->principleStress < 0) continue;

	//			averageField += neighTetra->vectorField;

	//			//std::cout << neighTetra->GetIndexNo() << endl;
	//			neighborCount++;
	//		}

	//		for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;

	//		if (Tetra->isTensileorCompressSelect && Tetra->principleStress > 0) {
	//			Eigen::Vector3d stressNorm = Tetra->principleStressNorm;
	//			stressNorm.normalized();
	//			Eigen::Vector3d A = stressNorm.cross(averageField.cross(stressNorm));
	//			Tetra->vectorField = A.normalized();
	//		}

	//		else
	//			Tetra->vectorField = averageField.normalized();
	//	}
	//}

	//build the laplacian for vector field
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		int index = Tetra->GetIndexNo();
		int neighborCount = 0;

		QMeshTetra *neighTetra;
		//find neighbor tetrahedral element
		for (int i = 0; i < 4; i++) {
			QMeshFace * Face = Tetra->GetFaceRecordPtr(i + 1);
			if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

			if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
			else neighTetra = Face->GetLeftTetra();

			//std::cout << neighTetra->GetIndexNo() << endl;
			neighborCount++;

			ParaTriplet.push_back(Eigen::Triplet<double>(index + selectedTetraNum, neighTetra->GetIndexNo(), -1)); // infill A
		}
		ParaTriplet.push_back(Eigen::Triplet<double>(index + selectedTetraNum, Tetra->GetIndexNo(), neighborCount)); // infill A
	}

	//---- Solve the linear system
	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(tetMesh->GetTetraNumber(), tetMesh->GetTetraNumber());
	ATA = Parameter.transpose()*Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	Solver.compute(ATA);

	for (int i = 0; i < 3; i++) {
		Eigen::VectorXd ATb(tetMesh->GetTetraNumber());
		ATb = Parameter.transpose() * b[i];
		vectorField[i] = Solver.solve(ATb);
	}

	//---- output the computed vector field and visulization
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		for (int i = 0; i < 3; i++)
			Tetra->vectorField(i) = vectorField[i](Tetra->GetIndexNo());
		Tetra->vectorField = Tetra->vectorField.normalized();
		//std::cout << Tetra->vectorField << std::endl;
	}
	tetMesh->drawVectorField = true;
}

//true to draw vector field, false to draw scalar field gradient
void GuidanceField::visualGuideFieldGradient(bool vector) {

	if (vector) {
		tetMesh->drawScalarFieldGradient = false; return;
	}

	//using equation to compute
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

		for (int i = 0; i < 4; i++) {
			fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->guideFieldValue;
			for (int j = 0; j < 3; j++) {
				gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
			}
		}

		Tetra->scalarFieldGradient = gradient.normalized();
	}

	//using numerical difference to compute

	//for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
	//	QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

	//	Eigen::Vector4d coordinate, nodeFieldValue;
	//	for(int i=0;i<4;i++) nodeFieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->guideFieldValue;
	//	
	//	//first compute center value
	//	this->bary_tet(Tetra, coordinate);
	//	double centerFieldValue = 0;
	//	for (int i = 0; i < 4; i++) centerFieldValue += coordinate(i)*nodeFieldValue(i);

	//	//compute x, y, z value;
	//	Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

	//	double gap = 0.01;
	//	Vector3d CenterP; Tetra->CalCenterPos(CenterP(0), CenterP(1), CenterP(2));
	//	for (int i = 0; i < 3; i++) {
	//		CenterP(i) += gap;
	//		this->bary_tet(Tetra, coordinate, CenterP);
	//		double coordFieldValue = 0;
	//		for (int j = 0; j < 4; j++) coordFieldValue += coordinate(j)*nodeFieldValue(j);
	//		gradient(i) = (coordFieldValue - centerFieldValue) / gap;
	//	}
	//	Tetra->scalarFieldGradient = gradient.normalized();
	//}

	tetMesh->drawScalarFieldGradient = true;
}

void GuidanceField::LocalGlobalMethod_Basic() {

	//----Step 1: initial the index
	//------------------------------------------------------------------
	//First step: set fixed region F(v) = 1
	//Also set the node index = -1 if selected, else index start from 0.

	nNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->isFixed == true) {
			Node->guideFieldValue = 0;
			Node->SetIndexNo(-1); continue;
		}
		Node->SetIndexNo(nNum);
		nNum++;
	}

	int index = 0;
	for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index);
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index);
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->SetIndexNo(index);
		index++;
	}
	//----Step 2: compute the initial normal for all tetrahedral element
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		/*Eigen::Vector3d stressNorm = Tetra->principleStressNorm;
		Eigen::Vector3d center;
		Tetra->CalCenterPos(center(0), center(1), center(2));

		Eigen::Vector3d printDir; printDir << 0, 1, 0;
		stressNorm.norm();

		Eigen::Vector3d fieldDir = printDir - (stressNorm.dot(printDir)) * stressNorm;*/

	}

	//----Step 3: transfer vector field to guidence field
	Eigen::SparseMatrix<double> Parameter(tetMesh->GetTetraNumber() * 3, nNum); //A
	Eigen::VectorXd vertexField(nNum); //x
	Eigen::VectorXd b(tetMesh->GetTetraNumber() * 3); //b

	b.setZero();
	std::vector<Eigen::Triplet<double>> ParaTriplet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Eigen::Vector3d stressNorm = Tetra->tau_max;
		//stressNorm << 1.0, 0.0, 0.0;
		stressNorm.norm();

		Eigen::Vector3d fieldDir;
		//if (Tetra->principleStress > 0) {
		Eigen::Vector3d printDir; printDir << 1, 0, 0;
		printDir.norm();
		fieldDir = printDir - (stressNorm.dot(printDir)) * stressNorm;
		fieldDir.norm();
		//}
		//else fieldDir = stressNorm;	

		for (int i = 0; i < 3; i++) b(3 * Tetra->GetIndexNo() + i) = fieldDir(i);

		//for (int i = 0; i < 3; i++) b(3 * Tetra->GetIndexNo() + i) = stressNorm(i);
	}

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		for (int i = 0; i < 3; i++) { //sort x, y, z
			for (int j = 0; j < 4; j++) { //sort vertex
				int nodeIndex = Tetra->GetNodeRecordPtr(j + 1)->GetIndexNo();
				double insertValue = Tetra->VolumeMatrix(j, i);

				if (nodeIndex == -1) {
					b(3 * Tetra->GetIndexNo() + i) -= insertValue * (Tetra->GetNodeRecordPtr(i + 1)->guideFieldValue);
					continue;
				}

				ParaTriplet.push_back(Eigen::Triplet<double>(3 * Tetra->GetIndexNo() + i, nodeIndex, insertValue));
			}
		}
	}
	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(nNum, nNum);
	ATA = Parameter.transpose()*Parameter;

	Eigen::VectorXd ATb(nNum);
	ATb = Parameter.transpose() * b;

	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;
	Solver.compute(ATA);
	vertexField = Solver.solve(ATb);

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo() > -1) {
			Node->guideFieldValue = vertexField(Node->GetIndexNo());
		}
		if (Node->selected == true) cout << Node->guideFieldValue << endl;
	}
}

void GuidanceField::LocalGlobalMethod_Laplacian()
{
	//----Step 1: initial the index
	//------------------------------------------------------------------
	//First step: set fixed region F(v) = 1
	//Also set the node index = -1 if selected, else index start from 0.

	int index = 0;
	for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index);
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index);
		index++;
	}

	int selectTensileTetraNum = 0;
	int selectCompressTetraNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect) {
			if (Tetra->sigma_max > 0) {
				Tetra->SetIndexNo(selectTensileTetraNum);
				selectTensileTetraNum++;
			}
			else {
				Tetra->SetIndexNo(selectCompressTetraNum);
				selectCompressTetraNum++;
			}

			for (int i = 0; i < 4; i++) {
				Tetra->GetNodeRecordPtr(i + 1)->tetraSelect = true;
			}
		}
		else Tetra->SetIndexNo(-1);
	}
	int selectTetraNum = selectTensileTetraNum + selectCompressTetraNum;

	nNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->isFixed == true && Node->tetraSelect == false) {
			Node->guideFieldValue = 0;
			Node->SetIndexNo(-1); continue;
		}
		Node->SetIndexNo(nNum);
		nNum++;
	}

	//----Step 3: transfer vector field to guidence field
	Eigen::SparseMatrix<double> Parameter(selectTetraNum * 3 + nNum, nNum); //A
	Eigen::VectorXd vertexField(nNum); //x
	Eigen::VectorXd b(selectTetraNum * 3 + nNum); //b

	b.setZero();
	std::vector<Eigen::Triplet<double>> ParaTriplet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->GetIndexNo() < 0)continue;
		Eigen::Vector3d stressNorm = Tetra->tau_max;
		stressNorm.norm();

		Eigen::Vector3d fieldDir;
		Eigen::Vector3d printDir; printDir << 1, 0, 0;
		printDir.norm();
		fieldDir = printDir - (stressNorm.dot(printDir)) * stressNorm;
		fieldDir.norm();

		for (int i = 0; i < 3; i++) {
			if (Tetra->sigma_max > 0) {
				//b(3 * Tetra->GetIndexNo() + i) = fieldDir(i);

				stressNorm << 1, 0, 0;
				b(3 * Tetra->GetIndexNo() + i) = stressNorm(i);

			}
			else {
				stressNorm << 0, 1, 0;
				b(3 * selectTensileTetraNum + 3 * Tetra->GetIndexNo() + i) = stressNorm(i);

			}
		}
		if (Tetra->sigma_max > 0) {
			for (int i = 0; i < 3; i++) { //sort x, y, z
				for (int j = 0; j < 4; j++) { //sort vertex
					int nodeIndex = Tetra->GetNodeRecordPtr(j + 1)->GetIndexNo();
					double insertValue = Tetra->VolumeMatrix(j, i);

					if (nodeIndex == -1) {
						b(3 * Tetra->GetIndexNo() + i) -= insertValue * (Tetra->GetNodeRecordPtr(i + 1)->guideFieldValue);
						continue;
					}

					ParaTriplet.push_back(Eigen::Triplet<double>(3 * Tetra->GetIndexNo() + i, nodeIndex, insertValue));
				}
			}
		}
		else {
			for (int i = 0; i < 3; i++) { //sort x, y, z
				for (int j = 0; j < 4; j++) { //sort vertex
					int nodeIndex = Tetra->GetNodeRecordPtr(j + 1)->GetIndexNo();
					double insertValue = Tetra->VolumeMatrix(j, i);

					if (nodeIndex == -1) {
						b(3 * selectTensileTetraNum + 3 * Tetra->GetIndexNo() + i) -= insertValue * (Tetra->GetNodeRecordPtr(i + 1)->guideFieldValue);
						continue;
					}

					ParaTriplet.push_back(Eigen::Triplet<double>(3 * selectTensileTetraNum + 3 * Tetra->GetIndexNo() + i, nodeIndex, insertValue));
				}
			}
		}
	}

	//--------------------------------------------------------
	//Step 3: Laplacian equation, with weighting
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);

		int nodeIndex = Node->GetIndexNo();
		if (nodeIndex == -1) continue;

		ParaTriplet.push_back(Eigen::Triplet<double>
			(nodeIndex + selectTetraNum * 3, nodeIndex, Node->GetEdgeNumber()*laplacianWeight));

		for (GLKPOSITION Pos = Node->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge *edge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos);
			QMeshNode* nodeNeighboor = edge->GetStartPoint();
			if (Node == nodeNeighboor) nodeNeighboor = edge->GetEndPoint();

			int neighboorNodeIndex = nodeNeighboor->GetIndexNo();
			if (neighboorNodeIndex > -1) {
				ParaTriplet.push_back(Eigen::Triplet<double>
					(nodeIndex + selectTetraNum * 3, neighboorNodeIndex, -1.0*laplacianWeight));
			}
			else if (neighboorNodeIndex == -1) {
				if (nodeNeighboor->selected)
					b(nodeIndex + selectTetraNum * 3) += nodeNeighboor->guideFieldValue*laplacianWeight;
			}
		}
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(nNum, nNum);
	ATA = Parameter.transpose()*Parameter;

	Eigen::VectorXd ATb(nNum);
	ATb = Parameter.transpose() * b;

	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;
	Solver.compute(ATA);
	vertexField = Solver.solve(ATb);

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo() > -1) {
			Node->guideFieldValue = vertexField(Node->GetIndexNo());
		}
		if (Node->selected == true) cout << Node->guideFieldValue << endl;
	}
}

void GuidanceField::adujstField_collisionRegion() {

	/* Conduct the tetrahedral as core*/
	for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);

		if (Edge->isCollisionCheckedEdge) {
			for (GLKPOSITION Pos = Edge->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)Edge->GetFaceList().GetNext(Pos);

				if (Face->GetLeftTetra()) Face->GetLeftTetra()->collisionTetra = true;
				if (Face->GetRightTetra()) Face->GetRightTetra()->collisionTetra = true;

			}

		}
			
	}

	/* Expand one ring neighbor */
	int expandRound = 3; //3 before， 11 for rendering
	for (int ringIndex = 0; ringIndex < expandRound; ringIndex++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->collisionTetra) {
				for (int i = 0; i < 4; i++) {
					QMeshFace* neighborFace = Tetra->GetFaceRecordPtr(i + 1);
					if (neighborFace->GetLeftTetra() != NULL)neighborFace->GetLeftTetra()->collisionTetra = true;
					if (neighborFace->GetRightTetra() != NULL)neighborFace->GetRightTetra()->collisionTetra = true;
				}
			}
		}
	}

	double fieldPlanarValue = 0.5; // 0.8 before
	Eigen::Vector3d hroizontal; hroizontal << 0.0, 1.0, 0.0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->collisionTetra)
			Tetra->vectorField = (Tetra->vectorField * fieldPlanarValue + hroizontal * (1 - fieldPlanarValue)).normalized();
			//Tetra->vectorField = hroizontal;
	}

	/*for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (abs(Tetra->vectorField.dot(hroizontal)) < 0.6){
			Tetra->vectorField =
				(Tetra->vectorField * fieldPlanarValue + hroizontal * (1 - fieldPlanarValue)).normalized();
		}
	}*/


	//smooth all field
	for (int iter = 0; iter < 300; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			if (Tetra->collisionTetra) continue;

			int neighborCount = 0;

			QMeshTetra* neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element
			for (int i = 0; i < 4; i++) {
				QMeshFace* Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField;

				//std::cout << neighTetra->GetIndexNo() << endl;
				neighborCount++;
			}

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;
			Tetra->vectorField = averageField.normalized();
		}
	}

	this->compGuideField();

	// update the flag for the next iteration.
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->collisionTetra = false;
	}
	for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);
		Edge->isCollisionCheckedEdge = false;
	}
}

