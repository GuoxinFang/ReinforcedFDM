#include "stdafx.h"
#include "VectorField.h"

using namespace std;
using namespace Eigen;

#define TENSILE 0
#define COMPRESS 1

void VectorField::initMeshVectorFieldCompute() {

	this->_initializeIndex();

	this->_detectSmallCriticalRegionandClear();

	//-------------------------------------
	// compute the inital guess for critical region
	this->_criticalRegionVectorFieldInitialGuess();
	std::cout << " Vector Field - Initialization finished! " << std::endl << std::endl;

	//-------------------------------------
	// find best orientation for single critical region (by flooding)
	this->_criticalRegionOrientationDetectionbyFlooding();
	this->_smoothVectorFieldCriticalRegion();
	std::cout << " Vector Field - Field computing finished! " << std::endl << std::endl;

	//-------------------------------------
	// find best orientation between different critical region (by sorting)
	this->_optOrientationBetweenRegions();

	//-------------------------------------
	// fill the rest unimportant region with laplacian and apply global smooth for vector field
	//this->_fillNIERegionandSmooth(200, true);

	tetMesh->drawVectorField = true;

}

void VectorField::_initializeIndex()
{
	tetMesh->initializeListIndex(); // initialize the index for all list, start from 0;

	int selectTensileTetraNum = 0; int selectCompressTetraNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect) {
			if (Tetra->sigma_max > 0) selectTensileTetraNum++;
			else  selectCompressTetraNum++;
		}
	}
	selectedTetraNum = selectTensileTetraNum + selectCompressTetraNum;
}

void VectorField::_criticalRegionVectorFieldInitialGuess() {

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (!Tetra->isTensileorCompressSelect) {
			Tetra->vectorField = Eigen::Vector3d::Zero(); continue;
		}

		Eigen::Vector3d stressNorm = Tetra->tau_max.normalized();
		Eigen::Vector3d tensile_Vector;
		tensile_Vector << 1, -stressNorm(0) / stressNorm(1), 0;
		//tensile_Vector << 1/ stressNorm(0), 0, 1/ stressNorm(2);

		stressNorm << 0.0, 1.0, 0.0;

		//Method 1: for 2D case, compute the vector on XY plane
		if (tetMesh->spaceComp == false) Tetra->vectorField = tensile_Vector.normalized();

		//Method 2: for 3D case, using the minimum principle stress direction
		else Tetra->vectorField = Tetra->tau_min;
		
	}
}

void VectorField::_detectSmallCriticalRegionandClear() {

	// ---- Delete all the tetra element (both tensile and compress region) that not have enough neighbor ---- //
	int smallRegionSize = 20;

	int stressEleNum = 0, removeEleNum_compress = 0;
	int tensileEleNum = 0, removeEleNum_tensile = 0;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->lessNeighborFlag = false;

		if (Tetra->isTensileorCompressSelect && Tetra->sigma_max < 0) {
			std::vector< QMeshTetra* > TetraSet; stressEleNum++;
			this->_detectNeighborTetrabyNode(TetraSet, Tetra, false, COMPRESS, false);
			if (TetraSet.size() < smallRegionSize) Tetra->lessNeighborFlag = true;
		}

		if (Tetra->isTensileorCompressSelect && Tetra->sigma_max >= 0) {
			std::vector< QMeshTetra* > TetraSet; tensileEleNum++;
			this->_detectNeighborTetrabyNode(TetraSet, Tetra, false, TENSILE, false);
			if (TetraSet.size() < smallRegionSize) Tetra->lessNeighborFlag = true;
		}
	}

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->lessNeighborFlag && Tetra->sigma_max < 0) { Tetra->isTensileorCompressSelect = false; removeEleNum_compress++; }
		if (Tetra->lessNeighborFlag && Tetra->sigma_max > 0) { Tetra->isTensileorCompressSelect = false; removeEleNum_tensile++; }
	}

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (!Tetra->isTensileorCompressSelect) Tetra->vectorField = Eigen::Vector3d::Zero();
	}

	std::cout << "For COMPRESS region, move " << removeEleNum_compress << " elements out of " << stressEleNum << std::endl;
	std::cout << "For TENSILE region, move " << removeEleNum_tensile << " elements out of " << tensileEleNum << std::endl;

}

void VectorField::_fillNIERegionandSmooth(int iterTime, bool globalSmooth) {

	// initialize the NIE region vector field value
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect) continue;
		else Tetra->vectorField = Eigen::Vector3d::Zero();
	}

	// apply laplacian to fill the NIE region (non-direct method)
	for (int iter = 0; iter < iterTime; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->isTensileorCompressSelect) continue;

			int neighborCount = 0;

			QMeshTetra* neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element, detect face-oriented neighbor tetrahedral
			for (int i = 0; i < 4; i++) {
				QMeshFace* Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField;
				neighborCount++;
			}

			//if (neighborCount == 0) std::cout << "ERROR, neighborhood number = 0" << std::endl;

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;
			Tetra->vectorField = averageField.normalized();
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
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

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
}

void VectorField::_criticalRegionOrientationDetectionbyFlooding() {

	bool stopIter; int iter = 0;
	do {
		stopIter = true; iter++;
		this->_singleRegionOrientationEstimation(COMPRESS, -iter);

		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

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
		this->_singleRegionOrientationEstimation(TENSILE, iter);

		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			// Tetra belongs to compressed region and not being processed
			if (Tetra->isTensileorCompressSelect && Tetra->sigma_max > 0 && !Tetra->flooding_Processed) {
				stopIter = false; break;
			}
		}
	} while (!stopIter);
	std::cout << "Tensile region number = " << iter << std::endl;
	this->tensileRegionNum = iter;

}

void VectorField::_smoothVectorFieldCriticalRegion() {

	for (int iter = 0; iter < 10; iter++) {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			if (Tetra->isTensileorCompressSelect) {
				if (Tetra->sigma_max > 0)
					this->_compVectorFieldwithLocalLaplacian(TENSILE, Tetra);
				else this->_compVectorFieldwithLocalLaplacian(COMPRESS, Tetra);
			}
		}
	}

}

void VectorField::_optOrientationBetweenRegions() {
	
	//-------------------------------------
	// Initialize the system, detect NIE node and face number and update index
	int NIETetNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (!Tetra->isTensileorCompressSelect) {
			Tetra->vectorCompIndex = NIETetNum; NIETetNum++;
			Tetra->floodingRegionIndex = 0;
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

	//----------------------------------------------------------
	//reorder the region index for tetra element

	std::vector<int> regionIndexSet;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (!Tetra->isTensileorCompressSelect) continue;
		else {
			bool exit = false;
			for (int i = 0; i < regionIndexSet.size(); i++) {
				if (regionIndexSet[i] == Tetra->floodingRegionIndex) exit = true;
			}
			if (!exit) regionIndexSet.push_back(Tetra->floodingRegionIndex);
		}
	}
	for (int i = 0; i < regionIndexSet.size(); i++) std::cout << regionIndexSet[i] << ","; std::cout << std::endl;
	int tensileNum = 0; for (int i = 0; i < regionIndexSet.size(); i++) { if (regionIndexSet[i] > 0)tensileNum++; }
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->floodingRegionIndex >= 0) Tetra->floodingRegionIndex -= -1;
		else Tetra->floodingRegionIndex = fabs(Tetra->floodingRegionIndex) + tensileNum;
	}

	//-----------------------------------------------------------------------
	// --------- First build the system of vector field optimization.

	long time = clock();
	Eigen::SparseMatrix<double> A; // A matrix for vector field computing
	this->_buildVectorFieldCompSystem(NIETetNum, NIEFaceNum, A);
	printf(" TIMER -- Build the vector field compute system takes %ld ms.\n", clock() - time);
	time = clock();

	std::cout << this->_iterFillNIEandComputeEnergy(NIETetNum, NIEFaceNum, A) << " initial energy!" << std::endl;

	//return;

	int iterNum = pow(2, regionIndexSet.size() - 2);
	//std::cout << regionIndexSet.size() << "," << iterNum << std::endl;

	//-----------------------------------------------------------------------
	// --------- Build the order for sorting
	Eigen::MatrixXi order(iterNum, compRegionNum + tensileRegionNum);
	for (int i = 0; i < iterNum; i++) {
		order(i, 0) = 0; // the orientation of first region is always remain constant
		for (int j = 0; j < compRegionNum + tensileRegionNum - 1; j++)  
			order(i, j + 1) = (i / ((int)pow(2, j))) % 2;
	}
	
	std::cout << order << std::endl;
	//time = clock();

	Eigen::VectorXd energyIter(iterNum);
	for (int i = 0; i < iterNum; i++) {
		Eigen::VectorXi orderiter = order.row(i);
		this->_changeCriticalRegionFieldDir(orderiter);
		energyIter(i) = this->_iterFillNIEandComputeEnergy(NIETetNum, NIEFaceNum, A);
		this->_changeCriticalRegionFieldDir(orderiter);
		std::cout << "energy for iter #" << i << " = " << energyIter(i) << " with order " << order.row(i) << std::endl;
	}

	int minIter; energyIter.minCoeff(&minIter);

	Eigen::VectorXi optOrder = order.row(minIter);

	this->_changeCriticalRegionFieldDir(optOrder);
	
	std::cout << "final energy order index" << minIter << " = " << 
		this->_iterFillNIEandComputeEnergy(NIETetNum, NIEFaceNum, A) << " with order " << order.row(minIter) << std::endl;


	//std::cout << energyIter << std::endl;

	/*for (int i = 0; i < iterNum; i++) {
		this->_changeRegionFieldDir(order.row(i));
		this->_fillBandCompute(NIETetNum, NIEFaceNum, A);
		energyIter(iterTime) = this->_compVectorLaplacianEnergyValue();
		this->_changeRegionFieldDir(iterTime);	
	}*/

	/*this->_fillBandCompute(NIETetNum, NIEFaceNum, A);
	double initEnergy = this->_compVectorLaplacianEnergyValue();
	std::cout << "init Energy = " << initEnergy << std::endl;*/

	//int critRegNum = compRegionNum + tensileRegionNum;
	//Eigen::VectorXd energyIter(critRegNum - 1);
	// 
	//int maxIter = 10;
	//for (int i = 0; i < maxIter; i++) {
	//	for (int iterTime = 0; iterTime < critRegNum - 1; iterTime++) {
	//		this->_changeCriticalRegionFieldDir(iterTime, true);
	//		energyIter(iterTime) = this->_iterFillNIEandComputeEnergy(NIETetNum, NIEFaceNum, A);
	//		this->_changeCriticalRegionFieldDir(iterTime, false);

	//		//energyIter(iterTime) = this->_compVectorLaplacianEnergyValue();
	//		//this->_changeRegionFieldDir(iterTime);
	//	}
	//	std::cout << " This iter, init energy = " << initEnergy
	//		<< ", updated energy = " << energyIter << std::endl;

	//	int minIter;
	//	if (energyIter.minCoeff(&minIter) >= initEnergy) {
	//		std::cout << "find best direction! " << std::endl;  break;
	//	}
	//	else {
	//		this->_changeRegionFieldDir(minIter);
	//		this->_fillBandCompute(NIETetNum, NIEFaceNum, A);
	//		initEnergy = this->_compVectorLaplacianEnergyValue();
	//	}
	//}

	printf(" TIMER -- find optimized field takes %ld ms.\n", clock() - time);
}

void VectorField::_changeCriticalRegionFieldDir(Eigen::VectorXi& orderiter) {

	// std::cout << detectRegion << std::endl;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect == false) continue;
		else {
			if (orderiter(Tetra->floodingRegionIndex) == 1) Tetra->vectorField = -Tetra->vectorField;
		}
	}
}

double VectorField::_iterFillNIEandComputeEnergy(
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

#pragma omp parallel   
	{
#pragma omp for  
		for (int i = 0; i < 3; i++) {
			Eigen::VectorXd ATb(NIETetNum);
			ATb = A.transpose() * b[i];
			vectorField[i] = vectorFieldSolver.solve(ATb);
		}
	}

	/*for (int i = 0; i < vectorField[0].rows(); i++)
		std::cout << vectorField[0](i) << "," << vectorField[1](i) << "," << vectorField[2](i) << std::endl;*/

	//---- output the computed vector field and visulization
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->isTensileorCompressSelect) continue;
		for (int i = 0; i < 3; i++)
			Tetra->vectorField(i) = vectorField[i](Tetra->vectorCompIndex);
		Tetra->vectorField = Tetra->vectorField.normalized();
	}

	double lapEnergy = 0.0;

	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

		/*lapEnergy += Face->CalArea() *
			pow(1 - Face->GetLeftTetra()->vectorField.dot(Face->GetRightTetra()->vectorField), 3);*/

		lapEnergy += Face->CalArea() *
			pow((Face->GetLeftTetra()->vectorField - Face->GetRightTetra()->vectorField).norm(), 4);
	}

	return lapEnergy;

}

void VectorField::_buildVectorFieldCompSystem(
	int NIETetNum, int NIEFaceNum, Eigen::SparseMatrix<double>& A) {

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

void VectorField::_compVectorFieldwithLocalLaplacian(int eleType, QMeshTetra* Tetra) {

	//we already make sure that all the elemnt have more than N neighbor before
	Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

	std::vector< QMeshTetra* > TetraSet; TetraSet.push_back(Tetra);

	_detectNeighborTetrabyNode(TetraSet, Tetra, false, eleType, false);
	for (int i = 0; i < TetraSet.size(); i++) {
		averageField += TetraSet[i]->vectorField;
	}
	for (int i = 0; i < 3; i++) averageField(i) /= TetraSet.size();
	Tetra->vectorField = averageField.normalized();

}

//ele_type = 0 for TENSIER, ele_type = 1 for COMPRESS
void VectorField::_detectNeighborTetrabyNode(std::vector< QMeshTetra* >& TetraSet, 
	QMeshTetra* Tetra, bool checkProcess, int ele_type, bool face_node) {

	//using node to find neighbor tetrahedral 
	if (face_node == false) {
		for (int i = 0; i < 4; i++) {
			QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
			for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);

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

			QMeshTetra* ConnectTetra = thisFace->GetRightTetra();
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
void VectorField::_singleRegionOrientationEstimation(int type, int index) {

	std::vector< QMeshTetra* > TetraSet;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (!Tetra->isTensileorCompressSelect || Tetra->flooding_Processed) continue;
		else if (type == COMPRESS && Tetra->sigma_max >= 0) continue;
		else if (type == TENSILE && Tetra->sigma_max < 0) continue;

		// initial tetra used in flooding
		TetraSet.push_back(Tetra);
		Tetra->flooding_Processed = true;

		//find fist ring neighbor
		this->_detectNeighborTetrabyNode(TetraSet, Tetra, true, type, false);

		for (int i = 0; i < TetraSet.size(); i++) {
			if (Tetra->vectorField.dot(TetraSet[i]->vectorField) < 0)
				TetraSet[i]->vectorField = -TetraSet[i]->vectorField;
		}
		break;
	}

	std::cout << " ## Region INDEX = " << index << ". Start flooding ... " << std::endl << TetraSet.size();

	int setNum;
	do {
		setNum = TetraSet.size();

		/*Eigen::Vector3d averageRegionField = Eigen::Vector3d::Zero();
		for (int j = 0; j < TetraSet.size(); j++) averageRegionField += TetraSet[j]->vectorField / TetraSet.size();
		averageRegionField = averageRegionField.normalized();

		std::cout << averageRegionField << std::endl << "--------------" << std::endl;*/

		/* #### This is important, used to forbid loop in one side #### Guoxin 2020-03-30 */
		std::vector< QMeshTetra* > TetraSetThisLoop(TetraSet.size());
		for (int i = 0; i < TetraSet.size(); i++) TetraSetThisLoop[i] = TetraSet[i];


		for (int i = 0; i < TetraSetThisLoop.size(); i++) {
			this->_detectNeighborTetrabyNode(TetraSet, TetraSetThisLoop[i], true, type, false);
		}
		std::cout << " -- " << TetraSet.size();

		for (int i = 0; i < TetraSet.size(); i++) {

			if (TetraSet[i]->flooding_Processed == true) continue;

			//detect neighbor normal
			std::vector< QMeshTetra* > thisTetraNeighborSet;
			this->_detectNeighborTetrabyNode(thisTetraNeighborSet, TetraSet[i], false, type, false);
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

	std::cout << std::endl<< "   This region contains " << TetraSet.size() << " elements." << std::endl << std::endl;
}
