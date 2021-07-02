#include "stdafx.h"
#include "PrincipleStressField.h"
#include <omp.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "GLKGeometry.h"

using namespace std;
using namespace Eigen;

PrincipleStressField::PrincipleStressField(QMeshPatch *mesh) { tetMesh = mesh; }

PrincipleStressField::~PrincipleStressField() { }

void PrincipleStressField::InputFEMResult(std::string filename)
{
	const char * c = filename.c_str();

	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
	strcpy(input_filename, "..\\Model\\FEM_Selection_file\\");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(input_filename, filetype);

	ifstream stressInfor(input_filename);
	if (!stressInfor)
		cerr << "Sorry!We were unable to open the file!\n";

	Eigen::VectorXi eleIndex = Eigen::VectorXi::Zero(tetMesh->GetTetraNumber());
	Eigen::MatrixXd stressMatrix = Eigen::MatrixXd::Zero(tetMesh->GetTetraNumber(), 7);

	//string line;
	int lineIndex = 0;
	string sss;
	while (getline(stressInfor, sss))
	{
		const char * c = sss.c_str();
		sscanf(c, "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
			&eleIndex(lineIndex), &stressMatrix(lineIndex, 0), &stressMatrix(lineIndex, 1), &stressMatrix(lineIndex, 2)
			, &stressMatrix(lineIndex, 3), &stressMatrix(lineIndex, 4), &stressMatrix(lineIndex, 5), &stressMatrix(lineIndex, 6));
		lineIndex++;
	}

	stressInfor.close();

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		element->eleStress = Eigen::VectorXd::Zero(7);
		//Index number start from 1
		for (int i = 0; i < 7; i++) element->eleStress(i) = stressMatrix(element->GetIndexNo() - 1, i);
		//cout << element->GetIndexNo() << " = " << element->eleStress << endl;
	}

	tetMesh->drawStressField = true;

	double max = 0, min = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		double thisStress = element->eleStress[0];
		if (thisStress > max) max = thisStress;
		if (thisStress < min) min = thisStress;
	}
	tetMesh->minStressValue = min;
	tetMesh->drawValue[0] = min;
	tetMesh->maxStressValue = max;
	tetMesh->drawValue[1] = max;

	printf(" Principle Stress Field -- Finish input FEM result!\n\n");
}

void PrincipleStressField::ComputeElementPrincipleStress() {

	//---- Compute the principle stress from the Tensor by SVD
	Eigen::Matrix3d stressTensor;

	int Core = 28;
	int EachCore = tetMesh->GetTetraNumber() / Core + 1;

#pragma omp parallel   
	{
#pragma omp for  
		for (int omptime = 0; omptime < Core; omptime++) {

			for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

				if (Tet->GetIndexNo() < omptime * EachCore) continue;
				else if (Tet->GetIndexNo() > (1 + omptime) * EachCore) break;

				stressTensor <<
					Tet->eleStress(1), Tet->eleStress(4), Tet->eleStress(5),
					Tet->eleStress(4), Tet->eleStress(2), Tet->eleStress(6),
					Tet->eleStress(5), Tet->eleStress(6), Tet->eleStress(3);

				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(stressTensor, Eigen::ComputeEigenvectors);
				Eigen::Matrix3d eigenvectorsMatrix = eigenSolver.eigenvectors();
				Eigen::Vector3d eigenvalue = eigenSolver.eigenvalues();

				std::ptrdiff_t first_index, last_index;
				//eigenvalue.maxCoeff(&index); //Max principle stress

				Eigen::Vector3d eigenvalueABS; for (int i = 0; i < 3; i++) eigenvalueABS(i) = abs(eigenvalue(i));

				//(eigenvalue.cwiseAbs()).maxCoeff(&index); //Max principle stress (ABS)

				eigenvalueABS.maxCoeff(&first_index); //Max principle stress (ABS)
				eigenvalueABS.minCoeff(&last_index); //Third principle stress (ABS)

				Tet->tau_max = eigenvectorsMatrix.col(first_index);
				Tet->tau_min = eigenvectorsMatrix.col(last_index);

				Tet->sigma_max = eigenvalue(first_index);
				Tet->sigma_min = eigenvalue(last_index);

				for (int i = 0; i < 3; i++) {
					if (i != first_index && i != last_index) {
						Tet->tau_mid = eigenvectorsMatrix.col(i);
						Tet->sigma_mid = eigenvalue(i);
					}
				}



			}
		}
	}
	tetMesh->drawPrincipleStressField = true;

	//---- Compute min and max principle stress value
	double max = 0, min = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		double thisStress = element->sigma_max;
		if (thisStress > max) max = thisStress;
		if (thisStress < min) min = thisStress;
	}
	tetMesh->minPrincipleStressValue = min;
	tetMesh->maxPrincipleStressValue = max;

}

void PrincipleStressField::DetermineCriticalTensileandCompressRegion(double rangeT, double rangeC) {
	
	tensileEleNum = 0;
	compressEleNum = 0;

	//--------------------------------------------------------------------
	//Step 1: initialize the index and flag, detect tensile and compress element number

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tet->stressIndex = -1;
		if (Tet->sigma_max >= 0.0) this->tensileEleNum++;
		else this->compressEleNum++;
	}

	Eigen::VectorXd tensileValue(tensileEleNum);
	Eigen::VectorXd compressValue(compressEleNum);

	int cEleNum = 0, tEleNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tet->sigma_max >= 0.0) {
			tensileValue(tEleNum) = Tet->sigma_max; tEleNum++;
		}
		else {
			compressValue(cEleNum) = fabs(Tet->sigma_max); cEleNum++;
		}
	}

	//--------------------------------------------------------------------
	//Step 2: sort the element by tensile or compress stress value

	VectorXi tind, cind;
	tind = VectorXi::LinSpaced(tEleNum, 0, tEleNum - 1); //[0 1 2 3 ... N-1]
	cind = VectorXi::LinSpaced(cEleNum, 0, cEleNum - 1); //[0 1 2 3 ... N-1]

	auto ruleT = [tensileValue](int i, int j)->bool {
		return tensileValue(i)>tensileValue(j); //sorting rules
	};

	std::sort(tind.data(), tind.data() + tind.size(), ruleT);

	auto ruleC = [compressValue](int i, int j)->bool {
		return compressValue(i)>compressValue(j); //sorting rules
	};

	std::sort(cind.data(), cind.data() + cind.size(), ruleC);

	tEleNum = 0; cEleNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tet->sigma_max >= 0.0) {
			for (int i = 0; i < tensileEleNum; i++) {
				if (tEleNum == tind(i)) {
					Tet->stressIndex = i; break;
				}
			}
			tEleNum++;
		}
		else {
			for (int i = 0; i < compressEleNum; i++) {
				if (cEleNum == cind(i)) {
					Tet->stressIndex = i; break;
				}
			}
			cEleNum++;
		}
	}

	//--------------------------------------------------------------------
	//Step 3: select compute region by ratio
	this->_selectTensileandCompressiveRegion(rangeT, rangeC);
}

void PrincipleStressField::_selectTensileandCompressiveRegion(double rangeT, double rangeC) {

	int initialGuessRegionNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tet->isTensileorCompressSelect = false;
		Tet->isSmallInfluence = false;

		int Tnumber = floor(tensileEleNum*rangeT);
		int Cnumber = floor(compressEleNum*rangeC);

		//if (Tet->sigma_max >= 0.0) {
		//	// ensure no singularity happen in mid/min stress
		//	if (fabs(Tet->sigma_mid) > 2 * fabs(Tet->sigma_min) && Tet->stressIndex < Tnumber ) 
		//		Tet->isTensileorCompressSelect = true;
		//}
		//else {
		//	if (fabs(Tet->sigma_mid) > 2 * fabs(Tet->sigma_min) && Tet->stressIndex < Cnumber)
		//		Tet->isTensileorCompressSelect = true;
		//}

		//if (fabs(Tet->sigma_max) < 3 * fabs(Tet->sigma_mid) && Tet->isTensileorCompressSelect) {
		//	std::cout << "initial guess region!" << std::endl;
		//	initialGuessRegionNum++;
		//}

		if (Tet->sigma_max >= 0.0) {
			if (Tet->stressIndex < Tnumber)
				Tet->isTensileorCompressSelect = true;
		}
		else {
			if (Tet->stressIndex < Cnumber)
				Tet->isTensileorCompressSelect = true;
		}
	}
	std::cout << "initialGuessRegionNum = " << initialGuessRegionNum << std::endl;
}
