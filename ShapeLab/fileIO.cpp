#include "stdafx.h"
#include "fileIO.h"
#include "GLKMatrixLib.h"
#include <omp.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "GLKGeometry.h"
#include <iomanip>

using namespace std;
using namespace Eigen;

void fileIO::saveSelectedFixedHandleRegion(PolygenMesh* polygenMesh) {

	QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char* c = filename.c_str();
	char* cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char* split = ".";
	char* p = strtok(cstr, split);

	char output_filename[256];
	strcpy(output_filename, "..\\Model\\FEM_Selection_file\\");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, "_selection.txt");
	strcat(output_filename, filetype);

	ofstream nodeSelection(output_filename);
	if (!nodeSelection)
		cerr << "Sorry!We were unable to build the file NodeSelect!\n";
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		nodeSelection << CheckNode->GetIndexNo() << ":";
		//for the selection of fixing part
		if (CheckNode->isFixed == true) nodeSelection << "1:";
		else nodeSelection << "0:";
		//for the selection of hard part
		if (CheckNode->isHandle == true) nodeSelection << "1:" << endl;
		else nodeSelection << "0:" << endl;
	}

	nodeSelection.close();
	printf(" fileIO -- Finish output selected handle / fixed region. \n");

}

bool fileIO::inputFixedHandleRegion(PolygenMesh* polygenMesh) {
	QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char* c = filename.c_str();

	char* cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char* split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
	strcpy(input_filename, "..\\Model\\FEM_Selection_file\\");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, "_selection.txt");
	strcat(input_filename, filetype);

	ifstream nodeSelect(input_filename);
	if (!nodeSelect) {
		cerr << "Sorry!We were unable to open the file!\n";
		return false;
	}
	vector<int> NodeIndex(patch->GetNodeNumber()), checkNodeFixed(patch->GetNodeNumber()), checkNodeHandle(patch->GetNodeNumber());
	//string line;
	int LineIndex1 = 0;
	string sss;
	while (getline(nodeSelect, sss)) {
		const char* c = sss.c_str();
		sscanf(c, "%d:%d:%d", &NodeIndex[LineIndex1], &checkNodeFixed[LineIndex1], &checkNodeHandle[LineIndex1]);
		LineIndex1++;
	}

	nodeSelect.close();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (checkNodeFixed[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isFixed = true;
		if (checkNodeHandle[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isHandle = true;
	}

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)
			Face->isHandleDraw = true;
		else Face->isHandleDraw = false;

		if (Face->GetNodeRecordPtr(0)->isFixed == true &&
			Face->GetNodeRecordPtr(1)->isFixed == true &&
			Face->GetNodeRecordPtr(2)->isFixed == true)
			Face->isFixedDraw = true;
		else Face->isFixedDraw = false;
	}
	printf(" fileIO -- Finish input handle / fixed region. \n");
}

void fileIO::exportMeshtoAbaqusFEM(PolygenMesh* polygenMesh) {

	QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char* c = filename.c_str();
	char* cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char* split = ".";
	char* p = strtok(cstr, split);

	char output_filename[256];
	strcpy(output_filename, "..\\Model\\Abaqus_INP\\");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".inp");
	strcat(output_filename, filetype);

	ofstream abaqusOutput(output_filename);
	if (!abaqusOutput)
		cerr << "Sorry!We were unable to build the file NodeSelect!\n";
	abaqusOutput << "*Part, name=" << filename << endl << "*Node" << endl;

	//First go through all the nodes
	double pp[3] = { 0 }, ppl[3] = { 0 }, ppr[3] = { 0 };
	int node_index = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		CheckNode->GetCoord3D(pp[0], pp[1], pp[2]);
		node_index++;
		abaqusOutput << node_index << "," << pp[0] << "," << pp[1] << "," << pp[2] << endl;
	}

	abaqusOutput << "*Element, type = C3D4" << endl;

	int node_indexTet[4] = { 0 };

	int tet_index = 0;
	for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
		tet_index++;
		for (int i = 0; i < 4; i++) node_indexTet[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		abaqusOutput << tet_index << ",";
		for (int i = 0; i < 3; i++) abaqusOutput << node_indexTet[i] << ",";
		abaqusOutput << node_indexTet[3];
		abaqusOutput << endl;
	}

	abaqusOutput << "*End Part" << endl;
	abaqusOutput.close();

}

void fileIO::outputISOSurfaceMesh(
	PolygenMesh* isosurfaceSet, bool splitMode, bool singleOutputMode, 
	std::string modelName, int maxLayerNum, bool offMode) {

	/*----------------- output the mesh -----------------*/

	for (GLKPOSITION posMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoSurface = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(posMesh);

		//----------------ISO layers spliting mode (modified by Guoxin on 19-01-2020)----------------//
		if (splitMode) {

			std::vector<QMeshPatch*>  isoSurfaceSet_splited;
			bool if_split = this->_splitSingleSurfacePatch_detect(isoSurface);

			std::string path = "../Model/IsoSurface/" + modelName + "/";

			if (if_split) {
				_splitSingleSurfacePatch_splitmesh(isoSurface, isoSurfaceSet_splited);
				for (int i = 0; i < isoSurfaceSet_splited.size(); i++) {
					if (isoSurface->includeSupportRegion == true)  isoSurfaceSet_splited[i]->includeSupportRegion = true;
					this->_outputSingleISOSurface(offMode, isoSurfaceSet_splited[i], path, true, false);
				}
				//this->outputSingleISOSurface(1, isoSurface, path, true);

			}
			else this->_outputSingleISOSurface(offMode, isoSurface, path, false, false);

			continue;
		}

		//----------------Single ISO layer output mode----------------//

		if (singleOutputMode && isoSurface->GetIndexNo() == maxLayerNum) {

			//std::string path = "../Model/surface/" + meshName->getModelName() + "_";
			std::string path = "../Model/IsoSurface/" + modelName + "/";
			this->_outputSingleISOSurface(offMode, isoSurface, path, false, false);

			break;
		}

		//----------------Multiple ISO layers output mode----------------//

		if (!singleOutputMode && isoSurface->GetIndexNo() < maxLayerNum) {

			std::string path = "../Model/IsoSurface/" + modelName + "/";
			this->_outputSingleISOSurface(offMode, isoSurface, path, false, false);

			continue;
		}

	}

}

void fileIO::outputPrincipleStressValueforCriticalRegion(PolygenMesh* initialModel) {
	std::string folderPath = "..\\Model\\RenderFile\\" + initialModel->getModelName();
	string command; command = "mkdir " + folderPath;
	system(command.c_str());

	QMeshPatch* tetModel = (QMeshPatch*)initialModel->GetMeshList().GetHead();

	//----------------principle stress value----------------//
	std::string stressValuePath = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_max_min_principle.txt";
	ofstream stressValue(stressValuePath);

	for (GLKPOSITION posNode = tetModel->GetTetraList().GetHeadPosition(); posNode != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetModel->GetTetraList().GetNext(posNode);

		double ratio = abs(tet->sigma_max / tet->sigma_min);
		if (ratio < 0.99) cout << "WARMING, min principle stress larger than max!" << std::endl;

		stressValue << tet->sigma_max << "," << tet->sigma_mid <<
			"," << tet->sigma_min << "," << ratio << std::endl;
	}

	stressValue.close();

}


void fileIO::outputVectorFieldforRendering(PolygenMesh* initialModel) {

	//----------------build the folder to save vector and sacalr field file----------------//
	std::string folderPath = "..\\Model\\RenderFile\\" + initialModel->getModelName();
	string command; command = "mkdir " + folderPath;
	system(command.c_str());

	QMeshPatch* tetModel = (QMeshPatch*)initialModel->GetMeshList().GetHead();

	//----------------output vector field----------------//
	std::string vectorFieldPath = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_vectorFieldwithRegionIndex.txt";
	ofstream vectorField(vectorFieldPath);

	for (GLKPOSITION posNode = tetModel->GetTetraList().GetHeadPosition(); posNode != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetModel->GetTetraList().GetNext(posNode);
		double pp[3];
		tet->CalCenterPos(pp[0], pp[1], pp[2]);

		int inner = 0;
		if (tet->IsInner()) inner = 1;

		double tetScalarField = 0;
		for (int i = 0; i < 4; i++)
			tetScalarField += tet->GetNodeRecordPtr(i + 1)->guideFieldValue;

		vectorField << pp[0] << "," << pp[1] << "," << pp[2] << ","
			<< tet->vectorField(0) << "," << tet->vectorField(1) << "," << tet->vectorField(2) << "," 
			<< tet->floodingRegionIndex << "," << inner << "," << tetScalarField / 4 << std::endl;
	}

	vectorField.close();

}

void fileIO::updateInitialSurfaceName(PolygenMesh* isosurfaceSet, PolygenMesh* isosurfaceSet_support) {

	Eigen::VectorXi layerIndex(isosurfaceSet->GetMeshList().GetCount());
	int index = 0; int support_index = 0;
	for (GLKPOSITION posMesh = isosurfaceSet_support->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoSurface = (QMeshPatch*)isosurfaceSet_support->GetMeshList().GetNext(posMesh);
		support_index++; isoSurface->SetIndexNo(support_index);
		if (isoSurface->includeSupportRegion == false) { 
			layerIndex(index) = isoSurface->GetIndexNo(); 
			std::cout << layerIndex(index) << ", "; index++;
		}
	}
	index = 0;
	for (GLKPOSITION posMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoSurface = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(posMesh);
		isoSurface->SetIndexNo(layerIndex(index)); index++;
	}

	printf("\n\n ---- Finish update the index of initial model isosurface \n\n");
}

void fileIO::outputISOSurfaceMesh_support(PolygenMesh* isosurfaceSet_support, std::string modelName) {

	std::string path = "../Model/IsoSurface/" + modelName + "/Support/";

	for (GLKPOSITION posMesh = isosurfaceSet_support->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoSurface = (QMeshPatch*)isosurfaceSet_support->GetMeshList().GetNext(posMesh);
		if (isoSurface->includeSupportRegion == false) continue;
		if (isoSurface->isInnerSlcingLayer == true) std::cout << "the surface" << isoSurface->layerName << "is inner layer!" << std::endl;
		this->_outputSingleISOSurface(false, isoSurface, path, false, false);
	}
}


void fileIO::inputInstalledIsoSurface(
	PolygenMesh* isosurfaceSet, std::vector<std::string>& layersFiles, std::vector<std::string>& fieldFiles, std::string folderPath) {
	
	std::cout << folderPath << std::endl;

	if (layersFiles.size() != fieldFiles.size()) printf("fileIO - SYSTEM FILE reading ERROR!\n");

	for (int i = 0; i < layersFiles.size(); i++) {

		char filename[1024]; sprintf(filename, "%s%s%s", folderPath.data(), "/", layersFiles[i].data());
		std::cout << filename << std::endl;
		QMeshPatch* layer = new QMeshPatch; layer->inputOBJFile(filename, false);
		layer->layerName = layersFiles[i].substr(0, layersFiles[i].length() - 4);
		isosurfaceSet->GetMeshList().AddTail(layer);
		layer->SetIndexNo(i);

		char fieldfilename[1024]; sprintf(fieldfilename, "%s%s%s", folderPath.data(), "/Field/", fieldFiles[i].data());
		std::cout << fieldfilename << std::endl;
		FILE* fp = fopen(fieldfilename, "r");
		if (!fp) { printf("Can not open the field data file\n"); return; }

		Eigen::MatrixXd fieldValue = Eigen::MatrixXd::Zero(layer->GetFaceNumber(), 3);
		float aa, bb, cc;
		for (int faceIndex = 0; faceIndex < layer->GetFaceNumber(); faceIndex++) {
			fscanf(fp, "%lf,%lf,%lf\n", &fieldValue(faceIndex, 0), &fieldValue(faceIndex, 1), &fieldValue(faceIndex, 2));
			//std::cout << aa << bb << cc << std::endl;
		}
		//&fieldValue(faceIndex, 0), & fieldValue(faceIndex, 1), & fieldValue(faceIndex, 2));
		fclose(fp);

		int index = 0;
		for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
			face->principleStressDir = fieldValue.row(index); index++;
			//std::cout << face->principleStressDir << std::endl;
		}

		printf("-- (heatMethodField) Finish input the stress field value !\n\n");
		layer->drawThisIsoLayer = true;
	}

}

void fileIO::inputInstalledIsoSurface(
	PolygenMesh* isosurfaceSet, std::vector<std::string>& layersFiles, std::string folderPath) {

	std::cout << folderPath << std::endl;

	for (int i = 0; i < layersFiles.size(); i++) {

		char filename[1024]; sprintf(filename, "%s%s%s", folderPath.data(), "/", layersFiles[i].data());
		std::cout << filename << std::endl;
		QMeshPatch* layer = new QMeshPatch; layer->inputOBJFile(filename, false);
		layer->layerName = layersFiles[i].substr(0, layersFiles[i].length() - 4);
		isosurfaceSet->GetMeshList().AddTail(layer);
		layer->SetIndexNo(i);
		layer->drawThisIsoLayer = true;

	}

}

void fileIO::inputInstalledIsoSurface_support(
	PolygenMesh* isosurfaceSet, std::vector<std::string>& layersFileSet_support, std::string folderPath, std::string modelName) {


	int initLayerNum = isosurfaceSet->GetMeshList().GetCount();
	std::string path_support_split = "../Model/IsoSurface/" + modelName + "/Support_Split/";


	for (int i = 0; i < layersFileSet_support.size(); i++) {

		char filename[1024]; sprintf(filename, "%s%s%s", folderPath.data(), "/Support_Trim/", layersFileSet_support[i].data());
		std::cout << filename << std::endl;

		QMeshPatch* layer_support = new QMeshPatch; layer_support->inputOBJFile(filename, false);
		layer_support->layerName = layersFileSet_support[i].substr(0, layersFileSet_support[i].length() - 4);
		layer_support->includeSupportRegion = true;

		// support layer require split
		if (this->_splitSingleSurfacePatch_detect(layer_support)) {

			std::cout << "split the support layer: " << layer_support->layerName << std::endl;

			std::vector<QMeshPatch*>  isoSurfaceSet_splited;
			int layerNameInt = stoi(layer_support->layerName.substr(0, layer_support->layerName.length() - 3));

			_splitSingleSurfacePatch_splitmesh(layer_support, isoSurfaceSet_splited);
			layer_support->ClearAll(); delete layer_support;

			for (int j = 0; j < isoSurfaceSet_splited.size(); j++) {

				std::cout << this->_supportLayerAera(isoSurfaceSet_splited[j]) << std::endl;
				if (this->_supportLayerAera(isoSurfaceSet_splited[j]) < 10.0) continue;

				isoSurfaceSet_splited[j]->layerName = to_string(layerNameInt * 100 + j) + "S";
				isoSurfaceSet_splited[j]->includeSupportRegion = true;
				isoSurfaceSet_splited[j]->drawThisIsoLayer = true;

				isosurfaceSet->GetMeshList().AddTail(isoSurfaceSet_splited[j]);
				isoSurfaceSet_splited[j]->SetIndexNo(initLayerNum + j);

				this->_outputSingleISOSurface(false, isoSurfaceSet_splited[j], path_support_split, false, true);

			}
			initLayerNum += isoSurfaceSet_splited.size();
		}
		else {
			isosurfaceSet->GetMeshList().AddTail(layer_support);
			layer_support->SetIndexNo(initLayerNum); initLayerNum++;
			layer_support->drawThisIsoLayer = true;

			this->_outputSingleISOSurface(false, layer_support, path_support_split, false, true);
		}
	
	}
	printf("-- Finish input the supporting layer !\n\n");

}

double fileIO::_supportLayerAera(QMeshPatch* isoSurface_support) {
	double aera = 0;
	for (GLKPOSITION posFace = isoSurface_support->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)isoSurface_support->GetFaceList().GetNext(posFace);
		aera += face->CalArea();
	}
	return aera;
}

bool fileIO::_splitSingleSurfacePatch_detect(QMeshPatch* isoSurface) {
	//Function for detecting if single path have more than one closed region - by flooding method//

	/*---------------------------
	Detect if given a closed mesh
	---------------------------*/

	bool closedSurface = true;
	for (GLKPOSITION Pos = isoSurface->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)isoSurface->GetEdgeList().GetNext(Pos);
		if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) { closedSurface = false; break; }
	}
	if (closedSurface) return false;

	/*-------------------------------
	give node split index by flooding
	-------------------------------*/
	bool allNodeChecked = true;
	int splitTime = 0;

	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
		thisNode->splitIndex = -1;
	}

	do {
		splitTime++;

		//-- Find start node
		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex < 0) { thisNode->splitIndex = splitTime; break; }
		}

		//-- Run flooding searching
		this->_splitSingleSurfacePatch_flooding(isoSurface, splitTime);

		//-- Detect if all the node has been checked
		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			allNodeChecked = true;
			if (thisNode->splitIndex < 0) { allNodeChecked = false; break; }
		}

	} while (!allNodeChecked);

	if (splitTime == 1) { std::cout << "This isoSurface contains single region" << std::endl; return false; }
	else {

		/*----------------------------------------------------------------------------------
		if contains more than one region, save them back to the isoSurfaceSet_splited vector
		----------------------------------------------------------------------------------*/
		isoSurface->splitIndex = splitTime;
		std::cout << "This isoSurface contains " << splitTime << " regions." << std::endl;
		return true;
	}
}

void fileIO::_splitSingleSurfacePatch_flooding(QMeshPatch* isoSurface, int index) {
	int StopFlag = 0; 			QMeshNode* NeighboorNode;

	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);

		if (thisNode->splitIndex == index) {
			for (int i = 0; i < thisNode->GetEdgeNumber(); i++) {

				QMeshEdge* thisEdge = thisNode->GetEdgeRecordPtr(i + 1);
				if (thisNode == thisEdge->GetStartPoint()) NeighboorNode = thisEdge->GetEndPoint();
				else NeighboorNode = thisEdge->GetStartPoint();

				if (NeighboorNode->splitIndex == index) continue;
				else {
					NeighboorNode->splitIndex = index;
					StopFlag++;
				}
			}
		}
	}
	//cout << "This iteration adding node number = " << StopFlag << endl;

	if (StopFlag > 0) _splitSingleSurfacePatch_flooding(isoSurface, index);
}

void fileIO::_splitSingleSurfacePatch_splitmesh(
	QMeshPatch* isoSurface, std::vector<QMeshPatch*>& isoSurfaceSet_splited) {
	for (int iter = 1; iter < isoSurface->splitIndex + 1; iter++) {

		QMeshPatch* patch = new QMeshPatch;
		patch->splitIndex = iter; patch->SetIndexNo(isoSurface->GetIndexNo());

		//build nodeNum and nodeTable
		float* nodeTable; int nodeNum = 0; 	double pp[3];

		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex == iter) nodeNum++;
		}
		nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

		int index = 0;
		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex == iter) {
				thisNode->GetCoord3D(pp[0], pp[1], pp[2]);
				for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
				thisNode->splitPatchIndex = index; //VolumetoSurfaceIndex start from 0
				index++;
			}
		}

		//build faceNum and faceTable
		unsigned int* faceTable; int faceNum = 0;
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->splitIndex == iter) {
					face->splitIndex = iter; faceNum++; break;
				}
			}
		}
		faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);
		index = 0;
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			if (face->splitIndex == iter) {
				for (int i = 0; i < 3; i++)
					faceTable[(index + 1) * 3 - 1 - i] = face->GetNodeRecordPtr(i)->splitPatchIndex;
				index++;
			}
		}

		// principle stress field transfer
		std::vector<Eigen::Vector3d> principleStressSet(faceNum);
		index = 0;
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			if (face->splitIndex == iter) {
				principleStressSet[index] = face->principleStressDir;
				index++;
			}
		}

		patch->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

		//transfer back principle stress field to splited new mesh
		index = 0;
		for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
			face->principleStressDir = principleStressSet[index]; index++;
		}
		isoSurfaceSet_splited.push_back(patch);
	}
}

void fileIO::_outputSingleISOSurface(
	bool offMode, QMeshPatch* isoSurface, std::string path, bool split, bool support_split_final){

	string fieldPath = path + "Field/"; //field install, only for initial layer without support region.
	string fieldRenderPath = path + "layerFieldRender/";
	//if (isoSurface->includeSupportRegion) path += "support/"; //support layer

	if (split) {
		path += to_string(isoSurface->GetIndexNo() * 100 + isoSurface->splitIndex - 1);
		fieldPath += to_string(isoSurface->GetIndexNo() * 100 + isoSurface->splitIndex - 1);
		fieldRenderPath += to_string(isoSurface->GetIndexNo() * 100 + isoSurface->splitIndex - 1);
	}
	else if (support_split_final) path += isoSurface->layerName;
	else {
		path += to_string(isoSurface->GetIndexNo() * 100);
		fieldPath += to_string(isoSurface->GetIndexNo() * 100);
		fieldRenderPath += to_string(isoSurface->GetIndexNo() * 100);
	}

	double pp[3];

	if (isoSurface->includeSupportRegion && support_split_final == false) path += "S"; //support layer

	//----OBJ file output
	if (!offMode) {
		path += ".obj";
		ofstream nodeSelection(path);

		int index = 0;
		for (GLKPOSITION posNode = isoSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)isoSurface->GetNodeList().GetNext(posNode);
			node->GetCoord3D(pp);
			nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << endl;
			index++; node->SetIndexNo(index);
		}
		for (GLKPOSITION posFace = isoSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(posFace);
			nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
		}
		nodeSelection.close();
	}

	//----OFF file output
	else {
		path += "layer.off";
		ofstream nodeSelection(path);

		nodeSelection << "OFF" << std::endl;
		nodeSelection << isoSurface->GetNodeNumber() << " " << isoSurface->GetFaceNumber() << " 0" << std::endl;

		for (GLKPOSITION posNode = isoSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)isoSurface->GetNodeList().GetNext(posNode);
			node->GetCoord3D(pp);
			nodeSelection << pp[0] << " " << pp[1] << " " << pp[2] << endl;
		}
		for (GLKPOSITION posFace = isoSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(posFace);

			nodeSelection << "3 " << face->GetNodeRecordPtr(0)->GetIndexNo() - 1
				<< " " << face->GetNodeRecordPtr(1)->GetIndexNo() - 1
				<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() - 1 << endl;
		}
		nodeSelection.close();
	}
	if (isoSurface->includeSupportRegion) 
		std::cout << "-- Finish output #SUPPORT# isosurface to: " << path << std::endl;
	else {
		std::cout << "-- Finish output #MODEL# isosurface to: " << path << std::endl;

		//---- Output vector field of iso-surface for futher tool-path generation
		fieldPath += "field.txt";
		fieldRenderPath += "field.txt";
		this->_outputISOSurfaceToolpathStressField(isoSurface, fieldPath, fieldRenderPath);
	}
}

void fileIO::_outputISOSurfaceToolpathStressField(QMeshPatch* isosurface, string fieldPath, string fieldRenderPath) {

	ofstream isoSurfaceStressField(fieldPath);

	/*for (GLKPOSITION posFace = isosurface->GetNodeList().GetHeadPosition(); posFace != nullptr;) {
		QMeshNode* node = (QMeshNode*)isosurface->GetNodeList().GetNext(posFace);
		isoSurfaceStressField << node->guideFieldValue << endl;
	}*/
	for (GLKPOSITION posFace = isosurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)isosurface->GetFaceList().GetNext(posFace);
		isoSurfaceStressField << face->principleStressDir(0) << "," << face->principleStressDir(1) << "," << face->principleStressDir(2) << endl;
	}
	isoSurfaceStressField.close();

	std::cout << "------- Also output the principle stress direction to " << fieldPath << std::endl << std::endl;

	ofstream isoSurfaceStressField_rendering(fieldRenderPath);

	for (GLKPOSITION posFace = isosurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)isosurface->GetFaceList().GetNext(posFace);

		double xx, yy, zz; face->CalCenterPos(xx, yy, zz); double length = 0.0;
		if (face->principleStress < 0) length = log10(50.0 * face->principleStress / minPrincipleStressValue);
		else length = log10(50.0 * face->principleStress / maxPrincipleStressValue);
		if (length < 0.01) length = 0;

		if (abs(face->principleStressColor[0]) + abs(face->principleStressColor[1]) + abs(face->principleStressColor[2]) < 0.001) {
			for (int i = 0; i < 3; i++) face->principleStressColor[i] = 0.8;
			length = 0.2;
		}
		//if (face->isNIEface == false) std::cout << isosurface->GetIndexNo() << ", ---- " << face->principleStress << std::endl;

		isoSurfaceStressField_rendering << xx << "," << yy << "," << zz << "," << length << ","
			<< face->principleStressDir(0) << "," << face->principleStressDir(1) << "," << face->principleStressDir(2) << ","
			<< face->principleStressColor[0] << "," << face->principleStressColor[1] << "," << face->principleStressColor[2] << endl;

		//isoSurfaceStressField_rendering << face->principleStressDir(0) << "," << face->principleStressDir(1) << "," << face->principleStressDir(2) << endl;
	}
	isoSurfaceStressField_rendering.close();
	std::cout << "------- Also output the principle stress direction, color to " << fieldRenderPath << std::endl << std::endl;

}

void fileIO::saveFieldforRendering(PolygenMesh* initialModel) {
	//----------------build the folder to save vector and sacalr field file----------------//
	std::string folderPath = "..\\Model\\RenderFile\\" + initialModel->getModelName();
	string command;
	//command = "mkdir -p " + folderPath;
	command = "mkdir " + folderPath;
	system(command.c_str());

	QMeshPatch* tetModel = (QMeshPatch*)initialModel->GetMeshList().GetHead();

	//----------------output scalar field----------------//
	std::string scalarFieldPath = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_scalarField.txt";
	ofstream scalarField(scalarFieldPath);

	for (GLKPOSITION posNode = tetModel->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetModel->GetNodeList().GetNext(posNode);
		double pp[3];
		node->GetCoord3D(pp);
		float color[3];
		node->GetColor(color[0], color[1], color[2]);
		scalarField << pp[0] << "," << pp[1] << "," << pp[2] << "," << node->guideFieldValue << "," <<
			color[0] << "," << color[1] << "," << color[2] << endl;
	}

	scalarField.close();

	//----------------output vector field----------------//
	// this part also need to compute the sacalr field value to visuilize the vector color

	std::string vectorFieldPath = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_vectorField.txt";
	ofstream vectorField(vectorFieldPath);

	for (GLKPOSITION posNode = tetModel->GetTetraList().GetHeadPosition(); posNode != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetModel->GetTetraList().GetNext(posNode);
		double pp[3];
		tet->CalCenterPos(pp[0], pp[1], pp[2]);
		double tetScalarField = 0;
		for (int i = 0; i < 4; i++)
			tetScalarField += tet->GetNodeRecordPtr(i + 1)->guideFieldValue;

		vectorField << pp[0] << "," << pp[1] << "," << pp[2] << ","
			<< tet->vectorField(0) << "," << tet->vectorField(1) << "," << tet->vectorField(2) << "," << tetScalarField / 4 << std::endl;
	}

	vectorField.close();

	//----------------output principle stress field----------------//
	std::string stressFieldPath = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_principleStressField.txt";
	ofstream stressField(stressFieldPath);

	for (GLKPOSITION posNode = tetModel->GetTetraList().GetHeadPosition(); posNode != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetModel->GetTetraList().GetNext(posNode);
		double pp[3];
		tet->CalCenterPos(pp[0], pp[1], pp[2]);

		double length = 0.0;
		if (tet->sigma_max < 0) {
			//length = tet->principleStress / tetModel->minPrincipleStressValue;
			length = log10(50.0 * tet->sigma_max / tetModel->minPrincipleStressValue);
			if (length < 0.01) length = 0;
		}
		else {
			//length = tet->principleStress / tetModel->maxPrincipleStressValue;
			length = log10(50.0 * tet->sigma_max / tetModel->maxPrincipleStressValue);
			if (length < 0.01) length = 0;
		}

		stressField << pp[0] << "," << pp[1] << "," << pp[2] << "," << length << "," <<
			tet->tau_max(0) << "," << tet->tau_max(1) << "," << tet->tau_max(2) << "," <<
			tet->principleStressColor[0] << "," << tet->principleStressColor[1] << "," << tet->principleStressColor[2] << endl;
	}

	stressField.close();

	//----------------output obj file----------------//
	std::string objPath = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_boundaryOBJ.obj";

	ofstream obj_output(objPath);
	for (GLKPOSITION posNode = tetModel->GetFaceList().GetHeadPosition(); posNode != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetModel->GetFaceList().GetNext(posNode);
		face->tetInnerFace = true;
		if (face->GetLeftTetra() == false || face->GetRightTetra() == false) {
			face->tetInnerFace = false;
			for (int i = 0; i < 3; i++)
				face->GetNodeRecordPtr(i)->tetInnerNode = false;
		}
	}
	int index = 0; double pp[3];
	for (GLKPOSITION posNode = tetModel->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetModel->GetNodeList().GetNext(posNode);
		if (node->tetInnerNode) continue;
		index++; node->tetInnerNodeIndex = index;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		obj_output << "v " << pp[0] << " " << pp[1] << " " << pp[2] << endl;
	}
	for (GLKPOSITION posNode = tetModel->GetFaceList().GetHeadPosition(); posNode != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetModel->GetFaceList().GetNext(posNode);
		if (face->tetInnerFace) continue;
		obj_output << "f " << face->GetNodeRecordPtr(0)->tetInnerNodeIndex << " "
			<< face->GetNodeRecordPtr(1)->tetInnerNodeIndex << " " << face->GetNodeRecordPtr(2)->tetInnerNodeIndex << endl;
	}
	obj_output.close();
}

void fileIO::outputCollisionSurface(PolygenMesh* isosurfaceSet) {

	std::cout << "output collision surface" << std::endl;
	for (GLKPOSITION Pos = isosurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(Pos);

		bool layerCollision = false;
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			if (Node->isCollisionHappenNode) { layerCollision = true; break; }
		}

		if (layerCollision) {
			for (int iter = 0; iter < 3; iter++) {
				for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
					QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
					if (Node->isCollisionHappenNode) {
						for (GLKPOSITION Pos = Node->GetFaceList().GetHeadPosition(); Pos;) {
							QMeshFace* face = (QMeshFace*)Node->GetFaceList().GetNext(Pos);
							face->isCollisionOutput = true;
						}
					}
				}
				for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
					QMeshFace* face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
					if (face->isCollisionOutput) {
						for (int i = 0; i < 3; i++) {
							face->GetNodeRecordPtr(i)->isCollisionOutput = true;
							face->GetNodeRecordPtr(i)->isCollisionHappenNode = true;
						}
					}
				}
			}
			
			std::cout << "Output No." << layer->GetIndexNo() << std::endl;
			this->_outputSingleCollisionSurface(layer);
		}


	}
}

void fileIO::_outputSingleCollisionSurface(QMeshPatch* layer) {
	std::string objPath = "../Model/RenderFile/CollisionLayer/" + to_string(layer->GetIndexNo()) + ".obj";
	std::cout << objPath << std::endl;

	ofstream objLayer(objPath);

	int index = 0;
	for (GLKPOSITION posNode = layer->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)layer->GetNodeList().GetNext(posNode);
		if (node->isCollisionOutput == false) { node->SetIndexNo(-1); continue; }
		double pp[3]; node->GetCoord3D(pp);
		objLayer << "v " << pp[0] << " " << pp[1] << " " << pp[2] << endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posFace = layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)layer->GetFaceList().GetNext(posFace);
		if (face->isCollisionOutput == false) continue;
		objLayer << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
	}
	objLayer.close();
}

void fileIO::outputSupportNodeMesh(QMeshPatch* supportNodePatch, PolygenMesh* initTetMesh) {

	std::cout << "begin output support node mesh!" << std::endl;
	Eigen::MatrixXd sphereNodeTable(26, 3);
	sphereNodeTable << 0, 0.5, 0,
		0, 0.3536, -0.3536,
		-0.25, 0.3536, -0.25,
		-0.3536, 0.3536, 0,
		-0.25, 0.3536, 0.25,
		0, 0.3536, 0.3536,
		0.25, 0.3536, 0.25,
		0.3536, 0.3536, 0,
		0.25, 0.3536, -0.25,
		0, 0, -0.5,
		-0.3536, 0, -0.3536,
		-0.5, 0, 0,
		-0.3536, 0, 0.3536,
		0, 0, 0.5,
		0.3536, 0, 0.3536,
		0.5, 0, 0,
		0.3536, 0, -0.3536,
		0, -0.3536, -0.3536,
		-0.25, -0.3536, -0.25,
		-0.3536, -0.3536, 0,
		-0.25, -0.3536, 0.25,
		0, -0.3536, 0.3536,
		0.25, -0.3536, 0.25,
		0.3536, -0.3536, 0,
		0.25, -0.3536, -0.25,
		0, -0.5, 0;

	Eigen::MatrixXd sphereFaceTable(48, 3);
	sphereFaceTable << 0, 1, 2,
		0, 2, 3,
		0, 3, 4,
		0, 4, 5,
		0, 5, 6,
		0, 6, 7,
		0, 7, 8,
		1, 0, 8,
		1, 9, 10,
		2, 1, 10,
		2, 10, 11,
		3, 2, 11,
		3, 11, 12,
		4, 3, 12,
		4, 12, 13,
		5, 4, 13,
		5, 13, 14,
		6, 5, 14,
		6, 14, 15,
		7, 6, 15,
		7, 15, 16,
		8, 7, 16,
		9, 8, 16,
		1, 8, 9,
		9, 17, 18,
		10, 9, 18,
		10, 18, 19,
		11, 10, 19,
		11, 19, 20,
		12, 11, 20,
		12, 20, 21,
		13, 12, 21,
		13, 21, 22,
		14, 13, 22,
		14, 22, 23,
		15, 14, 23,
		15, 23, 24,
		16, 15, 24,
		17, 16, 24,
		9, 16, 17,
		18, 17, 25,
		19, 18, 25,
		20, 19, 25,
		21, 20, 25,
		22, 21, 25,
		23, 22, 25,
		24, 23, 25,
		17, 24, 25;

	std::string supportNodepath = "../Model/RenderFile/" + 
		initTetMesh->getModelName() + "/polygensupport_node.obj";
	ofstream supportNode_output(supportNodepath);
	double pp[3];


	Eigen::MatrixXd nodeTable(26 * supportNodePatch->GetNodeNumber(), 3);

	int nodeIndex = 0;
	for (GLKPOSITION posNode = supportNodePatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)supportNodePatch->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp);
		for (int i = 0; i < 26; i++) {
			nodeTable.row(26 * nodeIndex + i) = sphereNodeTable.row(i);
			for (int j = 0; j < 3; j++) nodeTable.row(26 * nodeIndex + i)(j) += pp[j];
		}
		nodeIndex++;
	}

	Eigen::MatrixXd faceTable(48 * supportNodePatch->GetNodeNumber(), 3);
	for (int sphereIndex = 0; sphereIndex < supportNodePatch->GetNodeNumber(); sphereIndex++) {
		for (int i = 0; i < 48; i++) {
			faceTable.row(48 * sphereIndex + i) = sphereFaceTable.row(i);
			for (int j = 0; j < 3; j++)
				faceTable.row(48 * sphereIndex + i)(j) += sphereIndex * 26 + 1;
		}
	}

	for (int i = 0; i < 26 * supportNodePatch->GetNodeNumber(); i++)
		supportNode_output << "v " << nodeTable(i, 0) << " " << nodeTable(i, 1) << " " << nodeTable(i, 2) << endl;

	for (int i = 0; i < 48 * supportNodePatch->GetNodeNumber(); i++)
		supportNode_output << "f " << std::setprecision(8) << faceTable(i, 0) << " " << 
		setprecision(8) << faceTable(i, 1) << " " << setprecision(8) << faceTable(i, 2) << endl;


	supportNode_output.close();

	std::string supportPointPath = "../Model/RenderFile/" +
		initTetMesh->getModelName() + "/polygensupportNode.xyz";
	ofstream supportPoint(supportPointPath);

	for (GLKPOSITION posNode = supportNodePatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)supportNodePatch->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp);
		supportPoint << pp[0] << " " << pp[1] << " " << pp[2] << " " << std::endl;
	}
	QMeshPatch* initTet = (QMeshPatch*)initTetMesh->GetMeshList().GetHead();
	for (GLKPOSITION posNode = initTet->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)initTet->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp);
		supportPoint << pp[0] << " " << pp[1] << " " << pp[2] << " " << std::endl;
	}
	supportPoint.close();

}

void fileIO::outputToolpathRenderingFile(QMeshPatch* selectedSurface, bool isBoundaryField) {
	std::string boundaryField;
	if(isBoundaryField)
		boundaryField = "../Model/Surface_Mesh/isoSurfaceFile_boundaryField.txt";
	else 	boundaryField = "../Model/Surface_Mesh/isoSurfaceFile_zigzagField.txt";

	ofstream bField(boundaryField);

	float rr, gg, bb;
	for (GLKPOSITION posNode = selectedSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)selectedSurface->GetNodeList().GetNext(posNode);
		double pp[3]; node->GetCoord3D(pp);
		if(isBoundaryField)
			this->_changeValueToColor(1.0, 0.0, node->boundaryValue, rr, gg, bb);
		else this->_changeValueToColor(1.0, 0.0, node->zigzagValue, rr, gg, bb);

		bField << rr << " " << gg << " " << bb << std::endl;
	}
	bField.close();

	if (isBoundaryField == false) return;

	std::string objRefineField = "../Model/Surface_Mesh/refinement.obj";
	ofstream nodeSelection(objRefineField);

	int index = 0;
	for (GLKPOSITION posNode = selectedSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)selectedSurface->GetNodeList().GetNext(posNode);
		double pp[3]; node->GetCoord3D(pp);
		nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posFace = selectedSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)selectedSurface->GetFaceList().GetNext(posFace);
		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
	}
	nodeSelection.close();
}

void fileIO::outputToolpathRenderingFile_field(QMeshPatch* selectedSurface) {
	std::string vectorField;
	vectorField = "../Model/Surface_Mesh/isoSurfaceFile_vectorField.txt";

	ofstream vField(vectorField);

	for (GLKPOSITION posNode = selectedSurface->GetFaceList().GetHeadPosition(); posNode != nullptr;) {
		QMeshFace* face = (QMeshFace*)selectedSurface->GetFaceList().GetNext(posNode);
		double pp[3];  face->CalCenterPos(pp[0], pp[1], pp[2]);
		Eigen::Vector3d faceNorm; double D;
		face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
		Eigen::Vector3d vectordir = face->principleStressDir.cross(faceNorm);
		double zvalue = 0;
		for (int i = 0; i < 3; i++) zvalue += face->GetNodeRecordPtr(i)->zigzagValue / 3;
		vField << pp[0] << " " << pp[1] << " " << pp[2] << " " << vectordir(0) 
			<< " " << vectordir(1) << " " << vectordir(2) << " " << zvalue << std::endl;
	}
	vField.close();
}



void fileIO::_changeValueToColor(double maxValue, double minValue, double Value,
	float& nRed, float& nGreen, float& nBlue)
{
	//	Value=fabs(Value);

	if (Value <= minValue) { nRed = 0.0; nGreen = 0.0; nBlue = 0.0; return; } // black

	if ((maxValue - minValue) < 0.000000000001) { nRed = 0.0; nGreen = 0.0; nBlue = 1.0; return; } // blue

	double temp = (Value - minValue) / (maxValue - minValue);

	if (temp > 0.75)
	{
		nRed = 1;
		nGreen = (float)(1.0 - (temp - 0.75) / 0.25);
		if (nGreen < 0) nGreen = 0.0f;
		nBlue = 0;
		return;
	}
	if (temp > 0.5)
	{
		nRed = (float)((temp - 0.5) / 0.25);
		nGreen = 1;
		nBlue = 0;
		return;
	}
	if (temp > 0.25)
	{
		nRed = 0;
		nGreen = 1;
		nBlue = (float)(1.0 - (temp - 0.25) / 0.25);
		return;
	}
	else
	{
		nRed = 0;
		nGreen = (float)(temp / 0.25);
		nBlue = 1;
	}
}

