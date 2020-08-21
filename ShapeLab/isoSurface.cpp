#include "stdafx.h"
#include "isoSurface.h"
#include "GLKMatrixLib.h"
#include <omp.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "GLKGeometry.h"

#include "../Library/PQPLib/PQP.h"

using namespace std;
using namespace Eigen;

isoSurface::isoSurface(QMeshPatch* Patch) { tetMesh = Patch; }

isoSurface::~isoSurface() { }

void isoSurface::generateIsoSurface(PolygenMesh* isoSurface, int layerNum) {

	/* Direct substract iso-surface from tetrahedral mesh*/

	for (int i = 0; i < layerNum; i++) {

		double isoCurveValue = (0.5 + i) * 1 / (double)layerNum;
		QMeshPatch* layer = generatesingleIsoSurface(isoCurveValue, isoSurface, false);

		if (layer->GetNodeNumber() == 0) {
			cout << "this layer have no node!" << endl; continue;
		}

		isoSurface->meshList.AddTail(layer);
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount()); //index begin from 0
		cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue << ", nodeNum = " << layer->GetNodeNumber() << endl;
	
	}

}

void isoSurface::generateIsoSurface_supportStructure(PolygenMesh* isoSurface, int layerNum) {

	//iso-value range is [0,1], build vector of scalar field value
	Eigen::VectorXd guideFieldSupport(tetMesh->GetNodeNumber());

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		guideFieldSupport(Node->GetIndexNo()) = Node->guideFieldValue;
	}

	// compute max and min phis 
	double minPhi = 99999.999;
	double maxPhi = -99999.999;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++) {
		if (minPhi > guideFieldSupport(i)) minPhi = guideFieldSupport(i);
		if (maxPhi < guideFieldSupport(i)) maxPhi = guideFieldSupport(i);
	}

	double gap = 1.0 / layerNum;
	int minLayerNum = (int)((0.5*gap - minPhi) / gap);
	int maxLayerNum = (int)((maxPhi - (1 - 0.5*gap)) / gap);
	int allLayerNum = layerNum + minLayerNum + maxLayerNum;

	std::cout << "[min, max] = " << minPhi << "," << maxPhi << std::endl;
	std::cout << "[minLayerNum, maxLayerNum, allLayerNum] = "
		<< minLayerNum << "," << maxLayerNum << "," << allLayerNum << std::endl;

	//multi-layer
	for (int i = 0; i < allLayerNum; i++) {

		double isoCurveValue = (0.5 - minLayerNum + i) * gap;
		QMeshPatch* layer = generatesingleIsoSurface(isoCurveValue, isoSurface, true);

		if (layer->GetNodeNumber() == 0) {
			cout << "this layer have no node!" << endl; continue;
		}

		isoSurface->meshList.AddTail(layer);
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount()); //index begin from 0
		layer->drawThisIsoLayer = true;

		cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue 
			<< ", nodeNum = " << layer->GetNodeNumber() << endl;
	}

}

void isoSurface::generateIsoSurface_fromVoxelLayer(PolygenMesh* isoSurface, int layerNum) {
	for (int i = 0; i < layerNum; i++) {
		double isoCurveValue = 0.5 + i;
		QMeshPatch* layer = generatesingleIsoSurface(isoCurveValue, isoSurface, false);

		if (layer->GetNodeNumber() == 0) continue;

		isoSurface->meshList.AddTail(layer);
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount()); //index begin from 0
		cout << layer->GetIndexNo() << " Layer, isoValue = " << 9.99 << ", nodeNum = " << layer->GetNodeNumber() << endl;


	}
	//single-layer
	//generatesingleIsoSurface(5, isoSurface);
}

void isoSurface::splitSupportandModelSurface(PolygenMesh* isoSurface_support) {

	if (isoSurface_support->meshType != SUPPORT_LAYERS) return;

	std::vector<QMeshPatch *> splitMeshPatchSet(2* isoSurface_support->GetMeshList().GetCount());

	int layerIndex = 0; int singleLayerIndex = 0;
	for (GLKPOSITION posMesh = isoSurface_support->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *singleLayer = (QMeshPatch*)isoSurface_support->GetMeshList().GetNext(posMesh);
		singleLayerIndex++;

		bool layerFullSupport = true;
		for (GLKPOSITION Pos = singleLayer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace *face = (QMeshFace*)singleLayer->GetFaceList().GetNext(Pos);
			if (face->printLayerSupportFace == false) {
				layerFullSupport = false; break;
			}
		}

		if (layerFullSupport == true) {
			splitMeshPatchSet[layerIndex] = singleLayer; 
			singleLayer->includeSupportRegion = true;
			layerIndex += 1;
		}

		// ### Normally there is no such case this layer is without support region (03-15-2020)

		else {

			//seperate the body layer
			QMeshPatch *bodyLayer = new QMeshPatch;
			bodyLayer->isoSurfaceValue = singleLayer->isoSurfaceValue;
			bodyLayer->includeSupportRegion = false;

			this->splitSingleLayer(singleLayer, bodyLayer, false);

			//seperate the support layer
			QMeshPatch *supportLayer = new QMeshPatch;
			supportLayer->isoSurfaceValue = singleLayer->isoSurfaceValue;
			supportLayer->includeSupportRegion = true;

			this->splitSingleLayer(singleLayer, supportLayer, true);

			if (singleLayer->isInnerSlcingLayer == false) {

				// This 
				if (singleLayerIndex % 2 == 0) {
					splitMeshPatchSet[layerIndex] = bodyLayer;
					splitMeshPatchSet[layerIndex + 1] = supportLayer;
				}
				else {
					splitMeshPatchSet[layerIndex] = supportLayer;
					splitMeshPatchSet[layerIndex + 1] = bodyLayer;
				}

				layerIndex += 2;
			}
			else {
				splitMeshPatchSet[layerIndex] = supportLayer;
				supportLayer->isInnerSlcingLayer = true;
				layerIndex += 1;
			}

			singleLayer->ClearAll();
			
			std::cout << "finish split No. " << singleLayer->GetIndexNo() << " layer!" << std::endl;
			
		}


	}
	splitMeshPatchSet.resize(layerIndex);
	isoSurface_support->GetMeshList().RemoveAll();
	for (int i = 0; i < layerIndex; i++) {
		isoSurface_support->meshList.AddTail(splitMeshPatchSet[i]);
		splitMeshPatchSet[i]->SetIndexNo(isoSurface_support->GetMeshList().GetCount() + 1); //index begin from 1
		splitMeshPatchSet[i]->drawThisIsoLayer = true;
	}

}

void isoSurface::splitSingleLayer(QMeshPatch* initLayer, QMeshPatch* splitLayer, bool isSupport) {

	//build faceNum and faceTable

	for (GLKPOSITION Pos = initLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)initLayer->GetNodeList().GetNext(Pos);
		Node->printLayerSupportNode = (!isSupport);
		Node->splitSurfaceIndex = -1;
	}

	int faceNum = 0;
	for (GLKPOSITION Pos = initLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)initLayer->GetFaceList().GetNext(Pos);
		if (face->printLayerSupportFace == isSupport) {
			for (int i = 0; i < 3; i++) face->GetNodeRecordPtr(i)->printLayerSupportNode = isSupport;
			faceNum++;
		}
	}
	unsigned int* faceTable;
	faceTable = (unsigned int *)malloc(sizeof(unsigned int)*faceNum * 3);

	//build nodeNum and nodeTable
	int nodeNum = 0;
	for (GLKPOSITION Pos = initLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)initLayer->GetNodeList().GetNext(Pos);
		if (Node->printLayerSupportNode == isSupport) nodeNum++;
	}
	float* nodeTable;
	nodeTable = (float *)malloc(sizeof(float)*nodeNum * 3);

	int index = 0;
	for (GLKPOSITION Pos = initLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)initLayer->GetNodeList().GetNext(Pos);
		if (Node->printLayerSupportNode == isSupport) {
			double pp[3];
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
			Node->splitSurfaceIndex = index; //VolumetoSurfaceIndex start from 0
			index++;
		}
	}

	index = 0;
	for (GLKPOSITION Pos = initLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)initLayer->GetFaceList().GetNext(Pos);
		if (face->printLayerSupportFace == isSupport) {
			for (int i = 0; i < 3; i++)
				faceTable[(index + 1) * 3 - 1 - i] = face->GetNodeRecordPtr(i)->splitSurfaceIndex;
			index++;
		}
	}

	splitLayer->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	free(nodeTable);
	free(faceTable);
}



bool isoSurface::checkIsoSurfaceDistance(PolygenMesh* isoSurface) {

	std::vector<double> newSurfaceValue; std::vector<QMeshPatch *> lastLayer;
	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *bLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		if (bLayer == (QMeshPatch*)isoSurface->GetMeshList().GetTail()) break;

		GLKPOSITION PosNext = isoSurface->GetMeshList().Find(bLayer)->next;
		QMeshPatch* tLayer = (QMeshPatch*)isoSurface->GetMeshList().GetAt(PosNext);

		if (checkTwoLayerDistance(bLayer, tLayer)) {
			std::cout << "The layer distance of No. " << bLayer->GetIndexNo() << "is higher than given number." << std::endl;
			double newIsoCurveValue = (bLayer->isoSurfaceValue + tLayer->isoSurfaceValue) / 2;
			newSurfaceValue.push_back(newIsoCurveValue);
			lastLayer.push_back(bLayer);
		}
	}

	if (newSurfaceValue.size() == 0) return false;
	else
	{
		//build new iso surface
		for (int i = 0; i < newSurfaceValue.size(); i++) {
			QMeshPatch* layer = generatesingleIsoSurface(newSurfaceValue[i], isoSurface, false);

			GLKPOSITION Pos = isoSurface->meshList.Find(lastLayer[i]);
			isoSurface->meshList.InsertAfter(Pos, layer);
		}

		int layerIndex = 0;
		for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch *layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
			layer->SetIndexNo(layerIndex); layerIndex++;
		}

		return true;
	}
}


bool isoSurface::checkIsoSurfaceDistanceAndBuildNewLayer_supportStructure(PolygenMesh* isoSurface) {

	std::cout << "begin layer thickness checking!" << std::endl;

	std::vector<double> newSurfaceValue; std::vector<QMeshPatch*> lastLayer;
	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* bLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		if (bLayer == (QMeshPatch*)isoSurface->GetMeshList().GetTail()) break;

		GLKPOSITION PosNext = isoSurface->GetMeshList().Find(bLayer)->next;
		QMeshPatch* tLayer = (QMeshPatch*)isoSurface->GetMeshList().GetAt(PosNext);

		if (checkTwoLayerDistance_supportStructureLayerSubstraction(bLayer, tLayer)) {
			std::cout << "The layer distance of No. " << bLayer->GetIndexNo() << "is higher than given number." << std::endl;
			double newIsoCurveValue = (bLayer->isoSurfaceValue + tLayer->isoSurfaceValue) / 2;
			newSurfaceValue.push_back(newIsoCurveValue);
			lastLayer.push_back(bLayer);
		}
	}

	if (newSurfaceValue.size() == 0) return false;
	else
	{
		//build new iso surface
		for (int i = 0; i < newSurfaceValue.size(); i++) {
			QMeshPatch* innerLayer = generatesingleIsoSurface(newSurfaceValue[i], isoSurface, true);
			innerLayer->isInnerSlcingLayer = true;
			GLKPOSITION Pos = isoSurface->meshList.Find(lastLayer[i]);
			isoSurface->meshList.InsertAfter(Pos, innerLayer);
		}

		int layerIndex = 0;
		for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
			layer->SetIndexNo(layerIndex); layerIndex++;
		}

		return true;
	}
}


void isoSurface::computeLayerThicknessForNodeAndDraw(PolygenMesh* layerSetMesh, bool isSupportLayer) {
	

	std::vector<QMeshPatch*> layerSet;
	for (GLKPOSITION posMesh = layerSetMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)layerSetMesh->GetMeshList().GetNext(posMesh);
		if (isSupportLayer == false) layerSet.push_back(layer);
		else { if (layer->includeSupportRegion) layerSet.push_back(layer); }
	}
	
	int NodeNum = 0;
	for (int i = 0; i < layerSet.size() - 1; i++) {
		this->checkTwoLayerDistance(layerSet[i], layerSet[i + 1]);
		NodeNum += layerSet[i + 1]->GetNodeNumber();
	}

	Eigen::VectorXd pointDis; pointDis.resize(NodeNum); 
	int nodeIndex = 0;
	/* ---- start from the second layer ---- */
	for (int i = 1; i < layerSet.size(); i++) {
		for (GLKPOSITION Pos = layerSet[i]->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layerSet[i]->GetNodeList().GetNext(Pos);
			pointDis(nodeIndex) = Node->layerThickDistance; nodeIndex++;
		}
		layerSet[i]->drawLayerThickness = true;
	}

	layerSetMesh->maxLayerThickness = pointDis.maxCoeff();
	layerSetMesh->minLayerThickness = pointDis.minCoeff();

	if (isSupportLayer == false) std::cout << " ---- INITAL LAYER LAYERHIGHT ---- " << std::endl;
	else std::cout << " ---- SUPPORT LAYER LAYERHIGHT ---- " << std::endl;

	std::cout << "Layer Thickness Range = [ " << layerSetMesh->minLayerThickness <<
		" " << layerSetMesh->maxLayerThickness << " ]" << std::endl;
	std::cout << " --------------------------------- " << std::endl << std::endl;





	//compute the layer distance and save to node
	//for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
	//	QMeshPatch *bLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

	//	if (bLayer == (QMeshPatch*)isoSurface->GetMeshList().GetTail()) break;

	//	GLKPOSITION PosNext = isoSurface->GetMeshList().Find(bLayer)->next;
	//	QMeshPatch* tLayer = (QMeshPatch*)isoSurface->GetMeshList().GetAt(PosNext);

	//	checkTwoLayerDistance(bLayer, tLayer);
	//	NodeNum += tLayer->GetNodeNumber();
	//}

	//build the vector that install all the node thickness
	//Eigen::VectorXd pointDis;
	//pointDis.resize(NodeNum); NodeNum = 0;
	//for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
	//	QMeshPatch *layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
	//	if (layer == (QMeshPatch*)isoSurface->GetMeshList().GetHead()) continue;
	//	int index = 0;
	//	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
	//		pointDis(index + NodeNum) = Node->layerThickDistance;
	//		index++;
	//	}
	//	NodeNum += layer->GetNodeNumber();
	//	layer->drawLayerThickness = true;
	//}

	/*isoSurface->maxLayerThickness = pointDis.maxCoeff();
	isoSurface->minLayerThickness = pointDis.minCoeff();

	std::cout << "Layer Thickness Range = [ " << isoSurface->minLayerThickness <<
		" " << isoSurface->maxLayerThickness << " ]" << std::endl;*/

}

void isoSurface::deleteCloseIsoSurface(PolygenMesh* isoSurface) {
	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
		if (layer == (QMeshPatch*)isoSurface->GetMeshList().GetHead()) continue;

		vector<QMeshPatch*> bLayer;
		int layerNum = 10;
		//--detect the number of bottom layer
		if (layer->GetIndexNo() < layerNum + 1)
		{
			GLKPOSITION Pos = isoSurface->GetMeshList().Find(layer)->prev;
			bLayer.resize(layer->GetIndexNo());
			for (int i = 0; i < layer->GetIndexNo(); i++) {
				bLayer[i] = (QMeshPatch*)isoSurface->GetMeshList().GetPrev(Pos);
			}
		}
		else {
			bLayer.resize(layerNum);
			GLKPOSITION Pos = isoSurface->GetMeshList().Find(layer)->prev;
			for (int i = 0; i < layerNum; i++) {
				bLayer[i] = (QMeshPatch*)isoSurface->GetMeshList().GetPrev(Pos);
			}
		}

		//--build PQP model
		vector<PQP_Model*> bLayerPQP;
		bLayerPQP.resize(bLayer.size());
		for (int i = 0; i < bLayer.size(); i++) {
			if (bLayer[i]->GetNodeNumber() < 3) continue;
			// build PQP model for bottom layer
			PQP_Model* pqpModel = new PQP_Model();
			pqpModel->BeginModel();  int index = 0;
			PQP_REAL p1[3], p2[3], p3[3];

			for (GLKPOSITION Pos = bLayer[i]->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)bLayer[i]->GetFaceList().GetNext(Pos);

				Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
				Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
				Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

				pqpModel->AddTri(p1, p2, p3, index);
				index++;

			}
			pqpModel->EndModel();
			bLayerPQP[i] = pqpModel;
		}

		bool reconstruct = false;
		double pp[3];
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D(pp[0], pp[1], pp[2]);

			double minDist = 99999.99;
			for (int i = 0; i < bLayer.size(); i++) {
				if (bLayer[i]->GetNodeNumber() < 3) continue;

				PQP_DistanceResult dres; dres.last_tri = bLayerPQP[i]->last_tri;
				PQP_REAL p[3];
				p[0] = pp[0]; p[1] = pp[1]; p[2] = pp[2];
				PQP_Distance(&dres, bLayerPQP[i], p, 0.0, 0.0);

				double Dist = dres.Distance(); // distance of this layer
				if (minDist > Dist) minDist = Dist;
			}
			
			Node->isosurfaceDelete = false;

			if (minDist < minLayerThickness) {
				//deleteNodeinPatch(Node, tLayer);
				//std::cout << "delete this node" << std::endl;
				//break;
				reconstruct = true;
				Node->isosurfaceDelete = true;
			}
		}

		for (int i = 0; i < bLayer.size(); i++) delete bLayerPQP[i];

		if (reconstruct) {
			cout << "reconstruct No. " << layer->GetIndexNo() << " layer." << endl;
			reconstructSurface(layer); 
		}
	}
}

void isoSurface::deleteCloseIsoSurface_supportLayer(PolygenMesh* isoSurface) {
	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
		if (layer == (QMeshPatch*)isoSurface->GetMeshList().GetHead()) continue;

		vector<QMeshPatch*> bLayer;
		int layerNum = 2;
		//--detect the number of bottom layer
		if (layer->GetIndexNo() < layerNum + 1)
		{
			GLKPOSITION Pos = isoSurface->GetMeshList().Find(layer)->prev;
			bLayer.resize(layer->GetIndexNo());
			for (int i = 0; i < layer->GetIndexNo(); i++) {
				bLayer[i] = (QMeshPatch*)isoSurface->GetMeshList().GetPrev(Pos);
			}
		}
		else {
			bLayer.resize(layerNum);
			GLKPOSITION Pos = isoSurface->GetMeshList().Find(layer)->prev;
			for (int i = 0; i < layerNum; i++) {
				bLayer[i] = (QMeshPatch*)isoSurface->GetMeshList().GetPrev(Pos);
			}
		}

		//--build PQP model
		vector<PQP_Model*> bLayerPQP;
		bLayerPQP.resize(bLayer.size());
		for (int i = 0; i < bLayer.size(); i++) {
			if (bLayer[i]->GetNodeNumber() < 3) continue;
			// build PQP model for bottom layer
			PQP_Model* pqpModel = new PQP_Model();
			pqpModel->BeginModel();  int index = 0;
			PQP_REAL p1[3], p2[3], p3[3];

			for (GLKPOSITION Pos = bLayer[i]->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)bLayer[i]->GetFaceList().GetNext(Pos);

				Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
				Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
				Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

				pqpModel->AddTri(p1, p2, p3, index);
				index++;

			}
			pqpModel->EndModel();
			bLayerPQP[i] = pqpModel;
		}

		bool reconstruct = true;
		double pp[3];
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D(pp[0], pp[1], pp[2]);

			double minDist = 99999.99;
			for (int i = 0; i < bLayer.size(); i++) {
				if (bLayer[i]->GetNodeNumber() < 3) continue;

				PQP_DistanceResult dres; dres.last_tri = bLayerPQP[i]->last_tri;
				PQP_REAL p[3];
				p[0] = pp[0]; p[1] = pp[1]; p[2] = pp[2];
				PQP_Distance(&dres, bLayerPQP[i], p, 0.0, 0.0);

				double Dist = dres.Distance(); // distance of this layer
				if (minDist > Dist) minDist = Dist;
			}

			Node->isosurfaceDelete = false;

			if (minDist < minLayerThickness) {
				//deleteNodeinPatch(Node, tLayer);
				//std::cout << "delete this node" << std::endl;
				//break;
				reconstruct = true;
				Node->isosurfaceDelete = true;
			}
			if (Node->printLayerSupportNode == false) Node->isosurfaceDelete = true;

		}

		for (int i = 0; i < bLayer.size(); i++) delete bLayerPQP[i];

		if (reconstruct) {
			cout << "reconstruct No. " << layer->GetIndexNo() << " layer." << endl;
			reconstructSurface(layer);
		}
	}
}


void isoSurface::reconstructSurface(QMeshPatch* layer) {

	Eigen::MatrixXd nodePos;
	Eigen::MatrixXd faceIndex;

	int nodeNum = 0;
	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
		if (Node->isosurfaceDelete) Node->SetIndexNo(-1);
		else { Node->SetIndexNo(nodeNum); nodeNum++; }
	}

	int faceNum = 0;
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		Face->isoFaceDelete = false;
		for (int i = 0; i < 3; i++) {
			if (Face->GetNodeRecordPtr(i)->isosurfaceDelete) {
				Face->isoFaceDelete = true;
				break;
			}
		}
		if (!Face->isoFaceDelete) faceNum++;
	}
	

	if (nodeNum < 5 || faceNum < 5) {
		layer->ClearAll();
		return;
	}

	float* nodeTable;
	nodeTable = (float *)malloc(sizeof(float)*nodeNum * 3);
	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo() < 0) continue;

		double pp[3];
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) nodeTable[Node->GetIndexNo() * 3 + i] = (float)pp[i];
	}

	int index = 0;
	unsigned int* faceTable;
	faceTable = (unsigned int *)malloc(sizeof(unsigned int)*faceNum * 3);
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		if (face->isoFaceDelete) continue;

		for (int i = 0; i < 3; i++)
			faceTable[(index + 1) * 3 - 1 - i] = face->GetNodeRecordPtr(i)->GetIndexNo();
		index++;
	
	}
	layer->ClearAll();
	layer->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	std::cout << "Finish delete node and surface, rest " << index << " Node and " << faceNum << " Face" << endl;
}

void isoSurface::deleteNodeinPatch(QMeshNode* thisNode, QMeshPatch* layer) {
	
	layer->GetNodeList().Remove(thisNode);

	for (int i = 0; i < thisNode->GetFaceNumber(); i++) {
		QMeshFace* Face = thisNode->GetFaceRecordPtr(i+1);
		layer->GetFaceList().Remove(Face);

		for (int j = 0; j < 3; j++) {
			QMeshEdge* Edge = Face->GetEdgeRecordPtr(j + 1);
			if (thisNode == Edge->GetStartPoint() || thisNode == Edge->GetEndPoint())
			{
				QMeshNode* neighborNode = Edge->GetStartPoint();
				if (thisNode == neighborNode) neighborNode = Edge->GetEndPoint();

				neighborNode->GetEdgeList().Remove(Edge);
				neighborNode->GetFaceList().Remove(Face);

			}
			else {
				if (Edge->pLeftFace == Face) Edge->pLeftFace == nullptr;
				else Edge->pRightFace == nullptr;
			}
		}
		delete Face;
	}

	for (int i = 0; i < thisNode->GetEdgeNumber(); i++) {
		QMeshEdge* Edge = thisNode->GetEdgeRecordPtr(i + 1);

		layer->GetEdgeList().Remove(Edge);
		delete Edge;
	}

	delete thisNode;
}


void isoSurface::smoothingIsoSurface(PolygenMesh* isoSurface) {

	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *isoLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge *thisEdge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);
			if (thisEdge->IsBoundaryEdge()) {
				thisEdge->GetStartPoint()->isoSurfaceBoundary = true;
				thisEdge->GetEndPoint()->isoSurfaceBoundary = true;
			}
		}

		//laplacian smoothness
		for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *thisNode = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

			if (thisNode->isoSurfaceBoundary) continue;
			else {
				double pp[3] = { 0 }; int neighNum = 0;
				for (GLKPOSITION Pos = thisNode->GetEdgeList().GetHeadPosition(); Pos;) {
					QMeshEdge *neighEdge = (QMeshEdge*)thisNode->GetEdgeList().GetNext(Pos);

					QMeshNode* neighNode = neighEdge->GetStartPoint();
					if (neighNode == thisNode) neighNode = neighEdge->GetEndPoint();

					double p1[3];
					neighNode->GetCoord3D(p1[0], p1[1], p1[2]);

					for (int i = 0; i < 3; i++) pp[i] += p1[i];
					neighNum++;
				}
				for (int i = 0; i < 3; i++) pp[i] /= neighNum;
				thisNode->SetCoord3D(pp[0], pp[1], pp[2]);
			}
		}

	}
}

bool isoSurface::checkTwoLayerDistance(QMeshPatch* bLayer, QMeshPatch* tLayer) {
	bool isLarge = false; 
	double distance = maxLayerThickness;

	// build PQP model for bottom layer
	PQP_Model* pqpModel = new PQP_Model();
	pqpModel->BeginModel();  int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];

	for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel->EndModel();

	// compute minimal distance for every node at top layer
	double pp[3];
	for (GLKPOSITION Pos = tLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tLayer->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);

		PQP_DistanceResult dres; dres.last_tri = pqpModel->last_tri;
		PQP_REAL p[3];
		p[0] = pp[0]; p[1] = pp[1]; p[2] = pp[2];
		PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);

		float minDist = dres.Distance(); // minimal distance
		Node->layerThickDistance = minDist; //save the distance to node

		if (minDist > distance) {
			isLarge = true; //break;
		}
	}

	delete pqpModel;
	return isLarge;
}

bool isoSurface::checkTwoLayerDistance_supportStructureLayerSubstraction(QMeshPatch* bLayer, QMeshPatch* tLayer) {
	bool isLarge = false;
	double distance = maxLayerThickness;

	// build PQP model for bottom layer
	PQP_Model* pqpModel = new PQP_Model();
	pqpModel->BeginModel();  int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];

	for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel->EndModel();

	// compute minimal distance for every node at top layer
	double pp[3];
	for (GLKPOSITION Pos = tLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tLayer->GetNodeList().GetNext(Pos);

		if (Node->printLayerSupportNode == false) continue;	
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < 5.0) continue;

		PQP_DistanceResult dres; dres.last_tri = pqpModel->last_tri;
		PQP_REAL p[3];
		p[0] = pp[0]; p[1] = pp[1]; p[2] = pp[2];
		PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);

		float minDist = dres.Distance(); // minimal distance
		Node->layerThickDistance = minDist; //save the distance to node

		if (minDist > distance) {
			isLarge = true; //break;
		}
	}

	delete pqpModel;
	return isLarge;
}

void isoSurface::planeCutSurfaceMesh(QMeshPatch* surfaceMesh, QMeshFace* cutPlane) {

	int cutFaceCase[3][9] = {
		{1,4,5,1,2,4,4,3,5},
		{1,4,5,4,2,5,2,3,5},
		{1,4,5,4,2,5,1,5,3} };
	
	/*normal representation for given plane, use mesh center as the initial point*/

	//Eigen::Vector3d meshCenter = Eigen::Vector3d::Zero();
	//int nodeIndex = 0;
	//for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
	//	double pp[3]; Node->GetCoord3D(pp);
	//	Node->SetIndexNo(nodeIndex); nodeIndex++; //node index start from 0;
	//	for (int i = 0; i < 3; i++) meshCenter(i) += pp[i];
	//}
	//for (int i = 0; i < 3; i++) meshCenter(i) /= surfaceMesh->GetNodeNumber();

	//Vector3d PlaneDir; PlaneDir << 1.0, 0.0, 0.0;
	//double D = 0.0;
	//for (int i = 0; i < 3; i++) D -= meshCenter(i) * PlaneDir(i);

	Vector3d PlaneDir; double D = 0.0;
	cutPlane->CalPlaneEquation(PlaneDir(0), PlaneDir(1), PlaneDir(2), D);
	std::cout << PlaneDir << "," << D << std::endl;

	/*Plane equation: Ax+By+Cz+D = 0, detect the intersect edge, build intersect node*/
	Vector3d pointPos; int node_Index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(node_Index); node_Index++;
		Vector3d pp; Node->GetCoord3D(pp(0), pp(1), pp(2));
		Node->nodePlaneDis = pp.dot(PlaneDir) + D;
		//std::cout << Node->nodePlaneDis << std::endl;
	}

	int cutEdgeNum = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *thisEdge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
		double a = thisEdge->GetStartPoint()->nodePlaneDis;
		double b = thisEdge->GetEndPoint()->nodePlaneDis;

		if (a*b > 0) continue;

		cutEdgeNum++;
		double alpha = fabs(a) / fabs(b - a);
		double p1[3], p2[3], pp[3];
		thisEdge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
		thisEdge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

		for (int j = 0; j < 3; j++)
			pp[j] = (1.0 - alpha)*p1[j] + alpha*p2[j];

		QMeshNode* isoNode = new QMeshNode;
		isoNode->SetMeshPatchPtr(surfaceMesh);
		isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
		isoNode->SetIndexNo(surfaceMesh->GetNodeList().GetCount()); // index start from 0
		surfaceMesh->GetNodeList().AddTail(isoNode);
		isoNode->planeCutNewNode = true;

		thisEdge->intersectNodeIndex = isoNode->GetIndexNo(); // if exist, the node index will large than 0
	}
	std::cout << "finish generate the cutting node" << std::endl;

	if (cutEdgeNum == 0) {
		printf("non-cut plane!\n"); return;
	}

	///* Build topology */

	// -- node table
	float* nodeTable;
	nodeTable = (float *)malloc(sizeof(float)*surfaceMesh->GetNodeNumber() * 3);

	int index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);

		double pp[3]; Node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
		index++;
		//if (index > surfaceMesh->GetNodeNumber() - cutEdgeNum) Node->selected = true;
	}
	
	// -- face table
	int intersectFaceNum = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
		for (int i = 0; i < 3; i++) {
			if (Face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex > 0) {
				intersectFaceNum++; break; }
		}
	}

	unsigned int* faceTable;
	faceTable = (unsigned int *)malloc(sizeof(unsigned int)*(surfaceMesh->GetFaceNumber() + 2 * intersectFaceNum) * 3);

	int faceNodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		bool isCutFace = false;
		for (int i = 0; i < 3; i++) {
			if (face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex > 0) {
				isCutFace = true; break;
			}
		}

		//-- non cut face
		if (isCutFace == false) {
			for (int i = 0; i < 3; i++)
				faceTable[faceNodeIndex + i] = face->GetNodeRecordPtr(i)->GetIndexNo();
			faceNodeIndex += 3;
		}
		//-- cutting face
		else {
			int nodeIndexArray[5] = {0}; int noCutEdgeIndex = -1; int cutEdgeIndex = 0;
			for (int i = 0; i < 3; i++)
				nodeIndexArray[i] = face->GetNodeRecordPtr(i)->GetIndexNo();
			for (int i = 0; i < 3; i++) {
				if (face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex < 0) {
					noCutEdgeIndex = i; continue;
				}
				else {
					nodeIndexArray[3 + cutEdgeIndex] = face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex;
					cutEdgeIndex++;
				}
			}

			for (int i = 0; i < 3; i++) { //case
				if (noCutEdgeIndex == i) {
					for (int j = 0; j < 3; j++) { //face
						for (int k = 0; k < 3; k++)  //node
							faceTable[faceNodeIndex + 3 * j + k] = nodeIndexArray[cutFaceCase[i][3 * j + k]-1];
					}
				}
			}
			faceNodeIndex += 9;
		}
	}
	std::cout << "finish build node and face table" << std::endl;

	int nodeNum = surfaceMesh->GetNodeNumber();
	int faceNum = surfaceMesh->GetFaceNumber() + 2 * intersectFaceNum;
	surfaceMesh->ClearAll();

	surfaceMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	free(nodeTable);
	free(faceTable);

}

bool isoSurface::planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh, QMeshFace* cutPlane) {

	//printf("--- begin cut the mesh by convex hull plane \n\n");

	int cutFaceCase[3][9] = {
		{ 1,4,5,1,2,4,4,3,5 },
		{ 1,4,5,4,2,5,2,3,5 },
		{ 1,4,5,4,2,5,1,5,3 } };

	// Plane equation: Ax+By+Cz+D = 0

	Vector3d PlaneDir; double D = 0.0;
	cutPlane->CalPlaneEquation(PlaneDir(0), PlaneDir(1), PlaneDir(2), D);

	/*Eigen::Vector3d meshCenter = Eigen::Vector3d::Zero();
	int nodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++) meshCenter(i) += pp[i];
	}
	for (int i = 0; i < 3; i++) meshCenter(i) /= surfaceMesh->GetNodeNumber();

	Vector3d PlaneDir; PlaneDir << 1.0, 0.0, 0.0;
	double D = 0.0;
	for (int i = 0; i < 3; i++) D -= meshCenter(i) * PlaneDir(i);*/

	//--------------------------------------------------------------
	/* pre-process to compute node-plane distance*/

	int positiveNodeNum = 0; int negativeNodeNum = 0;

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Vector3d pp; Node->GetCoord3D(pp(0), pp(1), pp(2));
		Node->nodePlaneDis = pp.dot(PlaneDir) + D;

		if (Node->nodePlaneDis > 0) {
			positiveNodeNum++; Node->SetIndexNo(-1);
		}
		else {
			Node->SetIndexNo(negativeNodeNum); negativeNodeNum++; // node index start from 0
		}
	}

	if (positiveNodeNum == 0) return true; //do not do anything
	else if (negativeNodeNum == 0) return false; //delete this layer

	//--------------------------------------------------------------
	/* detect the intersect edge, build intersect node*/

	int cutNodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *thisEdge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

		double a = thisEdge->GetStartPoint()->nodePlaneDis; 
		double b = thisEdge->GetEndPoint()->nodePlaneDis;
		if (a*b > 0) continue;

		double alpha = fabs(a) / fabs(b - a);
		double p1[3], p2[3], pp[3];
		thisEdge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
		thisEdge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

		for (int j = 0; j < 3; j++)
			pp[j] = (1.0 - alpha)*p1[j] + alpha*p2[j];

		QMeshNode* isoNode = new QMeshNode;
		isoNode->SetMeshPatchPtr(surfaceMesh);
		isoNode->SetCoord3D(pp[0], pp[1], pp[2]);

		//std::cout << pp[0] << "," << pp[1] << "," << pp[2] << std::endl;

		surfaceMesh->GetNodeList().AddTail(isoNode);
		isoNode->planeCutNewNode = true;

		isoNode->SetIndexNo(negativeNodeNum + cutNodeIndex); // index start from 0
		cutNodeIndex++;

		thisEdge->intersectNodeIndex = isoNode->GetIndexNo(); // if exist, the node index will large than 0
	}
	//std::cout << "finish generate the cutting node" << std::endl;

	///* Build topology */

	// -- node table
	float* nodeTable;
	int nodeNum = surfaceMesh->GetNodeNumber() - positiveNodeNum;
	nodeTable = (float *)malloc(sizeof(float)* nodeNum * 3);

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo()<0) continue; //delete those face have node on the left plane
		double pp[3]; Node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++) nodeTable[Node->GetIndexNo() * 3 + i] = (float) pp[i];
		
		//std::cout << pp[0] << "," << pp[1] << "," << pp[2] << std::endl;

	}

	// -- face table
	int faceNum = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		int face_posNodeNum = 0;
		for (int i = 0; i < 3; i++) {
			if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) face_posNodeNum++;
		}
		if (face_posNodeNum == 0 || face_posNodeNum == 2) faceNum += 1;
		else if (face_posNodeNum == 1) faceNum += 2;	
	}

	unsigned int* faceTable;
	faceTable = (unsigned int *)malloc(sizeof(unsigned int)*faceNum * 3);

	int faceNodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		int face_posNodeNum = 0;
		for (int i = 0; i < 3; i++) {
			if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) face_posNodeNum++;
		}

		if (face_posNodeNum == 3) continue;
		else if (face_posNodeNum == 0) {
			for (int i = 0; i < 3; i++)
				faceTable[faceNodeIndex + i] = face->GetNodeRecordPtr(i)->GetIndexNo();
			faceNodeIndex += 3; continue;
		}

		int nodeIndexArray[5] = { 0 }; int noCutEdgeIndex = -1; int cutEdgeIndex = 0;
		for (int i = 0; i < 3; i++)
			nodeIndexArray[i] = face->GetNodeRecordPtr(i)->GetIndexNo();
		for (int i = 0; i < 3; i++) {
			if (face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex < 0) {
				noCutEdgeIndex = i; continue;
			}
			else {
				nodeIndexArray[3 + cutEdgeIndex] = face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex;
				cutEdgeIndex++;
			}
		}

		if (face_posNodeNum == 1) {

			int positiveNodeIndex = -1;
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) positiveNodeIndex = i;
			}

			for (int i = 0; i < 3; i++) { //case
				if (noCutEdgeIndex == i) {

					int addFaceIndex = 0;
					for (int j = 0; j < 3; j++) { //face

						bool keepThisFace = true;
						for (int k = 0; k < 3; k++) {
							int currentNodeIndex = cutFaceCase[i][3 * j + k] - 1;
							if (currentNodeIndex == positiveNodeIndex) keepThisFace = false;
						}
						if (keepThisFace == false) continue;

						for (int k = 0; k < 3; k++)  //node
							faceTable[faceNodeIndex + 3 * addFaceIndex + k] = nodeIndexArray[cutFaceCase[i][3 * j + k] - 1];
						addFaceIndex++;
					}
				}
			}

			faceNodeIndex += 6;
		}

		else if (face_posNodeNum == 2) {

			int negativeNodeIndex = -1;
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->nodePlaneDis < 0) negativeNodeIndex = i;
			}

			for (int i = 0; i < 3; i++) { //case
				if (noCutEdgeIndex == i) {
					for (int j = 0; j < 3; j++) { //face

						bool keepThisFace = false;
						for (int k = 0; k < 3; k++) {
							int currentNodeIndex = cutFaceCase[i][3 * j + k] - 1;
							if (currentNodeIndex == negativeNodeIndex) keepThisFace = true;
						}
						if (keepThisFace == false) continue;
						for (int k = 0; k < 3; k++)  //node
							faceTable[faceNodeIndex + k] = nodeIndexArray[cutFaceCase[i][3 * j + k] - 1];
					}
				}
			}

			faceNodeIndex += 3;
		}
	}

	/*for (int i = 0; i < nodeNum * 3; i++) {
		if (i % 3 == 0 && i > 0) std::cout << std::endl;
		std::cout << nodeTable[i] << ","; 
	}*/

	//for (int i = 0; i < faceNum * 3; i++) {
	//	if (i % 3 == 0 && i > 0) std::cout << std::endl;
	//	std::cout << faceTable[i] << ",";
	//}

	//std::cout << std::endl << nodeNum << "," << faceNum << std::endl;

	//std::cout << faceNodeIndex << "," << faceNum << std::endl;

	//std::cout << "finish build node and face table" << std::endl;

	surfaceMesh->ClearAll();

	surfaceMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);




	free(nodeTable);
	free(faceTable);


	return true;

	//printf("--- finish cut mesh by convex hull plane ---------");

}


QMeshPatch* isoSurface::generatesingleIsoSurface(double isoValue, PolygenMesh* isoSurface, bool support) {

	QMeshPatch *layer = new QMeshPatch;
	layer->isoSurfaceValue = isoValue;
	layer->includeSupportRegion = false;

	//when the node iso-value is equal to surface value, add this eps.
	double eps = 1.0e-5;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		if (abs(Node->scalarField - isoValue)<eps) {
			if (Node->scalarField > isoValue) Node->scalarField = isoValue + eps;
			else Node->scalarField = isoValue - eps;
		}
	}

	// build node list for isoSurface, this comes from the edge list
	for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);

		Edge->installedIsoNode = nullptr;
		Edge->isLocateIsoNode = false;

		double a = Edge->GetStartPoint()->scalarField;
		double b = Edge->GetEndPoint()->scalarField;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++)
				pp[j] = (1.0 - alpha)*p1[j] + alpha*p2[j];

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedInitTetEdge = Edge;
			isoNode->SetMeshPatchPtr(layer);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(layer->GetNodeList().GetCount() + 1);
			layer->GetNodeList().AddTail(isoNode);

			Edge->installedIsoNode = isoNode;
			Edge->isLocateIsoNode = true;


			//support structure checking
			if (support) {
				if (Edge->GetStartPoint()->tetSupportNode || Edge->GetEndPoint()->tetSupportNode) {
					if (layer->includeSupportRegion == false) layer->includeSupportRegion = true;
					isoNode->printLayerSupportNode = true;
				}
			}

		}
	}

	// build edge list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
		Face->installedIsoEdge = nullptr;
		Face->isLocatedIsoEdge = false;

		int positiveNum = 0;
		for (int i = 0; i < 3; i++) {
			if (Face->GetNodeRecordPtr(i)->scalarField > isoValue) positiveNum++;
		}

		if (positiveNum == 0 || positiveNum == 3) continue;
		else if (positiveNum == 1) {
			QMeshEdge* isoEdge = new QMeshEdge;

			//detect which node is positive
			QMeshNode* PostiveNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				PostiveNode = Face->GetNodeRecordPtr(index);
				if (PostiveNode->scalarField > isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
		else if (positiveNum == 2) {
			QMeshEdge* isoEdge = new QMeshEdge;
			//detect which node is negative
			QMeshNode* NegativeNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				NegativeNode = Face->GetNodeRecordPtr(index);
				if (NegativeNode->scalarField < isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
	}

	// build face list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		int isoEdgeNum = 0;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) isoEdgeNum++;
		}
		if (isoEdgeNum == 0) continue;
		else if (isoEdgeNum == 2 || isoEdgeNum == 1) cout << "Error! isoEdgeNum cannot equal to 1 or 2!" << endl << endl;
		else if (isoEdgeNum == 3) {
			QMeshFace* isoFace = new QMeshFace;
			if (Tetra->isTensileorCompressSelect == true) {
				isoFace->isCriticalFace = true;
				isoFace->principleStressDir = Tetra->tau_max;
				isoFace->principleStress = Tetra->sigma_max;
				for (int i = 0; i < 3; i++) isoFace->principleStressColor[i] = Tetra->principleStressColor[i];

				//std::cout << Tetra->principleStressColor[0] << ", " << isoFace->principleStressColor[i] << std::endl;
			}
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(3);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) {
				if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) {
					FaceList[faceIndex] = Tetra->GetFaceRecordPtr(i + 1);
					faceIndex++;
				}
			}

			//sorting
			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
			if (firstDir == true) {
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}
			else {
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}

			//using the first face and add its edge into the isoFace.
			for (int i = 0; i < 3; i++) {
				isoFace->SetEdgeRecordPtr(i, FaceList[i]->installedIsoEdge);
				isoFace->SetDirectionFlag(i, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])));
				if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
					FaceList[i]->installedIsoEdge->SetLeftFace(isoFace);
				else FaceList[i]->installedIsoEdge->SetRightFace(isoFace);
			}

			//push this isoFace back to layer and compute norm
			isoFace->SetEdgeNum(3);
			isoFace->CalPlaneEquation();
			isoFace->SetMeshPatchPtr(layer);
			isoFace->SetIndexNo(layer->GetFaceList().GetCount() + 1);
			layer->GetFaceList().AddTail(isoFace);
		}

		else if (isoEdgeNum == 4) {
			std::vector<QMeshFace*> isoFace; isoFace.resize(2);
			for (int i = 0; i < 2; i++) {
				isoFace[i] = new QMeshFace;
				if (Tetra->isTensileorCompressSelect == true) {
					isoFace[i]->isCriticalFace = true;
					isoFace[i]->principleStress = Tetra->sigma_max;
					isoFace[i]->principleStressDir = Tetra->tau_max;
					for (int j = 0; j < 3; j++) isoFace[i]->principleStressColor[j] = Tetra->principleStressColor[j];
				}
			}
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(4);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) FaceList[i] = Tetra->GetFaceRecordPtr(i + 1);

			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
																						//sorting edge
			if (firstDir == true) {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);
			}
			else {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);

			}
			////test the sorting
			//cout << Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])) << endl;
			//for (int i = 0; i < 4; i++) {
			//	cout << FaceList[i]->installedIsoEdge->GetStartPoint()->GetIndexNo() << " , " <<
			//		FaceList[i]->installedIsoEdge->GetEndPoint()->GetIndexNo() << endl;	
			//}

			QMeshEdge* midEdge1 = new QMeshEdge;
			midEdge1->isMiddleEdge1 = true;
			if (firstDir == true) {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else cout << "Wrong case 1" << endl;
			}
			else {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else cout << "Wrong case 2" << endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge1->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge1->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/


			QMeshEdge* midEdge2 = new QMeshEdge;
			midEdge2->isMiddleEdge = true;
			if (firstDir == true) {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());

				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else cout << "Wrong case 1" << endl;
			}
			else {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());

				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else cout << "Wrong case 2" << endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge2->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge2->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/

			//midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
			/*
			bool dir = false;
			if (FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetStartPoint());
			dir = true;
			}
			else if(FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetEndPoint());
			else cout << "Error!" << endl;*/

			QMeshEdge* midEdge;

			if (midEdge1->CalLength() <= midEdge2->CalLength()) {

				midEdge = midEdge1;
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, FaceList[1]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[0]->SetEdgeRecordPtr(2, midEdge);
				isoFace[0]->SetDirectionFlag(2, false);

				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 1) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetRightFace(isoFace[0]);

				isoFace[1]->SetEdgeRecordPtr(0, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[3]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, true);
				for (int i = 0; i < 4; i++) {
					if (i == 2 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[1]);
			}

			else {
				midEdge = midEdge2;
				//first triangle
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, midEdge);
				isoFace[0]->SetDirectionFlag(1, true);
				isoFace[0]->SetEdgeRecordPtr(2, FaceList[3]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(2, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[0]);

				//first triangle
				isoFace[1]->SetEdgeRecordPtr(0, FaceList[1]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, false);
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 1 || i == 2) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[1]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[1]);
					}
				}
				midEdge->SetRightFace(isoFace[1]);
			}

			//push back midEdge
			midEdge->SetMeshPatchPtr(layer);
			midEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge);


			/*midEdge1->SetMeshPatchPtr(layer);
			midEdge1->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge1);*/

			//midEdge2->SetMeshPatchPtr(layer);
			//midEdge2->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			//layer->GetEdgeList().AddTail(midEdge2);



			//push this isoFace back to layer and compute norm
			for (int i = 0; i < 2; i++) {
				isoFace[i]->SetEdgeNum(3);
				isoFace[i]->CalPlaneEquation();
				isoFace[i]->SetMeshPatchPtr(layer);
				isoFace[i]->SetIndexNo(layer->GetFaceList().GetCount() + 1);
				layer->GetFaceList().AddTail(isoFace[i]);
			}
		}
	}

	//give each node face list
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetFaceList().AddTail(Face);
		}
	}

	//check support region layer structure, update flag
	if (support) {
		for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace *Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
			for (int i = 0; i < 3; i++) {
				// as long as this face include one singe support node, its a support face
				if (Face->GetNodeRecordPtr(i)->printLayerSupportNode) {
					Face->printLayerSupportFace = true; break;
				}
			}
		}

		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge *Edge = (QMeshEdge*)layer->GetEdgeList().GetNext(Pos);
			if(Edge->GetStartPoint()->printLayerSupportNode || Edge->GetEndPoint()->printLayerSupportNode)
				Edge->printLayerSupportEdge = true;
		}
	}

	return layer;
}

