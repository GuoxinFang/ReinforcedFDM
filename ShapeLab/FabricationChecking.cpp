#include "stdafx.h"
#include <omp.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "FabricationChecking.h"
#include "../Library/QHull/qhull_a.h"
#include "../Library/PQPLib/PQP.h"
#include "PMBody.h"

using namespace std;
using namespace Eigen;

#define DEGREE_TO_ROTATE(x)		0.0174532922222*x

void FabricationProcess::runBestFabricationDirectionDetection( 
	bool compute, double _theta, double _beta, bool inverse) {

	/* -- 3D rotation function 2020-04-27 -- */
	this->_initializeSystem_RotationSystem();
	
	int sampleRate = 36;	Eigen::Vector3d initNorm = { 0, 1.0, 0 };
	Eigen::MatrixXd shadowVolume = Eigen::MatrixXd::Zero(sampleRate + 1, sampleRate + 1);
	Eigen::Matrix3d bestRotationMatrix;


	if (compute == true) {

		for (int xaxisIndex = 0; xaxisIndex < sampleRate + 1; xaxisIndex++) {

			//#pragma omp parallel   
			{
				//#pragma omp for

				for (int yaxisIndex = 0; yaxisIndex < sampleRate + 1; yaxisIndex++) {



					double thetaDegree = xaxisIndex * 360 / sampleRate;
					double betaDegree = yaxisIndex * 180 / sampleRate;

					double theta = DEGREE_TO_ROTATE(thetaDegree);
					double beta = DEGREE_TO_ROTATE(betaDegree);


					Eigen::Vector3d rotateDir = { cos(theta) * sin(beta), cos(beta),sin(theta) * sin(beta) };

					Eigen::Matrix3d rotationMatrix;
					rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(initNorm, rotateDir);

					std::vector<Eigen::MatrixXd> layerCoord(isoSurfaceSet->GetMeshList().GetCount());
					bool fliporder = this->_rotateLayerSetbyMatrix(rotationMatrix, false, layerCoord);

					QHULLSET* currentConvexFront = NULL;

					if (fliporder == false) {		// ---- first to last

						long convexBuildTime = 0;
						long volumeCompTime = 0;

						for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
							QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);

							long time = clock();
							currentConvexFront = _constructNewConvexFront(
								layer, currentConvexFront, printingPlatform, layerCoord[layer->GetIndexNo()]);
							convexBuildTime += clock() - time;

							time = clock();
							shadowVolume(xaxisIndex, yaxisIndex) +=
								_compShadowVolume(layer, currentConvexFront, layerCoord[layer->GetIndexNo()]);
							volumeCompTime += clock() - time;
						}
						std::cout << "compute convex time = " << convexBuildTime << std::endl;
						std::cout << "compute shadow volume time = " << volumeCompTime << std::endl;


					}
					else {		// ---- last to first

						std::vector<Eigen::MatrixXd> layerCoord_flip(isoSurfaceSet->GetMeshList().GetCount());
						for (int i = 0; i < isoSurfaceSet->GetMeshList().GetCount(); i++)
							layerCoord_flip[i] = layerCoord[isoSurfaceSet->GetMeshList().GetCount() - i - 1];

						for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetTailPosition(); posMesh != nullptr;) {
							QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetPrev(posMesh);

							currentConvexFront = _constructNewConvexFront(
								layer, currentConvexFront, printingPlatform, layerCoord[layer->GetIndexNo()]);
							shadowVolume(xaxisIndex, yaxisIndex) +=
								_compShadowVolume(layer, currentConvexFront, layerCoord[layer->GetIndexNo()]);

						}
						layerCoord_flip.clear();
					}

					_freeMemoryConvexHull(currentConvexFront);
					layerCoord.clear();

					std::cout << " ### theta = " << thetaDegree << ", beta = " << betaDegree <<
						". Shadow Volume = " << shadowVolume(xaxisIndex, yaxisIndex) << std::endl << std::endl;

					//break;

				}
			}
			//break;

		}

		int i, j;
		shadowVolume.minCoeff(&i, &j);

		double thetaDegree = i * 360 / sampleRate;
		double betaDegree = j * 180 / sampleRate;

		double theta = DEGREE_TO_ROTATE(thetaDegree);
		double beta = DEGREE_TO_ROTATE(betaDegree);

		//double theta = DEGREE_TO_ROTATE(i * 360 / sampleRate);
		//double beta = DEGREE_TO_ROTATE(j * 180 / sampleRate);
		std::cout << "Best Printing Dir: theta = " << thetaDegree << ", beta = " << betaDegree << std::endl;

		//Eigen::Vector3d rotateDir = { cos(theta) * sin(beta),  sin(theta) * sin(beta), cos(beta) };
		Eigen::Vector3d rotateDir = { cos(theta) * sin(beta), cos(beta),sin(theta) * sin(beta) };

		//rotateDir << 1.0, 1.0, 1.0; rotateDir = rotateDir.normalized();
		bestRotationMatrix = Eigen::Quaterniond().setFromTwoVectors(initNorm, rotateDir);
	}

	else {
		//double theta = DEGREE_TO_ROTATE(_theta);
		//double beta = DEGREE_TO_ROTATE(_beta);
		//Eigen::Vector3d rotateDir = { cos(_theta) * sin(_beta),  cos(_beta), sin(_theta) * sin(_beta)};

		// -- yoga
		double theta = 4.01; double beta = 0.44;
		Eigen::Vector3d rotateDir = { cos(theta) * sin(beta),sin(theta) * sin(beta), cos(beta) };

		///*if(inverse == false)
		//	bestRotationMatrix = Eigen::Quaterniond().setFromTwoVectors(initNorm, rotateDir);
		//else bestRotationMatrix = Eigen::Quaterniond().setFromTwoVectors(rotateDir, initNorm);*/

		//bestRotationMatrix = Eigen::Quaterniond().setFromTwoVectors(initNorm, rotateDir);

		// -- csquare
		//double theta = DEGREE_TO_ROTATE(110.0);
		//double beta = DEGREE_TO_ROTATE(100.0);

		//double theta = DEGREE_TO_ROTATE(i * 360 / sampleRate);
		//double beta = DEGREE_TO_ROTATE(j * 180 / sampleRate);

		//Eigen::Vector3d rotateDir = { cos(theta) * sin(beta),  sin(theta) * sin(beta), cos(beta) };
		
		//Eigen::Vector3d rotateDir = { cos(theta) * sin(beta), cos(beta),sin(theta) * sin(beta) };

		//rotateDir << 1.0, 1.0, 1.0; rotateDir = rotateDir.normalized();
		bestRotationMatrix = Eigen::Quaterniond().setFromTwoVectors(initNorm, rotateDir);


	}
	std::vector<Eigen::MatrixXd> layerCoord(isoSurfaceSet->GetMeshList().GetCount());
	bool fliporder = this->_rotateLayerSetbyMatrix(bestRotationMatrix, true, layerCoord);

}

void FabricationProcess::_initializeSystem_RotationSystem() {
	int layerindex = 0;
	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		layer->SetIndexNo(layerindex); layerindex++;
		int nodeIndex = 0;
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->SetIndexNo(nodeIndex); nodeIndex++;
			double pp[3];
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
		}
	}

	QMeshPatch* tetMesh = (QMeshPatch*)tetrahedralModel->GetMeshList().GetHead();
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		double pp[3];
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
	}
}

void FabricationProcess::initializeSystem_RotationSystem() {
	
	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);	
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			double pp[3];
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
		}
	}

	QMeshPatch* tetMesh = (QMeshPatch*)tetrahedralModel->GetMeshList().GetHead();
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		double pp[3];
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
	}
}

bool FabricationProcess::_rotateLayerSetbyMatrix(
	Eigen::Matrix3d& rotationMatrix, bool updateVisual, std::vector<Eigen::MatrixXd>& layerCoord) {

	// build the coordinate matrix
	//std::vector<Eigen::MatrixXd> layerCoord(isoSurfaceSet->GetMeshList().GetCount());

	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		//std::cout << layer->GetIndexNo() << std::endl;
		layerCoord[layer->GetIndexNo()] = Eigen::MatrixXd::Zero(layer->GetNodeNumber(), 3); 
		//std::cout << layerCoord[layer->GetIndexNo()].rows() << ", " << layerCoord[layer->GetIndexNo()].cols() << std::endl;
		/*rotate the isosurface potision*/
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) { 
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Eigen::Vector3d pp; Node->GetCoord3D_last(pp(0), pp(1), pp(2));
			Eigen::Vector3d rotatedpp = rotationMatrix * pp;
			for (int i = 0; i < 3; i++)
				layerCoord[layer->GetIndexNo()](Node->GetIndexNo(), i) = rotatedpp(i);
		}
	}

	// detect after rotation, wheather the top layer is lower than the bottom layer
	bool flipOrder = false;
	double ymin_firstLayer = layerCoord[0].col(1).minCoeff();
	double ymin_lastLayer = layerCoord[isoSurfaceSet->GetMeshList().GetCount() - 1].col(1).minCoeff();

	double minY;
	if (ymin_firstLayer > ymin_lastLayer) flipOrder = true;

	Eigen::VectorXd minYLayer(isoSurfaceSet->GetMeshList().GetCount());
	for (int i = 0; i < isoSurfaceSet->GetMeshList().GetCount(); i++) {
		minYLayer(i) = layerCoord[i].col(1).minCoeff();
	}
	minY = minYLayer.minCoeff();

	// center the model and move up, cal center coord of XZ / bottom coord and shift

	/* Update: the center and hight should be computed by the tetmesh 2020-05-01 */

	QMeshPatch* initTetModel = (QMeshPatch*) tetrahedralModel->GetMeshList().GetHead();
	Eigen::MatrixXd initTetPos(initTetModel->GetNodeNumber(), 3); int nIndex = 0;
	for (GLKPOSITION posMesh = initTetModel->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)initTetModel->GetNodeList().GetNext(posMesh);

		Eigen::Vector3d pp; node->GetCoord3D(pp(0), pp(1), pp(2));
		Eigen::Vector3d rotatedpp = rotationMatrix * pp;
		for (int i = 0; i < 3; i++)
			initTetPos(nIndex, i) = rotatedpp(i);
		nIndex++;
	}
	
	Eigen::Vector3d centerPos;  
	for (int i = 0; i < 3; i++) centerPos(i) = initTetPos.col(i).mean();
	//std::cout << centerPos << std::endl;

	//double center[3] = { 0 }; int elementNum = 0;
	//for (int i = 0; i < isoSurfaceSet->GetMeshList().GetCount(); i++) {
	//	for (int j = 0; j < 3; j++) {
	//		center[j] += layerCoord[i].col(j).mean() * layerCoord[i].rows();
	//		elementNum += layerCoord[i].rows();
	//	}
	//}
	//for (int j = 0; j < 3; j++) center[j] /= elementNum;
	//std::cout << center[0]<<"," << center[1] << "," << center[2] << std::endl;

	for (int i = 0; i < isoSurfaceSet->GetMeshList().GetCount(); i++) {
		for (int j = 0; j < layerCoord[i].rows(); j++) {
			layerCoord[i](j, 0) -= centerPos(0);
			layerCoord[i](j, 2) -= centerPos(2);
			layerCoord[i](j, 1) += this->Yshift - minY;
		}
	}

	if (updateVisual) {
		for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
			QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);		
			for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
				Eigen::Vector3d pp = layerCoord[layer->GetIndexNo()].row(Node->GetIndexNo());
				Node->SetCoord3D(pp(0), pp(1), pp(2));
			}

			for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
				Face->principleStressDir = (rotationMatrix * Face->principleStressDir).normalized();
			}
		}

		for (GLKPOSITION posMesh = initTetModel->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshNode* node = (QMeshNode*)initTetModel->GetNodeList().GetNext(posMesh);
			Eigen::Vector3d pp; node->GetCoord3D(pp(0), pp(1), pp(2));
			Eigen::Vector3d rotatedpp = rotationMatrix * pp;
			node->SetCoord3D(rotatedpp(0)- centerPos(0), rotatedpp(1) + this->Yshift - minY, rotatedpp(2)- centerPos(2));
		}
		for (GLKPOSITION posMesh = initTetModel->GetTetraList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshTetra* tetra = (QMeshTetra*)initTetModel->GetTetraList().GetNext(posMesh);
			tetra->vectorField = rotationMatrix * tetra->vectorField;
		}
	}

	return flipOrder;

}


void FabricationProcess::runBestFabricationDirectionDetection(bool compute, int rotateDegree)
{
	this->_initializeSystem_RotationSystem();

	if (compute) {
		int searchNum = 72;
		Eigen::VectorXd shadowVolume = Eigen::VectorXd::Zero(searchNum);

		for (int i = 0; i < searchNum; i++) {
			bool order = this->rotateLayerandCenter_checkOrder(i * 5);

			QHULLSET* currentConvexFront;
			currentConvexFront = NULL;
			if (order) {  //first to last

				for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
					QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);

					currentConvexFront = _constructNewConvexFront(layer, currentConvexFront, printingPlatform);
					shadowVolume(i) += compShadowVolume(layer, currentConvexFront);
				}
			}
			else { //last to first

				for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetTailPosition(); posMesh != nullptr;) {
					QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetPrev(posMesh);

					currentConvexFront = _constructNewConvexFront(layer, currentConvexFront, printingPlatform);
					shadowVolume(i) += compShadowVolume(layer, currentConvexFront);

				}
			}
			_freeMemoryConvexHull(currentConvexFront);
			std::cout << "degree = " << i << ", volume = " << shadowVolume(i) << std::endl;
		}

		//sort the smallest one
		int finalDegree = 0; double valueSearch = shadowVolume(0);
		for (int i = 0; i < searchNum; i++) {
			if (shadowVolume(i) < valueSearch) {
				valueSearch = shadowVolume(i);
				finalDegree = i;
			}
		}
		rotateDegree = finalDegree * 5;
	}

	/* -- 190 for J model (before)
	-- 280 for S model, 135
	-- 295 for new J model
	-- 295 for bunny head 
	-- 210 for topo-opt model -- 220 （0422）
	-- 90 for yoga
	*/

	//double finalDegree = 210 / 5;
	std::cout << "The final rotate degree is " << rotateDegree << std::endl;
	bool order = this->rotateLayerandCenter_checkOrder(rotateDegree);
	//_drawConvexHull(order);
}

bool FabricationProcess::runFabricationCollisionDetection(QMeshPatch* printNozzle) {

	printf(" BEGIN run collision detection! \n\n ");
	bool allLayerCollision = false;
	// This function is not finished 2020-04-06

	/*  First initialize the printNozzle position  */
	double tipPos[3] = {0}; double bottomY = 9999999.999;
	for (GLKPOSITION Pos = printNozzle->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)printNozzle->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp);
		if (pp[1] < bottomY) {
			bottomY = pp[1];  for (int i = 0; i < 3; i++) tipPos[i] = pp[i];
		}
	}
	for (GLKPOSITION Pos = printNozzle->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)printNozzle->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp); for (int i = 0; i < 3; i++) pp[i] -= tipPos[i];
		Node->SetCoord3D(pp[0], pp[1], pp[2]); 	Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
	}
	Eigen::Vector3d nozzleInitNorm = { 0, 1.0, 0 };

	/* Begin checking the collision */
	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		bool totalCollisionHappen = false;

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

			Eigen::Vector3d nodePos, norm;
			Node->GetCoord3D(nodePos(0), nodePos(1), nodePos(2));
			Node->CalNormal(); Node->GetNormal(norm(0), norm(1), norm(2));
			//norm = -norm;
			// rotate the nozzle mesh to tip position, compute the normal of every face
			Eigen::Matrix3d rotationMatrix;
			norm = -norm;
			rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(nozzleInitNorm, norm);

			//std::cout << rotationMatrix << std::endl;

			for (GLKPOSITION Pos = printNozzle->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode* nozzleNode = (QMeshNode*)printNozzle->GetNodeList().GetNext(Pos);
				Eigen::Vector3d nozzleNodePos;

				nozzleNode->GetCoord3D_last(nozzleNodePos(0), nozzleNodePos(1), nozzleNodePos(2));
				//nozzleNodePos = rotationMatrix * nozzleNodePos;
				nozzleNodePos = rotationMatrix * nozzleNodePos + nodePos;

				nozzleNode->SetCoord3D(nozzleNodePos(0), nozzleNodePos(1), nozzleNodePos(2));
				//std::cout << nozzleNodePos(0) << "," << nozzleNodePos(1) << "," << nozzleNodePos(2) << std::endl;
			}

			//if (layer->GetIndexNo() == 13 && Node->GetIndexNo() == 20) return true;
			
			// detect collision
			if (this->checkSingleNodeCollision_withinItsLayer(printNozzle, layer, Node)) {
				//std::cout << "collision node!" << std::endl;
				Node->isCollisionHappenNode = true;
				Node->relatedInitTetEdge->isCollisionCheckedEdge = true;
				totalCollisionHappen = true;
				std::cout << " -- " << Node->GetIndexNo();
				//if (layer->GetIndexNo() == 15 && Node->GetIndexNo() != 1048) return true;
			}
		}
		if (totalCollisionHappen) { 
			allLayerCollision = true;
			std::cout << std::endl << " #### Above are No." << layer->GetIndexNo() << "collision nodes!" << std::endl << std::endl;

			//break;
		}
		else std::cout << "No." << layer->GetIndexNo() << " is Collision free!" << std::endl;
		//break;
	}

	if (allLayerCollision) return true;
	else return false;
}

bool FabricationProcess::checkSingleNodeCollision_withinItsLayer(
	QMeshPatch* printNozzle, QMeshPatch* layer, QMeshNode* checkNode) {

	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* layerNode = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
		bool nodeCollision = true;

		if (checkNode == layerNode) continue;

		Eigen::Vector3d pp;
		layerNode->GetCoord3D(pp(0), pp(1), pp(2));

		for (GLKPOSITION Pos = printNozzle->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* nozzleFace = (QMeshFace*)printNozzle->GetFaceList().GetNext(Pos);

			Eigen::Vector3d n1, n2, n3;
			nozzleFace->GetNodeRecordPtr(0)->GetCoord3D(n1(0), n1(1), n1(2));
			nozzleFace->GetNodeRecordPtr(1)->GetCoord3D(n2(0), n2(1), n2(2));
			nozzleFace->GetNodeRecordPtr(2)->GetCoord3D(n3(0), n3(1), n3(2));

			Eigen::Vector3d planeNormal = (n2 - n1).cross(n3 - n1);
			planeNormal = planeNormal.normalized();
			//std::cout << planeNormal(0) << "," << planeNormal(1) << "," << planeNormal(2) << std::endl;

			double dd = -n3.dot(planeNormal);
			//std::cout << planeNormal(0) << "," << planeNormal(1) << "," << planeNormal(2) << "," << dd << "," << std::endl;
			if (pp.dot(planeNormal) + dd > -0.01) {
				//std::cout << "outside convex hull!" << std::endl;
				nodeCollision = false; break;
			}
		}

		if (nodeCollision) return true;
	}
	return false;

}

void FabricationProcess::support_tet_link_topology(QMeshPatch* initTet, QMeshPatch* supportTet) {

	supportPatch = supportTet;
	//the initTet and supportTet share same vertex table, first build tetrahedral table
	
	//--------------------//
	/*initialize the index*/
	int index = 0;
	for (GLKPOSITION Pos = initTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)initTet->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index); index++;
	}
	index = 0;
	for (GLKPOSITION Pos = supportTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)supportTet->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index); index++;
	}
	index = 0;
	for (GLKPOSITION Pos = initTet->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *tet = (QMeshTetra*)initTet->GetTetraList().GetNext(Pos);
		tet->SetIndexNo(index); index++;
	}
	index = 0;
	for (GLKPOSITION Pos = supportTet->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *tet = (QMeshTetra*)supportTet->GetTetraList().GetNext(Pos);
		tet->SetIndexNo(index); index++;
	}

	//--------------------------//
	/*first allign mesh together*/

	double centerInit[3] = { 0 };
	double centerSupport[3] = { 0 };

	for (GLKPOSITION Pos = initTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)initTet->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++) centerInit[i] += pp[i];
	}
	for (int i = 0; i < 3; i++) centerInit[i] /= initTet->GetNodeNumber();


	for (GLKPOSITION Pos = supportTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)supportTet->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo() < initTet->GetNodeNumber()) {
			double pp[3]; Node->GetCoord3D(pp);
			for (int i = 0; i < 3; i++) centerSupport[i] += pp[i];
		}	
	}
	for (int i = 0; i < 3; i++) centerSupport[i] /= initTet->GetNodeNumber();

	for (GLKPOSITION Pos = supportTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)supportTet->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++) pp[i] += (centerInit[i] - centerSupport[i]);
		Node->SetCoord3D(pp[0], pp[1], pp[2]);
	}

	//---------------------//
	/*scalar field transfer*/
	index = 0;
	Eigen::VectorXd initScalarField = Eigen::VectorXd::Zero(initTet->GetNodeNumber());
	
	for (GLKPOSITION Pos = initTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)initTet->GetNodeList().GetNext(Pos);
		initScalarField(index) = Node->guideFieldValue_no_normalize; index++;
	}
	for (GLKPOSITION Pos = supportTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)supportTet->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo() < initTet->GetNodeNumber()) {

			Node->guideFieldValue = initScalarField(Node->GetIndexNo());
			Node->guideFieldValue_no_normalize = Node->guideFieldValue;

			Node->tetSupportNode = false;
		}
		else {
			Node->tetSupportNode = true; Node->guideFieldValue = 0;
		}
	}

	std::cout << "transfer guide field value to support tetrahedral mesh!" << std::endl;

	//---------------------//
	/*vector field transfer*/

	//----Step 1: find linkage of tetrahedral table 
	// (2020-03-09 no need, Prof. Zhong already link the element table for me.)

	//int supportEleNum = 0;
	//for (GLKPOSITION Pos = supportTet->GetTetraList().GetHeadPosition(); Pos;) {
	//	QMeshTetra *tet = (QMeshTetra*)supportTet->GetTetraList().GetNext(Pos);
	//	if (tet->GetNodeRecordPtr(1)->tetSupportNode == false && tet->GetNodeRecordPtr(2)->tetSupportNode == false
	//		&& tet->GetNodeRecordPtr(3)->tetSupportNode == false && tet->GetNodeRecordPtr(4)->tetSupportNode == false)
	//	{
	//		tet->tetSupportElement = false; supportEleNum++;
	//	}
	//	else  tet->tetSupportElement = true;  
	//}
	//std::cout << "support tetrahedral number = " << supportEleNum << endl;

	////build initial tet mesh node table
	//std::vector<Eigen::MatrixXi> eleNodeTable; eleNodeTable.resize(supportTet->GetNodeNumber());
	//std::vector<Eigen::VectorXi> eleTetIndex; eleTetIndex.resize(supportTet->GetNodeNumber());

	//for (GLKPOSITION Pos = initTet->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode *Node = (QMeshNode*)initTet->GetNodeList().GetNext(Pos);

	//	eleNodeTable[Node->GetIndexNo()] = Eigen::MatrixXi::Zero(Node->GetTetraNumber(), 4);
	//	eleTetIndex[Node->GetIndexNo()] = Eigen::VectorXi::Zero(Node->GetTetraNumber());
	//	
	//	index = 0;
	//	for (GLKPOSITION Pos = Node->GetTetraList().GetHeadPosition(); Pos;) {
	//		QMeshTetra *linkTetra = (QMeshTetra*)Node->GetTetraList().GetNext(Pos);
	//		eleTetIndex[Node->GetIndexNo()](index) = linkTetra->GetIndexNo();
	//		for (int i = 0; i < 4; i++) 
	//			eleNodeTable[Node->GetIndexNo()](index, i) = linkTetra->GetNodeRecordPtr(i + 1)->GetIndexNo();
	//		index++;
	//	}
	//}

	//for (GLKPOSITION Pos = supportTet->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode *Node = (QMeshNode*)supportTet->GetNodeList().GetNext(Pos);
	//	if (Node->tetSupportNode) continue;

	//	for (GLKPOSITION Pos = Node->GetTetraList().GetHeadPosition(); Pos;) {
	//		QMeshTetra *linkTetra = (QMeshTetra*)Node->GetTetraList().GetNext(Pos);
	//		if (linkTetra->tetSupportElement || linkTetra->linkIndex > -1) continue;

	//		Eigen::Vector4i thisIndexVec; 
	//		for (int i = 0; i < 4; i++) thisIndexVec(i) = linkTetra->GetNodeRecordPtr(i + 1)->GetIndexNo();


	//		Eigen::MatrixXi indexMatrix = eleNodeTable[Node->GetIndexNo()];
	//		for (int i = 0; i < indexMatrix.rows(); i++) {
	//			Eigen::Vector4i initIndexVec = indexMatrix.row(i);
	//			if (compareVector(thisIndexVec, initIndexVec)) {
	//				linkTetra->linkIndex = eleTetIndex[Node->GetIndexNo()](i); 
	//				break;
	//			}
	//		}
	//	}
	//}

	//----Step 1: find linkage of tetrahedral table , meanwhile check the topology of given mesh
	Eigen::MatrixXi tetTable = Eigen::MatrixXi::Zero(initTet->GetTetraNumber(), 4);
	for (GLKPOSITION Pos = initTet->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *tet = (QMeshTetra*)initTet->GetTetraList().GetNext(Pos);
		for (int i = 0; i < 4; i++) {
			tetTable(tet->GetIndexNo(), i) = tet->GetNodeRecordPtr(i + 1)->GetIndexNo();
		}
	}
	//std::cout << tetTable << std::endl;

	std::cout << "initTet tetnumber = " << initTet->GetTetraNumber() << std::endl << std::endl;

	for (GLKPOSITION Pos = supportTet->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *tet = (QMeshTetra*)supportTet->GetTetraList().GetNext(Pos);
		if (tet->GetIndexNo() < initTet->GetTetraNumber())
		{
			tet->tetSupportElement = false;
			
			//check topology
			Eigen::Vector4i initIndexVec = tetTable.row(tet->GetIndexNo());
			Eigen::Vector4i thisIndexVec;
			for (int i = 0; i < 4; i++) 
				thisIndexVec(i) = tet->GetNodeRecordPtr(i + 1)->GetIndexNo();
			if(!this->compareVector(thisIndexVec, initIndexVec))
				std::cerr << "Wrong tet table for non-support region!" << std::endl;

		}
		else {
			tet->tetSupportElement = true;

			//check topology
			if (tet->GetNodeRecordPtr(1)->GetIndexNo() < initTet->GetNodeNumber() &&
				tet->GetNodeRecordPtr(2)->GetIndexNo() < initTet->GetNodeNumber() &&
				tet->GetNodeRecordPtr(3)->GetIndexNo() < initTet->GetNodeNumber() &&
				tet->GetNodeRecordPtr(4)->GetIndexNo() < initTet->GetNodeNumber())

				//std::cerr << "Wrong tet table for support region! No. " << tet->GetIndexNo() << std::endl;
				std::cerr <<  tet->GetIndexNo()	<< "," << tet->GetNodeRecordPtr(1)->GetIndexNo()  << "," << tet->GetNodeRecordPtr(2)->GetIndexNo()  << "," <<
				tet->GetNodeRecordPtr(3)->GetIndexNo() << "," << tet->GetNodeRecordPtr(4)->GetIndexNo() << std::endl;
		}
	}

	//----Step 2: transfer vector field
	std::vector<Eigen::Vector3d> vectorFieldSet; vectorFieldSet.resize(initTet->GetTetraNumber());
	for (GLKPOSITION Pos = initTet->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *tet = (QMeshTetra*)initTet->GetTetraList().GetNext(Pos);
		vectorFieldSet[tet->GetIndexNo()] = tet->vectorField;
	}

	//printf("The element have no-linked topology is: \n");

	int nonSupportTetNum = 0;
	for (GLKPOSITION Pos = supportTet->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *tet = (QMeshTetra*)supportTet->GetTetraList().GetNext(Pos);

		/*if (tet->tetSupportElement == false) {

			if(tet->linkIndex == -1)
				std::cout << tet->GetIndexNo() << " - " << 
				tet->GetNodeRecordPtr(1)->GetIndexNo()  << "," << tet->GetNodeRecordPtr(2)->GetIndexNo() <<","<<
				tet->GetNodeRecordPtr(3)->GetIndexNo() << "," << tet->GetNodeRecordPtr(4)->GetIndexNo()
				<< " - " << tet->linkIndex << std::endl;

		}*/

		//if (tet->linkIndex > -1) {

		if (tet->tetSupportElement == false) {
			tet->vectorField = vectorFieldSet[tet->GetIndexNo()];
			nonSupportTetNum++;
		}
		else {
			tet->vectorField << 0, 0, 0;
		}

	}

	std::cout << "nonSupportTetNum = " << nonSupportTetNum << endl;
	supportTet->drawVectorField = true;
	supportTet->sparseVectorDraw = true;
	std::cout << "transfer guide field value to support tetrahedral mesh!" << std::endl;
}

void FabricationProcess::support_surface_delete_nonimportant_region(
	PolygenMesh *supportRegion, PolygenMesh *supportIsoSurface) {

	/*Node-detection method (2020-03-13)*/

	QMeshPatch *supportNodePatch = (QMeshPatch *)supportRegion->GetMeshList().GetHead();
	std::vector<Eigen::VectorXd> checkBox(supportNodePatch->GetNodeNumber());

	int index = 0;
	for (GLKPOSITION Pos = supportNodePatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)supportNodePatch->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp);
		Eigen::VectorXd boxRange = Eigen::VectorXd::Zero(6);
		for (int i = 0; i < 3; i++) {
			boxRange(2 * i) = pp[i] + supportNodeBoxSize;
			boxRange(2 * i + 1) = pp[i] - supportNodeBoxSize;
		}
		checkBox[index] = boxRange; index++;
	}


#pragma omp parallel   
	{
#pragma omp for
	for (int CPUcore = 0; CPUcore < CPUCoreNum; CPUcore++) {
		for (GLKPOSITION Pos = supportIsoSurface->GetMeshList().GetHeadPosition(); Pos;) {
			QMeshPatch *layer = (QMeshPatch*)supportIsoSurface->GetMeshList().GetNext(Pos);

			if (layer->GetIndexNo() % CPUCoreNum != CPUcore) continue;

			if (layer->includeSupportRegion == false) continue;

			bool supportLayerPrint = false;
			//check node
			for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

				Node->importantSupportNode = false;

				double pp[3]; Node->GetCoord3D(pp);
				for (int i = 0; i < supportNodePatch->GetNodeNumber(); i++) {
					if (pp[0] < checkBox[i](0) && pp[0] > checkBox[i](1) &&
						pp[1] < checkBox[i](2) && pp[1] > checkBox[i](3) &&
						pp[2] < checkBox[i](4) && pp[2] > checkBox[i](5))
					{
						Node->importantSupportNode = true;
						supportLayerPrint = true;
						break;
					}
				}
			}

			//set face flag - together with edge flag

			//for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos;) {
			//	QMeshEdge *edge = (QMeshEdge*)layer->GetEdgeList().GetNext(Pos);
			//	edge->importantSupportEdge = false;
			//}
			//for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
			//	QMeshFace *face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);

			//	face->importantSupportFace = true;
			//	for (int i = 0; i < 3; i++) {
			//		if (face->GetNodeRecordPtr(i)->importantSupportNode == false) {
			//			face->importantSupportFace = false;
			//		}
			//	}
			//	if (face->importantSupportFace == true) {
			//		for (int i = 0; i < 3; i++) 
			//			face->GetEdgeRecordPtr(i + 1)->importantSupportEdge = true;
			//	}

			//}

			////set node flag

			//for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			//	QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			//	Node->importantSupportNode = false;
			//}
			//for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
			//	QMeshFace *face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
			//	if (face->importantSupportFace == true) {
			//		for (int i = 0; i < 3; i++)
			//			face->GetNodeRecordPtr(i)->importantSupportNode = true;
			//	}
			//}

			if (supportLayerPrint == false) {
				layer->ClearAll();
			}

			std::cout << " ## No. " << layer->GetIndexNo() << "support layer checked!" << std::endl;
		}
	}
}

	std::cout << " -- finish checking the imporatnt of support curve surface!" << std::endl;
	
	/*Ray-intersecting method with initial iso-surface*/
	
	/*
	//first build a vector to install all the support-point ray from initial iso surface

	//int supportNodeNum = 0;
	//for (GLKPOSITION Pos = initIsoSurface->GetMeshList().GetHeadPosition(); Pos;) {
	//	QMeshPatch *layer = (QMeshPatch*)initIsoSurface->GetMeshList().GetNext(Pos);

	//	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

	//		if (Node->supportNode) supportNodeNum++;

	//	}
	//}

	//std::cout << "in total " << supportNodeNum << "support ray." << endl;

	//std::vector<Eigen::Vector3d> startPoint(supportNodeNum);
	//std::vector<Eigen::Vector3d> endPoint(supportNodeNum);

	//int index = 0;
	//for (GLKPOSITION Pos = initIsoSurface->GetMeshList().GetHeadPosition(); Pos;) {
	//	QMeshPatch *layer = (QMeshPatch*)initIsoSurface->GetMeshList().GetNext(Pos);

	//	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

	//		if (Node->supportNode) {
	//			double pp[3];
	//			Node->GetCoord3D(pp);
	//			startPoint[index] << pp[0], pp[1], pp[2];

	//			endPoint[index] = Node->supportEndPos;
	//			index++;
	//		}

	//	}
	//}

	////detect intersect
	//for (GLKPOSITION Pos = supportIsoSurface->GetMeshList().GetHeadPosition(); Pos;) {
	//	QMeshPatch *layer = (QMeshPatch*)supportIsoSurface->GetMeshList().GetNext(Pos);

	//	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
	//		QMeshFace *face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
	//		if (!face->printLayerSupportFace) continue;

	//		Eigen::Vector3d v1, v2, v3;
	//		face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
	//		face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
	//		face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

	//		for (int i = 0; i < supportNodeNum; i++) {
	//			Eigen::Vector3d dir = (endPoint[i] - startPoint[i]).normalized();
	//			Eigen::Vector3d insertPP;
	//			//if (intersetTriangle(startPoint[i], dir, v1, v2, v3, insertPP)) 
	//			if (intersetTriangle_supportStructure(startPoint[i], endPoint[i], v1, v2, v3))
	//			{
	//				face->importantSupportFace = true;
	//				break;
	//			}
	//		}
	//	}
	//}

	//for (GLKPOSITION Pos = supportIsoSurface->GetMeshList().GetHeadPosition(); Pos;) {
	//	QMeshPatch *layer = (QMeshPatch*)supportIsoSurface->GetMeshList().GetNext(Pos);

	//	bool supportLayer = false;
	//	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
	//		QMeshFace *face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
	//		if (face->importantSupportFace && face->printLayerSupportFace)
	//		{
	//			supportLayer = true; break;
	//		}
	//	}

	//	if (supportLayer) {
	//		for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
	//			QMeshFace *face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
	//			if (face->printLayerSupportFace)
	//				face->importantSupportFace = true;
	//		}
	//	}


	//}
	printf("finish delete non-importnat support face\n\n");
	*/
}


bool FabricationProcess::compareVector(Eigen::Vector4i& a, Eigen::Vector4i& b) {

	bool match[4] = { false };
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (a(i) == b(j)) {
				match[i] = true; break;
			}
		}
	}
	for (int i = 0; i < 4; i++) {
		if (match[i] == false) return false;
	}
	return true;
}

void FabricationProcess::runSupportRegionCompute(bool checkSingleBelow) {

	//---Step 1: recompute the normal for the mesh
	int layerindex = 0;
	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		layer->SetIndexNo(layerindex + 1); layerindex++; //start from 1
		for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace *Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
			Face->CalPlaneEquation();
		}

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->CalNormal();
		}
	}

	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge *Edge = (QMeshEdge*)layer->GetEdgeList().GetNext(Pos);
			if (Edge->GetLeftFace() == NULL || Edge->GetRightFace() == NULL) {
				Edge->GetStartPoint()->inner = true;
				Edge->GetEndPoint()->inner = true;
			}
		}
	}

	//QMeshPatch *supportStructure = new QMeshPatch;
	//supportStructure->SetIndexNo(isoSurfaceSet->GetMeshList().GetCount()); //index begin from 0
	//isoSurfaceSet->GetMeshList().AddTail(supportStructure);
	//supportStructure->drawThisIsoLayer = true;


	if (checkSingleBelow) {

		//---Step 2: for the first layer, always add the support with initial plane

		QMeshPatch *fLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetHead();
		for (GLKPOSITION Pos = fLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)fLayer->GetNodeList().GetNext(Pos);
			Node->supportNode = true;
			Eigen::Vector3d orig; Node->GetCoord3D(orig(0), orig(1), orig(2));
			Eigen::Vector3d dir; Node->GetNormal(dir(0), dir(1), dir(2));

			Eigen::Vector3d n; n << 0, 1, 0;
			double t = -n.dot(orig) / n.dot(dir);
			Eigen::Vector3d insertP = orig + t*dir;

			Node->supportEndPos = insertP;

			//buildSupportingNode(supportStructure, insertP, orig);

		}

		//---Step 3: for the rest layer, always add the support with initial plane

		int layerIndex = 0;
		for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch *bLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);
			layerIndex++;

			//if (layerIndex != 120) continue;
			if (layerIndex > 180) break;

			GLKPOSITION PosNext = isoSurfaceSet->GetMeshList().Find(bLayer)->next;
			QMeshPatch* tLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetAt(PosNext);
			if (tLayer->isSupport) break;

			for (GLKPOSITION Pos = tLayer->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode *Node = (QMeshNode*)tLayer->GetNodeList().GetNext(Pos);

				if (Node->inner == true) continue;

				Eigen::Vector3d orig; Node->GetCoord3D(orig(0), orig(1), orig(2));
				Eigen::Vector3d dir; Node->GetNormal(dir(0), dir(1), dir(2));

				int neighNum = 0;
				//if (Node->inner == true) {
				//	for (GLKPOSITION Pos = Node->GetEdgeList().GetHeadPosition(); Pos;) {
				//		QMeshEdge *thisEdge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos);

				//		QMeshNode *neighNode = thisEdge->GetStartPoint();
				//		if (thisEdge->GetStartPoint() == Node) neighNode = thisEdge->GetEndPoint();

				//		if (neighNode->inner) continue;

				//		Eigen::Vector3d dirNeighbor;
				//		Node->GetNormal(dirNeighbor(0), dirNeighbor(1), dirNeighbor(2));

				//		dir << 0.0, 0.0, 0.0;
				//		dir += dirNeighbor;
				//		neighNum++;
				//	}
				//	//dir /= Node->GetEdgeNumber();
				//	if (neighNum == 0) continue;
				//	dir /= neighNum;

				//}
				dir = dir.normalized();

				Eigen::Vector3d insertP, v1, v2, v3;

				bool overhang = true;

				for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
					QMeshFace *Face = (QMeshFace*)tLayer->GetFaceList().GetNext(Pos);

					Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
					Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
					Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

					Vector3d va = (v1 + v2 + v3) / 3; double ratio = 0.1;
					v1 = v1 + ratio*(v1 - va);
					v2 = v2 + ratio*(v2 - va);
					v3 = v3 + ratio*(v3 - va);


					if (intersetTriangle(orig, dir, v1, v2, v3, insertP)) {
						//std::cout << "find interset Point" << endl;
						overhang = false;
						break;
					}

				}

				if (overhang) {
					Node->supportNode = true;

					//std::cout << "overhang node! " << endl;
					Eigen::Vector3d n; n << 0, 1, 0;
					double t = -n.dot(orig) / n.dot(dir);
					Eigen::Vector3d insertP = orig + t*dir;

					Node->supportEndPos = insertP;

					//buildSupportingNode(supportStructure, insertP, orig);

				}

			}
			std::cout << layerIndex << std::endl;
		}
	}

	else {
		//boundary node to detect 2-ring boundary

		for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);
			for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
				if (Node->inner == true) {

					for (GLKPOSITION Pos = Node->GetEdgeList().GetHeadPosition(); Pos;) {
						QMeshEdge* edge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos);
						QMeshNode* neighborNode = edge->GetStartPoint();
						if (Node == neighborNode) neighborNode = edge->GetEndPoint();

						neighborNode->tworingBoundary = true;
					}
					Node->tworingBoundary = true;
				}
			}
		}

		//compute the boundary of given model
		QMeshPatch *tetMesh = (QMeshPatch *)tetrahedralModel->GetMeshList().GetHead();

		minZBox = 99999.999, maxZBox = -99999.999;
		for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
			double pp[3]; Node->GetCoord3D(pp);
			if (pp[2] > maxZBox) maxZBox = pp[2];
			if (pp[2] < minZBox) minZBox = pp[2];
		}
		std::cout << minZBox << "," << maxZBox << std::endl;

#pragma omp parallel   
		{
#pragma omp for
			for (int CPUcore = 0; CPUcore < CPUCoreNum; CPUcore++) {
				for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
					QMeshPatch *tLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);


					if (tLayer->GetIndexNo() % CPUCoreNum != CPUcore) continue;
					//if (tLayer->GetIndexNo() > 32) break;
					std::cout << "Check Layer No. " << tLayer->GetIndexNo() << std::endl;

					Eigen::Vector3d insertP, v1, v2, v3;
					Eigen::Vector3d platform_Normal; platform_Normal << 0, 1, 0;

					/*-- First layer always need to have support with platform --*/
					if (tLayer->GetIndexNo() == 1) {

						for (GLKPOSITION Pos = tLayer->GetNodeList().GetHeadPosition(); Pos;) {
							QMeshNode *Node = (QMeshNode*)tLayer->GetNodeList().GetNext(Pos);
							Node->supportNode = true;

							//if (Node->inner == true) continue;
							Eigen::Vector3d orig; Node->GetCoord3D(orig(0), orig(1), orig(2));
							Eigen::Vector3d dir; Node->GetNormal(dir(0), dir(1), dir(2));

							double t = -platform_Normal.dot(orig) / platform_Normal.dot(dir);
							insertP = orig + t*dir;

							Node->supportEndPos = insertP;

							//this->buildSupportingNode(supportStructure, insertP, orig);

						}

						continue;
					}

					/*-- For the rest, build a bottom layer set and check --*/

					std::vector<QMeshPatch*> bLayerSet; bLayerSet.resize(tLayer->GetIndexNo() - 1);
					for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
						QMeshPatch *bLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);
						if (bLayer->GetIndexNo() < tLayer->GetIndexNo())
							bLayerSet[bLayer->GetIndexNo() - 1] = bLayer;
					}

					for (GLKPOSITION Pos = tLayer->GetNodeList().GetHeadPosition(); Pos;) {
						QMeshNode *Node = (QMeshNode*)tLayer->GetNodeList().GetNext(Pos);

						//if (Node->inner == true) continue;
						//if (Node->tworingBoundary == true) continue;

						Eigen::Vector3d orig; Node->GetCoord3D(orig(0), orig(1), orig(2));
						Eigen::Vector3d dir; Node->GetNormal(dir(0), dir(1), dir(2));

						bool overhang_Node = true;

						//first check if this node have interset with its below layer
						QMeshPatch *bLayer = bLayerSet[bLayerSet.size() - 1];
						for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
							QMeshFace *Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);

							Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
							Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
							Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

							Vector3d va = (v1 + v2 + v3) / 3; double ratio = 0.2; //expanding
							v1 = v1 + ratio*(v1 - va);
							v2 = v2 + ratio*(v2 - va);
							v3 = v3 + ratio*(v3 - va);

							if (intersetTriangle(orig, dir, v1, v2, v3, insertP)) {
								overhang_Node = false; break;
							}
						}

						if (!overhang_Node) continue;
						else {
							Node->supportNode = true;

							//if this node is overhang node, check it it will interset with all the surface below
							bool support_withplatform = true;

							for (int i = bLayerSet.size() - 1; i > 0; i--) {
								QMeshPatch *bLayer = bLayerSet[i - 1];
								for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
									QMeshFace *Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);

									Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
									Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
									Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

									Vector3d va = (v1 + v2 + v3) / 3; double ratio = 0.1; //expanding
									v1 = v1 + ratio*(v1 - va);
									v2 = v2 + ratio*(v2 - va);
									v3 = v3 + ratio*(v3 - va);

									if (intersetTriangle(orig, dir, v1, v2, v3, insertP)) {
										support_withplatform = false; break;
									}
								}
								if (!support_withplatform) break;
							}

							if (support_withplatform) {
								//std::cout << "overhang node! " << endl;
								Eigen::Vector3d n; n << 0, 1, 0;
								
								if (fabs(n.dot(dir)) < 0.8) {
									dir = (dir * 0.5 - n * 0.5).normalized();
									std::cout << "flat the normal!" << std::endl;
									//dir = n;
								}
									

								double t = -n.dot(orig) / n.dot(dir);
								insertP = orig + t*dir;
							}

							Node->supportEndPos = insertP;

							//this->buildSupportingNode(supportStructure, insertP, orig);


						}
					}


				}
			}
		}
	}

	printf("finish detect the support node, all support ray end point is installed!\n\n");

	/*std::cout << " -- Finish build the supporting node, Number = " << 
		supportStructure->GetNodeNumber() << std::endl << std::endl;*/
}

void FabricationProcess::computeSupportNode(PolygenMesh *supportRegion) {

	QMeshPatch *supportNode = new QMeshPatch;
	supportNode->SetIndexNo(supportRegion->GetMeshList().GetCount()); //index begin from 0
	supportRegion->GetMeshList().AddTail(supportNode);

	supportNode->isSupport = true;


	for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
	
			if (Node->supportNode) {

				Eigen::Vector3d orig; 
				Node->GetCoord3D(orig(0), orig(1), orig(2));
				this->buildSupportingNode(supportNode, Node->supportEndPos, orig, Node);

			}
		
		}
	}

	std::cout << " -- Finish build the supporting node, Number = " <<
		supportNode->GetNodeNumber() << std::endl << std::endl;

}

void FabricationProcess::outputConvexHullandTetrahedralMesh(PolygenMesh *supportRegion) {

	//---- export tet mesh ----//

	QMeshPatch *tetMesh = (QMeshPatch *)tetrahedralModel->GetMeshList().GetHead();
	std::string path = "../Model/IsoSurface/tetMesh_rotate.tet";

	double pp[3];

	//make sure the inner / outer is detect 
	for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode *node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);
		node->inner = true;
	}
	for (GLKPOSITION posFace = tetMesh->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace *face = (QMeshFace*)tetMesh->GetFaceList().GetNext(posFace);
		if (face->GetLeftTetra() != nullptr && face->GetRightTetra() != nullptr)
			face->inner = true;
		else {
			face->inner = false;
			for (int i = 0; i < 3; i++) {
				face->GetNodeRecordPtr(i)->inner = false;
			}
		}

	}

	//make order of the boundary node output
	int outerNum = 0;
	for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode *node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);
		if (!node->inner) {
			node->outerIndex = outerNum;
			outerNum++;
		}
	}

	//----tet file output
	ofstream tet_output(path);
	tet_output << tetMesh->GetNodeNumber() << " vertices" << endl;
	tet_output << tetMesh->GetTetraNumber() << " tets" << endl;

	int index = 0;
	for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode *node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);
		node->SetIndexNo(index); index++;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		tet_output << pp[0] << " " << pp[1] << " " << pp[2] << endl;
	}
	for (GLKPOSITION posFace = tetMesh->GetTetraList().GetHeadPosition(); posFace != nullptr;) {
		QMeshTetra *tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(posFace);
		tet_output << "4 " << tet->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(2)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(3)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(4)->GetIndexNo() << endl;
	}
	tet_output.close();

	//----obj file output
	path = "../Model/IsoSurface/tetMesh_rotate_boundary.obj";

	ofstream obj_output(path);
	for (GLKPOSITION posNode = tetMesh->GetFaceList().GetHeadPosition(); posNode != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(posNode);
		face->tetInnerFace = true;
		if (face->GetLeftTetra() == false || face->GetRightTetra() == false) {
			face->tetInnerFace = false;
			for (int i = 0; i < 3; i++) 
				face->GetNodeRecordPtr(i)->tetInnerNode = false;	
		}
	}
	index = 0;
	for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);
		if (node->tetInnerNode) continue;
		index++; node->tetInnerNodeIndex = index;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		obj_output << "v " << pp[0] << " " << pp[1] << " " << pp[2] << endl;
	}
	for (GLKPOSITION posNode = tetMesh->GetFaceList().GetHeadPosition(); posNode != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(posNode);
		if (face->tetInnerFace) continue;
		obj_output << "f " << face->GetNodeRecordPtr(0)->tetInnerNodeIndex << " "
			<< face->GetNodeRecordPtr(1)->tetInnerNodeIndex << " " << face->GetNodeRecordPtr(2)->tetInnerNodeIndex << endl;
	}
	obj_output.close();

	//---- export convexHULL mesh ----//

	QMeshPatch *convexHull = (QMeshPatch *)supportRegion->GetMeshList().GetTail();
	
	path = "../Model/IsoSurface/convex_hull.obj";
	ofstream obj_Output(path);

	for (GLKPOSITION posNode = convexHull->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode *node = (QMeshNode*)convexHull->GetNodeList().GetNext(posNode);
		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		float r, g, b;
		node->GetColor(r, g, b);
		obj_Output << "v " << xx << " " << yy << " " << zz << " " << node->value1 << endl;
	}
	for (GLKPOSITION posFace = convexHull->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace *face = (QMeshFace*)convexHull->GetFaceList().GetNext(posFace);
		obj_Output << "f " << face->GetNodeRecordPtr(0)->GetIndexNo() << " " << 
			face->GetNodeRecordPtr(1)->GetIndexNo() << " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
	}

	obj_Output.close();

}

void FabricationProcess::buildSupportingNode(
	QMeshPatch *supportStructure, Eigen::Vector3d& insertP, Eigen::Vector3d& orig, QMeshNode* connectedNode){

	//if (insertP(2) > maxZBox || insertP(2) < minZBox) return; //only for 2D case
	//if (fabs(insertP(2)) > 300 || fabs(insertP(0)) > 300) return;
	//else {

		/*double radius = sqrt(insertP(2) * insertP(2) + insertP(0) * insertP(0));
		if (radius > 90) {
			insertP(2) /= (radius / 90);
			insertP(0) /= (radius / 90);
		}*/

	//}
	//int nodeNum = 5;

	int nodeNum = (insertP - orig).norm() / supportNodeGap + 1;

	for (int i = 0; i < nodeNum + 1; i++) {
		Eigen::Vector3d supportNode = insertP * i * 1.0 / nodeNum + orig*(1 - i* 1.0 / nodeNum);

		QMeshNode* isoNode = new QMeshNode;

		isoNode->supportNode = true;
		isoNode->connectedNode = connectedNode;
		isoNode->SetMeshPatchPtr(supportStructure);
		isoNode->SetCoord3D(supportNode[0], supportNode[1], supportNode[2]);
		isoNode->SetIndexNo(supportStructure->GetNodeList().GetCount() + 1);
		supportStructure->GetNodeList().AddTail(isoNode);

	}
}

void FabricationProcess::computeSupportingConvexHull(PolygenMesh *supportRegion) {

	//build Convex-Hull (inclouding supporing point and surface of tetrahedral mesh)
	int pntNum = 0;
	QMeshPatch* supportNodePatch = (QMeshPatch*)supportRegion->GetMeshList().GetTail();
	pntNum += supportNodePatch->GetNodeNumber();

	QMeshPatch *tetMesh = (QMeshPatch *)tetrahedralModel->GetMeshList().GetHead();
	pntNum += tetMesh->GetNodeNumber();

	facetT *facet;		vertexT *vertex, **vertexp;
	int i, index, num, stIndex;			float pos[3];
	double vec[3][3], dir[3], v1[3], v2[3], pp[3];
	QHULLSET *newConvexFront = NULL; // new convexhull used for checking

	double *pntArray = (double*)malloc(sizeof(double) * 3 * pntNum); //all the point use to compute convex hull

	int nodeIndex = 0;
	for (GLKPOSITION posMesh = supportNodePatch->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode *node = (QMeshNode*)supportNodePatch->GetNodeList().GetNext(posMesh);
		node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++)
			pntArray[nodeIndex * 3 + i] = pp[i];
		nodeIndex++;
	}
	for (GLKPOSITION posMesh = tetMesh->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode *node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posMesh);
		node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++)
			pntArray[nodeIndex * 3 + i] = pp[i];
		nodeIndex++;
	}

	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	//printf("Convex-Hull: %d faces with %d vertices\n",faceNum,nodeNum);
	if (faceNum>0 && nodeNum>0) {
		newConvexFront = _mallocMemoryConvexHull(faceNum, nodeNum);
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		newConvexFront->vertPos[index * 3] = vertex->point[0];
		newConvexFront->vertPos[index * 3 + 1] = vertex->point[1];
		newConvexFront->vertPos[index * 3 + 2] = vertex->point[2];
		index++;
		}
			//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			newConvexFront->normalVec[index * 3] = facet->normal[0];
		newConvexFront->normalVec[index * 3 + 1] = facet->normal[1];
		newConvexFront->normalVec[index * 3 + 2] = facet->normal[2];
		newConvexFront->offset[index] = facet->offset;
		//	It has been verified all normal[] vectors generated by qhull library are pointing outwards and are unit-vectors 
		//		(verified by the function -- QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)  ).

		int i = 0;
		FOREACHvertex_(facet->vertices) {
			newConvexFront->faceTable[index * 3 + i] = vertex->id + 1; //index start from 1;
																	   //newConvexFront->faceTable[index * 3 + i] = vertex->id; //index start from 0;

			SET(vec[i],vertex->point);
			i++;
			if (i >= 3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		if (DOT(dir,facet->normal)<0) {
			unsigned int temp = newConvexFront->faceTable[index * 3];
			newConvexFront->faceTable[index * 3] = newConvexFront->faceTable[index * 3 + 2];
			newConvexFront->faceTable[index * 3 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);

	_drawConvexHull(newConvexFront, supportRegion);
}

QHULLSET* FabricationProcess::computeNozzleConvexHull(QMeshPatch* nozzleMesh) {

	//build Convex-Hull (inclouding supporing point and surface of tetrahedral mesh)

	facetT* facet;		vertexT* vertex, ** vertexp;
	int i, index, pntNum, num, stIndex;			float pos[3];
	double vec[3][3], dir[3], v1[3], v2[3], pp[3];
	QHULLSET* newConvexFront = NULL; // new convexhull used for checking

	pntNum = nozzleMesh->GetNodeNumber();
	double* pntArray = (double*)malloc(sizeof(double) * 3 * pntNum); //all the point use to compute convex hull

	int nodeIndex = 0;
	for (GLKPOSITION posMesh = nozzleMesh->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)nozzleMesh->GetNodeList().GetNext(posMesh);
		double pp[3]; node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++)
			pntArray[nodeIndex * 3 + i] = pp[i];
		nodeIndex++;
	}
	
	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	//printf("Convex-Hull: %d faces with %d vertices\n",faceNum,nodeNum);
	if (faceNum > 0 && nodeNum > 0) {
		newConvexFront = _mallocMemoryConvexHull(faceNum, nodeNum);
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		newConvexFront->vertPos[index * 3] = vertex->point[0];
		newConvexFront->vertPos[index * 3 + 1] = vertex->point[1];
		newConvexFront->vertPos[index * 3 + 2] = vertex->point[2];
		index++;
		}
			//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			newConvexFront->normalVec[index * 3] = facet->normal[0];
		newConvexFront->normalVec[index * 3 + 1] = facet->normal[1];
		newConvexFront->normalVec[index * 3 + 2] = facet->normal[2];
		newConvexFront->offset[index] = facet->offset;
		//	It has been verified all normal[] vectors generated by qhull library are pointing outwards and are unit-vectors 
		//		(verified by the function -- QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)  ).

		int i = 0;
		FOREACHvertex_(facet->vertices) {
			newConvexFront->faceTable[index * 3 + i] = vertex->id + 1; //index start from 1;
																	   //newConvexFront->faceTable[index * 3 + i] = vertex->id; //index start from 0;

			SET(vec[i],vertex->point);
			i++;
			if (i >= 3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		if (DOT(dir,facet->normal) < 0) {
			unsigned int temp = newConvexFront->faceTable[index * 3];
			newConvexFront->faceTable[index * 3] = newConvexFront->faceTable[index * 3 + 2];
			newConvexFront->faceTable[index * 3 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);
	return newConvexFront;
}


bool FabricationProcess::intersetTriangle(Eigen::Vector3d& orig, Eigen::Vector3d& dir,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& insertP) {

	/* https://www.cnblogs.com/graphics/archive/2010/08/09/1795348.html */

	double t, u, v;

	// E1,E2,P
	Vector3d E1 = v1 - v0;	Vector3d E2 = v2 - v0;
	Vector3d P = dir.cross(E2);

	// determinant
	double det = E1.dot(P);

	// keep det > 0, modify T accordingly
	Vector3d T;
	if (det >0)
	{
		T = orig - v0;
	}
	else
	{
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001) {
		//std::cout << "this node lies in plane!!!" << std::endl;
		return false;
	}

	// Calculate u and make sure u <= 1
	u = T.dot(P);
	if (u < 0.0f || u > det)
	//if (u < -0.5f || u > det*1.5)
		return false;

	// Q
	Vector3d Q = T.cross(E1);

	// Calculate v and make sure u + v <= 1
	v = dir.dot(Q);
	if (v < 0.0f || u + v > det)
	//if (v < -0.5f || u + v > det*1.5)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = E2.dot(Q);

	float fInvDet = 1.0f / det;
	t *= fInvDet;
	u *= fInvDet;
	v *= fInvDet;

	insertP = orig + t*dir;
	//std::cout << insertP << endl;
	//std::cout << (1 - u - v)*v0 + u*v1 + v*v2 << endl << endl;

	return true;
}

bool FabricationProcess::intersetTriangle_supportStructure(Eigen::Vector3d& orig, Eigen::Vector3d& endNode,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2) {

	Eigen::Vector3d dir = (endNode - orig).normalized();
	double linePara = (endNode - orig).norm();

	/* https://www.cnblogs.com/graphics/archive/2010/08/09/1795348.html */

	double t, u, v;

	// E1,E2,P
	Vector3d E1 = v1 - v0;	Vector3d E2 = v2 - v0;
	Vector3d P = dir.cross(E2);

	// determinant
	double det = E1.dot(P);

	// keep det > 0, modify T accordingly
	Vector3d T;
	if (det >0)
	{
		T = orig - v0;
	}
	else
	{
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001) {
		//std::cout << "this node lies in plane!!!" << std::endl;
		return false;
	}

	// Calculate u and make sure u <= 1
	u = T.dot(P);
	if (u < 0.0f || u > det)
		//if (u < -0.5f || u > det*1.5)
		return false;

	// Q
	Vector3d Q = T.cross(E1);

	// Calculate v and make sure u + v <= 1
	v = dir.dot(Q);
	if (v < 0.0f || u + v > det)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = E2.dot(Q);

	float fInvDet = 1.0f / det;
	t *= fInvDet;
	u *= fInvDet;
	v *= fInvDet;

	if (t<0 || t>linePara) return false;

	return true;
}



double FabricationProcess::compShadowVolume(QMeshPatch* Layer, QHULLSET *currentConvexFront){

	//first detect if all the point in layer is inside the convexhull
	double pnt[3]; 
	double shadowVolume = 0.0;
	
	for (GLKPOSITION Pos = Layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)Layer->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pnt[0], pnt[1], pnt[2]);
		Node->isShadowNode = _isPntInsideConvexHull(currentConvexFront, pnt);
		if (Node->isShadowNode) { Node->shadowDis = _compDistanceToConvexFront(pnt, currentConvexFront); }
	}

	for (GLKPOSITION Pos = Layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)Layer->GetFaceList().GetNext(Pos);
		double shadowDistance = 0.0;
		if (Face->GetNodeRecordPtr(0)->isShadowNode&&
			Face->GetNodeRecordPtr(1)->isShadowNode&&Face->GetNodeRecordPtr(2)->isShadowNode) {
			for (int i = 0; i < 3; i++) 
				shadowDistance += Face->GetNodeRecordPtr(i)->shadowDis;
			shadowVolume += Face->CalArea()*shadowDistance / 3;
		}
	}
	
	return shadowVolume;
}

double FabricationProcess::_compShadowVolume(
	QMeshPatch* Layer, QHULLSET* currentConvexFront, Eigen::MatrixXd& layerCoord) {

	//first detect if all the point in layer is inside the convexhull
	double pnt[3];
	double shadowVolume = 0.0;
	Eigen::VectorXd nodeShadowDis = Eigen::VectorXd::Zero(Layer->GetNodeNumber());
	std::vector<int> nodeShadowFlag(Layer->GetNodeNumber());

	for (GLKPOSITION Pos = Layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)Layer->GetNodeList().GetNext(Pos);


		Node->GetCoord3D(pnt[0], pnt[1], pnt[2]);
		for (int i = 0; i < 3; i++) pnt[i] = layerCoord(Node->GetIndexNo(), i);

		if (_isPntInsideConvexHull(currentConvexFront, pnt)) {
			nodeShadowDis(Node->GetIndexNo()) = _compDistanceToConvexFront(pnt, currentConvexFront);
			nodeShadowFlag[Node->GetIndexNo()] = true;
		}
		else nodeShadowFlag[Node->GetIndexNo()] = false;
	}
		
	for (GLKPOSITION Pos = Layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)Layer->GetFaceList().GetNext(Pos);
		double shadowDistance = 0.0;

		if (nodeShadowFlag[Face->GetNodeRecordPtr(0)->GetIndexNo()] &&
			nodeShadowFlag[Face->GetNodeRecordPtr(1)->GetIndexNo()] &&
			nodeShadowFlag[Face->GetNodeRecordPtr(2)->GetIndexNo()]) {
			for (int i = 0; i < 3; i++)
				shadowDistance += nodeShadowDis(Face->GetNodeRecordPtr(i)->GetIndexNo());
			shadowVolume += Face->CalArea() * shadowDistance / 3;
		}
	}

	return shadowVolume;
}

bool FabricationProcess::rotateLayerandCenter_checkOrder(int degree) {
	double pp[3];
	double ymin = 1e+8;
	// double offsetY = 10.0 * 1.62; // topopt model
	//double offsetY = 20.0; // bridge model
	double offsetY = 10.0; // bridge model

	double modelCentral[3] = { 0 };
	int nodeNumSum = 0;

	//rotate the model

	double theta = DEGREE_TO_ROTATE(degree);
	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);

		/*rotate the isosurface potision*/
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D_last(pp[0], pp[1], pp[2]);
			double xnew = pp[0] * cos(theta) - pp[1] * sin(theta);
			double ynew = pp[0] * sin(theta) + pp[1] * cos(theta);
			Node->SetCoord3D(xnew, ynew, pp[2]);
		}
		/*rotate the principle stress direction*/
		for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
			Eigen::Vector3d tetVector = Face->principleStressDir;
			double vectorX = tetVector(0) * cos(theta) - tetVector(1) * sin(theta);
			double vectorY = tetVector(0) * sin(theta) + tetVector(1) * cos(theta);
			Face->principleStressDir << vectorX, vectorY, tetVector(2);
		}
	}

	//also move and rotate the input initial tetrahderal model
	QMeshPatch *tetMesh = (QMeshPatch *)tetrahedralModel->GetMeshList().GetHead();
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->GetCoord3D_last(pp[0], pp[1], pp[2]);

		double xnew = pp[0] * cos(theta) - pp[1] * sin(theta);
		double ynew = pp[0] * sin(theta) + pp[1] * cos(theta);
		Node->SetCoord3D(xnew, ynew, pp[2]);
	}

	//also rotate the direction of vector field
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Eigen::Vector3d tetVector = tet->vectorField;
		double vectorX = tetVector(0) * cos(theta) - tetVector(1) * sin(theta);
		double vectorY = tetVector(0) * sin(theta) + tetVector(1) * cos(theta);
		tet->vectorField << vectorX, vectorY, tetVector(2);
	}

	//-------------------------------------
	//move model to the center and slightly higher than the platform.	
	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			if (pp[1] < ymin) ymin = pp[1];
			for (int i = 0; i < 3; i++) modelCentral[i] += pp[i];
		}
		nodeNumSum += layer->GetNodeNumber();
	}

	for (int i = 0; i < 3; i++) modelCentral[i] = modelCentral[i] / nodeNumSum;

	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			pp[0] -= modelCentral[0];
			pp[2] -= modelCentral[2];
			pp[1] -= (ymin - offsetY);
			Node->SetCoord3D(pp[0], pp[1], pp[2]);
		}
	}

	//-------------------------------------
	//also move and rotate the input initial tetrahderal model
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);

		pp[0] -= modelCentral[0];
		pp[2] -= modelCentral[2];
		pp[1] -= (ymin - offsetY);

		Node->SetCoord3D(pp[0], pp[1], pp[2]);
	}




	//Detect the order of the layer, if the top layer is lower than the bottom one switch the order.
	bool order = true;
	QMeshPatch *firstLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetHead();
	QMeshPatch *lastLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetTail();
	double yfmin = 1e+8, ylmin = 1e+8;
	for (GLKPOSITION Pos = firstLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)firstLayer->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < yfmin) yfmin = pp[1];
	}
	for (GLKPOSITION Pos = lastLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)lastLayer->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < ylmin) ylmin = pp[1];
	}
	if (yfmin > ylmin) order = false;

	return order;
}


QHULLSET* FabricationProcess::_mallocMemoryConvexHull(int faceNum, int vertNum)
{
	QHULLSET* pConvexHull;

	pConvexHull = (QHULLSET*)malloc(sizeof(QHULLSET));
	pConvexHull->faceNum = faceNum;
	pConvexHull->normalVec = (double*)malloc(sizeof(double) * 3 * faceNum);
	pConvexHull->offset = (double*)malloc(sizeof(double)*faceNum);

	pConvexHull->faceTable = (unsigned int*)malloc(sizeof(unsigned int) * 3 * faceNum);

	pConvexHull->vertNum = vertNum;
	pConvexHull->vertPos = (double*)malloc(sizeof(double) * 3 * vertNum);

	return pConvexHull;
}

void FabricationProcess::_freeMemoryConvexHull(QHULLSET *&pConvexHull)
{
	free((pConvexHull->normalVec));
	free((pConvexHull->offset));
	free((pConvexHull->faceTable));
	free((pConvexHull->vertPos));
	free(pConvexHull);

	pConvexHull = NULL;
}

QHULLSET* FabricationProcess::_constructNewConvexFront(
	QMeshPatch* curveLayer, QHULLSET* currentConvexFront, PolygenMesh* platform)
{
	facetT *facet;		vertexT *vertex, **vertexp;
	int i, index, pntNum, num, stIndex;			float pos[3];
	double vec[3][3], dir[3], v1[3], v2[3], pp[3];
	QHULLSET *newConvexFront = NULL; // new convexhull used for checking

	//-------------------------------------------------------------------------------------
	//	Step 1: initialization
	pntNum = curveLayer->GetNodeNumber();
	QMeshPatch* platformMesh;
	if (printingPlatform != NULL) platformMesh = (QMeshPatch*)platform->meshList.GetHead();
	else platformMesh = NULL;
	if (currentConvexFront != NULL) { pntNum += currentConvexFront->vertNum; }
	if (platformMesh != NULL) { pntNum += platformMesh->GetNodeNumber(); }
	double *pntArray = (double*)malloc(sizeof(double) * 3 * pntNum); //all the point use to compute convex hull
	
	int nodeIndex = 0;
	for (GLKPOSITION posMesh = curveLayer->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode *node = (QMeshNode*)curveLayer->GetNodeList().GetNext(posMesh);
		node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++) 
			pntArray[nodeIndex * 3 + i] = pp[i]; 
		nodeIndex++;
	}
	stIndex = nodeIndex;
	if (currentConvexFront != NULL) {
		for (i = 0; i<currentConvexFront->vertNum; i++) {
			for (int j = 0; j < 3; j++)
				pntArray[stIndex * 3 + j] = currentConvexFront->vertPos[i * 3 + j];
			stIndex++;
		}
	}
	if (platformMesh != NULL) {
		for (GLKPOSITION posMesh = platformMesh->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshNode *node = (QMeshNode*)platformMesh->GetNodeList().GetNext(posMesh);
			node->GetCoord3D(pp);
			for (int i = 0; i < 3; i++) 
				pntArray[stIndex * 3 + i] = pp[i]; 
			stIndex++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	//printf("Convex-Hull: %d faces with %d vertices\n",faceNum,nodeNum);
	if (faceNum>0 && nodeNum>0) {
		newConvexFront = _mallocMemoryConvexHull(faceNum, nodeNum);
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		newConvexFront->vertPos[index * 3] = vertex->point[0];
		newConvexFront->vertPos[index * 3 + 1] = vertex->point[1];
		newConvexFront->vertPos[index * 3 + 2] = vertex->point[2];
		index++;
		}
			//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			newConvexFront->normalVec[index * 3] = facet->normal[0];
		newConvexFront->normalVec[index * 3 + 1] = facet->normal[1];
		newConvexFront->normalVec[index * 3 + 2] = facet->normal[2];
		newConvexFront->offset[index] = facet->offset;
		//	It has been verified all normal[] vectors generated by qhull library are pointing outwards and are unit-vectors 
		//		(verified by the function -- QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)  ).

		int i = 0;
		FOREACHvertex_(facet->vertices) {
			newConvexFront->faceTable[index * 3 + i] = vertex->id + 1; //index start from 1;
			//newConvexFront->faceTable[index * 3 + i] = vertex->id; //index start from 0;

			SET(vec[i],vertex->point);
			i++;
			if (i >= 3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		if (DOT(dir,facet->normal)<0) {
			unsigned int temp = newConvexFront->faceTable[index * 3];
			newConvexFront->faceTable[index * 3] = newConvexFront->faceTable[index * 3 + 2];
			newConvexFront->faceTable[index * 3 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);

	return newConvexFront;
}

QHULLSET* FabricationProcess::_constructNewConvexFront(
	QMeshPatch* curveLayer, QHULLSET* currentConvexFront, PolygenMesh* platform, Eigen::MatrixXd& layerCoord)
{
	facetT* facet;		vertexT* vertex, ** vertexp;
	int i, index, pntNum, num, stIndex;			float pos[3];
	double vec[3][3], dir[3], v1[3], v2[3], pp[3];
	QHULLSET* newConvexFront = NULL; // new convexhull used for checking

	//-------------------------------------------------------------------------------------
	//	Step 1: initialization
	pntNum = curveLayer->GetNodeNumber();
	QMeshPatch* platformMesh;
	if (printingPlatform != NULL) platformMesh = (QMeshPatch*)platform->meshList.GetHead();
	else platformMesh = NULL;
	if (currentConvexFront != NULL) { pntNum += currentConvexFront->vertNum; }
	if (platformMesh != NULL) { pntNum += platformMesh->GetNodeNumber(); }
	double* pntArray = (double*)malloc(sizeof(double) * 3 * pntNum); //all the point use to compute convex hull

	int nodeIndex = 0;
	for (GLKPOSITION posMesh = curveLayer->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)curveLayer->GetNodeList().GetNext(posMesh);
		for (int i = 0; i < 3; i++) pntArray[nodeIndex * 3 + i] = layerCoord(node->GetIndexNo(), i);
		nodeIndex++;
	}
	stIndex = nodeIndex;
	if (currentConvexFront != NULL) {
		for (i = 0; i < currentConvexFront->vertNum; i++) {
			for (int j = 0; j < 3; j++)
				pntArray[stIndex * 3 + j] = currentConvexFront->vertPos[i * 3 + j];
			stIndex++;
		}
	}
	if (platformMesh != NULL) {
		for (GLKPOSITION posMesh = platformMesh->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshNode* node = (QMeshNode*)platformMesh->GetNodeList().GetNext(posMesh);
			node->GetCoord3D(pp);
			for (int i = 0; i < 3; i++)
				pntArray[stIndex * 3 + i] = pp[i];
			stIndex++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	//printf("Convex-Hull: %d faces with %d vertices\n",faceNum,nodeNum);
	if (faceNum > 0 && nodeNum > 0) {
		newConvexFront = _mallocMemoryConvexHull(faceNum, nodeNum);
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		newConvexFront->vertPos[index * 3] = vertex->point[0];
		newConvexFront->vertPos[index * 3 + 1] = vertex->point[1];
		newConvexFront->vertPos[index * 3 + 2] = vertex->point[2];
		index++;
		}
			//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			newConvexFront->normalVec[index * 3] = facet->normal[0];
		newConvexFront->normalVec[index * 3 + 1] = facet->normal[1];
		newConvexFront->normalVec[index * 3 + 2] = facet->normal[2];
		newConvexFront->offset[index] = facet->offset;
		//	It has been verified all normal[] vectors generated by qhull library are pointing outwards and are unit-vectors 
		//		(verified by the function -- QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)  ).

		int i = 0;
		FOREACHvertex_(facet->vertices) {
			newConvexFront->faceTable[index * 3 + i] = vertex->id + 1; //index start from 1;
			//newConvexFront->faceTable[index * 3 + i] = vertex->id; //index start from 0;

			SET(vec[i],vertex->point);
			i++;
			if (i >= 3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		if (DOT(dir,facet->normal) < 0) {
			unsigned int temp = newConvexFront->faceTable[index * 3];
			newConvexFront->faceTable[index * 3] = newConvexFront->faceTable[index * 3 + 2];
			newConvexFront->faceTable[index * 3 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);

	return newConvexFront;
}

bool FabricationProcess::_isPntInsideConvexHull(QHULLSET *pConvexHull, double pnt[]) {
	double normVec[3], offValue;

	for (int i = 0; i<pConvexHull->faceNum; i++) {
		normVec[0] = pConvexHull->normalVec[i * 3];
		normVec[1] = pConvexHull->normalVec[i * 3 + 1];
		normVec[2] = pConvexHull->normalVec[i * 3 + 2];
		offValue = pConvexHull->offset[i];
		if ((DOT(pnt, normVec) + offValue) >= 0.0) return false;

	}
	return true;
}

double FabricationProcess::_compDistanceToConvexFront(double pos[], QHULLSET *pConvexHull)
{
	double pc[3], dist, minDist;    int i, pIndex;

	//----------------------------------------------------------------------------------------------------------------------------------------
	//	Search the closest plane in the convex hull
	minDist = 1.0e+5;
	for (i = 0; i<pConvexHull->faceNum; i++) {
		pIndex = pConvexHull->faceTable[i * 3] - 1;
		pc[0] = pConvexHull->vertPos[pIndex * 3];		pc[1] = pConvexHull->vertPos[pIndex * 3 + 1];		pc[2] = pConvexHull->vertPos[pIndex * 3 + 2];

		dist = fabs((pos[0] - pc[0])*pConvexHull->normalVec[i * 3] + 
			(pos[1] - pc[1])*pConvexHull->normalVec[i * 3 + 1] + (pos[2] - pc[2])*pConvexHull->normalVec[i * 3 + 2]);

		if (dist<minDist) { minDist = dist; }
	}
	return minDist;
}

void FabricationProcess::_drawConvexHull(bool order) {
	QHULLSET *currentConvexFront = NULL;
	if (order) {  //first to last
		for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);
			currentConvexFront = _constructNewConvexFront(layer, currentConvexFront, printingPlatform);
		}
	}
	else { //last to first
		for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetTailPosition(); posMesh != nullptr;) {
			QMeshPatch *layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetPrev(posMesh);
			currentConvexFront = _constructNewConvexFront(layer, currentConvexFront, printingPlatform);
		}
	}
	//after checking all the layer, this is used to visulize the convex hull

	QMeshPatch *convexHullVisual = new QMeshPatch;
	convexHullVisual->SetIndexNo(isoSurfaceSet->GetMeshList().GetCount()); //index begin from 0
	isoSurfaceSet->GetMeshList().AddTail(convexHullVisual);

	float* nodeTable;
	nodeTable = (float *)malloc(sizeof(float)*currentConvexFront->vertNum * 3);
	for (int i = 0; i < currentConvexFront->vertNum * 3; i++)
		nodeTable[i] = (float)currentConvexFront->vertPos[i];
	unsigned int* faceTable;
	faceTable = (unsigned int *)malloc(sizeof(unsigned int)*currentConvexFront->faceNum * 3);
	for (int i = 0; i < currentConvexFront->faceNum * 3; i++)
		faceTable[i] = currentConvexFront->faceTable[i] - 1;

	convexHullVisual->constructionFromVerFaceTable
	(currentConvexFront->vertNum, nodeTable, currentConvexFront->faceNum, faceTable);
	_freeMemoryConvexHull(currentConvexFront);
	free(nodeTable);	free(faceTable);
}

void FabricationProcess::_drawConvexHull(QHULLSET *ConvexHULL, PolygenMesh *supportRegion) {

	QMeshPatch *convexHullVisual = new QMeshPatch;
	convexHullVisual->isSupportConvexHull = true;
	convexHullVisual->SetIndexNo(supportRegion->GetMeshList().GetCount()); //index begin from 0
	supportRegion->GetMeshList().AddTail(convexHullVisual);

	float* nodeTable;
	nodeTable = (float *)malloc(sizeof(float)*ConvexHULL->vertNum * 3);
	for (int i = 0; i < ConvexHULL->vertNum * 3; i++)
		nodeTable[i] = (float)ConvexHULL->vertPos[i];
	unsigned int* faceTable;
	faceTable = (unsigned int *)malloc(sizeof(unsigned int)*ConvexHULL->faceNum * 3);
	for (int i = 0; i < ConvexHULL->faceNum * 3; i++)
		faceTable[i] = ConvexHULL->faceTable[i] - 1;

	convexHullVisual->constructionFromVerFaceTable
	(ConvexHULL->vertNum, nodeTable, ConvexHULL->faceNum, faceTable);
	_freeMemoryConvexHull(ConvexHULL);
	free(nodeTable);	free(faceTable);
}