#include "stdafx.h"
#include "FabricationDirectionDetection.h"

#define CONVHULL_3D_ENABLE
#include "../QMeshLib/convhull_3d.h"

using namespace std;
using namespace Eigen;

#define DEGREE_TO_ROTATE(x)		0.0174532922222*x

FabricationDirectionDetection::FabricationDirectionDetection(
	PolygenMesh* initMesh, PolygenMesh* platformMesh, PolygenMesh* isoSurfaceMeshSet){
	tetMesh = (QMeshPatch*)initMesh->GetMeshList().GetHead();
	platform = (QMeshPatch*)platformMesh->GetMeshList().GetHead();
	isoSurfaceSet = isoSurfaceMeshSet;
	platformMeshInstall = platformMesh;
}

void FabricationDirectionDetection::runFabDir2DCaseDetection(bool compute, double installedAngle) {

	this->_initializeSystemIndex();
	
	if (compute) {
		
		int searchNum = 72;
		Eigen::VectorXd shadowVolume = Eigen::VectorXd::Zero(searchNum);
		
		// compute shadow volume of every rotation angle - accelerated by openmp
#pragma omp parallel
		{
#pragma omp for
			for (int i = 0; i < searchNum; i++)
				shadowVolume(i) = this->_compShadowVolume_OpenMP(i * 5);
		}
		std::cout << shadowVolume << std::endl;
		// sort the smallest one
		int optimizedDegree = 0; double valueSearch = shadowVolume(0);
		for (int i = 0; i < searchNum; i++) {
			if (shadowVolume(i) < valueSearch) { valueSearch = shadowVolume(i); optimizedDegree = i; }
		}
		installedAngle = optimizedDegree * 5;
		std::cout << "The final detected rotate degree is: " << installedAngle << std::endl;

	}
	else std::cout << "Installed rotate degree is: " << installedAngle << std::endl;

	//-------------------------------------------------------------------------------------------------
	//  Rotate the mesh and iso-surface set
	bool order = this->_finalizePrintingDirandRotate(installedAngle);
	printf("build convex hull takes %ld ms.\n", timeConvexHull);
	/*std::vector<Eigen::MatrixXd> layerCoord(isoSurfaceSet->GetMeshList().GetCount());
	bool order = this->_rotateLayerSetbyMatrixandDetectOrder(0, layerCoord, true);*/

}

void FabricationDirectionDetection::_initializeSystemIndex() {
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

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		double pp[3];
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
	}
}

double FabricationDirectionDetection::_compShadowVolume_OpenMP(int degree) {

	double shadowVolume = 0.0;
	int layerNum = isoSurfaceSet->GetMeshList().GetCount();
	//-------------------------------------------------------------------------------------------------
	// Build iso-surface coordinate matrix and detect order flip

	std::vector<Eigen::MatrixXd> layerCoord(layerNum);
	bool fliporder = this->_rotateLayerSetbyMatrixandDetectOrder(degree, layerCoord, false);

	//-------------------------------------------------------------------------------------------------
	// Compute the shadow volume for each iso-surface

	QMeshPatch* convexHull = new QMeshPatch;
	if (fliporder == false) {
		//std::cout << "not flip the order!" << std::endl;
		for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);


			/* update the convexhull mesh */
			if(layer->GetIndexNo() == 0)
				this->_constructConvexHullfromPatchSet(layerCoord[layer->GetIndexNo()], convexHull, true);
			else this->_constructConvexHullfromPatchSet(layerCoord[layer->GetIndexNo()], convexHull, false);
			

			/* compute the shadow volume */
			shadowVolume += _compShadowVolume(layer, convexHull, layerCoord[layer->GetIndexNo()]);
			//break;
		}
	}
	else { // flip case, sort the isosurface from the list end to start
		//std::cout << "flip the order!" << std::endl;

		for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetTailPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetPrev(posMesh);

			if (layer->GetIndexNo() == layerNum - 1)
				this->_constructConvexHullfromPatchSet(layerCoord[layer->GetIndexNo()], convexHull, true);
			else this->_constructConvexHullfromPatchSet(layerCoord[layer->GetIndexNo()], convexHull, false);

			double thisLayerShadowVolume = _compShadowVolume(layer, convexHull, layerCoord[layer->GetIndexNo()]);
			// if (thisLayerShadowVolume < 2.0) break;
			shadowVolume += thisLayerShadowVolume;
			//std::cout << "This layer shadow volume = " << shadowVolume << std::endl;
		}
	}
	convexHull->ClearAll(); delete convexHull;
	//convexHull->SetIndexNo(isoSurfaceSet->GetMeshList().GetCount()); //index begin from 0
	//isoSurfaceSet->GetMeshList().AddTail(convexHull);
	//convexHull->drawThisIsoLayer = true;

	std::cout << "degree = " << degree << ", shadow volume = " << shadowVolume << std::endl;

	return shadowVolume;
}

double FabricationDirectionDetection::_compShadowVolume(
	QMeshPatch* layer, QMeshPatch* convexHull, Eigen::MatrixXd& layerCoord)
{
	//-------------------------------------------------------------------------------------------------
	//  detect the vertex of single layer is inside the convexhull

	double shadowVolume = 0.0;
	Eigen::VectorXd nodeShadowDis = Eigen::VectorXd::Zero(layer->GetNodeNumber());
	std::vector<bool> nodeShadowFlag(layer->GetNodeNumber());

	for (GLKPOSITION Pos = convexHull->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(Pos);
		Face->CalPlaneEquation(); // conpute plane equation for convex-hull inside checking
	}


	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

		double pnt[3]; for (int i = 0; i < 3; i++) pnt[i] = layerCoord(Node->GetIndexNo(), i);

		nodeShadowDis(Node->GetIndexNo()) = _detectPntInsideConvexHullandMinDist(convexHull, pnt);
		if (nodeShadowDis(Node->GetIndexNo()) > 0.0) {
			nodeShadowFlag[Node->GetIndexNo()] = true;
			Node->isShadowNode = true;
		}
		else nodeShadowFlag[Node->GetIndexNo()] = false;

	}

	/*for (int i = 0; i < layer->GetNodeNumber(); i++) std::cout << nodeShadowFlag[i] << ",";
	std:cout << std::endl;*/

	int shadowFaceNum = 0;
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		
		/* only when three node of the trianlge all inside the convexhull, this face is used to compute shadow volume*/
		bool shadowFace = true;
		for(int i=0;i<3;i++) {
			if (nodeShadowFlag[Face->GetNodeRecordPtr(0)->GetIndexNo()] == false) { shadowFace = false; break;}
		}
		if (shadowFace) {			
			double shadowDistance = 0.0; shadowFaceNum++;
			for (int i = 0; i < 3; i++) shadowDistance += nodeShadowDis(Face->GetNodeRecordPtr(i)->GetIndexNo());
			shadowVolume += Face->CalArea() * shadowDistance / 3;
		}
	}
	//std::cout << "for this layer shadow Face num = " << shadowFaceNum << std::endl;
	return shadowVolume;
}

double FabricationDirectionDetection::_detectPntInsideConvexHullandMinDist(QMeshPatch* convexHull, double pnt[3]) {

	Eigen::Vector3d pp; for (int i = 0; i < 3; i++) pp(i) = pnt[i];

	double dis = 999999.999;
	for (GLKPOSITION Pos = convexHull->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(Pos);
		Eigen::Vector3d planeNormal; double dd;
		Face->GetPlaneEquation(planeNormal(0), planeNormal(1), planeNormal(2), dd);
		
		//dd = dd / planeNormal.norm(); planeNormal = planeNormal.normalized();

		double nodeFaceDis = pp.dot(planeNormal) + dd;
		
		/*Eigen::Vector3d n1, n2, n3;
		Face->GetNodeRecordPtr(0)->GetCoord3D(n1(0), n1(1), n1(2));
		Face->GetNodeRecordPtr(1)->GetCoord3D(n2(0), n2(1), n2(2));
		Face->GetNodeRecordPtr(2)->GetCoord3D(n3(0), n3(1), n3(2));

		Eigen::Vector3d planeNormal = (n2 - n1).cross(n3 - n1);
		planeNormal = planeNormal.normalized();

		double dd = -n3.dot(planeNormal);
		double nodeFaceDis = pp.dot(planeNormal) + dd;*/

		if (nodeFaceDis > -0.00001) {
			//std::cout << "node outside convex hull, dis = " << nodeFaceDis << ". FaceIndex = " << Face->GetIndexNo() << std::endl;
			return -1.0;  //the node is outside the convex hull
		}
		else { if (fabs(nodeFaceDis) < dis) dis = fabs(nodeFaceDis); }
	}

	//std::cout << "node inside convex hull" << std::endl;
	return dis;

}

bool FabricationDirectionDetection::_finalizePrintingDirandRotate(int degree) {

	//-------------------------------------------------------------------------------------------------
	// Rotate iso-surface set and initial tetrahedral mesh based on input degree, and update the vector field

	this->_rotateMeshandUpdateVectorField(DEGREE_TO_ROTATE(degree));

	//--------------------------------------------------------------------------------------
	// Move model to the center and slightly higher than the platform.	

	this->_computeLayerSetCenterandMove();

	//--------------------------------------------------------------------------------------
	// Detect the order of the layer, if the top layer is lower than the bottom one then switch the layer order.

	return _detectLayerOrderFlip();
	
}

void FabricationDirectionDetection::_constructConvexHullfromPatchSet(
	Eigen::MatrixXd layerCoord, QMeshPatch* convexHull, bool isFirstLayer) 
{
	//--------------------------------------------------------------------------------------
	// build the point set for convex-hull generation
	Eigen::MatrixXd pointSet;
	if (isFirstLayer) { // first layer convex hull = platform + layer
		pointSet.resize(layerCoord.rows() + platform->GetNodeNumber(), 3);
		for (int i = 0; i < layerCoord.rows(); i++) pointSet.row(i) = layerCoord.row(i);

		int index = 0;
		for (GLKPOSITION Pos = platform->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)platform->GetNodeList().GetNext(Pos);
			Vector3d pp; Node->GetCoord3D(pp(0), pp(1), pp(2));
			pointSet.row(layerCoord.rows() + index) = pp; index++;
		}
	}
	else { // other layer convex hull = old convexhull + layer
		pointSet.resize(layerCoord.rows() + convexHull->GetNodeNumber(), 3);
		for (int i = 0; i < layerCoord.rows(); i++) pointSet.row(i) = layerCoord.row(i);
		int index = 0;
		for (GLKPOSITION Pos = convexHull->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)convexHull->GetNodeList().GetNext(Pos);
			Vector3d pp; Node->GetCoord3D(pp(0), pp(1), pp(2));
			pointSet.row(layerCoord.rows() + index) = pp; index++;
		}
	}
	
	this->_convexHullGenerator(pointSet, convexHull);

}

bool FabricationDirectionDetection::_rotateLayerSetbyMatrixandDetectOrder(
	int degree, std::vector<Eigen::MatrixXd>& layerCoord, bool updateVisual)
{
	//--------------------------------------------------------------------------------------
	// build the coordinate matrix and rotate the model
	double theta = DEGREE_TO_ROTATE(degree);
	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);

		layerCoord[layer->GetIndexNo()] = Eigen::MatrixXd::Zero(layer->GetNodeNumber(), 3);

		/*rotate the isosurface potision*/
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Eigen::Vector3d pp; Node->GetCoord3D_last(pp(0), pp(1), pp(2));
			Eigen::Vector3d rotatedpp; 
			rotatedpp << pp(0) * cos(theta) - pp(1) * sin(theta), pp(0) * sin(theta) + pp(1) * cos(theta), pp(2);
			for (int i = 0; i < 3; i++) layerCoord[layer->GetIndexNo()](Node->GetIndexNo(), i) = rotatedpp(i);
		}
	}

	// detect after rotation, wheather the top layer is lower than the bottom layer
	bool flipOrder = false;
	double ymin_firstLayer = layerCoord[0].col(1).minCoeff();
	double ymin_lastLayer = layerCoord[isoSurfaceSet->GetMeshList().GetCount() - 1].col(1).minCoeff();
	if (ymin_firstLayer > ymin_lastLayer) flipOrder = true;

	//--------------------------------------------------------------------------------------
	// move the surface set into center position
	Eigen::VectorXd minYLayer(isoSurfaceSet->GetMeshList().GetCount());
	for (int i = 0; i < isoSurfaceSet->GetMeshList().GetCount(); i++) {
		minYLayer(i) = layerCoord[i].col(1).minCoeff();
	}
	double minY = minYLayer.minCoeff();

	/* Update: the center and hight should be computed by the tetmesh 2020-05-01 */
	Eigen::MatrixXd initTetPos(tetMesh->GetNodeNumber(), 3); int nIndex = 0;
	for (GLKPOSITION posMesh = tetMesh->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posMesh);
		Eigen::Vector3d pp; node->GetCoord3D(pp(0), pp(1), pp(2));
		Eigen::Vector3d rotatedpp;
		rotatedpp << pp(0) * cos(theta) - pp(1) * sin(theta), pp(0)* sin(theta) + pp(1) * cos(theta), pp(2);
		for (int i = 0; i < 3; i++) initTetPos(nIndex, i) = rotatedpp(i);
		nIndex++;
	}
	Eigen::Vector3d centerPos; for (int i = 0; i < 3; i++) centerPos(i) = initTetPos.col(i).mean();

	for (int i = 0; i < isoSurfaceSet->GetMeshList().GetCount(); i++) {
		for (int j = 0; j < layerCoord[i].rows(); j++) {
			layerCoord[i](j, 0) -= centerPos(0); layerCoord[i](j, 2) -= centerPos(2);
			layerCoord[i](j, 1) += this->offsetY - minY;
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
		}
	}

	return flipOrder;

}

void FabricationDirectionDetection::_rotateMeshandUpdateVectorField(double theta) {

	double pp[3];

	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		/*rotate the isosurface potision*/
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D_last(pp[0], pp[1], pp[2]);
			Node->SetCoord3D(pp[0] * cos(theta) - pp[1] * sin(theta), pp[0] * sin(theta) + pp[1] * cos(theta), pp[2]);
		}
		/*rotate the principle stress direction*/
		for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
			Eigen::Vector3d tetVector = Face->principleStressDir;
			Face->principleStressDir << tetVector(0) * cos(theta) - tetVector(1) * sin(theta),
				tetVector(0)* sin(theta) + tetVector(1) * cos(theta), tetVector(2);
		}
	}

	/*rotate the mesh potision*/
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->GetCoord3D_last(pp[0], pp[1], pp[2]);
		Node->SetCoord3D(pp[0] * cos(theta) - pp[1] * sin(theta), pp[0] * sin(theta) + pp[1] * cos(theta), pp[2]);
	}
	/*rotate the vector field*/
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Eigen::Vector3d tetVector = tet->vectorField;
		tet->vectorField << tetVector(0) * cos(theta) - tetVector(1) * sin(theta),
			tetVector(0)* sin(theta) + tetVector(1) * cos(theta), tetVector(2);
	}

}

void FabricationDirectionDetection::_computeLayerSetCenterandMove() {

	double pp[3]; 	double ymin = 1e+8;
	double modelCentral[3] = { 0 };
	int nodeNumSum = 0;

	/* compute the center position and Ymin for the given isoSurfaceSet */

	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D(pp); if (pp[1] < ymin) ymin = pp[1];
			for (int i = 0; i < 3; i++) modelCentral[i] += pp[i];
		}
		nodeNumSum += layer->GetNodeNumber();
	}

	for (int i = 0; i < 3; i++) modelCentral[i] = modelCentral[i] / nodeNumSum;

	/* move the isosurface set and initial tetrahedral mesh */

	for (GLKPOSITION Pos = isoSurfaceSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(Pos);
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			Node->SetCoord3D(pp[0] - modelCentral[0], pp[1] - (ymin - offsetY), pp[2] - modelCentral[2]);
		}
	}

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		Node->SetCoord3D(pp[0] - modelCentral[0], pp[1] - (ymin - offsetY), pp[2] - modelCentral[2]);
	}
}

bool FabricationDirectionDetection::_detectLayerOrderFlip() {

	double pp[3];
	bool order = true;
	QMeshPatch* firstLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetHead();
	QMeshPatch* lastLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetTail();
	double yfmin = 1e+8, ylmin = 1e+8;
	for (GLKPOSITION Pos = firstLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)firstLayer->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < yfmin) yfmin = pp[1];
	}
	for (GLKPOSITION Pos = lastLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)lastLayer->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < ylmin) ylmin = pp[1];
	}
	if (yfmin > ylmin) order = false;

	return order;

}

void FabricationDirectionDetection::convexHullGenerator(PolygenMesh* polygenMesh) {

	//-------------------------------------------------------------------------------------------------
	//  Generate convex-hull with initial point
	QMeshPatch* inputMesh = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	int nVertices = inputMesh->GetNodeNumber();
	ch_vertex* vertices;
	vertices = (ch_vertex*)malloc(nVertices * sizeof(ch_vertex));

	int index = 0;
	for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(vertices[index].x, vertices[index].y, vertices[index].z);
		Node->SetIndexNo(index);
		index++;
	}

	int* faceIndices = NULL; int nFaces;
	convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);

	std::cout << " fabrication direction detection - finish compute convex hull" << std::endl;

	for (int i = 0; i < nFaces; i++)
		std::cout << faceIndices[3 * i] << "," << faceIndices[3 * i + 1] << "," << faceIndices[3 * i + 2] << std::endl;

	//-------------------------------------------------------------------------------------------------
	//  Rebuid nodelist and facelist to generate the convex hull mesh
	std::vector<int> nodeIndex;
	for (int i = 0; i < 3 * nFaces; i++) {
		bool indicesExist = false;
		for (int j = 0; j < nodeIndex.size(); j++) { if (nodeIndex[j] == faceIndices[i]) { indicesExist = true; break; } }
		if (!indicesExist) nodeIndex.push_back(faceIndices[i]);
	}
	sort(nodeIndex.begin(), nodeIndex.end());
	std::cout << "nodeIndex size = " << nodeIndex.size() << std::endl;
	for (int i = 0; i < nodeIndex.size(); i++) std::cout << nodeIndex[i] << ",";

	VectorXi nodeCorresTable(inputMesh->GetNodeNumber());
	float* nodeTable;
	nodeTable = (float*)malloc(sizeof(float) * nodeIndex.size() * 3);
	int relatedNodeIndex = 0;
	for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
		nodeCorresTable(Node->GetIndexNo()) = -1;
		if (Node->GetIndexNo() == nodeIndex[relatedNodeIndex]) {
			double pp[3]; Node->GetCoord3D(pp);
			for (int i = 0; i < 3; i++) nodeTable[relatedNodeIndex * 3 + i] = pp[i];
			nodeCorresTable(Node->GetIndexNo()) = relatedNodeIndex;
			relatedNodeIndex++; if (relatedNodeIndex == nodeIndex.size()) break;
		}
	}
	std::cout << " build node table! " << std::endl;
	std::cout << "nodeCorresTable" << nodeCorresTable << std::endl;

	//build faceNum and faceTable
	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * nFaces * 3);
	index = 0;
	for (int i = 0; i < nFaces; i++) {
		for (int j = 0; j < 3; j++) faceTable[3 * i + j] = nodeCorresTable(faceIndices[3 * i + j]);
	}
	std::cout << " build face table! " << std::endl;

	QMeshPatch* convexHull = new QMeshPatch;
	convexHull->SetIndexNo(polygenMesh->GetMeshList().GetCount()); //index begin from 0
	polygenMesh->GetMeshList().AddTail(convexHull);
	convexHull->constructionFromVerFaceTable(nodeIndex.size(), nodeTable, nFaces, faceTable);

	free(vertices);
	free(faceIndices);
	free(nodeTable);
	free(faceTable);

}

void FabricationDirectionDetection::_convexHullGenerator(Eigen::MatrixXd& pointSet, QMeshPatch* convexHull) {

	long time = clock();

	//-------------------------------------------------------------------------------------------------
	//  Generate convex-hull with the point set
	int nVertices = pointSet.rows();
	ch_vertex* vertices;
	vertices = (ch_vertex*)malloc(nVertices * sizeof(ch_vertex));

	for (int index = 0; index < nVertices; index++) {
		vertices[index].x = pointSet(index, 0);
		vertices[index].y = pointSet(index, 1);
		vertices[index].z = pointSet(index, 2);
	}

	int* faceIndices = NULL; int nFaces;
	convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);

	timeConvexHull += clock() - time;

	//std::cout << " fabrication direction detection - finish compute convex hull" << std::endl;

	//for (int i = 0; i < nFaces; i++)
	//	std::cout << faceIndices[3 * i] << "," << faceIndices[3 * i + 1] << "," << faceIndices[3 * i + 2] << std::endl;

	//-------------------------------------------------------------------------------------------------
	//  Rebuid nodelist and facelist to generate the convex hull mesh
	std::vector<int> nodeIndex;
	for (int i = 0; i < 3 * nFaces; i++) {
		bool indicesExist = false;
		for (int j = 0; j < nodeIndex.size(); j++) { if (nodeIndex[j] == faceIndices[i]) { indicesExist = true; break; } }
		if (!indicesExist) nodeIndex.push_back(faceIndices[i]);
	}
	sort(nodeIndex.begin(), nodeIndex.end());
	//std::cout << "nodeIndex size = " << nodeIndex.size() << std::endl;
	//for (int i = 0; i < nodeIndex.size(); i++) std::cout << nodeIndex[i] << ",";

	VectorXi nodeCorresTable = Eigen::VectorXi::Zero(nVertices);
	float* nodeTable;
	nodeTable = (float*)malloc(sizeof(float) * nodeIndex.size() * 3);
	int relatedNodeIndex = 0;
	for (int index = 0; index < nVertices; index++) {
		if (index == nodeIndex[relatedNodeIndex]) {
			for (int i = 0; i < 3; i++) nodeTable[relatedNodeIndex * 3 + i] = pointSet(index, i);
			nodeCorresTable(index) = relatedNodeIndex;
			relatedNodeIndex++; if (relatedNodeIndex == nodeIndex.size()) break;
		}
	}

	//std::cout << std::endl<< " ### build node table! " << std::endl;
	//std::cout << "nodeCorresTable" << std::endl << nodeCorresTable << std::endl;

	//build faceNum and faceTable
	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * nFaces * 3);
	for (int i = 0; i < nFaces; i++) {
		for (int j = 0; j < 3; j++) faceTable[3 * i + j] = nodeCorresTable(faceIndices[3 * i + j]);
	}
	//std::cout << " build face table! " << std::endl;

	convexHull->ClearAll();
	convexHull->constructionFromVerFaceTable(nodeIndex.size(), nodeTable, nFaces, faceTable);

	free(vertices);
	free(faceIndices);
	free(nodeTable);
	free(faceTable);

}

