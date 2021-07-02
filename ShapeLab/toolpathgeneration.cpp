#include "toolpathgeneration.h"
using namespace std;
using namespace Eigen;
#include <iostream>
#include <fstream>
#include "GLKGeometry.h"

toolPathGeneration::toolPathGeneration(QMeshPatch* inputMesh)
{
	surfaceMesh = inputMesh;
	minZigZagtoolpathNodeNum = 5;
}

toolPathGeneration::~toolPathGeneration() {}

//////////////////////////////////////////////////////////////////////////////////
/* function for CCF project usage */
void toolPathGeneration::mergeFieldforCCF() {

	Eigen::VectorXd guideField(surfaceMesh->GetNodeNumber());
	Eigen::VectorXd guideFieldNormalize(surfaceMesh->GetNodeNumber());
	int index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Node->zigzagValue = 2 * abs(Node->zigzagValue - 0.5);
		guideField(index) = (Node->zigzagValue + (1-Node->boundaryValue)) / 2; index++;
	}


	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < surfaceMesh->GetNodeNumber(); i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i < surfaceMesh->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Node->zigzagValue = guideFieldNormalize(index); index++;

		//Node->zigzagValue = 2 * abs(Node->zigzagValue - 0.5);
	}

}


//////////////////////////////////////////////////////////////////////////////////
/* Below are the function for tool path generation from geodistance field value
- IO function to Mainwindow */

void toolPathGeneration::generateZigzagToolPath(PolygenMesh* tPath, int Num)
{
	//initialize variables
	initialize(Num);

	//build isonode without connection
	if (generateZigZagIsoNode(Num) == false) { printf("No isonode for zigzag \n"); return; }
	else {
		//verify the correctness of zigzag node generation - by visuilization
	/*	for (GLKPOSITION Pos = zigzagPathList.GetHeadPosition(); Pos;) {
			QMeshPatch* patch = (QMeshPatch*)zigzagPathList.GetNext(Pos);
			tPath->GetMeshList().AddTail(patch);
			patch->isToolPathPatch = true;
		}*/

	//connect all isonode to continues toolpath
		int zigzagRNum = 0;
		do {
			connectZigZagToolPathToRegion(tPath); zigzagRNum++;
		} while (detectAllZigZagToolPathisProcessed() == false);

		std::cout << "Zig-zag toolpath contains " << zigzagRNum << " region(s)." << std::endl;
	}
	
}

void toolPathGeneration::generateBundaryToolPath(PolygenMesh* tPath, int Num, double offset) {

	//initialize variables
	initialize(Num, offset);

	//build isonode without connection
	generateBoundaryIsoNode(Num, offset);

	if (boundPathList.GetCount() == 0) return;

	//connect all isonode to continues toolpath
	do {
		//buildOutRingToolPathandConnectZigzagPath(tPath);
		buildOutRingToolPath_without_ConnectZigzagPath(tPath);
		//std::cout << "run single iter once!" << endl;
	} while (detectAllBoundaryToolPathisProcessed() == false);

}

void toolPathGeneration::resampling(PolygenMesh *tPath) {
	
	for (GLKPOSITION Pos = tPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch *patch = (QMeshPatch*)tPath->GetMeshList().GetNext(Pos);
		
		if(patch == (QMeshPatch*)tPath->GetMeshList().GetHead()) resamplingLength = resamplingSinglePatch(patch);
		else resamplingSinglePatch(patch);

		if (resamplingLength < 0) resamplingLength = 1.0;

		if (patch->GetEdgeNumber() < 4) {

			//patch->isSmallPatch = true;
			patch->ClearAll();
			tPath->GetMeshList().Remove(patch);
		}		
	}
}

///////////////////////////////////////////////
/*Below are the function for zig-zag toolpath*/

void toolPathGeneration::initialize(int Num) {
	zigzagPathList.RemoveAll();
	edgePIndex.resize(Num, surfaceMesh->GetEdgeNumber());
	for (int i = 0; i< Num; i++) {
		for (int j = 0; j< surfaceMesh->GetEdgeNumber(); j++)
			edgePIndex(i, j) = -1;
	}
	/*for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		std::cout << thisNode->zigzagValue << std::endl;
	}*/
}

bool toolPathGeneration::generateZigZagIsoNode(int Num) {

	int isoNodeNum = 0;

	double isoVStep = 1.0 / Num;
	for (int i = 0; i < Num; i++) {
		double isoValue = isoVStep / 2 + isoVStep * i;
		QMeshPatch *singleToolPath = new QMeshPatch;
		singleToolPath->isoNum = i;

		//Here only the patch contains isonode will be installed!
		if (this->generateSingleZigzagToolPathIsoNode(singleToolPath, isoValue)) {
			//std::cout << singleToolPath->GetNodeNumber() << std::endl;
			singleToolPath->SetIndexNo(zigzagPathList.GetCount()); //index begin from 0
			zigzagPathList.AddTail(singleToolPath);
			isoNodeNum += singleToolPath->GetNodeNumber();
		}
	}

	if (isoNodeNum > 0) return true; else return false;
	//std::cout<<"Zig-zag isoNode is installed inside "<< zigzagPathList.GetCount() << " segment(s)."<< std::endl <<std::endl;
}

/*This function is used to generate all the isonode and detect which one is the boudnary node*/
bool toolPathGeneration::generateSingleZigzagToolPathIsoNode(QMeshPatch *singlePath, double isoValue)
{
	//----First build the node and install back to the meshpatch
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
		//Edge->relatedToolPathPointIndex = -1;

		double a = Edge->GetStartPoint()->zigzagValue;
		double b = Edge->GetEndPoint()->zigzagValue;
		double boundaryValue = 0;
		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1);
			Edge->GetEndPoint()->GetCoord3D(p2);

			for (int j = 0; j < 3; j++) {
				//compute the position for this isonode
				pp[j] = (1.0 - alpha)*p1[j] + alpha*p2[j];

				//compute boundary value for this isonode
				boundaryValue = (1.0 - alpha) * Edge->GetStartPoint()->boundaryValue +
					alpha * Edge->GetEndPoint()->boundaryValue;
			}

			// #### detect if the node inside the boundary region ####

			if (breakZigzagbyBoundary && boundaryValue > minBoundaryFieldValue) continue;

			QMeshNode* isoNode = new QMeshNode;
			isoNode->SetMeshPatchPtr(singlePath);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(singlePath->GetNodeList().GetCount()); //index start from 0

			isoNode->boundaryValue = boundaryValue;
			isoNode->relatedEdgeIndex = Edge->GetIndexNo();
			isoNode->connectTPathProcessed = false;

			//cal normal of iso point, consider boundary condition
			double n1[3] = { 0 }; double n2[3] = { 0 }; Eigen::Vector3d n3;
			if(Edge->GetLeftFace() != NULL) Edge->GetLeftFace()->GetNormal(n1[0], n1[1], n1[2]);
			if (Edge->GetRightFace() != NULL) Edge->GetRightFace()->GetNormal(n2[0], n2[1], n2[2]);
			for (int i = 0; i < 3; i++) n3(i) = (n1[i] + n2[i]) / 2;
			if (n3.norm() < 0.01) std::cerr << "toolpath - zigzag - NORMAL ERROR!" << std::endl;
			n3 = n3.normalized();

			isoNode->SetNormal(n3(0), n3(1), n3(2));

			edgePIndex(singlePath->isoNum, Edge->GetIndexNo()) = isoNode->GetIndexNo();

			singlePath->GetNodeList().AddTail(isoNode);
		}
	}
	if (singlePath->GetNodeNumber() == 0)
		return false;  //means no node is detected

														 //----Detect the iso node located at the boundary of single tool-path, used for connection

														 //Detect the boundary node by finding the connected edge (in original mesh) which have only one face covers 2 isonode
														 //Notice that, the edge contains zig-zag node normally not located at the boundary. If its a boundary edge, we directly
														 //set this isonode as boundary of zig-zag tool-path.

	int zigzagPathBoundaryNode = 0;
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		QMeshEdge* thisEdge = surfaceMesh->GetEdgeRecordPtr(thisNode->relatedEdgeIndex + 1);

		thisNode->isZigzagBoundaryNode = false;

		if (thisEdge->IsBoundaryEdge()) {
			thisNode->isZigzagBoundaryNode = true; zigzagPathBoundaryNode++;
			if(breakZigzagbyBoundary)
				cout << "Warning:: Notice that, the zigzag toolpath node related to a boundary edge! " <<
				"You can try to make the boundary tool-path offset higher." << endl;
			continue;
		}

		int lFaceIsoNodeNum = 0; //left face
		for (int i = 0; i < 3; i++) {
			QMeshEdge *NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
			int index = edgePIndex(singlePath->isoNum, NeighborEdge->GetIndexNo());
			if (index < 0) continue;
			else lFaceIsoNodeNum++;
		}
		int rFaceIsoNodeNum = 0; //right face
		for (int i = 0; i < 3; i++) {
			QMeshEdge *NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);
			int index = edgePIndex(singlePath->isoNum, NeighborEdge->GetIndexNo());
			if (index < 0) continue;
			else rFaceIsoNodeNum++;
		}
		// isolated node, with no left and right boundary node connected, we ignore this region.
		if (lFaceIsoNodeNum == 1 && rFaceIsoNodeNum == 1) {
			thisNode->connectTPathProcessed = true; continue;
		}
		if (lFaceIsoNodeNum == 1 || rFaceIsoNodeNum == 1) {
			thisNode->isZigzagBoundaryNode = true;
			zigzagPathBoundaryNode++;
		}
	}
	singlePath->boundNodeNum = zigzagPathBoundaryNode;
	if (zigzagPathBoundaryNode % 2 != 0) printf("zigzag path boundary node number error! should be even number\n");
	//std::cout << "boundarynodeNum = "<< zigzagPathBoundaryNode << std::endl;
	return true;
}

/*This function is used to connect isonode to a region*/
void toolPathGeneration::connectZigZagToolPathToRegion(PolygenMesh* tPath) {
	QMeshPatch *tPathRegion = new QMeshPatch;

	GLKGeometry geo;
	//Detect the start tool path (not all the node being processed)
	//Then find the unprocessed boundary iso-node as the start node
	QMeshPatch *startToolPath;
	for (GLKPOSITION Pos = zigzagPathList.GetHeadPosition(); Pos;) {
		startToolPath = (QMeshPatch*)zigzagPathList.GetNext(Pos);
		if (detectSingleZigZagToolPathisProcessed(startToolPath) == false) break;
	}

	QMeshNode* startNode; bool startNodeFind = false;

	double compareBoundValue = -9999.999;
	for (GLKPOSITION Pos = startToolPath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)startToolPath->GetNodeList().GetNext(Pos);
		//if (Node->isZigzagBoundaryNode && Node->connectTPathProcessed == false && Node->boundaryValue > compareBoundValue) {
		if (Node->isZigzagBoundaryNode && Node->connectTPathProcessed == false) {
			startNode = Node; startNodeFind = true;
			//compareBoundValue = Node->boundaryValue;
		}
	}

	if (startNodeFind == false && breakZigzagbyBoundary)
		std::cout << "the start node of this region is not found for No." << startToolPath->GetIndexNo() << " region!" << endl;

	double pp[3]; startNode->GetCoord3D(pp);
	double normal[3]; startNode->GetNormal(normal[0], normal[1], normal[2]);
	QMeshNode* edgeSNode = buildNewNodetoQMeshPatch(tPathRegion, pp, normal);
	double endPointPos[3];
	QMeshNode* lastTPathendNode =
		connectSingleZigZagToolPathandGenerateEdge(startNode, edgeSNode, tPathRegion, startToolPath, endPointPos);

	if (detectAllZigZagToolPathisProcessed()) return;

	do {
		GLKPOSITION sTPathPos = zigzagPathList.Find(startToolPath)->next;
		if (sTPathPos == nullptr) break;

		startToolPath = (QMeshPatch*)zigzagPathList.GetNext(sTPathPos);

		if (detectSingleZigZagToolPathisProcessed(startToolPath)) break;

		double tPathDis = 100000;
		for (GLKPOSITION Pos = startToolPath->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)startToolPath->GetNodeList().GetNext(Pos);
			if (thisNode->isZigzagBoundaryNode && thisNode->connectTPathProcessed == false) {
				double pp[3];
				thisNode->GetCoord3D(pp[0], pp[1], pp[2]);
				double bDis = geo.Distance_to_Point(pp, endPointPos);
				if (tPathDis>bDis) {
					startNode = thisNode;
					tPathDis = bDis;
				}
			}
		}

		if (tPathDis > maxGapDist) break;

		startNode->GetCoord3D(pp);
		double normal[3]; startNode->GetNormal(normal[0], normal[1], normal[2]);
		QMeshNode* edgeSNode = buildNewNodetoQMeshPatch(tPathRegion, pp, normal);
		buildNewEdgetoQMeshPatch(tPathRegion, lastTPathendNode, edgeSNode)->isConnectEdge = true;

		lastTPathendNode = connectSingleZigZagToolPathandGenerateEdge(startNode, edgeSNode, tPathRegion, startToolPath, endPointPos);

	} while ((QMeshPatch*)zigzagPathList.GetTail() != startToolPath);

	if (tPathRegion->GetNodeNumber() > minZigZagtoolpathNodeNum) //this can be adjusted
	{
		tPathRegion->isToolPathPatch = true;
		tPathRegion->SetIndexNo(tPath->GetMeshList().GetCount()); //index begin from 0
		tPath->GetMeshList().AddTail(tPathRegion);
	}
}

QMeshNode* toolPathGeneration::connectSingleZigZagToolPathandGenerateEdge
(QMeshNode *startNode, QMeshNode *edgeSNode, QMeshPatch *tPathRegion, QMeshPatch* thisTPath, double endPointPos[]) {

	//first add startNode to tPathRegion
	QMeshNode* sNode = startNode;
	QMeshNode* eNode;
	sNode->connectTPathProcessed = true;
	double pp[3];
	sNode->GetCoord3D(pp[0], pp[1], pp[2]);
	//QMeshNode* edgeSNode = buildNewNodetoQMeshPatch(tPathRegion,pp);
	QMeshNode* edgeENode;

	do {
		//Step1: find next node by mesh topology
		sNode->connectTPathProcessed = true;

		QMeshEdge* thisEdge = surfaceMesh->GetEdgeRecordPtr(sNode->relatedEdgeIndex + 1);
		if (thisEdge->IsBoundaryEdge() && breakZigzagbyBoundary)
			cout << "ERROR, this zigzag toolpath node related edge should not belongs to boundary!" << endl;

		bool eNodeDetect = false;

		//detect if eNode located at left face
		for (int i = 0; i < 3; i++) {
			QMeshEdge *NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);

			if (NeighborEdge == thisEdge) continue;
			int index = edgePIndex(thisTPath->isoNum, NeighborEdge->GetIndexNo());
			if (index<0) continue;
			else {
				QMeshNode* dNode = thisTPath->GetNodeRecordPtr(index + 1);
				if (dNode->connectTPathProcessed) continue;
				else {
					eNode = dNode;
					eNodeDetect = true;
				}

			}
		}

		//if eNode not find, check rightface
		if (eNodeDetect == false) {
			for (int i = 0; i < 3; i++) {
				QMeshEdge *NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);

				if (NeighborEdge == thisEdge) continue;
				int index = edgePIndex(thisTPath->isoNum, NeighborEdge->GetIndexNo());
				if (index<0) continue;
				else {
					QMeshNode* dNode = thisTPath->GetNodeRecordPtr(index + 1);
					if (dNode->connectTPathProcessed) continue;
					else {
						eNode = dNode;
						eNodeDetect = true;
					}
				}
			}
		}

		//cout<<detectNumLeft<<","<<detectNumRight<<endl;

		if (eNodeDetect == false) {
			cout << surfaceMesh->layerName <<" ERROR, the next node is not founded! this should not happen for a zigzag region boundary node!" << endl;
			break;
		}

		//set eNode as sNode and install them back to the tPathRegion
		eNode->connectTPathProcessed = true;
		eNode->GetCoord3D(pp[0], pp[1], pp[2]);
		double normal[3]; eNode->GetNormal(normal[0], normal[1], normal[2]);
		edgeENode = buildNewNodetoQMeshPatch(tPathRegion, pp, normal);
		buildNewEdgetoQMeshPatch(tPathRegion, edgeSNode, edgeENode);

		sNode = eNode;
		edgeSNode = edgeENode;

	} while (!sNode->isZigzagBoundaryNode);

	//return the eNode for next toolpath isonode checking
	edgeENode->GetCoord3D(endPointPos[0], endPointPos[1], endPointPos[2]);
	return edgeENode;
}


////////////////////////////////////////////////////////////
/*Below are the function for boundary tool path generation*/

void toolPathGeneration::initialize(int Num, double offset) {
	boundPathList.RemoveAll();
	edgeBPIndex.resize(Num, surfaceMesh->GetEdgeNumber());
	for (int i = 0; i<Num; i++) {
		for (int j = 0; j<surfaceMesh->GetEdgeNumber(); j++)
			edgeBPIndex(i, j) = -1;
	}
}

void toolPathGeneration::generateBoundaryIsoNode(int Num, double offset) {
	for (int i = 0; i < Num; i++) {
		double isoValue = 1 - (Num - i)*offset + offset*0.5; //the order is from inside to outside
		QMeshPatch *singleBoundPath = new QMeshPatch;
		singleBoundPath->isoNum = i;
		generateSingleBoundaryToolPathIsoNode(singleBoundPath, isoValue);
		singleBoundPath->SetIndexNo(boundPathList.GetCount());
		boundPathList.AddTail(singleBoundPath);
		//singleBoundPath->SetIndexNo(tPath->GetMeshList().GetCount());
		//tPath->GetMeshList().AddTail(singleBoundPath);
	}

	//std::cout<<"Boundary isoNode is included inside " << boundPathList.GetCount() << " segment(s)."<< std::endl <<std::endl;
}

void toolPathGeneration::generateSingleBoundaryToolPathIsoNode(QMeshPatch *singlePath, double isoValue)
{
	//----build the node and install back to the meshpatch
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
		//Edge->relatedToolPathPointIndex = -1;

		double a = Edge->GetStartPoint()->boundaryValue;
		double b = Edge->GetEndPoint()->boundaryValue;
		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++) {
				//compute the position for this isonode
				pp[j] = (1.0 - alpha)*p1[j] + alpha*p2[j];
			}

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedEdgeIndex = Edge->GetIndexNo();
			isoNode->connectTPathProcessed = false;

			isoNode->SetMeshPatchPtr(singlePath);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(singlePath->GetNodeList().GetCount()); //index should start from 0

																	   //cal normal of iso point
			double n1[3], n2[3], n3[3];
			Edge->GetLeftFace()->GetNormal(n1[0], n1[1], n1[2]);
			Edge->GetRightFace()->GetNormal(n2[0], n2[1], n2[2]);
			for (int i = 0; i<3; i++) n3[i] = (n1[i] + n2[i]) / 2;
			isoNode->SetNormal(n3[0], n3[1], n3[2]);

			edgeBPIndex(singlePath->isoNum, Edge->GetIndexNo()) = isoNode->GetIndexNo();
			//Edge->relatedToolPathPointIndex = isoNode->GetIndexNo();

			singlePath->GetNodeList().AddTail(isoNode);
		}
	}

	if (singlePath->GetNodeNumber() == 0)
		std::cout << "Warning! the boundary toolpath contains no isonode!" << endl;  //means no node is detected
}

void toolPathGeneration::buildOutRingToolPathandConnectZigzagPath(PolygenMesh* tPath)
{

	//QMeshPatch* zigzagFirstPatch = (QMeshPatch*) tPath->GetMeshList().GetHead();
	//QMeshNode* zigzagFirstNode = (QMeshNode*) zigzagFirstPatch->GetNodeList().GetTail();

	QMeshPatch* zigzagFirstPatch;
	QMeshNode* zigzagFirstNode;

	QMeshPatch* boundIsoPatch;
	for (GLKPOSITION Pos = boundPathList.GetHeadPosition(); Pos; ) {
		boundIsoPatch = (QMeshPatch*)boundPathList.GetNext(Pos);
		if (detectSingleZigZagToolPathisProcessed(boundIsoPatch) == false) break;
	}

	QMeshNode* boundFirstNode = nullptr;

	for (GLKPOSITION Pos = tPath->GetMeshList().GetHeadPosition(); Pos; ) {
		zigzagFirstPatch = (QMeshPatch*)tPath->GetMeshList().GetNext(Pos);
		if (zigzagFirstPatch->connectWithBoundary == true) continue;

		zigzagFirstNode = (QMeshNode*)zigzagFirstPatch->GetNodeList().GetTail();
		boundFirstNode = findNextNearestPoint(zigzagFirstNode, boundIsoPatch);
		if (boundFirstNode != nullptr) {
			zigzagFirstPatch->connectWithBoundary = true;
			break;
		}
	}

	double p1[3];
	QMeshNode* boundSNode;

	if (boundFirstNode == nullptr) {
		zigzagFirstPatch = new QMeshPatch;
		zigzagFirstPatch->SetIndexNo(tPath->GetMeshList().GetCount());
		zigzagFirstPatch->isToolPathPatch = true;
		tPath->GetMeshList().AddTail(zigzagFirstPatch);

		for (GLKPOSITION Pos = boundIsoPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)boundIsoPatch->GetNodeList().GetNext(Pos);
			if (thisNode->connectTPathProcessed == false) {
				boundFirstNode = thisNode;
				break;
			}
		}
		boundFirstNode->GetCoord3D(p1);
		double normal[3]; boundFirstNode->GetNormal(normal[0], normal[1], normal[2]);
		boundSNode = buildNewNodetoQMeshPatch(zigzagFirstPatch, p1, normal);

	}
	else {
		boundFirstNode->GetCoord3D(p1);
		double normal[3]; boundFirstNode->GetNormal(normal[0], normal[1], normal[2]);
		boundSNode = buildNewNodetoQMeshPatch(zigzagFirstPatch, p1, normal);
		buildNewEdgetoQMeshPatch(zigzagFirstPatch, zigzagFirstNode, boundSNode)->isConnectEdge = true;
	}

	//search the loop
	//QMeshNode* startNode = boundFirstNode;
	QMeshNode* lastBoundEndNode = buildOneRingBoundaryToolPath(boundSNode, boundFirstNode, zigzagFirstPatch, boundIsoPatch);

	int itertime = boundPathList.GetCount() - 1 - boundIsoPatch->GetIndexNo();
	for (int i = 0; i<itertime; i++)
	{
		//find next one
		GLKPOSITION pos = boundPathList.Find(boundIsoPatch)->next;
		boundIsoPatch = (QMeshPatch*)boundPathList.GetNext(pos);
		boundFirstNode = findNextNearestPoint(lastBoundEndNode, boundIsoPatch);
		//startNode = boundFirstNode;
		if (boundFirstNode == nullptr) continue;

		boundFirstNode->GetCoord3D(p1);
		//boundFirstNode = buildNewNodetoQMeshPatch(zigzagFirstPatch,p1);
		double normal[3]; boundFirstNode->GetNormal(normal[0], normal[1], normal[2]);

		boundSNode = buildNewNodetoQMeshPatch(zigzagFirstPatch, p1, normal);

		QMeshEdge * connectEdge = buildNewEdgetoQMeshPatch(zigzagFirstPatch, lastBoundEndNode, boundSNode);
		connectEdge->isConnectEdge = true;
		//std::cout<<connectEdge->GetIndexNo()<<std::endl;

		//search the loop
		//startNode = boundSNode;
		lastBoundEndNode = buildOneRingBoundaryToolPath(boundSNode, boundFirstNode, zigzagFirstPatch, boundIsoPatch);

		//std::cout<<lastBoundEndNode->GetIndexNo()<<std::endl;
	}


}

void toolPathGeneration::buildOutRingToolPath_without_ConnectZigzagPath(PolygenMesh* tPath)
{
	/*Modified by Guoxin on 2020-01-22*/

	QMeshPatch* boundIsoPatch;
	for (GLKPOSITION Pos = boundPathList.GetHeadPosition(); Pos; ) {
		boundIsoPatch = (QMeshPatch*)boundPathList.GetNext(Pos);
		if (detectSingleZigZagToolPathisProcessed(boundIsoPatch) == false) break;
	}

	QMeshNode* boundFirstNode = nullptr; double p1[3];

	QMeshPatch* boundaryFirstPatch = new QMeshPatch;
	boundaryFirstPatch->SetIndexNo(tPath->GetMeshList().GetCount());
	boundaryFirstPatch->isToolPathPatch = true;
	tPath->GetMeshList().AddTail(boundaryFirstPatch);

	for (GLKPOSITION Pos = boundIsoPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)boundIsoPatch->GetNodeList().GetNext(Pos);
		if (thisNode->connectTPathProcessed == false) {
			boundFirstNode = thisNode;
			break;
		}
	}
	boundFirstNode->GetCoord3D(p1);
	double normal[3]; boundFirstNode->GetNormal(normal[0], normal[1], normal[2]);
	QMeshNode* boundSNode = buildNewNodetoQMeshPatch(boundaryFirstPatch, p1, normal);


	//search the loop
	QMeshNode* lastBoundEndNode = buildOneRingBoundaryToolPath(boundSNode, boundFirstNode, boundaryFirstPatch, boundIsoPatch);

	int itertime = boundPathList.GetCount() - 1 - boundIsoPatch->GetIndexNo();
	for (int i = 0; i<itertime; i++)
	{
		//find next one
		GLKPOSITION pos = boundPathList.Find(boundIsoPatch)->next;
		boundIsoPatch = (QMeshPatch*)boundPathList.GetNext(pos);
		boundFirstNode = findNextNearestPoint(lastBoundEndNode, boundIsoPatch);
		//startNode = boundFirstNode;
		if (boundFirstNode == nullptr) continue;

		boundFirstNode->GetCoord3D(p1);
		//boundFirstNode = buildNewNodetoQMeshPatch(zigzagFirstPatch,p1);
		double normal[3]; boundFirstNode->GetNormal(normal[0], normal[1], normal[2]);

		boundSNode = buildNewNodetoQMeshPatch(boundaryFirstPatch, p1, normal);

		QMeshEdge * connectEdge = buildNewEdgetoQMeshPatch(boundaryFirstPatch, lastBoundEndNode, boundSNode);
		connectEdge->isConnectEdge = true;
		//std::cout<<connectEdge->GetIndexNo()<<std::endl;

		//search the loop
		//startNode = boundSNode;
		lastBoundEndNode = buildOneRingBoundaryToolPath(boundSNode, boundFirstNode, boundaryFirstPatch, boundIsoPatch);

		//std::cout<<lastBoundEndNode->GetIndexNo()<<std::endl;
	}

}


QMeshNode* toolPathGeneration::buildOneRingBoundaryToolPath(QMeshNode *startNode, QMeshNode* sNode, QMeshPatch *zigzagPatch, QMeshPatch* boundIsoPatch)
{
	GLKGeometry geo;
	//QMeshNode* sNode = startNode;
	double startPP[3]; startNode->GetCoord3D(startPP);

	QMeshNode* eNode;
	QMeshNode* endNode;
	//Step1: find next node by mesh topology
	sNode->connectTPathProcessed = true;

	QMeshEdge* thisEdge = surfaceMesh->GetEdgeRecordPtr(sNode->relatedEdgeIndex + 1);
	for (int i = 0; i < 3; i++) {
		QMeshEdge *NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;
		int index = edgeBPIndex(boundIsoPatch->GetIndexNo(), NeighborEdge->GetIndexNo());
		if (index >= 0) {
			eNode = boundIsoPatch->GetNodeRecordPtr(index + 1); break;
		}
	}

	double p1[3];
	eNode->GetCoord3D(p1);
	double normal[3];
	eNode->GetNormal(normal[0], normal[1], normal[2]);

	endNode = buildNewNodetoQMeshPatch(zigzagPatch, p1, normal);
	buildNewEdgetoQMeshPatch(zigzagPatch, startNode, endNode);
	startNode = endNode;

	int iter = 0;
	std::vector<Eigen::Vector3d> pos;
	std::vector<Eigen::Vector3d> normalSet;
	pos.resize(boundIsoPatch->GetNodeNumber());
	normalSet.resize(boundIsoPatch->GetNodeNumber());

	for (int i = 0; i<3; i++) {
		pos[0](i) = p1[i];
		normalSet[0](i) = normal[i];
	}

	do {
		sNode = eNode;
		sNode->connectTPathProcessed = true;
		//startNode = endNode;
		eNode = findNextBoundaryToolPath(sNode, boundIsoPatch);
		if (eNode == nullptr) break;

		eNode->GetCoord3D(p1);
		eNode->GetNormal(normal[0], normal[1], normal[2]);
		//endNode = buildNewNodetoQMeshPatch(zigzagPatch,p1);
		//buildNewEdgetoQMeshPatch(zigzagPatch,startNode,endNode);
		iter++;

		for (int i = 0; i<3; i++) {
			pos[iter](i) = p1[i];
			normalSet[iter](i) = normal[i];
		}

	} while (eNode->connectTPathProcessed == false);

	//here the process is used to fobid the connection between two boundary toolpath is too close!
	for (int j = 0; j<iter; j++) {
		for (int i = 0; i<3; i++) {
			p1[i] = pos[j + 1](i);
			normal[i] = normalSet[j + 1](i);
		}

		double dis = geo.Distance_to_Point(startPP, p1);
		if (dis < boundaryGapDist && j > 4) break;

		//double normal[3] = {0};
		endNode = buildNewNodetoQMeshPatch(zigzagPatch, p1, normal);
		buildNewEdgetoQMeshPatch(zigzagPatch, startNode, endNode);
		startNode = endNode;
	}

	return endNode;
}


QMeshNode* toolPathGeneration::findNextNearestPoint(QMeshNode *startNode, QMeshPatch *boundIsoPatch)
{
	GLKGeometry geo;
	double pp[3]; startNode->GetCoord3D(pp);
	double p1[3];
	double dist = 10000;
	QMeshNode *nextNode;

	for (GLKPOSITION Pos = boundIsoPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)boundIsoPatch->GetNodeList().GetNext(Pos);
		if (Node->connectTPathProcessed == true) continue;
		Node->GetCoord3D(p1);
		double distancePP = geo.Distance_to_Point(pp, p1);
		if (distancePP < dist) {
			nextNode = Node;
			dist = distancePP;
		}
	}

	if (dist > maxConnectDist) return nullptr;
	return nextNode;
}


QMeshNode* toolPathGeneration::findNextBoundaryToolPath(QMeshNode *startNode, QMeshPatch* boundIsoPatch) {

	QMeshNode* sNode = startNode;
	QMeshNode* eNode;

	bool nextNodeDetected = false;
	sNode->connectTPathProcessed = true;
	QMeshEdge* thisEdge = surfaceMesh->GetEdgeRecordPtr(sNode->relatedEdgeIndex + 1);

	//detect left face
	for (int i = 0; i < 3; i++) {
		QMeshEdge *NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;
		int index = edgeBPIndex(boundIsoPatch->GetIndexNo(), NeighborEdge->GetIndexNo());
		if (index >= 0) {
			QMeshNode* bNode = boundIsoPatch->GetNodeRecordPtr(index + 1);
			if (bNode->connectTPathProcessed == false) {
				nextNodeDetected = true;
				eNode = bNode;
				break;
			}
		}
	}

	if (nextNodeDetected) return eNode;

	//detect right face
	for (int i = 0; i < 3; i++) {
		QMeshEdge *NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;
		int index = edgeBPIndex(boundIsoPatch->GetIndexNo(), NeighborEdge->GetIndexNo());
		if (index >= 0) {
			QMeshNode* bNode = boundIsoPatch->GetNodeRecordPtr(index + 1);
			if (bNode->connectTPathProcessed == false) {
				nextNodeDetected = true;
				eNode = bNode;
				break;
			}
		}
	}
	if (nextNodeDetected) return eNode;
	else {
		//std::cout<<"Error, the next node is not found!"<<std::endl;
		return nullptr;
	}
}

/////////////////////////////
/* Resampling the toolpath */

double toolPathGeneration::resamplingSinglePatch(QMeshPatch *patch) {
	double length = computeAverageConnectEdgeLength(patch);

	//for boundary edge which not contains connection edge.
	if (length < 0) length = this->resamplingLength;

	//before resample, output the index - for debug
	//    for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
	//        QMeshEdge *Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
	//        std::cout<<Edge->GetIndexNo()<<" Edge, Node index = "
	//                <<Edge->GetStartPoint()->GetIndexNo() <<" , " <<Edge->GetEndPoint()->GetIndexNo()<<std::endl;
	//    }

	//std::cout<<"The average connect edge is "<<length<< std::endl;

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->resampleChecked = false;
	}

	double lsum = 0.0;
	//length = 0.5;
	QMeshNode* sNode = (QMeshNode*)patch->GetNodeList().GetHead();
	sNode->resampleChecked = true;

	QMeshNode* sPoint = sNode;
	QMeshNode* ePoint;

	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		ePoint = Edge->GetStartPoint();
		if (ePoint == sPoint) ePoint = Edge->GetEndPoint();

		if (Edge->isConnectEdge == true) {
			//std::cout<<"Connection Edge "<< Edge->GetIndexNo()<< "::" << sNode->GetIndexNo()<<","<<ePoint->GetIndexNo()<<endl;
			sPoint->resampleChecked = true;
			ePoint->resampleChecked = true;

			sNode = ePoint;
			sPoint = sNode;
			lsum = 0;
			continue;
		}

		if (ePoint == patch->GetNodeList().GetTail()) {
			//std::cout<<"End Node connected Edge"<< Edge->GetIndexNo()<< "::"<<sNode->GetIndexNo()<<","<<ePoint->GetIndexNo()<<endl;
			ePoint->resampleChecked = true;
			break;
		}

		lsum += Edge->CalLength();
		if (lsum > length) {
			//std::cout<<"Inner Edge"<< Edge->GetIndexNo()<< "::"<<sNode->GetIndexNo()<<","<<ePoint->GetIndexNo()<<endl;
			ePoint->resampleChecked = true;

			sNode = ePoint;
			sPoint = sNode;
			lsum = 0;
		}
		else {
			sPoint = ePoint;
		}
	}

	//rebuild the edge
	patch->GetEdgeList().RemoveAll();
	sNode = (QMeshNode*)patch->GetNodeList().GetHead();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (Node->resampleChecked) {
			buildNewEdgetoQMeshPatch(patch, sNode, Node);
			sNode = Node;
		}
	}

	return length;
}

double toolPathGeneration::computeAverageConnectEdgeLength(QMeshPatch *patch) {

	double length = 0;
	int num = 0;

	bool detectConnectEdge = false;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		if (Edge->isConnectEdge) {
			length += Edge->CalLength(); num++;
			detectConnectEdge = true;
		}
	}
	if(detectConnectEdge) return length / num;
	else return -1.0;
}


///////////////////////////////////////////////
/* Other assistant function */
QMeshNode* toolPathGeneration::buildNewNodetoQMeshPatch(QMeshPatch *patch, double pp[], double normal[])
{
	QMeshNode* isoNode = new QMeshNode;
	isoNode->SetMeshPatchPtr(patch);
	isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
	//isoNode->SetIndexNo(patch->GetNodeList().GetCount() + 1);
	isoNode->SetIndexNo(patch->GetNodeList().GetCount());

	patch->GetNodeList().AddTail(isoNode);

	isoNode->SetNormal(normal[0], normal[1], normal[2]);
	//std::cout<<normal[0]<<","<< normal[1]<<","<< normal[2]<<std::endl;
	isoNode->normalSet = true;

	return isoNode;
}

QMeshEdge* toolPathGeneration::buildNewEdgetoQMeshPatch
(QMeshPatch *patch, QMeshNode *startNode, QMeshNode *endNode) {

	QMeshEdge* isoEdge = new QMeshEdge;
	isoEdge->SetStartPoint(startNode);
	isoEdge->SetEndPoint(endNode);

	isoEdge->SetMeshPatchPtr(patch);
	//isoEdge->SetIndexNo(patch->GetEdgeList().GetCount() + 1);
	isoEdge->SetIndexNo(patch->GetEdgeList().GetCount());

	(startNode->GetEdgeList()).AddTail(isoEdge);
	(endNode->GetEdgeList()).AddTail(isoEdge);
	patch->GetEdgeList().AddTail(isoEdge);

	return isoEdge;
}

bool toolPathGeneration::detectSingleZigZagToolPathisProcessed(QMeshPatch *singlePath) {
	//if all the node being processed return true, else return false.
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (thisNode->connectTPathProcessed == false) return false;
	}
	return true;
}

bool toolPathGeneration::detectAllZigZagToolPathisProcessed() {
	//if all the patch being processed return true, else return false.
	for (GLKPOSITION Pos = zigzagPathList.GetHeadPosition(); Pos;) {
		QMeshPatch* patch = (QMeshPatch*)zigzagPathList.GetNext(Pos);
		if (detectSingleZigZagToolPathisProcessed(patch) == false) return false;
	}
	return true;
}

bool toolPathGeneration::detectAllBoundaryToolPathisProcessed() {
	//if all the patch being processed return true, else return false.
	for (GLKPOSITION Pos = boundPathList.GetHeadPosition(); Pos;) {
		QMeshPatch* patch = (QMeshPatch*)boundPathList.GetNext(Pos);
		if (detectSingleZigZagToolPathisProcessed(patch) == false) return false;
	}
	return true;
}

//true for zigzag, false for boundary
int toolPathGeneration::autoComputeTPathNum(bool type) {
	double distanceTPath = this->toolpathOffset;

	int iter = 10;
	std::vector<Eigen::MatrixXd> isoPoint(10);
	Eigen::VectorXd isoPointNum(iter);

	int maxNodeNum = 10000;

	for (int i = 0; i < iter; i++) {
		isoPoint[i] = Eigen::MatrixXd::Zero(maxNodeNum, 3);

		double isoValue = (0.5 + i) * 1.0 / iter;

		int index = 0;
		for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
			//Edge->relatedToolPathPointIndex = -1;
			double a, b;
			if (type) {
				a = Edge->GetStartPoint()->zigzagValue;
				b = Edge->GetEndPoint()->zigzagValue;
			}
			else {
				a = Edge->GetStartPoint()->boundaryValue;
				b = Edge->GetEndPoint()->boundaryValue;
			}

			if ((isoValue - a) * (isoValue - b) < 0.0) {
				double alpha = (isoValue - a) / (b - a);
				double p1[3], p2[3], pp[3];
				Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
				Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

				for (int j = 0; j < 3; j++) {
					//compute the position for this isonode
					if (index > maxNodeNum) { printf("ERROR, node number too high!\n"); break; }
					isoPoint[i](index, j) = (1.0 - alpha)*p1[j] + alpha*p2[j];
				}
				index++;
			}
		}
		isoPointNum(i) = index;
		//std::cout << "isovalue " << isoValue << " layer has " << index << " points" << std::endl;
	}

	Eigen::VectorXd distance(iter - 1);
	for (int i = 0; i < iter - 1; i++) {
		distance(i) = 1000000.0;

		for (int j = 0; j < isoPointNum(i); j++) {
			for (int k = 0; k < isoPointNum(i + 1); k++) {
				double dis = (isoPoint[i].row(j) - isoPoint[i + 1].row(k)).norm();
				if (dis < distance(i)) distance(i) = dis;
			}
		}
	}
	//std::cout << distance << std::endl;
	return floor(distance.sum() / distanceTPath);
}

//used to make sure the first toolpath will not be cut (change the shrinkoffset value)
double toolPathGeneration::autoComputeZigZagTPathOffestingValue(
	int zigzagTPathNum, int boundaryTPathNum, double boundaryTPathOffset) {

	double shrinkOffset = 0.0;
	double minBoundaryFieldValue = 1 - (boundaryTPathNum + shrinkOffset) * boundaryTPathOffset;

	double firstCurve_IsoValue;
	bool find = false;
	int indexNum = 0;
	for (indexNum = 0; indexNum < zigzagTPathNum; indexNum++) {
		double isoValue = (indexNum + 0.5) * 1.0 / zigzagTPathNum;

		for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

			double a = Edge->GetStartPoint()->zigzagValue;
			double b = Edge->GetEndPoint()->zigzagValue;

			if ((isoValue - a) * (isoValue - b) < 0.0) {
				double alpha = (isoValue - a) / (b - a);

				//compute boundary value for this isonode
				double boundaryValue = (1.0 - alpha) * Edge->GetStartPoint()->boundaryValue +
					alpha * Edge->GetEndPoint()->boundaryValue;

				if (boundaryValue < minBoundaryFieldValue) {
					find = true; break;
				}

			}
		}

		if (find) {
			firstCurve_IsoValue = isoValue;
			break;
		}
	}
	//std::cout << "MinBoundaryFieldValue = " << minBoundaryFieldValue << std::endl;
	//std::cout << "The non-zero tool path index = " << indexNum << std::endl;

	find = false;
	for (int i = 0; i < 7; i++) {
		if (this->runBreakingChecking(firstCurve_IsoValue, minBoundaryFieldValue)) {
			find = true;
			break;
		}
		else {
			shrinkOffset -= 0.1;
			minBoundaryFieldValue = 1 - (boundaryTPathNum + shrinkOffset) * boundaryTPathOffset;
		}
	}
	if (!find) std::cout << "Didn't find a proper offset value!" << std::endl;

	if (shrinkOffset < -0.5) return 0.0;
	return shrinkOffset;
}

bool toolPathGeneration::runBreakingChecking(double firstCurve_IsoValue, double minBoundaryFieldValue) {

	std::vector<QMeshNode*> nodeSet;

	std::vector<int> edgePIndexBreaking(surfaceMesh->GetEdgeNumber());
	for (int i = 0; i < edgePIndexBreaking.size(); i++) edgePIndexBreaking[i] = -1;

	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

		double a = Edge->GetStartPoint()->zigzagValue;
		double b = Edge->GetEndPoint()->zigzagValue;

		if ((firstCurve_IsoValue - a) * (firstCurve_IsoValue - b) < 0.0) {
			double alpha = (firstCurve_IsoValue - a) / (b - a);

			//compute boundary value for this isonode
			double boundaryValue = (1.0 - alpha) * Edge->GetStartPoint()->boundaryValue +
				alpha * Edge->GetEndPoint()->boundaryValue;

			if (boundaryValue < minBoundaryFieldValue) {

				QMeshNode* isoNode = new QMeshNode;
				nodeSet.push_back(isoNode);
				isoNode->relatedEdgeIndex = Edge->GetIndexNo(); // this is important
				isoNode->connectTPathProcessed = false;
				edgePIndexBreaking[Edge->GetIndexNo()] = nodeSet.size(); //index start from 1
			}
		}
	}

	int zigzagPathBoundaryNode = 0;
	for (int i = 0; i < nodeSet.size(); i++) {
		QMeshNode* thisNode = nodeSet[i];
		QMeshEdge* thisEdge = surfaceMesh->GetEdgeRecordPtr(thisNode->relatedEdgeIndex + 1);

		thisNode->isZigzagBoundaryNode = false;

		if (thisEdge->IsBoundaryEdge()) {
			thisNode->isZigzagBoundaryNode = true;
			zigzagPathBoundaryNode++;
			cout << "Warning:: Notice that, the zigzag toolpath node related to a boundary edge! you can try to make the boundary tool-path offset higher." << endl;
			continue;
		}

		int lFaceIsoNodeNum = 0; //left face
		for (int i = 0; i < 3; i++) {
			QMeshEdge *NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
			int index = edgePIndexBreaking[NeighborEdge->GetIndexNo()];
			if (index < 0) continue;
			else lFaceIsoNodeNum++;
		}
		int rFaceIsoNodeNum = 0; //right face
		for (int i = 0; i < 3; i++) {
			QMeshEdge *NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);
			int index = edgePIndexBreaking[NeighborEdge->GetIndexNo()];
			if (index < 0) continue;
			else rFaceIsoNodeNum++;
		}
		if (lFaceIsoNodeNum == 1 || rFaceIsoNodeNum == 1) {
			thisNode->isZigzagBoundaryNode = true;
			zigzagPathBoundaryNode++;
		}
	}

	if (zigzagPathBoundaryNode != 2)
		//std::cout << "ERROR, the first path have more than two boundary node!" << std::endl;
		return false;

	//QMeshNode* sNode; QMeshNode* eNode;
	//for (int i = 0; i < nodeSet.size(); i++) {
	//	if (nodeSet[i]->isZigzagBoundaryNode) {
	//		sNode = nodeSet[i];
	//		break;
	//	}
	//}
	//do {
	//	//Step1: find next node by mesh topology
	//	sNode->connectTPathProcessed = true;

	//	QMeshEdge* thisEdge = surfaceMesh->GetEdgeRecordPtr(sNode->relatedEdgeIndex);
	//	if (thisEdge->IsBoundaryEdge())
	//		cout << "ERROR, this zigzag toolpath node related edge should not belongs to boundary!" << endl;

	//	bool eNodeDetect = false;

	//	//detect if eNode located at left face
	//	for (int i = 0; i < 3; i++) {
	//		QMeshEdge *NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);

	//		if (NeighborEdge == thisEdge) continue;
	//		int index = edgePIndex[NeighborEdge->GetIndexNo()];
	//		if (index<0) continue;
	//		else {
	//			QMeshNode* dNode = nodeSet[index-1];
	//			if (dNode->connectTPathProcessed) continue;
	//			else {
	//				eNode = dNode;
	//				eNodeDetect = true;
	//			}

	//		}
	//	}

	//	//if eNode not find, check rightface
	//	if (eNodeDetect == false) {
	//		for (int i = 0; i < 3; i++) {
	//			QMeshEdge *NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);

	//			if (NeighborEdge == thisEdge) continue;
	//			int index = edgePIndex[NeighborEdge->GetIndexNo()];
	//			if (index<0) continue;
	//			else {
	//				QMeshNode* dNode = nodeSet[index - 1];
	//				if (dNode->connectTPathProcessed) continue;
	//				else {
	//					eNode = dNode;
	//					eNodeDetect = true;
	//				}
	//			}
	//		}
	//	}

	//	//cout<<detectNumLeft<<","<<detectNumRight<<endl;

	//	if (eNodeDetect == false) {
	//		cout << "ERROR, the next node is not founded! this should not happen for a zigzag region boundary node!" << endl;
	//		break;
	//	}

	//	//set eNode as sNode and install them back to the tPathRegion
	//	eNode->connectTPathProcessed = true;
	//	sNode = eNode;

	//} while (!sNode->isZigzagBoundaryNode);

	//for (int i = 0; i < nodeSet.size(); i++) {
	//	if (nodeSet[i]->connectTPathProcessed == false) return false;
	//}

	return true;
}












