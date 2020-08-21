// GLKGraph.cpp: implementation of the GLKGraph class.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "GLKHeap.h"
#include "GLKGraph.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKGraph::GLKGraph()
{
	nodeList.RemoveAll();	edgeList.RemoveAll();
}

GLKGraph::~GLKGraph()
{
	clearAll();
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

void GLKGraph::_Debug_for_MST_TSP()
{
	bool bForMinimumSpanningTreeOrTravelingSalesman=true;

/////////////////////////////////////////////////////////////////////////////////////////////
//
//                   8            8
//           B ------------- D ------ G
//         / |             /   \      | \
//    10 /   |         7 /      \     |  \ 2 
//     /     |         /       5 \    |9  \
//   A      9|       E            \   |    I
//     \     |     /   \           \  |   /
//    12 \   |  3/       \3         \ |  / 11 
//         \ | /           \         \| /
//           C ------------- F ------ H
//                   1           6
//
/////////////////////////////////////////////////////////////////////////////////////////////
	GLKGraphNode *node_a;	GLKGraphNode *node_b;	GLKGraphNode *node_c;	GLKGraphNode *node_d;	GLKGraphNode *node_e;
	GLKGraphNode *node_f;	GLKGraphNode *node_g;	GLKGraphNode *node_h;	GLKGraphNode *node_i;

	GLKGraphEdge *edgeAB;	GLKGraphEdge *edgeAC;	GLKGraphEdge *edgeBC;	GLKGraphEdge *edgeBD;	GLKGraphEdge *edgeCE;
	GLKGraphEdge *edgeCF;	GLKGraphEdge *edgeDE;	GLKGraphEdge *edgeDG;	GLKGraphEdge *edgeDH;	GLKGraphEdge *edgeEF;
	GLKGraphEdge *edgeFH;	GLKGraphEdge *edgeGH;	GLKGraphEdge *edgeGI;	GLKGraphEdge *edgeHI;

	node_a=new GLKGraphNode;	AddNode(node_a);	node_a->m_height=1;
	node_b=new GLKGraphNode;	AddNode(node_b);	node_b->m_height=2;
	node_c=new GLKGraphNode;	AddNode(node_c);	node_c->m_height=3;
	node_d=new GLKGraphNode;	AddNode(node_d);	node_d->m_height=4;
	node_e=new GLKGraphNode;	AddNode(node_e);	node_e->m_height=5;
	node_f=new GLKGraphNode;	AddNode(node_f);	node_f->m_height=6;
	node_g=new GLKGraphNode;	AddNode(node_g);	node_g->m_height=7;
	node_h=new GLKGraphNode;	AddNode(node_h);	node_h->m_height=8;
	node_i=new GLKGraphNode;	AddNode(node_i);	node_i->m_height=9;

	edgeAB=new GLKGraphEdge;	edgeAB->startNode=node_a;	edgeAB->endNode=node_b; edgeAB->m_weight=10.0;	AddEdge(edgeAB);
	edgeAC=new GLKGraphEdge;	edgeAC->startNode=node_a;	edgeAC->endNode=node_c; edgeAC->m_weight=12.0;	AddEdge(edgeAC);
	edgeBC=new GLKGraphEdge;	edgeBC->startNode=node_b;	edgeBC->endNode=node_c; edgeBC->m_weight=9.0;	AddEdge(edgeBC);
	edgeBD=new GLKGraphEdge;	edgeBD->startNode=node_b;	edgeBD->endNode=node_d; edgeBD->m_weight=8.0;	AddEdge(edgeBD);
	edgeCE=new GLKGraphEdge;	edgeCE->startNode=node_c;	edgeCE->endNode=node_e; edgeCE->m_weight=3.0;	AddEdge(edgeCE);

	edgeCF=new GLKGraphEdge;	edgeCF->startNode=node_c;	edgeCF->endNode=node_f; edgeCF->m_weight=1.0;	AddEdge(edgeCF);
	edgeDE=new GLKGraphEdge;	edgeDE->startNode=node_d;	edgeDE->endNode=node_e; edgeDE->m_weight=7.0;	AddEdge(edgeDE);
	edgeDG=new GLKGraphEdge;	edgeDG->startNode=node_d;	edgeDG->endNode=node_g; edgeDG->m_weight=8.0;	AddEdge(edgeDG);
	edgeDH=new GLKGraphEdge;	edgeDH->startNode=node_d;	edgeDH->endNode=node_h; edgeDH->m_weight=5.0;	AddEdge(edgeDH);
	edgeEF=new GLKGraphEdge;	edgeEF->startNode=node_e;	edgeEF->endNode=node_f; edgeEF->m_weight=3.0;	AddEdge(edgeEF);

	edgeFH=new GLKGraphEdge;	edgeFH->startNode=node_f;	edgeFH->endNode=node_h; edgeFH->m_weight=6.0;	AddEdge(edgeFH);
	edgeGH=new GLKGraphEdge;	edgeGH->startNode=node_g;	edgeGH->endNode=node_h; edgeGH->m_weight=9.0;	AddEdge(edgeGH);
	edgeGI=new GLKGraphEdge;	edgeGI->startNode=node_g;	edgeGI->endNode=node_i; edgeGI->m_weight=2.0;	AddEdge(edgeGI);
	edgeHI=new GLKGraphEdge;	edgeHI->startNode=node_h;	edgeHI->endNode=node_i; edgeHI->m_weight=11.0;	AddEdge(edgeHI);

	FillInEdgeLinkersOnNodes();

	if (bForMinimumSpanningTreeOrTravelingSalesman) {
		GLKObList trRootList,trEdgeList;

		double pathLength=MinimumSpanningTree(&trRootList, &trEdgeList, true);

		GLKPOSITION Pos;
		printf("%d roots are found!\n",trRootList.GetCount());
		printf("The first root is node %d\n",((GLKGraphNode*)(trRootList.GetHead()))->m_height);
		printf("--------------------------------------\n");
		for(Pos=trEdgeList.GetHeadPosition();Pos!=NULL;) {
			GLKGraphEdge *edge=(GLKGraphEdge *)(trEdgeList.GetNext(Pos));
			printf("Edge: %d-%d (cost=%lf)\n",edge->startNode->m_height,edge->endNode->m_height,edge->m_weight);
		}
		printf("--------------------------------------\n");
		printf("Total cost: %lf\n",pathLength);
	}
	else {
		GLKObList nodeVisitList;

		ApproximateTravelingSalesmanProblemTour(&nodeVisitList);

		GLKPOSITION Pos;
		printf("--------------------------------------\n");
		printf("The tour of Traveling Salesman is:\n");
		printf("--------------------------------------\n");
		for(Pos=nodeVisitList.GetHeadPosition();Pos!=NULL;) {
			GLKGraphNode *node=(GLKGraphNode *)(nodeVisitList.GetNext(Pos));
			printf("%d ",node->m_height);
		}
		printf("\n");
	}
}

void GLKGraph::_Debug_for_MinimumCut() 
{
	GLKGraphNode *node1;
	GLKGraphNode *node2;
	GLKGraphNode *node3;
	GLKGraphNode *node4;
	GLKGraphNode *node5;
	GLKGraphEdge *edge1;
	GLKGraphEdge *edge2;
	GLKGraphEdge *edge3;
	GLKGraphEdge *edge4;
	GLKGraphEdge *edge5;
	GLKGraphEdge *edge6;
	GLKGraphEdge *edge7;
	GLKObList srList,trList;

	node1=new GLKGraphNode;	AddNode(node1);
	node2=new GLKGraphNode;	AddNode(node2);
	node3=new GLKGraphNode;	AddNode(node3);
	node4=new GLKGraphNode;	AddNode(node4);
	node5=new GLKGraphNode;	AddNode(node5);

	edge1=new GLKGraphEdge;	edge1->startNode=node1;	edge1->endNode=node2; edge1->m_weight=12.0;	AddEdge(edge1);
	edge2=new GLKGraphEdge;	edge2->startNode=node3;	edge2->endNode=node1; edge2->m_weight=14.0;	AddEdge(edge2);
	edge3=new GLKGraphEdge;	edge3->startNode=node2;	edge3->endNode=node3; edge3->m_weight=5.0;	AddEdge(edge3);
	edge4=new GLKGraphEdge;	edge4->startNode=node3;	edge4->endNode=node4; edge4->m_weight=8.0;	AddEdge(edge4);
	edge5=new GLKGraphEdge;	edge5->startNode=node4;	edge5->endNode=node5; edge5->m_weight=10.0;	AddEdge(edge5);
	edge6=new GLKGraphEdge;	edge6->startNode=node4;	edge6->endNode=node2; edge6->m_weight=7.0;	AddEdge(edge6);
	edge7=new GLKGraphEdge;	edge7->startNode=node2;	edge7->endNode=node5; edge7->m_weight=16.0;	AddEdge(edge7);

	FillInEdgeLinkersOnNodes();
	double maxFlow=MinimumCut(node1,node5,&srList,&trList,true);

/*	GLKPOSITION Pos;
	for(Pos=srList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(srList.GetNext(Pos));
	}*/
}

void GLKGraph::ApproximateTravelingSalesmanProblemTour(GLKObList *nodeVisitList)
{
	GLKPOSITION Pos;
	GLKObList trRootList,trEdgeList;

	nodeVisitList->RemoveAll();	
	MinimumSpanningTree(&trRootList,&trEdgeList,true);
	
	//--------------------------------------------------------------------------------
	//	Preparation
	int maxEdgeNum=0;
	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		edge->m_bFlag=false;
	}

	//--------------------------------------------------------------------------------
	//	Tree-traversal for generating the tour
	for(Pos=trRootList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *rootNode=(GLKGraphNode *)(trRootList.GetNext(Pos));
		_traversalOfMSTForTSP(rootNode, nodeVisitList);
	}
}

void GLKGraph::_traversalOfMSTForTSP(GLKGraphNode *rootNode, GLKObList *nodeVisitList)
{
	GLKPOSITION Pos;
	GLKGraphNode *linkedNode;

	nodeVisitList->AddTail(rootNode);
	for(Pos=rootNode->edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(rootNode->edgeList.GetNext(Pos));
		if (edge->m_bFlag) continue;
		if (edge->startNode==rootNode) linkedNode=edge->endNode; else linkedNode=edge->startNode;

		edge->m_bFlag=true;
		_traversalOfMSTForTSP(linkedNode,nodeVisitList);
	}
}

double GLKGraph::MinimumSpanningTree(GLKObList *trRootList, GLKObList *trEdgeList, 
									 bool bBuildConnectivityForTree /* When this is set to true, the edgeList of each graph-node will be updated*/)
{
	GLKPOSITION Pos;	double cost=0.0;
	GLKHeap *minHeap;	
	int visitedNodeNum=0;

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		node->m_bFlag=false;
	}
	trRootList->RemoveAll();	trEdgeList->RemoveAll();

	while(visitedNodeNum<nodeList.GetCount()) {
		//--------------------------------------------------------------------------------
		//	Find the root node randomly
		GLKGraphNode *rootNode=NULL,*linkedNode,*nextNode;
		for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
			GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
			if (!(node->m_bFlag)) {rootNode=node; break;}
		}
		if (rootNode==NULL) break;
		

		//--------------------------------------------------------------------------------
		//	Intialization for the heap
		minHeap=new GLKHeap(edgeList.GetCount(),true);	trRootList->AddTail(rootNode);
		//--------------------------------------------------------------------------------
		rootNode->m_bFlag=true;	visitedNodeNum++;
		for(Pos=rootNode->edgeList.GetHeadPosition();Pos!=NULL;) {
			GLKGraphEdge *edge=(GLKGraphEdge *)(rootNode->edgeList.GetNext(Pos));
			if (edge->startNode==rootNode) linkedNode=edge->endNode; else linkedNode=edge->startNode;
			if (linkedNode->m_bFlag) continue;

			GLKHeapNode *heapNode=new GLKHeapNode;
			heapNode->attachedObj=edge;
			heapNode->SetValue((float)edge->m_weight);
			minHeap->Insert(heapNode);
		}

		//--------------------------------------------------------------------------------
		//	Using heap to construct the minimum spanning tree rooted at the rootNode
		while(!(minHeap->ListEmpty())) {
			GLKHeapNode *topHeapNode=minHeap->RemoveTop();
			GLKGraphEdge *edge=(GLKGraphEdge *)(topHeapNode->attachedObj);
			if (edge->startNode->m_bFlag) nextNode=edge->endNode; else nextNode=edge->startNode;
			delete topHeapNode;
			if (nextNode->m_bFlag) continue;

			cost+=edge->m_weight;	trEdgeList->AddTail(edge);	
			nextNode->m_bFlag=true;	visitedNodeNum++;
			for(Pos=nextNode->edgeList.GetHeadPosition();Pos!=NULL;) {
				GLKGraphEdge *edge=(GLKGraphEdge *)(nextNode->edgeList.GetNext(Pos));
				if (edge->startNode==nextNode) linkedNode=edge->endNode; else linkedNode=edge->startNode;
				if (linkedNode->m_bFlag) continue;

				GLKHeapNode *heapNode=new GLKHeapNode;
				heapNode->attachedObj=edge;
				heapNode->SetValue((float)edge->m_weight);
				minHeap->Insert(heapNode);
			}
		}

		//--------------------------------------------------------------------------------
		//	Release memory of the heap
		delete minHeap;
	}

	if (bBuildConnectivityForTree) {
		for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
			GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
			node->edgeList.RemoveAll();
		}
		for(Pos=trEdgeList->GetHeadPosition();Pos!=NULL;) {
			GLKGraphEdge *edge=(GLKGraphEdge *)(trEdgeList->GetNext(Pos));
			edge->startNode->edgeList.AddTail(edge);
			edge->endNode->edgeList.AddTail(edge);
		}
	}

	return cost;
}

double GLKGraph::MinimumCut(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
							GLKObList *sourceRegionNodeList, GLKObList *targetRegionNodeList,
							bool bComputeMaxFlow)
{
	GLKObList linkList;
	GLKPOSITION Pos;

	_initializePreflow(sourceNode);

	linkList.RemoveAll();
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		if (node==sourceNode) continue;
		if (node==targetNode) continue;
		linkList.AddTail(node);
	}

	int oldHeight;	GLKGraphNode *uNode;
	Pos=linkList.GetHeadPosition();
	while(Pos!=NULL) {
		uNode=(GLKGraphNode *)(linkList.GetAt(Pos));
	
		oldHeight=uNode->m_height;
		
		_discharge(uNode);
		
		if (uNode->m_height>oldHeight) {
			linkList.RemoveAt(Pos);
			linkList.AddHead(uNode);
			Pos=linkList.GetHeadPosition();
		}
		
		linkList.GetNext(Pos);
	}

	_partitionByResidualGraph(sourceNode,targetNode,sourceRegionNodeList,targetRegionNodeList);
	if (bComputeMaxFlow) return _computeMaxFlow();
	return 0.0;
}

void GLKGraph::_partitionByResidualGraph(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
										 GLKObList *sourceRegionNodeList, 
										 GLKObList *targetRegionNodeList)
{
	GLKPOSITION Pos;

	sourceRegionNodeList->RemoveAll();
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		node->m_height=0;
	}

	_propagateInResidualGraph(sourceNode,sourceRegionNodeList);

	targetRegionNodeList->RemoveAll();
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		if (node->m_height==0) targetRegionNodeList->AddTail(node);
	}
}

double GLKGraph::_computeMaxFlow()
{
	GLKPOSITION Pos;
	double value=0.0;

	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		if ((edge->startNode->m_height==1) && (edge->endNode->m_height==0))
			value+=edge->m_weight;
		if ((edge->endNode->m_height==1) && (edge->startNode->m_height==0))
			value+=edge->m_weight;
	}

	return value;
}

void GLKGraph::_propagateInResidualGraph(GLKGraphNode *node, GLKObList *regionNodeList)
{
	GLKPOSITION Pos;
	GLKGraphNode *otherNode;

	regionNodeList->AddTail(node);
	node->m_height=1;	// to specify that it has been added into the list

	for(Pos=node->edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(node->edgeList.GetNext(Pos));
		double cf;

		if (edge->startNode==node) {
			cf=edge->m_weight-edge->m_flow;
			if (cf<=1.0e-8) continue;
			otherNode=edge->endNode;
		}
		else {
			cf=edge->m_weight+edge->m_flow;
			if (cf<=1.0e-8) continue;
			otherNode=edge->startNode;
		}

		if (otherNode->m_height==0) // to detect whether it has been added into the list
			_propagateInResidualGraph(otherNode, regionNodeList);
	}
}

void GLKGraph::_discharge(GLKGraphNode *uNode)
{
	GLKPOSITION Pos=uNode->edgeList.GetHeadPosition();

	while(uNode->m_excess>0.0) {
		if (Pos==NULL) {
			_relable(uNode);
			Pos=uNode->edgeList.GetHeadPosition();
		}
		else {
			GLKGraphEdge *edge=(GLKGraphEdge *)(uNode->edgeList.GetAt(Pos));
			_push(uNode,edge);
			uNode->edgeList.GetNext(Pos);
		}
	}
}

void GLKGraph::_push(GLKGraphNode *uNode, GLKGraphEdge *edge)
{
	GLKGraphNode *vNode;
	double df,cf;

	if (uNode->m_excess==0.0) return;

	if (uNode==edge->startNode) {
		vNode=edge->endNode;
		if (uNode->m_height!=(vNode->m_height+1)) return;
		cf=edge->m_weight-edge->m_flow;	
		if (cf<=0.0) return;

		df=uNode->m_excess;
		if (cf<df) df=cf;

		edge->m_flow=edge->m_flow+df;
		uNode->m_excess-=df;
		vNode->m_excess+=df;
	}
	else {	// the edge pointing to vNode to uNode but with some flow filled
		vNode=edge->startNode;
		if (uNode->m_height!=(vNode->m_height+1)) return;
		cf=edge->m_weight+edge->m_flow;
		if (cf<=0.0) return;

		df=uNode->m_excess;
		if (cf<df) df=cf;

		edge->m_flow=edge->m_flow-df;
		uNode->m_excess-=df;
		vNode->m_excess+=df;
	}
}

void GLKGraph::_relable(GLKGraphNode *uNode)
{
	if (uNode->m_excess==0.0) return;

	GLKGraphNode *vNode;
	GLKPOSITION Pos;	int minH;	double cf;	
	
	minH=-1;
	for(Pos=uNode->edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(uNode->edgeList.GetNext(Pos));
		if (edge->startNode==uNode) {
			vNode=edge->endNode;
			cf=edge->m_weight-edge->m_flow;
			if (cf<=0.0) continue;
//			if (uNode->m_height>vNode->m_height) continue;
			if ((minH<0) || (vNode->m_height<minH))	minH=vNode->m_height;
		}
		else {
			vNode=edge->startNode;
			cf=edge->m_weight+edge->m_flow;
			if (cf<=0.0) continue;
//			if (uNode->m_height>vNode->m_height) continue;
			if ((minH<0) || (vNode->m_height<minH))	minH=vNode->m_height;
		}
	}
	if (minH<0) return;

	uNode->m_height=1+minH;
}

void GLKGraph::_initializePreflow(GLKGraphNode *sourceNode)
{
	GLKPOSITION Pos;

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		node->m_height=0;
		node->m_excess=0.0;
	}
	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		edge->m_flow=0.0;
	}
	sourceNode->m_height=nodeList.GetCount();

	for(Pos=sourceNode->edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(sourceNode->edgeList.GetNext(Pos));
		if (edge->startNode==sourceNode) {	// pointing out direction
			edge->m_flow=edge->m_weight; // capacity
			edge->endNode->m_excess=edge->m_weight;
			sourceNode->m_excess=sourceNode->m_excess-edge->m_weight;
		}
		else {	// pointing out direction
			edge->m_flow=-(edge->m_weight);
			edge->startNode->m_excess=edge->m_weight;
			sourceNode->m_excess=sourceNode->m_excess-edge->m_weight;
		}
	}
}

void GLKGraph::AddNode(GLKGraphNode *node)
{
	nodeList.AddTail(node);
}

void GLKGraph::FillInEdgeLinkersOnNodes()
{
	GLKPOSITION Pos;

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		node->edgeList.RemoveAll();
	}

	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		edge->startNode->edgeList.AddTail(edge);
		edge->endNode->edgeList.AddTail(edge);
	}
}

void GLKGraph::AddEdge(GLKGraphEdge *edge)
{
	edgeList.AddTail(edge);
}

void GLKGraph::clearAll()
{
	GLKPOSITION Pos;

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		delete node;
	}
	nodeList.RemoveAll();

	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		delete edge;
	}
	edgeList.RemoveAll();
}
