#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <time.h>
//#include <omp.h>

#include "../GLKLib/GLKHeap.h"
#include "../GLKLib/GLKObList.h"
#include "../GLKLib/GLKGeometry.h"

#include "../LinearSolver/SpMatrix.h"
#include "../LinearSolver/SuperLU.h"

#include "../PQP/PQP.h"

#include "QMeshPatch.h"
#include "QMeshFace.h"
#include "QMeshEdge.h"
#include "QMeshNode.h"

#include "QDiscreteShortestPath.h"
#include "QBndSpiral.h"

#ifdef _DEBUG	// for detecting memory leak, using this - you need to use MFC DLL setting in compiling
#include <afx.h>         // MFC core and standard components
#define new DEBUG_NEW
#endif

class QMeshEdgeNode : public GLKObject {
public:
	QMeshEdgeNode() {
		hostEdgeOrNode=NULL; heapNodePtr=NULL;	bEdgeOrNode=false; 
		locOnEdge=0.5f; 
	};
public:
	void *hostEdgeOrNode;	
	void *heapNodePtr;
	float locOnEdge;	
	bool bEdgeOrNode;	//	true:	ptrEdgeOrNode is for QMeshEdge
						//	false:	ptrEdgeOrNode is for QMeshNode
};

int pqpFaceNum;		
QMeshFace** pqpFaceArray;
PQP_Model *pqpModel;
bool pqpSnappingBOOL;

QBndSpiral::QBndSpiral(void)
{
}

QBndSpiral::~QBndSpiral(void)
{
}
	
int QBndSpiral::CheckMaxPatchID(QMeshPatch *mesh)
{
	GLKPOSITION Pos;		QMeshFace *face;
	int maxPatchID=-1;

	for(Pos=mesh->GetFaceList().GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace *)(mesh->GetFaceList().GetNext(Pos));
		maxPatchID=MAX(face->m_nIdentifiedPatchIndex,maxPatchID);
	}

	return maxPatchID;
}

double QBndSpiral::CompAverageEdgeLength(QMeshPatch *mesh)
{
	double div,avgLength;
	GLKPOSITION Pos;

	div=avgLength=0.0;
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		QMeshEdge *edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
		div+=1.0;	avgLength+=edge->CalLength();
	}
	avgLength=avgLength/div;
	printf("Average Length = %lf \n",avgLength);

	return avgLength;
}

bool QBndSpiral::AnalyzeBndDistMap(QMeshPatch *mesh, double maxValue, double offsetValue, int maxPatchID)
{
	GLKPOSITION Pos;	int peakNum;	bool bPeakPnt,bWithProblematicContour=false;
	QMeshNode *node,*linkedNode;	
	QMeshEdge *edge;
	GLKPOSITION PosLink;

	int i,curveNum=(int)(maxValue/offsetValue);
	double isoValue,deltaIsoValue=maxValue-offsetValue*(double)curveNum;
	GLKObList isoCurveNodeList;
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		node->SetAttribFlag(4,false);
	}
	for(int patchID=-1;patchID<=maxPatchID;patchID++) {
		if (maxPatchID>0 && patchID<=0) continue;
		for(i=0;i<=curveNum;i++) {
			isoValue=offsetValue*(double)i+deltaIsoValue*0.5;
			bool bRes=_generateIsocurve(mesh, isoValue, &isoCurveNodeList, patchID);
			if (isoCurveNodeList.IsEmpty()) bRes=true;
			if (!bRes) { // the topology of this isocurve is complex (there are multiple loops)
				printf("Warning: the %d-th isocurve has complex topology - isovalue = %f\n",i,(float)isoValue);
				for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
					node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
					if (node->GetBoundaryDis()>isoValue) node->SetAttribFlag(4);
				}
				bWithProblematicContour=true;
				break;
			}

			for(Pos=isoCurveNodeList.GetHeadPosition();Pos!=NULL;) {
				QMeshNode *node=(QMeshNode *)(isoCurveNodeList.GetNext(Pos));
				delete node;
			}
			isoCurveNodeList.RemoveAll();
		}
	}

	return bWithProblematicContour;
}

double QBndSpiral::GenerateBndDistMap(QMeshPatch *mesh, int patchID)
{
	GLKPOSITION Pos;
	QMeshNode *node,*linkedNode;
	QMeshEdge *edge;	
	QMeshFace *face;

	//--------------------------------------------------------------------------------------------
	//	Step 1: Initialization
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
		edge->SetAttribFlag(0,false);
		edge->CalLength();	
		edge->SetAttribFlag(7,false);	// to specify if the edge belongs to the patch
		if (edge->GetLeftFace()==NULL || edge->GetRightFace()==NULL) edge->SetAttribFlag(0,true);
		if (edge->GetLeftFace()!=NULL && edge->GetLeftFace()->m_nIdentifiedPatchIndex==patchID) edge->SetAttribFlag(7,true);
		if (edge->GetRightFace()!=NULL && edge->GetRightFace()->m_nIdentifiedPatchIndex==patchID) edge->SetAttribFlag(7,true);
	}
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		node->SetAttribFlag(0,false);	// boundary flag
		node->SetAttribFlag(4,false);	// boundary flag
		node->SetAttribFlag(7,false);	// flag to specify whether it has been processed by the heap
		node->attachedPointer=NULL;
	}
	for(Pos=mesh->GetFaceList().GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace *)(mesh->GetFaceList().GetNext(Pos));
		if (face->m_nIdentifiedPatchIndex!=patchID) continue;
		int edgeNum=face->GetEdgeNum();
		for(int i=0;i<edgeNum;i++) {face->GetNodeRecordPtr(i)->SetBoundaryDis(1.0e+5);}
	}
	//--------------------------------------------------------------------------------------------
	GLKHeap *heap=new GLKHeap(mesh->GetNodeNumber(),true);
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
		if (edge->GetAttribFlag(0) 
			|| (edge->GetLeftFace()->m_nIdentifiedPatchIndex!=edge->GetRightFace()->m_nIdentifiedPatchIndex)) {
			edge->GetStartPoint()->SetAttribFlag(0,true);	
			edge->GetEndPoint()->SetAttribFlag(0,true);
			if (edge->GetAttribFlag(7)) {
				edge->GetStartPoint()->SetBoundaryDis(0.0); edge->GetEndPoint()->SetBoundaryDis(0.0);
				if (edge->GetStartPoint()->attachedPointer==NULL) {
					GLKHeapNode *heapNode=new GLKHeapNode;	heapNode->SetValue(0.0f);
					heapNode->attachedObj=edge->GetStartPoint();	
					heap->Insert(heapNode);	
					edge->GetStartPoint()->SetAttribFlag(7,true);
					edge->GetStartPoint()->attachedPointer=heapNode;
				}
				if (edge->GetEndPoint()->attachedPointer==NULL) {
					GLKHeapNode *heapNode=new GLKHeapNode;	heapNode->SetValue(0.0f);
					heapNode->attachedObj=edge->GetEndPoint();	
					heap->Insert(heapNode);	
					edge->GetEndPoint()->SetAttribFlag(7,true);
					edge->GetEndPoint()->attachedPointer=heapNode;
				}
			}
		}
	}

	//--------------------------------------------------------------------------------------------
	//	Step 2: Heap based discrete boundary distance map
	float weight;	
	while(!(heap->ListEmpty())){
		GLKHeapNode *heapNode=heap->RemoveTop();
		node=(QMeshNode *)heapNode->attachedObj;	
		node->SetBoundaryDis(heapNode->GetValue());
		for(Pos=node->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
			edge=(QMeshEdge *)(node->GetEdgeList().GetNext(Pos));
			if (!(edge->GetAttribFlag(7))) continue;	// i.e., the path is NOT on the specified patch

			weight=(float)(edge->GetLength())+heapNode->GetValue();
			if (edge->GetStartPoint()==node) linkedNode=edge->GetEndPoint(); else linkedNode=edge->GetStartPoint();
			if (linkedNode->attachedPointer!=NULL) {
				GLKHeapNode *heapNode=(GLKHeapNode *)(linkedNode->attachedPointer);
				if (heapNode->GetValue()>weight) {
					heapNode->SetValue(weight);		heap->AdjustPosition(heapNode);
				}
			}
			else {
				if (!(linkedNode->GetAttribFlag(7))) {	// Not processed nodes need to be inserted into the heap
					GLKHeapNode *heapNode=new GLKHeapNode;	heapNode->SetValue(weight);
					heapNode->attachedObj=linkedNode;	
					heap->Insert(heapNode);		linkedNode->SetAttribFlag(7,true);
					linkedNode->attachedPointer=heapNode;
				}
			}
		}
		node->attachedPointer=NULL;	delete heapNode;
	}

	//--------------------------------------------------------------------------------------------
	//	Step 3: Check the range of the distances
	double maxValue=0.0;	QMeshNode *maxValueNode=NULL;
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		if (!(node->GetAttribFlag(7))) continue;
		if (node->GetBoundaryDis()>maxValue) {maxValue=node->GetBoundaryDis(); maxValueNode=node;}
	}
	if (maxValueNode!=NULL) maxValueNode->SetAttribFlag(4);

	//--------------------------------------------------------------------------------------------
	//	Step 4: Free the memory
	delete heap;

	return maxValue;
}

double QBndSpiral::GenerateBndDistMap2(QMeshPatch *mesh, short refinedNodeNum)	// Approximate version - by refined edge-nodes
{
	GLKPOSITION Pos;		GLKGeometry geo;
	GLKPOSITION Pos2;
	QMeshNode *node;
	QMeshEdge *edge,*oppEdge;	QMeshFace *face;
	QMeshEdgeNode *attachedNode,*linkedAttachedNode;

	//--------------------------------------------------------------------------------------------
	//	Step 1: Initialization
	//--------------------------------------------------------------------------------------------
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		node->SetBoundaryDis(1.0e+5);	
		node->SetAttribFlag(0,false);	// boundary flag
		attachedNode=new QMeshEdgeNode;	
		attachedNode->hostEdgeOrNode=node;	attachedNode->bEdgeOrNode=false;
		node->attachedPointer=attachedNode;
	}
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
		edge->SetAttribFlag(0,false);
		if (edge->GetLeftFace()==NULL || edge->GetRightFace()==NULL) {
			edge->SetAttribFlag(0,true);
			edge->GetStartPoint()->SetAttribFlag(0,true);
			edge->GetEndPoint()->SetAttribFlag(0,true);
		}
		edge->CalLength();		
		edge->GetAttachedList().RemoveAll();
		for(short i=0;i<refinedNodeNum;i++) {
			attachedNode=new QMeshEdgeNode;	
			attachedNode->hostEdgeOrNode=edge;	attachedNode->bEdgeOrNode=true;
			edge->GetAttachedList().AddTail(attachedNode);
		}
	}
	//--------------------------------------------------------------------------------------------
	GLKHeap *heap=new GLKHeap(mesh->GetNodeNumber()+mesh->GetEdgeNumber()*(int)refinedNodeNum,true);
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		GLKHeapNode *heapNode=new GLKHeapNode;	
		heap->Insert(heapNode);		
		attachedNode=(QMeshEdgeNode *)(node->attachedPointer);
		attachedNode->heapNodePtr=heapNode;		heapNode->attachedObj=attachedNode;
		if (node->GetAttribFlag(0)) heapNode->SetValue(0.0f); else heapNode->SetValue(1.0e+5f);
	}
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
		for(Pos2=edge->GetAttachedList().GetHeadPosition();Pos2!=NULL;) {
			attachedNode=(QMeshEdgeNode *)(edge->GetAttachedList().GetNext(Pos2));
			GLKHeapNode *heapNode=new GLKHeapNode;	
			if (edge->GetAttribFlag(0)) heapNode->SetValue(0.0f); else heapNode->SetValue(1.0e+5f);
			heap->Insert(heapNode);	
			attachedNode->heapNodePtr=heapNode;		heapNode->attachedObj=attachedNode;
		}
	}

	//--------------------------------------------------------------------------------------------
	//	Step 2: Heap based discrete boundary distance map
	float weight;	
	while(!(heap->ListEmpty())){
		GLKHeapNode *heapNode=heap->RemoveTop();
		attachedNode=(QMeshEdgeNode *)(heapNode->attachedObj);
		if (attachedNode->bEdgeOrNode) {  // on a QMeshEdge	
			double cp[3],sp[3],ep[3],pp[3],tt;

			edge=(QMeshEdge *)(attachedNode->hostEdgeOrNode);
			int index,i=0;
			for(Pos2=edge->GetAttachedList().GetHeadPosition();Pos2!=NULL;i++) {
				linkedAttachedNode=(QMeshEdgeNode *)(edge->GetAttachedList().GetNext(Pos2));
				if (linkedAttachedNode==attachedNode) {index=i; tt=(double)(i+1)/(double)(refinedNodeNum+1); break;}
			}
			
			//-----------------------------------------------------------------------------------
			//	for nodes on the same edge
			i=0;
			for(Pos2=edge->GetAttachedList().GetHeadPosition();Pos2!=NULL;i++) {
				linkedAttachedNode=(QMeshEdgeNode *)(edge->GetAttachedList().GetNext(Pos2));
				if (linkedAttachedNode==attachedNode) continue;
				weight=heapNode->GetValue()+(float)(fabs((double)i-(double)index)/(double)(refinedNodeNum+1)*edge->GetLength());
				GLKHeapNode *heapNode2=(GLKHeapNode *)(linkedAttachedNode->heapNodePtr);
				if (heapNode2==NULL) continue;
				if (heapNode2->GetValue()>weight) {heapNode2->SetValue(weight); heap->AdjustPosition(heapNode2);}
			}

			//-----------------------------------------------------------------------------------
			//	for nodes on the same face
			edge->GetStartPoint()->GetCoord3D(sp[0],sp[1],sp[2]);		
			edge->GetEndPoint()->GetCoord3D(ep[0],ep[1],ep[2]);
			cp[0]=(1.0-tt)*sp[0]+tt*ep[0];	
			cp[1]=(1.0-tt)*sp[1]+tt*ep[1];
			cp[2]=(1.0-tt)*sp[2]+tt*ep[2];
			for(int k=0;k<2;k++) {
				if (k==0) face=edge->GetLeftFace(); else face=edge->GetRightFace();
				if (face==NULL) continue;
				int j,edgeNum=face->GetEdgeNum();
				for(j=0;j<edgeNum;j++) {
					node=face->GetNodeRecordPtr(j);
					node->GetCoord3D(pp[0],pp[1],pp[2]);
					weight=heapNode->GetValue()+(float)(geo.Distance_to_Point(cp,pp));

					linkedAttachedNode=(QMeshEdgeNode *)(node->attachedPointer);
					GLKHeapNode *heapNode2=(GLKHeapNode *)(linkedAttachedNode->heapNodePtr);
					if (heapNode2==NULL) continue;
					if (heapNode2->GetValue()>weight) {heapNode2->SetValue(weight); heap->AdjustPosition(heapNode2);}
				}

				for(j=0;j<edgeNum;j++) {
					oppEdge=face->GetEdgeRecordPtr(j);	
					if (edge==oppEdge) continue;
					i=0;
					for(Pos2=oppEdge->GetAttachedList().GetHeadPosition();Pos2!=NULL;i++) {
						linkedAttachedNode=(QMeshEdgeNode *)(oppEdge->GetAttachedList().GetNext(Pos2));
						tt=tt=(double)(i+1)/(double)(refinedNodeNum+1);
						oppEdge->GetStartPoint()->GetCoord3D(sp[0],sp[1],sp[2]);		
						oppEdge->GetEndPoint()->GetCoord3D(ep[0],ep[1],ep[2]);
						pp[0]=(1.0-tt)*sp[0]+tt*ep[0];	
						pp[1]=(1.0-tt)*sp[1]+tt*ep[1];
						pp[2]=(1.0-tt)*sp[2]+tt*ep[2];

						weight=heapNode->GetValue()+(float)(geo.Distance_to_Point(cp,pp));
						GLKHeapNode *heapNode2=(GLKHeapNode *)(linkedAttachedNode->heapNodePtr);
						if (heapNode2==NULL) continue;
						if (heapNode2->GetValue()>weight) {heapNode2->SetValue(weight); heap->AdjustPosition(heapNode2);}
					}
				}
			}
		}
		else {	// on a QMeshNode
			node=(QMeshNode *)(attachedNode->hostEdgeOrNode);
			//-----------------------------------------------------------------------------------
			//	for nodes on the adjacent edges
			for(Pos=node->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
				edge=(QMeshEdge *)(node->GetEdgeList().GetNext(Pos));
				if (edge->GetStartPoint()==node) {
					linkedAttachedNode=(QMeshEdgeNode *)(edge->GetAttachedList().GetHead());
				}
				else {
					linkedAttachedNode=(QMeshEdgeNode *)(edge->GetAttachedList().GetTail());
				}
				weight=heapNode->GetValue()+(float)(edge->GetLength())/(float)(refinedNodeNum+1);
				GLKHeapNode *heapNode2=(GLKHeapNode *)(linkedAttachedNode->heapNodePtr);
				if (heapNode2==NULL) continue;
				if (heapNode2->GetValue()>weight) {heapNode2->SetValue(weight); heap->AdjustPosition(heapNode2);}
			}

			//-----------------------------------------------------------------------------------
			//	for nodes on the opporsite edge of adjacent faces
			double cp[3],sp[3],ep[3],pp[3],tt;
			node->GetCoord3D(cp[0],cp[1],cp[2]);
			for(Pos=node->GetFaceList().GetHeadPosition();Pos!=NULL;) {
				face=(QMeshFace *)(node->GetFaceList().GetNext(Pos));
				int edgeNum=face->GetEdgeNum();
				for(int j=0;j<edgeNum;j++) {
					edge=face->GetEdgeRecordPtr(j);
					if (edge->GetStartPoint()==node || edge->GetEndPoint()==node) continue;

					edge->GetStartPoint()->GetCoord3D(sp[0],sp[1],sp[2]);		
					edge->GetEndPoint()->GetCoord3D(ep[0],ep[1],ep[2]);
					int i=0;
					for(Pos2=edge->GetAttachedList().GetHeadPosition();Pos2!=NULL;i++) {
						linkedAttachedNode=(QMeshEdgeNode *)(edge->GetAttachedList().GetNext(Pos2));
						tt=(double)(i+1)/(double)(refinedNodeNum+1);
						pp[0]=(1.0-tt)*sp[0]+tt*ep[0];
						pp[1]=(1.0-tt)*sp[1]+tt*ep[1];
						pp[2]=(1.0-tt)*sp[2]+tt*ep[2];
						weight=heapNode->GetValue()+geo.Distance_to_Point(cp,pp);

						GLKHeapNode *heapNode2=(GLKHeapNode *)(linkedAttachedNode->heapNodePtr);
						if (heapNode2==NULL) continue;
						if (heapNode2->GetValue()>weight) {heapNode2->SetValue(weight); heap->AdjustPosition(heapNode2);}
					}
				}
			}

			node->SetBoundaryDis(heapNode->GetValue());
		}
		attachedNode->locOnEdge=heapNode->GetValue();
		attachedNode->heapNodePtr=NULL;		delete heapNode;	
	}

	//--------------------------------------------------------------------------------------------
	//	Step 3: Check the range of the distances
	double maxValue=0.0;	QMeshNode *maxValueNode=NULL;
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		if (node->GetBoundaryDis()>maxValue) {maxValue=node->GetBoundaryDis(); maxValueNode=node;}
	}
	if (maxValueNode!=NULL) maxValueNode->SetAttribFlag(4);

	//--------------------------------------------------------------------------------------------
	//	Step 4: Free the memory
	delete heap;
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		attachedNode=(QMeshEdgeNode *)(node->attachedPointer);	if (attachedNode!=NULL) delete attachedNode;
		node->attachedPointer=NULL;
	}
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
		for(Pos2=edge->GetAttachedList().GetHeadPosition();Pos2!=NULL;) {
			attachedNode=(QMeshEdgeNode *)(edge->GetAttachedList().GetNext(Pos2));
			delete attachedNode;
		}
		edge->GetAttachedList().RemoveAll();	
	}

	return maxValue;
}

void QBndSpiral::GenerateHarmonicMap(QMeshPatch *mesh, bool bCot)
{
	GLKPOSITION Pos;	GLKPOSITION Pos2;
	QMeshNode *node;
	QMeshEdge *edge;
	QMeshFace *face;
	GLKArray *pLpcs;
	int i,edgeNum;		const bool bWithOneRingAnchor=false;

	//--------------------------------------------------------------------------------------------
	//	Step 1: preparation
	if (bCot) _attachCotLaplacianWeight(mesh);
	//--------------------------------------------------------------------------------------------
	i=1;
	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;i++) {
		node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		node->SetAttribFlag(0,false);	// boundary flag
		node->SetAttribFlag(7,false);	// flag for anchor points
		node->SetIndexNo(i);
		if (node->GetAttribFlag(4)) node->SetAttribFlag(7,true);	// anchor points
	}
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
		edge->SetAttribFlag(0,false);
		edge->SetAttribFlag(7,false);
		if (edge->GetLeftFace()==NULL || edge->GetRightFace()==NULL) {
			edge->SetAttribFlag(0,true); 
			edge->GetStartPoint()->SetAttribFlag(0,true);		edge->GetEndPoint()->SetAttribFlag(0,true);
			edge->GetStartPoint()->SetAttribFlag(7,true);		edge->GetEndPoint()->SetAttribFlag(7,true);
			edge->GetStartPoint()->SetBoundaryDis(0.0);			edge->GetEndPoint()->SetBoundaryDis(0.0);
		}
		else if (edge->GetLeftFace()->m_nIdentifiedPatchIndex!=edge->GetRightFace()->m_nIdentifiedPatchIndex) {
			edge->SetAttribFlag(7,true); 
			edge->GetStartPoint()->SetAttribFlag(7,true);		edge->GetEndPoint()->SetAttribFlag(7,true);
			edge->GetStartPoint()->SetBoundaryDis(0.0);			edge->GetEndPoint()->SetBoundaryDis(0.0);
		}
	}
	//--------------------------------------------------------------------------------------------
	if (bWithOneRingAnchor) {
		printf("One more ring of vertices adjacent to anchor points is also enforced in the Harmonic field computation.\n");
		GLKObList seedNodeList;
		for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
			node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
//			if (node->GetAttribFlag(4) || node->GetAttribFlag(0)) 
			if (node->GetAttribFlag(4))	seedNodeList.AddTail(node);
		}
		for(Pos=seedNodeList.GetHeadPosition();Pos!=NULL;) {
			node=(QMeshNode *)(seedNodeList.GetNext(Pos));
			for(Pos2=node->GetEdgeList().GetHeadPosition();Pos2!=NULL;) {
				edge=(QMeshEdge *)(node->GetEdgeList().GetNext(Pos2));
				QMeshNode *linkedNode=(edge->GetStartPoint()==node)?(edge->GetEndPoint()):(edge->GetStartPoint());
				linkedNode->SetAttribFlag(7);
				//linkedNode->SetBoundaryDis(node->GetBoundaryDis());
				linkedNode->SetAttribFlag(4);
			}			
		}
		for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
			edge=(QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
			if (!(edge->GetAttribFlag(0))) continue;
			QMeshFace *face;
			if (edge->GetLeftFace()==NULL) face=edge->GetRightFace(); else face=edge->GetLeftFace();
			face->GetNodeRecordPtr(0)->SetAttribFlag(7);
			face->GetNodeRecordPtr(1)->SetAttribFlag(7);
			face->GetNodeRecordPtr(2)->SetAttribFlag(7);
			face->GetNodeRecordPtr(0)->SetAttribFlag(4);
			face->GetNodeRecordPtr(1)->SetAttribFlag(4);
			face->GetNodeRecordPtr(2)->SetAttribFlag(4);
		}
	}
	//--------------------------------------------------------------------------------------------
	int iRow = mesh->GetNodeNumber();
	double *x = new double[iRow], *b = new double[iRow];
	CSpMatrix matA(iRow, iRow);

	//--------------------------------------------------------------------------------------------
	//	Step 2: specify the matrix coefficients for the Laplacian equation
	if (bCot) {
		iRow=0;
		for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;iRow++) {
			node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
			pLpcs=(GLKArray*)(node->attachedPointer);
			i=0; b[iRow]=0.0;
			edgeNum=node->GetEdgeNumber();	b[iRow]=0.0;
			for(Pos2=node->GetEdgeList().GetHeadPosition();Pos2!=NULL;i++) {
				edge=(QMeshEdge *)(node->GetEdgeList().GetNext(Pos2));
				QMeshNode *linkedNode=(edge->GetStartPoint()==node)?(edge->GetEndPoint()):(edge->GetStartPoint());
				if (linkedNode->GetAttribFlag(7)) {	// anchor points
					b[iRow] -= linkedNode->GetBoundaryDis()*pLpcs->GetDoubleAt(i);
				}
				else {
					matA.Set(iRow, linkedNode->GetIndexNo()-1, pLpcs->GetDoubleAt(i));
				}
			}
			matA.Set(iRow, node->GetIndexNo()-1, pLpcs->GetDoubleAt(node->GetEdgeNumber()) );
		}
	}
	else {
		iRow=0;
		for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;iRow++) {
			node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
			edgeNum=node->GetEdgeNumber();	b[iRow]=0.0;
			for(Pos2=node->GetEdgeList().GetHeadPosition();Pos2!=NULL;) {
				edge=(QMeshEdge *)(node->GetEdgeList().GetNext(Pos2));
				QMeshNode *linkedNode=(edge->GetStartPoint()==node)?(edge->GetEndPoint()):(edge->GetStartPoint());
				if (linkedNode->GetAttribFlag(7)) {	// anchor points
					b[iRow] += linkedNode->GetBoundaryDis()/(double)edgeNum;
				}
				else {
					matA.Set(iRow, linkedNode->GetIndexNo()-1, -1.0/(double)edgeNum);
				}
			}
			matA.Set(iRow, node->GetIndexNo()-1, 1.0);
		}
	}
//	CSpMatrix::MulATA(matA,b);
//	CSpMatrix::MulATA(matA,b);

	//--------------------------------------------------------------------------------------------
	//	Step 3: solving the Laplacian equation to determined the Harmonic field
	if (CSuperLU::Solve(matA, x, b)) {
		i=0;
		for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;i++) {
			node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
			if (node->GetAttribFlag(7)) continue;
			node->SetBoundaryDis(x[i]);
		}
	}

	//--------------------------------------------------------------------------------------------
	//	Step X: free the memory
	delete x;	delete b;
	if (bCot) _releaseCotLaplacianWeight(mesh);
}

QMeshPatch* QBndSpiral::GenerateSpiralCurves2(QMeshPatch *mesh, double maxValue, double offsetValue, int patchID, bool bWithSmoothing)
{
	int i,curveNum,j;
	const int pntsNum=72;		
	const bool bGeodesicDistBasedBlending=true;
	double isoValue,shiftIsoValue;
	GLKPOSITION Pos;
	GLKObList **isoNodeList,newNodeList,newEdgeList,newNodeList2,newEdgeList2;
	QMeshNode *startNode;		double closestPntToStart[3],curvePnt[3],tt;
	double *xPos,*yPos,*zPos;	QMeshPatch *newMesh;

	//--------------------------------------------------------------------------------------------
	//	Step 1: Initialization
	curveNum=(int)(maxValue/offsetValue);	shiftIsoValue=maxValue-offsetValue*(double)curveNum;
	newNodeList.RemoveAll();	newEdgeList.RemoveAll();
	isoNodeList=(GLKObList **)new long[curveNum+1];
	for(i=0;i<=curveNum;i++) isoNodeList[i]=new GLKObList;

	//--------------------------------------------------------------------------------------------
	//	Step 2: Generate the curves
	startNode=NULL;
	QMeshFace **faceArray=NULL;
//	for(i=0;i<=curveNum;i++) {
	for(i=curveNum/2;i<=curveNum;i++) {
		isoValue=offsetValue*(double)i+1.0e-5;//+shiftIsoValue*0.5;

		bool bRes=_generateIsocurve(mesh, isoValue, isoNodeList[i], patchID);
		if (!bRes) {
			printf("Warning: the %d-th isocurve has complex topology - isovalue = %f\n",i,(float)isoValue);
		}
		if (isoNodeList[i]->IsEmpty()) continue;

		if (startNode!=NULL) {
			startNode->GetCoord3D(closestPntToStart[0],closestPntToStart[1],closestPntToStart[2]);
			_reIndexNodeOnContourByStartPoint(isoNodeList[i],closestPntToStart);
		}
		startNode=(QMeshNode*)(isoNodeList[i]->GetHead());

		_resamplingContourCurve(isoNodeList[i],pntsNum,xPos,yPos,zPos,faceArray);
		for(j=0;j<pntsNum;j++) {
			tt=(double)j/(double)pntsNum;		//tt=0.0;
			
			double targetFieldValue=tt*offsetValue+isoValue;
			curvePnt[0]=xPos[j];	curvePnt[1]=yPos[j];	curvePnt[2]=zPos[j];

			//////////////////////////////////////////////////////////////////////////
			//	for Debug purpose
			QMeshNode *stNode=new QMeshNode;	stNode->SetMeshPatchPtr(mesh);
			stNode->SetCoord3D(curvePnt[0],curvePnt[1],curvePnt[2]);	
			stNode->SetAttribFlag(4); 			newNodeList2.AddTail(stNode);

			bool bNotTruncated;
			if (j!=0) bNotTruncated=_pushingPntToPlaceWithGivenBDMValue(targetFieldValue,curvePnt,faceArray[j]);

			QMeshNode *newNode=new QMeshNode;	newNode->SetMeshPatchPtr(mesh);
			newNode->SetCoord3D(curvePnt[0],curvePnt[1],curvePnt[2]);	
//			if (bNotTruncated) newNode->SetAttribFlag(4); else 
			newNode->SetAttribFlag(3);
			newNodeList.AddTail(newNode);

			//////////////////////////////////////////////////////////////////////////
			//	for Debug purpose
			QMeshEdge *edge=new QMeshEdge;		edge->SetMeshPatchPtr(mesh);
			edge->SetStartPoint(stNode);		edge->SetEndPoint(newNode);
			newEdgeList2.AddTail(edge);
		}
		free(xPos);	free(yPos);	free(zPos);
		if (faceArray!=NULL) free(faceArray);
		break;
	}
	if (!(newNodeList.IsEmpty())) {
		if (bWithSmoothing) _smoothingSpiralCurve(&newNodeList,0.5,5);
		_createEdgesByNodeList(&newNodeList,&newEdgeList,false);  // need also further improve this function by using geodesic curve
	}

	//--------------------------------------------------------------------------------------------
	//	Step 3: Finalize
	char filename[200];
	sprintf(filename,"Data//spiral%d.txt",patchID);
	if (!(newNodeList.IsEmpty())) _saveCurveFile(&newNodeList, filename);
	//--------------------------------------------------------------------------------------------
	for(i=0;i<=curveNum;i++) {
		for(Pos=isoNodeList[i]->GetHeadPosition();Pos!=NULL;) {
			startNode=(QMeshNode*)(isoNodeList[i]->GetNext(Pos));	delete startNode;
		}
		delete ((GLKObList*)(isoNodeList[i]));
	}
	delete []isoNodeList;
	//--------------------------------------------------------------------------------------------
	if (newNodeList.IsEmpty() && newEdgeList.IsEmpty()) {
		newMesh=NULL;
	}
	else {
		newMesh=new QMeshPatch;
		newMesh->GetNodeList().AttachListTail(&newNodeList);	newNodeList.RemoveAllWithoutFreeMemory();
		newMesh->GetEdgeList().AttachListTail(&newEdgeList);	newEdgeList.RemoveAllWithoutFreeMemory();

		//////////////////////////////////////////////////////////////////////////////////////////////////
		//	for Debug purpose
		newMesh->GetNodeList().AttachListTail(&newNodeList2);	newNodeList2.RemoveAllWithoutFreeMemory();
		newMesh->GetEdgeList().AttachListTail(&newEdgeList2);	newEdgeList2.RemoveAllWithoutFreeMemory();
	}

	return newMesh;
}

QMeshPatch* QBndSpiral::GenerateSpiralCurves(QMeshPatch *mesh, double maxValue, double offsetValue, int patchID, bool bWithSmoothing)
{
	int i,curveNum,j;
	const int pntsNum=72;		
	const bool bGeodesicDistBasedBlending=true;
	double isoValue,shiftIsoValue;
	GLKPOSITION Pos;
	GLKObList **isoNodeList,newNodeList,newEdgeList;
	QMeshNode *startNode;		double closestPntToStart[3],curvePnt[3],tt;
	double *xPos,*yPos,*zPos,*lastXPos,*lastYPos,*lastZPos;
	QMeshPatch *newMesh;

	//--------------------------------------------------------------------------------------------
	//	Step 1: Initialization
	pqpSnappingBOOL=true;
	if (pqpSnappingBOOL) _pqpInitialization(mesh);
	//--------------------------------------------------------------------------------------------
	curveNum=(int)(maxValue/offsetValue);	shiftIsoValue=maxValue-offsetValue*(double)curveNum;
	newNodeList.RemoveAll();	newEdgeList.RemoveAll();
	isoNodeList=(GLKObList **)new long[curveNum+1];
	for(i=0;i<=curveNum;i++) isoNodeList[i]=new GLKObList;
	lastXPos=lastYPos=lastZPos=NULL;
	//--------------------------------------------------------------------------------------------
	QDiscreteShortestPath::Initialization(mesh);

	//--------------------------------------------------------------------------------------------
	//	Step 2: Generate the curves
	startNode=NULL;
	QMeshFace **faceArray=NULL,**lastFaceArray=NULL;
	for(i=0;i<=curveNum;i++) {
		isoValue=offsetValue*(double)i+shiftIsoValue*0.5+1.0e-5;

		bool bRes=_generateIsocurve(mesh, isoValue, isoNodeList[i], patchID);
		if (!bRes) {
			printf("Warning: the %d-th isocurve has complex topology - isovalue = %f\n",i,(float)isoValue);
		}
		if (isoNodeList[i]->IsEmpty()) continue;

		if (startNode!=NULL) {
			startNode->GetCoord3D(closestPntToStart[0],closestPntToStart[1],closestPntToStart[2]);
			_reIndexNodeOnContourByStartPoint(isoNodeList[i],closestPntToStart);
		}
		startNode=(QMeshNode*)(isoNodeList[i]->GetHead());

		lastFaceArray=faceArray;	_resamplingContourCurve(isoNodeList[i],pntsNum,xPos,yPos,zPos,faceArray);
		if (i>0) {	
			for(j=0;j<pntsNum;j++) {
				tt=(double)j/(double)pntsNum;		//tt=0.0;

				QMeshFace *pntFace=NULL;		double queryPnt[3],dist;
				//-------------------------------------------------------------------------------------------------------------------
				// PQP-projection-based blending
				if (pqpSnappingBOOL && (!bGeodesicDistBasedBlending)) {
					queryPnt[0]=(1.0-tt)*lastXPos[j]+tt*xPos[j];
					queryPnt[1]=(1.0-tt)*lastYPos[j]+tt*yPos[j];
					queryPnt[2]=(1.0-tt)*lastZPos[j]+tt*zPos[j];
					pntFace=_pqpClosestPntQuery(queryPnt,curvePnt,dist);
				}
				//-------------------------------------------------------------------------------------------------------------------
				// Geodesic-distance-based blending, which has better performance than the Euclidean-distance-based blending
				if (bGeodesicDistBasedBlending) {
					double startPnt[3],endPnt[3];
					startPnt[0]=lastXPos[j];	startPnt[1]=lastYPos[j];	startPnt[2]=lastZPos[j];
					endPnt[0]=xPos[j];		endPnt[1]=yPos[j];		endPnt[2]=zPos[j];
					QDiscreteShortestPath::ComputePntOnPath(mesh,startPnt,lastFaceArray[j],endPnt,faceArray[j],tt,queryPnt);
					pntFace=_pqpClosestPntQuery(queryPnt,curvePnt,dist);
				}

				QMeshNode *newNode=new QMeshNode;	newNode->SetMeshPatchPtr(mesh);
				newNode->SetCoord3D(curvePnt[0],curvePnt[1],curvePnt[2]);	newNode->SetAttribFlag(4);
				newNodeList.AddTail(newNode);		newNode->attachedPointer=pntFace;
			}
			free(lastXPos);	free(lastYPos);	free(lastZPos);	
		}
		lastXPos=xPos;	lastYPos=yPos;	lastZPos=zPos;
		if (lastFaceArray!=NULL) free(lastFaceArray);
	}
	if (faceArray!=NULL) free(faceArray);
	if (lastXPos!=NULL) free(lastXPos);	
	if (lastYPos!=NULL) free(lastYPos);	
	if (lastZPos!=NULL) free(lastZPos);
	if (!(newNodeList.IsEmpty())) {
		if (bWithSmoothing) _smoothingSpiralCurve(&newNodeList,0.25,5);
		_createEdgesByNodeList(&newNodeList,&newEdgeList,false);  // need also further improve this function by using geodesic curve
	}

	//--------------------------------------------------------------------------------------------
	//	Step 3: Finalize
	char filename[200];
	sprintf(filename,"Data//spiral%d.txt",patchID);
	if (!(newNodeList.IsEmpty())) _saveCurveFile(&newNodeList, filename);
	//--------------------------------------------------------------------------------------------
	for(i=0;i<=curveNum;i++) {
		for(Pos=isoNodeList[i]->GetHeadPosition();Pos!=NULL;) {
			startNode=(QMeshNode*)(isoNodeList[i]->GetNext(Pos));	delete startNode;
		}
		delete ((GLKObList*)(isoNodeList[i]));
	}
	delete []isoNodeList;
	//--------------------------------------------------------------------------------------------
	if (newNodeList.IsEmpty() && newEdgeList.IsEmpty()) {
		newMesh=NULL;
	}
	else {
		newMesh=new QMeshPatch;
		newMesh->GetNodeList().AttachListTail(&newNodeList);	newNodeList.RemoveAllWithoutFreeMemory();
		newMesh->GetEdgeList().AttachListTail(&newEdgeList);	newEdgeList.RemoveAllWithoutFreeMemory();
	}
	//--------------------------------------------------------------------------------------------
	if (pqpSnappingBOOL) _pqpMemoryRelease();

	return newMesh;
}

QMeshPatch* QBndSpiral::GenerateIsocurves(QMeshPatch *mesh, double maxValue, double offsetValue, int patchID)
{
	int i,j,curveNum,edgeNum,num;		
	double isoValue,shiftIsoValue,value,alpha,sx,sy,sz,ex,ey,ez,xx,yy,zz;
	GLKPOSITION Pos;
	QMeshEdge *edge,*newEdge;
	QMeshNode *newNode,*contourNodes[2];
	QMeshFace *face;
	GLKObList newNodeList,newEdgeList;
	QMeshPatch *newMesh=NULL;

	//--------------------------------------------------------------------------------------------
	//	Step 1: Initialization
	curveNum=(int)(maxValue/offsetValue);	shiftIsoValue=maxValue-offsetValue*(double)curveNum;
	newNodeList.RemoveAll();	newEdgeList.RemoveAll();
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos));
		edge->SetAttribFlag(7,false);	// used to specify if the edge is on the specified patch
	}
	for(Pos=mesh->GetFaceList().GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace*)(mesh->GetFaceList().GetNext(Pos));
		if (face->m_nIdentifiedPatchIndex!=patchID) continue;
		edgeNum=face->GetEdgeNum();
		for(j=0;j<edgeNum;j++) face->GetEdgeRecordPtr(j)->SetAttribFlag(7);
	}

	for(i=0;i<=curveNum;i++) {
		isoValue=offsetValue*(double)i+shiftIsoValue*0.5+1.0e-5;

		for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
			edge=(QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos));
			edge->attachedPointer=NULL;
		}

		//--------------------------------------------------------------------------------------------
		//	Step 2: create nodes on the edges
		for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
			edge=(QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos));
			if (!(edge->GetAttribFlag(7))) continue;

			if (edge->GetEndPoint()->GetBoundaryDis()==edge->GetStartPoint()->GetBoundaryDis()) continue;
			value=(edge->GetStartPoint()->GetBoundaryDis()-isoValue)*(edge->GetEndPoint()->GetBoundaryDis()-isoValue);
			if (value<=0.0) {
				alpha=fabs(isoValue-edge->GetStartPoint()->GetBoundaryDis())
					/fabs(edge->GetEndPoint()->GetBoundaryDis()-edge->GetStartPoint()->GetBoundaryDis());
				edge->GetStartPoint()->GetCoord3D(sx,sy,sz);
				edge->GetEndPoint()->GetCoord3D(ex,ey,ez);
				xx=(1.0-alpha)*sx+alpha*ex;
				yy=(1.0-alpha)*sy+alpha*ey;
				zz=(1.0-alpha)*sz+alpha*ez;

				newNode=new QMeshNode;	newNode->SetMeshPatchPtr(mesh);
				newNode->SetCoord3D(xx,yy,zz);
				newNodeList.AddTail(newNode);		newNode->SetAttribFlag(0);
				newNode->SetIndexNo(mesh->GetNodeNumber()+newNodeList.GetCount());
				newNode->attachedPointer=edge;

				edge->attachedPointer=newNode;
			}
		}

		//--------------------------------------------------------------------------------------------
		//	Step 3: create edges on the faces
		for(Pos=mesh->GetFaceList().GetHeadPosition();Pos!=NULL;) {
			face=(QMeshFace*)(mesh->GetFaceList().GetNext(Pos));
			if (face->m_nIdentifiedPatchIndex!=patchID) continue;

			edgeNum=face->GetEdgeNum();		num=0;
			for(j=0;j<edgeNum;j++) {
				edge=face->GetEdgeRecordPtr(j);
				if (edge->attachedPointer!=NULL) {contourNodes[num]=(QMeshNode*)(edge->attachedPointer); num++;}
				if (num==2) break;
			}
			if (num==2) {
				newEdge=new QMeshEdge;	newEdge->SetMeshPatchPtr(mesh);
				newEdge->SetStartPoint(contourNodes[0]);	newEdge->SetEndPoint(contourNodes[1]);
				newEdgeList.AddTail(newEdge);				newEdge->SetAttribFlag(0);
				newEdge->SetIndexNo(mesh->GetEdgeNumber()+newEdgeList.GetCount());
				newEdge->attachedPointer=face;
			}
		}
	}

	//--------------------------------------------------------------------------------------------
	//	Step 4: Finalize
	if (newNodeList.IsEmpty() && newEdgeList.IsEmpty()) return NULL;
	newMesh=new QMeshPatch;
	newMesh->GetNodeList().AttachListTail(&newNodeList);	newNodeList.RemoveAllWithoutFreeMemory();
	newMesh->GetEdgeList().AttachListTail(&newEdgeList);	newEdgeList.RemoveAllWithoutFreeMemory();

	return newMesh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	The private functions
//
bool QBndSpiral::_generateIsocurve(QMeshPatch *mesh, double isoValue, GLKObList *contourNodeList, int patchID)
{
	GLKPOSITION Pos;
	QMeshEdge *edge,*faceEdge;	QMeshFace *face;
	QMeshNode *newNode,*currentNode;
	double value,xx,yy,zz,sx,sy,sz,ex,ey,ez,alpha;
	int i,j,edgeNum;
	GLKObList newNodeList;		bool bAllSliced=false;

	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos));
		edge->SetAttribFlag(7,false);
	}
	for(Pos=mesh->GetFaceList().GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace*)(mesh->GetFaceList().GetNext(Pos));
		if (face->m_nIdentifiedPatchIndex!=patchID) continue;
		edgeNum=face->GetEdgeNum();
		for(j=0;j<edgeNum;j++) face->GetEdgeRecordPtr(j)->SetAttribFlag(7);
	}

	//--------------------------------------------------------------------------------------------
	//	Step 1: create nodes on the edges
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos));
		edge->attachedPointer=NULL;
		if (!(edge->GetAttribFlag(7))) continue;
		if (edge->GetEndPoint()->GetBoundaryDis()==edge->GetStartPoint()->GetBoundaryDis()) continue;

		value=(edge->GetStartPoint()->GetBoundaryDis()-isoValue)*(edge->GetEndPoint()->GetBoundaryDis()-isoValue);
		if (value<=0.0) {
			alpha=fabs(isoValue-edge->GetStartPoint()->GetBoundaryDis())
				/fabs(edge->GetEndPoint()->GetBoundaryDis()-edge->GetStartPoint()->GetBoundaryDis());
			edge->GetStartPoint()->GetCoord3D(sx,sy,sz);
			edge->GetEndPoint()->GetCoord3D(ex,ey,ez);
			xx=(1.0-alpha)*sx+alpha*ex;
			yy=(1.0-alpha)*sy+alpha*ey;
			zz=(1.0-alpha)*sz+alpha*ez;

			newNode=new QMeshNode;	newNode->SetMeshPatchPtr(mesh);
			newNode->SetCoord3D(xx,yy,zz);
			newNodeList.AddTail(newNode);		newNode->SetAttribFlag(4);
			newNode->SetIndexNo(mesh->GetNodeNumber()+newNodeList.GetCount());
			newNode->attachedPointer=edge;		newNode->SetAttribFlag(7,false);

			edge->attachedPointer=newNode;
		}
	}
	if (newNodeList.IsEmpty()) return false;

	//--------------------------------------------------------------------------------------------
	//	Step 2: create the first edge - the direction is very important for the direction of isocurve
	contourNodeList->RemoveAll();	
	currentNode=(QMeshNode *)(newNodeList.GetHead());
	edge=(QMeshEdge*)(currentNode->attachedPointer);	face=edge->GetLeftFace(); 
	edgeNum=face->GetEdgeNum();		int currentIndex;
	for(j=0;j<edgeNum;j++) {
		faceEdge=face->GetEdgeRecordPtr(j);
		if (faceEdge==edge) currentIndex=j;
		if (faceEdge->attachedPointer==NULL) continue;
		newNode=(QMeshNode*)(faceEdge->attachedPointer);
		if (newNode!=currentNode) break;
	}
	if (face->GetNodeRecordPtr((currentIndex+1)%edgeNum)->GetBoundaryDis()<isoValue) {
		//	"currentNode => newNode" gives the anti-clockwise diretion of the loop
		contourNodeList->AddTail(currentNode);			currentNode->SetAttribFlag(7,true);
		edge=(QMeshEdge*)(currentNode->attachedPointer);	currentNode->GetFaceList().RemoveAll();
		if (edge->GetLeftFace()==face)
			{	if (edge->GetRightFace()!=NULL) currentNode->GetFaceList().AddTail(edge->GetRightFace());	}
		else
			{	if (edge->GetLeftFace()!=NULL) currentNode->GetFaceList().AddTail(edge->GetLeftFace());	}
		contourNodeList->AddTail(newNode);				newNode->SetAttribFlag(7,true);
		currentNode=newNode;
		currentNode->GetFaceList().RemoveAll();			currentNode->GetFaceList().AddTail(face);	// the face containing "currentNode-newNode"
	}
	else {
		//	"newNode => currentNode" gives the anti-clockwise diretion of the loop
		contourNodeList->AddTail(newNode);				newNode->SetAttribFlag(7,true);
		edge=(QMeshEdge*)(newNode->attachedPointer);	newNode->GetFaceList().RemoveAll();
		if (edge->GetLeftFace()==face)
			{	if (edge->GetRightFace()!=NULL) newNode->GetFaceList().AddTail(edge->GetRightFace());	}
		else
			{	if (edge->GetLeftFace()!=NULL) newNode->GetFaceList().AddTail(edge->GetLeftFace());	}
		contourNodeList->AddTail(currentNode);			currentNode->SetAttribFlag(7,true);
		currentNode->GetFaceList().RemoveAll();			currentNode->GetFaceList().AddTail(face);	// the face containing "newNode-currentNode"
	}

	//--------------------------------------------------------------------------------------------
	//	Step 3: slicing the nodes
	do{
		edge=(QMeshEdge*)(currentNode->attachedPointer);
		currentNode=NULL;

		for(i=0;i<2;i++) {
			if (i==0) 
				face=edge->GetLeftFace(); 
			else 
				face=edge->GetRightFace();
			if (face==NULL) continue;
			
			edgeNum=face->GetEdgeNum();
			for(j=0;j<edgeNum;j++) {
				faceEdge=face->GetEdgeRecordPtr(j);
				if (faceEdge->attachedPointer==NULL) continue;
				newNode=(QMeshNode*)(faceEdge->attachedPointer);
				if (!(newNode->GetAttribFlag(7))) {
					currentNode=newNode;	
					contourNodeList->AddTail(currentNode);
					currentNode->SetAttribFlag(7);
					currentNode->GetFaceList().RemoveAll();		currentNode->GetFaceList().AddTail(face);
					break;
				}
			}
			if (currentNode!=NULL) break;
		}
	}while(currentNode!=NULL);

	//--------------------------------------------------------------------------------------------
	//	Step 3: finalize
	bAllSliced=true;
	for(Pos=newNodeList.GetHeadPosition();Pos!=NULL;) {
		QMeshNode *node=(QMeshNode *)(newNodeList.GetNext(Pos));
		((QMeshEdge*)(node->attachedPointer))->attachedPointer=NULL;
		if (!(node->GetAttribFlag(7))) {bAllSliced=false;	delete node;} else {node->SetAttribFlag(7,false);}
	}
	for(Pos=mesh->GetEdgeList().GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos));
		edge->SetAttribFlag(7,false);
	}

	
	for(Pos=contourNodeList->GetHeadPosition();Pos!=NULL;) {
		QMeshNode *node=(QMeshNode *)(contourNodeList->GetNext(Pos));
		if (node->GetFaceList().GetCount()!=1) printf("ERROR!");
	}

	return bAllSliced;
}

void QBndSpiral::_reIndexNodeOnContourByStartPoint(GLKObList *contourNodeList, double closestPntToStart[])
{
	GLKPOSITION Pos;	QMeshNode *node;
	int i,minIndex;		double xx,yy,zz,dd,dMin;
	GLKObList beforeList,afterList;

	if (contourNodeList->IsEmpty()) return;

	i=0;	dMin=1.0e+8;
	for(Pos=contourNodeList->GetHeadPosition();Pos!=NULL;i++) {
		node=(QMeshNode*)(contourNodeList->GetNext(Pos));
		node->GetCoord3D(xx,yy,zz);
		dd=(xx-closestPntToStart[0])*(xx-closestPntToStart[0])
			+(yy-closestPntToStart[1])*(yy-closestPntToStart[1])
			+(zz-closestPntToStart[2])*(zz-closestPntToStart[2]);
		if (dd<dMin) {dMin=dd;	minIndex=i;}
	}

	i=0;	
	for(Pos=contourNodeList->GetHeadPosition();Pos!=NULL;i++) {
		node=(QMeshNode*)(contourNodeList->GetNext(Pos));
		if (i<minIndex) beforeList.AddTail(node); else afterList.AddTail(node);
	}

	contourNodeList->RemoveAll();	
	contourNodeList->AttachListTail(&afterList);	afterList.RemoveAllWithoutFreeMemory();
	contourNodeList->AttachListTail(&beforeList);	beforeList.RemoveAllWithoutFreeMemory();
}

void QBndSpiral::_createEdgesByNodeList(GLKObList *nodeList, GLKObList *edgeList, bool bPeriodic)
{
	GLKPOSITION Pos;
	QMeshNode *currentNode,*nextNode;		QMeshEdge *newEdge;
	int i,nodeNum;

	if (nodeList->IsEmpty()) return;

	edgeList->RemoveAll();

	Pos=nodeList->GetHeadPosition();	nodeNum=nodeList->GetCount();
	if (!bPeriodic) nodeNum--;
	currentNode=(QMeshNode*)(nodeList->GetNext(Pos));
	for(i=0;i<nodeNum;i++) {
		if (Pos!=NULL)
			nextNode=(QMeshNode*)(nodeList->GetNext(Pos));
		else 
			nextNode=(QMeshNode*)(nodeList->GetHead());
		newEdge=new QMeshEdge;
		newEdge->SetStartPoint(currentNode);	newEdge->SetEndPoint(nextNode);
		edgeList->AddTail(newEdge);				newEdge->SetAttribFlag(0);

		currentNode=nextNode;
	}
}

void QBndSpiral::_resamplingContourCurve(GLKObList *contourNodeList, int pntsNum, double* &xPos, double* &yPos, double* &zPos)
{
	int i,edgeNum,pIndex;
	double *edgeLength;
	GLKPOSITION Pos;	QMeshNode **nodeArray;
	double sx,sy,sz,ex,ey,ez,passedLength,totalLength,targetLength,alpha;

	edgeNum=contourNodeList->GetCount();
	nodeArray=(QMeshNode**)new long[edgeNum+1];
	i=0;
	for(Pos=contourNodeList->GetHeadPosition();Pos!=NULL;i++) {nodeArray[i]=(QMeshNode *)(contourNodeList->GetNext(Pos)); }
	nodeArray[edgeNum]=(QMeshNode *)(contourNodeList->GetHead());

	totalLength=0.0;
	edgeLength=(double *)malloc(edgeNum*sizeof(double));	
	for(i=0;i<edgeNum;i++) {
		nodeArray[i]->GetCoord3D(sx,sy,sz);
		nodeArray[i+1]->GetCoord3D(ex,ey,ez);
		edgeLength[i]=sqrt((ex-sx)*(ex-sx)+(ey-sy)*(ey-sy)+(ez-sz)*(ez-sz));
		totalLength += edgeLength[i];
	}

	xPos=(double *)malloc(pntsNum*sizeof(double));
	yPos=(double *)malloc(pntsNum*sizeof(double));
	zPos=(double *)malloc(pntsNum*sizeof(double));
	nodeArray[0]->GetCoord3D(xPos[0],yPos[0],zPos[0]);		passedLength=0.0;	pIndex=0;
	for(i=1;i<pntsNum;i++) {
		targetLength=(double)i*totalLength/(double)pntsNum;
		while(passedLength+edgeLength[pIndex]<targetLength){passedLength+=edgeLength[pIndex];	pIndex++;}
		alpha=(targetLength-passedLength)/edgeLength[pIndex];
		nodeArray[pIndex]->GetCoord3D(sx,sy,sz);
		nodeArray[pIndex+1]->GetCoord3D(ex,ey,ez);
		xPos[i]=(1.0-alpha)*sx+alpha*ex;
		yPos[i]=(1.0-alpha)*sy+alpha*ey;
		zPos[i]=(1.0-alpha)*sz+alpha*ez;
	}

	free(edgeLength);	delete []nodeArray;
}

void QBndSpiral::_resamplingContourCurve(GLKObList *contourNodeList, int pntsNum, double* &xPos, double* &yPos, double* &zPos, QMeshFace **&faceArray)
{
	int i,edgeNum,pIndex;
	double *edgeLength;
	GLKPOSITION Pos;	QMeshNode **nodeArray;
	double sx,sy,sz,ex,ey,ez,passedLength,totalLength,targetLength,alpha;

	edgeNum=contourNodeList->GetCount();
	nodeArray=(QMeshNode**)new long[edgeNum+1];
	i=0;
	for(Pos=contourNodeList->GetHeadPosition();Pos!=NULL;i++) {nodeArray[i]=(QMeshNode *)(contourNodeList->GetNext(Pos)); }
	nodeArray[edgeNum]=(QMeshNode *)(contourNodeList->GetHead());

	totalLength=0.0;
	edgeLength=(double *)malloc(edgeNum*sizeof(double));	
	for(i=0;i<edgeNum;i++) {
		nodeArray[i]->GetCoord3D(sx,sy,sz);
		nodeArray[i+1]->GetCoord3D(ex,ey,ez);
		edgeLength[i]=sqrt((ex-sx)*(ex-sx)+(ey-sy)*(ey-sy)+(ez-sz)*(ez-sz));
		totalLength += edgeLength[i];
	}

	xPos=(double *)malloc(pntsNum*sizeof(double));
	yPos=(double *)malloc(pntsNum*sizeof(double));
	zPos=(double *)malloc(pntsNum*sizeof(double));
	faceArray=(QMeshFace **)malloc(pntsNum*sizeof(long));
	nodeArray[0]->GetCoord3D(xPos[0],yPos[0],zPos[0]);		passedLength=0.0;	pIndex=0;
	faceArray[0]=(QMeshFace *)(nodeArray[0]->GetFaceList().GetHead());
	for(i=1;i<pntsNum;i++) {
		targetLength=(double)i*totalLength/(double)pntsNum;
		while(passedLength+edgeLength[pIndex]<targetLength){passedLength+=edgeLength[pIndex];	pIndex++;}
		alpha=(targetLength-passedLength)/edgeLength[pIndex];
		nodeArray[pIndex]->GetCoord3D(sx,sy,sz);
		nodeArray[pIndex+1]->GetCoord3D(ex,ey,ez);
		xPos[i]=(1.0-alpha)*sx+alpha*ex;
		yPos[i]=(1.0-alpha)*sy+alpha*ey;
		zPos[i]=(1.0-alpha)*sz+alpha*ez;
		faceArray[i]=(QMeshFace *)(nodeArray[pIndex+1]->GetFaceList().GetHead());
	}

	free(edgeLength);	delete []nodeArray;
}

void QBndSpiral::_smoothingSpiralCurve(GLKObList *contourNodeList, double relaxationFactor, int iterNum)
{
	int i;
	GLKPOSITION Pos;	
	QMeshNode *lastLastNode,*lastNode,*currentNode;
	double p1[3],pp[3],p2[3];

	for(i=0;i<iterNum;i++) {
		lastLastNode=lastNode=currentNode=NULL;
		for(Pos=contourNodeList->GetHeadPosition();Pos!=NULL;) {
			currentNode=(QMeshNode*)(contourNodeList->GetNext(Pos));
			if (lastLastNode!=NULL && lastNode!=NULL) {
				lastLastNode->GetCoord3D(p1[0],p1[1],p1[2]);
				lastNode->GetCoord3D(pp[0],pp[1],pp[2]);
				currentNode->GetCoord3D(p2[0],p2[1],p2[2]);

				pp[0]=(1.0-relaxationFactor)*pp[0]+relaxationFactor*0.5*(p1[0]+p2[0]);
				pp[1]=(1.0-relaxationFactor)*pp[1]+relaxationFactor*0.5*(p1[1]+p2[1]);
				pp[2]=(1.0-relaxationFactor)*pp[2]+relaxationFactor*0.5*(p1[2]+p2[2]);

				if (pqpSnappingBOOL) {
					double closestPnt[3],dist;
					_pqpClosestPntQuery(pp,closestPnt,dist);
					pp[0]=closestPnt[0];	pp[1]=closestPnt[1];	pp[2]=closestPnt[2];
				}

				lastNode->SetCoord3D(pp[0],pp[1],pp[2]);
			}
			lastLastNode=lastNode;	lastNode=currentNode;
		}
	}
}

bool QBndSpiral::_pushingPntToPlaceWithGivenBDMValue(double targetF, double inputPnt[], QMeshFace *currentFace)
{
	double pos[3],verPos[3][3],alpha,beta,fa,fb,fc,dt,dfa,dfb;

	currentFace->GetNodePos(0,verPos[0][0],verPos[0][1],verPos[0][2]);
	currentFace->GetNodePos(1,verPos[1][0],verPos[1][1],verPos[1][2]);
	currentFace->GetNodePos(2,verPos[2][0],verPos[2][1],verPos[2][2]);
	_3DCoordToBarycentricCoord(inputPnt,verPos[0],verPos[1],verPos[2],alpha,beta);

	fa=currentFace->GetNodeRecordPtr(0)->GetBoundaryDis();
	fb=currentFace->GetNodeRecordPtr(1)->GetBoundaryDis();
	fc=currentFace->GetNodeRecordPtr(2)->GetBoundaryDis();

	dfa=fc-fa;		dfb=fc-fb;
	if (fabs(dfa)<1.0e-5 && fabs(dfb)<1.0e-5) return false;
	dt=((alpha*fa+beta*fb+(1.0-alpha-beta)*fc)-targetF)/(dfa*dfa+dfb*dfb);
	if (dt<0.0) {dfa=-dfa; dfb=-dfb; dt=-dt;}

	short iTruncateFlag=-1;		double tAB,tBC,tCA,tmin;		tAB=tBC=tCA=1.0e+5;
	if (dfa<0.0) tBC=-alpha/dfa;
	if (dfb<0.0) tCA=-beta/dfb;
	if (dfa+dfb>0.0) tAB=(1.0-alpha-beta)/(dfa+dfb);
	tmin=dt;
	if (tAB<tmin) {tmin=tAB; iTruncateFlag=0;}
	if (tBC<tmin) {tmin=tBC; iTruncateFlag=1;}
	if (tCA<tmin) {tmin=tCA; iTruncateFlag=2;}

	dt=tmin;
	alpha=alpha+dfa*dt;		beta=beta+dfb*dt;

	if (alpha<0.0 || beta<0.0 || alpha>1.0 || beta>1.0 || alpha+beta>1.0) printf("ERROR ");
//	if (fabs((alpha*fa+beta*fb+(1.0-alpha-beta)*fc)-targetF)>1.0e-8) printf("ERROR ");

	_barycentricCoordTo3DCoord(alpha,beta,verPos[0],verPos[1],verPos[2],pos);

	inputPnt[0]=pos[0];
	inputPnt[1]=pos[1];
	inputPnt[2]=pos[2];
	if (iTruncateFlag>=0) return false;

	return true;
}

void QBndSpiral::_saveCurveFile(GLKObList *contourNodeList, char *filename)
{
	FILE *fp;
	GLKPOSITION Pos;
	QMeshNode *node;	double xx,yy,zz;

	fp = fopen(filename, "w");
	fprintf(fp,"Part.InsertCurveFileBegin\n");

	for(Pos=contourNodeList->GetHeadPosition();Pos!=NULL;) {
		node=(QMeshNode*)(contourNodeList->GetNext(Pos));
		node->GetCoord3D(xx,yy,zz);
		fprintf(fp,"boolstatus = Part.InsertCurveFilePoint(%f, %f, %f)\n",(float)xx*0.001f,(float)yy*0.001f,(float)zz*0.001f);
//		fprintf(fp,"boolstatus = Part.InsertCurveFilePoint(%f, %f, %f)\n",(float)xx,(float)yy,(float)zz);
	}

	fprintf(fp,"boolstatus = Part.InsertCurveFileEnd()\n");
	fclose(fp);
}

void QBndSpiral::_pqpInitialization(QMeshPatch *mesh)
{
	PQP_REAL p1[3], p2[3], p3[3];
	GLKPOSITION Pos;	QMeshFace *face;	int i;
	double xx,yy,zz;

	pqpModel = new PQP_Model();
	pqpFaceNum=mesh->GetFaceNumber();	if (pqpFaceNum>0) pqpFaceArray=(QMeshFace**)new long[pqpFaceNum];

	pqpModel->BeginModel();		i=0;
	for(Pos=mesh->GetFaceList().GetHeadPosition();Pos!=NULL;i++) {
		face=(QMeshFace*)(mesh->GetFaceList().GetNext(Pos));
		face->SetIndexNo(i);	
		pqpFaceArray[i]=face;

		face->GetNodePos(0,xx,yy,zz);	
		p1[0]=(PQP_REAL)(xx);	p1[1]=(PQP_REAL)(yy);	p1[2]=(PQP_REAL)(zz);
		face->GetNodePos(1,xx,yy,zz);	
		p2[0]=(PQP_REAL)(xx);	p2[1]=(PQP_REAL)(yy);	p2[2]=(PQP_REAL)(zz);
		face->GetNodePos(2,xx,yy,zz);	
		p3[0]=(PQP_REAL)(xx);	p3[1]=(PQP_REAL)(yy);	p3[2]=(PQP_REAL)(zz);

		pqpModel->AddTri(p1,p2,p3,i);
	}
	pqpModel->EndModel();
}

void QBndSpiral::_pqpMemoryRelease()
{
	if (pqpFaceNum>0) delete (QMeshFace**)pqpFaceArray;
	delete pqpModel; 
}

QMeshFace* QBndSpiral::_pqpClosestPntQuery(double queryPnt[], double closestPt[], double &dist)
{
	PQP_DistanceResult dres;	PQP_REAL p[3];		int minTriIndex;	

	p[0] = queryPnt[0];	p[1] = queryPnt[1];	p[2] = queryPnt[2];
	dres.last_tri = pqpModel->last_tri;	
	PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);

	closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];	// closest point
	minTriIndex = dres.last_tri->id;	// closest triangle
	dist = dres.Distance();				// minimal distance

	return pqpFaceArray[minTriIndex];
}

bool QBndSpiral::_attachCotLaplacianWeight(QMeshPatch *mesh)
{
	double aAvg = 0.0;
	GLKPOSITION pos1;
	QMeshNode *pVer;
	GLKObList *pList;

	for (pos1 = (pList = &(mesh->GetFaceList()))->GetHeadPosition(); pos1 != NULL; )
		aAvg += ((QMeshFace*)pList->GetNext(pos1))->CalArea()/3.0;

	if ((aAvg /= pList->GetCount()) < TOL) return false;

	double aMix, cotA, cotB, cotC, value;
	GLKArray *pLpcs;

	// Determine and store cotangent Laplacian coefficients
	for (pos1 = (pList = &(mesh->GetNodeList()))->GetHeadPosition(); pos1 != NULL; ) {
		pVer = (QMeshNode*)(pList->GetNext(pos1));

		(pVer->attachedPointer == NULL) ?
			pVer->attachedPointer = pLpcs = new GLKArray(20, 20, 3) : (pLpcs = (GLKArray*)pVer->attachedPointer)->RemoveAll();
		aMix = aAvg/_nodeVoronoiArea(pVer);
		cotC = 0.0;

		for (GLKPOSITION pos2 = (pList = &pVer->GetEdgeList())->GetHeadPosition(); pos2 != NULL; )
		{
			if (!_cotLaplacianWeight(pVer, ((QMeshEdge*)(pList->GetNext(pos2))), cotA, cotB)) return false;
			pLpcs->Add(-(value = aMix*(cotA + cotB)));
			cotC += value;
		}

		pLpcs->Add(cotC);
	}

	return true;
}

void QBndSpiral::_releaseCotLaplacianWeight(QMeshPatch *mesh)
{
	GLKPOSITION Pos;

	for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
		QMeshNode *node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
		if (node->attachedPointer!=NULL) {delete ((GLKArray *)(node->attachedPointer));  node->attachedPointer=NULL;}
	}
}

bool QBndSpiral::_cotLaplacianWeight(QMeshNode *node, QMeshEdge *edge, double &cotA, double &cotB)
{
	int i, num;
	double lSq, v[3], e[3], v1[3], v2[3];
	QMeshFace *pFct1, *pFct2;

	if (edge->GetStartPoint() == node) {
		edge->GetEndPoint()->GetCoord3D(e[0], e[1], e[2]);
		pFct1 = edge->GetRightFace(); pFct2 = edge->GetLeftFace();
	}
	else
	{
		edge->GetStartPoint()->GetCoord3D(e[0], e[1], e[2]);
		pFct1 = edge->GetLeftFace(); pFct2 = edge->GetRightFace();
	}

	cotA = cotB = 0.0;

	if (pFct1!=NULL) {
		for (i = 0, num = pFct1->GetEdgeNum(); i < num; i++) if (pFct1->GetNodeRecordPtr(i) == node) break; 
		if (i == num) return false;
		pFct1->GetNodeRecordPtr((i + 1)%num)->GetCoord3D(v1[0], v1[1], v1[2]);
	}

	if (pFct2!=NULL) {
		for (i = 0, num = pFct2->GetEdgeNum(); i < num; i++) if (pFct2->GetNodeRecordPtr(i) == node) break; 
		if (i == num) return false;
		pFct2->GetNodeRecordPtr((i - 1 + num)%num)->GetCoord3D(v2[0], v2[1], v2[2]);
	}

	node->GetCoord3D(v[0], v[1], v[2]); lSq = _squareDist(v, e);
	if (pFct1!=NULL) cotA = _cotan(lSq, _squareDist(v1, e), _squareDist(v1, v), true);
	if (pFct2!=NULL) cotB = _cotan(lSq, _squareDist(v2, e), _squareDist(v2, v), true);

	return true;
}

double QBndSpiral::_nodeVoronoiArea(QMeshNode *node)
{
	int i, num;
	double lSq, lSq0, lSq1, cs, cos0, cos1, value = 0.0, v[3], v0[3], v1[3];
	QMeshNode *pVer0, *pVer1;
	QMeshFace *pFct;
	GLKObList *pFcts = &(node->GetFaceList());

	node->GetCoord3D(v[0], v[1], v[2]);

	for (GLKPOSITION pos = pFcts->GetHeadPosition(); pos != NULL; ) {
		pFct = (QMeshFace*)pFcts->GetNext(pos);

		for (i = 0, num = pFct->GetEdgeNum(); i < num; i++) if (pFct->GetNodeRecordPtr(i) == node) break;
		pVer0 = pFct->GetNodeRecordPtr((i - 1 + num)%num); pVer0->GetCoord3D(v0[0], v0[1], v0[2]);
		pVer1 = pFct->GetNodeRecordPtr((i + 1)%num);	   pVer1->GetCoord3D(v1[0], v1[1], v1[2]);
		lSq = _squareDist(v1, v0);
		lSq0 = _squareDist(v0, v);
		lSq1 = _squareDist(v, v1);
		cs = _cosine(lSq, lSq1, lSq0, true);
		cos0 = _cosine(lSq0, lSq, lSq1, true);
		cos1 = _cosine(lSq1, lSq0, lSq, true);
		value += (cs > 0.0 && cs > 0.0 && cs > 0.0) ?
				 (lSq0*cos0/sqrt(1.0 - cos0*cos0) + lSq1*cos1/sqrt(1.0 - cos1*cos1))/8.0 :	// Non-obtuse: Voronoi is safe
				 _area(v, v0, v1)/((cs > 0.0) ? 4.0 : 2.0);						// Obtuse: Voronoi is inappropriate
	}

	return value;
}

double QBndSpiral::_squareDist(const double ver1[3], const double ver2[3]) 
{
	return (ver1[0] - ver2[0])*(ver1[0] - ver2[0]) + (ver1[1] - ver2[1])*(ver1[1] - ver2[1]) + (ver1[2] - ver2[2])*(ver1[2] - ver2[2]);
}

double QBndSpiral::_cotan(const double len, const double len1, const double len2, const bool bSqLen) 
{
	if (fabs(len1) < TOL || fabs(len2) < TOL) return 0.0;
	const double cs = bSqLen ? (len1 + len2 - len)/(2.0*sqrt(len1)*sqrt(len2)) : (len1*len1 + len2*len2 - len*len)/(2.0*len1*len2);
	if (cs < -1.0 || cs > 1.0 || fabs(fabs(cs) - 1.0) < TOL) return 0.0;
	return cs/sqrt(1.0 - cs*cs);
}

double QBndSpiral::_area(const double ver1[3], const double ver2[3], const double ver3[3])
{
	const double vec1[3] = {ver2[0] - ver1[0], ver2[1] - ver1[1], ver2[2] - ver1[2]};
	const double vec2[3] = {ver3[0] - ver1[0], ver3[1] - ver1[1], ver3[2] - ver1[2]};
	const double vec[3] = {vec1[1]*vec2[2] - vec1[2]*vec2[1],
						   vec1[2]*vec2[0] - vec1[0]*vec2[2],
						   vec1[0]*vec2[1] - vec1[1]*vec2[0]};

	return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])/2.0;
}

double QBndSpiral::_cosine(const double len, const double len1, const double len2, const bool bSqLen/*false*/)
{
	if (fabs(len1) < TOL || fabs(len2) < TOL) return 0.0;

	const double value = bSqLen ? (len1 + len2 - len)/(2.0*sqrt(len1)*sqrt(len2)) :
								  (len1*len1 + len2*len2 - len*len)/(2.0*len1*len2);

	if (value < -1.0) return -1.0;
	if (value > 1.0)  return 1.0;
	return value;
}

void QBndSpiral::_3DCoordToBarycentricCoord(double pos[], double ver0[], double ver1[], double ver2[], double &alpha, double &beta)
{
	double trglArea=_area(ver0,ver1,ver2);
	double aArea=_area(pos,ver1,ver2);
	double bArea=_area(pos,ver2,ver0);
	alpha=aArea/trglArea;	if (alpha>1.0) alpha=1.0;	
	beta=bArea/trglArea;	if (beta>1.0) beta=1.0;	
	if (alpha+beta>1.0) beta=1.0-alpha;
}

void QBndSpiral::_barycentricCoordTo3DCoord(double alpha, double beta, double ver0[], double ver1[], double ver2[], double pos[])
{
	pos[0]=alpha*ver0[0]+beta*ver1[0]+(1.0-alpha-beta)*ver2[0];
	pos[1]=alpha*ver0[1]+beta*ver1[1]+(1.0-alpha-beta)*ver2[1];
	pos[2]=alpha*ver0[2]+beta*ver1[2]+(1.0-alpha-beta)*ver2[2];
}
