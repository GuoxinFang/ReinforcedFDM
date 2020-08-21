// GLKGraph.h: interface for the GLKGraph class.
//
//////////////////////////////////////////////////////////////////////

#include "GLKObList.h"

#ifndef _CW_GLKGRAPHNODE
#define _CW_GLKGRAPHNODE

class GLKGraphNode : public GLKObject
{
public:
	GLKGraphNode() {edgeList.RemoveAll();attachedObj=NULL;};
	virtual ~GLKGraphNode() {};
	void *attachedObj;
	GLKObList edgeList;

	//---------------------------------------------------------------
	//	the following variables are for minimum cut
	double m_excess;
	int m_height;
	GLKGraphNode *nextNode;

	//---------------------------------------------------------------
	//	the following variable is for minimum spanning tree
	bool m_bFlag;
};

#endif

#ifndef _CW_GLKGRAPHEDGE
#define _CW_GLKGRAPHEDGE

class GLKGraphEdge : public GLKObject
{
public:
	GLKGraphEdge() {startNode=NULL;	endNode=NULL;	m_weight=0.0;};
	virtual ~GLKGraphEdge() {};

	GLKGraphNode* startNode;
	GLKGraphNode* endNode;
	double m_weight;	
	void *attachedObj;

	//---------------------------------------------------------------
	//	the following variables are for minimum cut
	double m_flow;

	//---------------------------------------------------------------
	//	the following variable is for traveling on minimum spanning tree
	bool m_bFlag;
};

#endif

#ifndef _CW_GLKGRAPH
#define _CW_GLKGRAPH

class GLKGraphCutNode;

class GLKGraph  
{
public:
	GLKGraph();
	virtual ~GLKGraph();

	void AddNode(GLKGraphNode *node);
	void AddEdge(GLKGraphEdge *edge);
	void FillInEdgeLinkersOnNodes();

	void _Debug_for_MinimumCut();
	void _Debug_for_MST_TSP();

public:
	//-------------------------------------------------------------------------------
	//	The following function is implemented by the approximation using Minimum Spanning Tree
	void ApproximateTravelingSalesmanProblemTour(GLKObList *nodeVisitList);

	//-------------------------------------------------------------------------------
	//	The following function is implemented by the Prim's algorithm
	double MinimumSpanningTree(GLKObList *trRootList, GLKObList *trEdgeList, 
			bool bBuildConnectivityForTree /* When this is set to true, the edgeList of each graph-node will be updated*/);	

	//-------------------------------------------------------------------------------
	//	The following function is implemented by the relabel-to-front algorithm
	double MinimumCut(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
			GLKObList *sourceRegionNodeList, GLKObList *targetRegionNodeList, 
			bool bComputeMaxFlow=false);

private:
	void _initializePreflow(GLKGraphNode *sourceNode);
	void _discharge(GLKGraphNode *uNode);
	void _push(GLKGraphNode *uNode, GLKGraphEdge *edge);
	void _relable(GLKGraphNode *uNode);
	void _partitionByResidualGraph(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
			GLKObList *sourceRegionNodeList, GLKObList *targetRegionNodeList);
	void _propagateInResidualGraph(GLKGraphNode *node, GLKObList *regionNodeList);
	double _computeMaxFlow();

	void _traversalOfMSTForTSP(GLKGraphNode *rootNode, GLKObList *nodeVisitList);

private:
	void clearAll();

	GLKObList nodeList;
	GLKObList edgeList;
};

#endif
