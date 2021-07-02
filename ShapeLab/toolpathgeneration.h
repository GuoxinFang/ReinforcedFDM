#ifndef TOOLPATHGENERATION_H
#define TOOLPATHGENERATION_H

#include "QMeshPatch.h"
#include "PolygenMesh.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "../GLKLib/GLKObList.h"

class toolPathGeneration
{
public:
	toolPathGeneration(QMeshPatch* inputMesh);
	~toolPathGeneration();

	void generateZigzagToolPath(PolygenMesh* tPath, int Num);
	void generateBundaryToolPath(PolygenMesh* tPath, int Num, double offset);

	void resampling(PolygenMesh* tPath);

	int autoComputeTPathNum(bool type);
	double autoComputeZigZagTPathOffestingValue(
		int zigzagTPathNum, int boundaryTPathNum, double boundaryTPathOffset);

	void mergeFieldforCCF();

	double minBoundaryFieldValue;
	double maxGapDist;
	double maxConnectDist;
	double boundaryGapDist;
	double toolpathOffset;

	bool breakZigzagbyBoundary = true;


private:
	GLKObList zigzagPathList; //install zigzag isonode
	GLKObList boundPathList; //install boundary isonode

	Eigen::MatrixXi edgePIndex;    //connection between isonode and surfaceMesh edge (zigzag)
	Eigen::MatrixXi edgeBPIndex;   //connection between isonode and surfaceMesh edge (boundary)

	QMeshPatch* surfaceMesh;

	double resamplingLength;
	int minZigZagtoolpathNodeNum;

	void initialize(int Num);
	void initialize(int Num, double offset);

	/* Below are the function for zigzag tool path generation */
	bool generateZigZagIsoNode(int Num);

	bool generateSingleZigzagToolPathIsoNode(QMeshPatch* singlePath, double isoValue);
	void connectZigZagToolPathToRegion(PolygenMesh* tPath);
	QMeshNode* connectSingleZigZagToolPathandGenerateEdge(
		QMeshNode* startNode, QMeshNode *edgeSNode, QMeshPatch* tPathRegion, QMeshPatch* thisTPath, double endPointPos[]);

	/* Below are the function for boundary tool path generation */
	void generateBoundaryIsoNode(int Num, double offset);

	void generateSingleBoundaryToolPathIsoNode(QMeshPatch* singlePath, double isoValue);
	void buildOutRingToolPathandConnectZigzagPath(PolygenMesh* tPath);
	void buildOutRingToolPath_without_ConnectZigzagPath(PolygenMesh* tPath);



	void generateSingleBoundaryToolPath(PolygenMesh* tPath, double isoValue);
	QMeshNode* buildOneRingBoundaryToolPath(QMeshNode* startNode, QMeshNode*sNode, QMeshPatch* patch, QMeshPatch* boundIsoPatch);
	QMeshNode* findNextBoundaryToolPath(QMeshNode* startNode, QMeshPatch* boundIsoPatch);
	QMeshNode* findNextNearestPoint(QMeshNode* startNode, QMeshPatch* boundIsoPatch);

	/* Other assistant function */
	double computeAverageConnectEdgeLength(QMeshPatch* patch);
	double resamplingSinglePatch(QMeshPatch* patch);

	/* Other assistant function */
	bool detectSingleZigZagToolPathisProcessed(QMeshPatch* singlePath);
	bool detectAllZigZagToolPathisProcessed();
	bool detectAllBoundaryToolPathisProcessed();

	

	QMeshNode* buildNewNodetoQMeshPatch(QMeshPatch* patch, double pp[], double normal[]);
	QMeshEdge* buildNewEdgetoQMeshPatch(QMeshPatch* patch, QMeshNode* startNode, QMeshNode* endNode);

	bool runBreakingChecking(double firstCurve_IsoValue, double minBoundaryFieldValue);
};

#endif // TOOLPATHGENERATION_H
