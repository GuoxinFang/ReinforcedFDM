#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"
#include <omp.h>
#include <QTimer>
#include <QLabel>

#include "../QMeshLib/BSPTree.h"
#include "../QMeshLib/QMeshVoxel.h"

#include "../QMeshLib/BSPTreeOperation.h"
#include "../QMeshLib/QMeshVoxelOperation.h"
#include "../QMeshLib/VOXSetStructure.h"

#include "fileIO.h"
#include "meshOperator.h"
#include "PrincipleStressField.h"
#include "GuidanceField.h"
#include "VectorField.h"
#include "ScalarField.h"
#include "SurfaceGuidanceField.h"
#include "FabricationDirectionDetection.h"

#include "toolpathgeneration.h"
#include "heatmethodfield.h"
#include "heatMethod.h"
#include "FabricationChecking.h"
#include "isoSurface.h"

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

	/*---------------------------------*/
	/*----System Function / File IO----*/

public:
	explicit MainWindow(QWidget* parent = 0);
	~MainWindow();

protected:
	void dragEnterEvent(QDragEnterEvent* event);
	void dropEvent(QDropEvent* event);

private:
	fileIO* fileIOObject;
	meshOperator* meshOperatorObject;
	PrincipleStressField* stressFieldComp;
	GuidanceField* GuideFieldComp;

private slots:
	void open();
	void save();

	void saveSelection();
	void readSelection();

private slots:

	void exportMeshtoAbaqus(); // -- ABAQUS file IO

	void autoScroll();

	void signalNavigation(int flag);
	void updateTree();
	void mouseMoveEvent(QMouseEvent* event);
	void on_pushButton_clearAll_clicked();
	void on_treeView_clicked(const QModelIndex& index);
	void changeColorValue();
	void usingFaceNormalShade();
	void usingNodeNormalShade();
	void changeIsoLayerDisplay();
	void changeVoxelLayerDisplay();
	void saveVoxelLayerOrder();
	void loadVoxelLayerOrder();
	void generateVoxelfromPolygonMesh();
	void doConvexPeelingOrderCompute();
	void viewAllIsoLayerandOffsetDisplay();
	void transformTetrahedralIsoValuetoVoxel();
	void compute_LayerThickness_and_Visulize();

	void drawSparseVectorField();

	void shiftToOrigin();
	void scalarModelSize(); // -- scale - rotate model (Tetrahedral mesh)

public slots:

	/*--------------------------------------------*/
	/* Principle Stress Field Computing Functions */

	void compPrincipleStressField();
	void changeTensileandCompressRegion();
	void inputFEMResult(); // -- ABAQUS file IO

	/*--------------------------------------------------------*/
	/* Guidence Field Computing (Vector Field + Scalar Field) */

	void compGuidenceField();

	void doComputeStressField_vectorFieldComp();
	void doComputeStressField_vectorFlipNormal();
	void doComputeStressField_vectorDeleteRegion();
	void doComputeStressField_scalarFieldComp();

	void drawScalarFieldGradient();

	/*-------------------------------------*/
	/*---- ISO surface generation / IO ----*/

	void buildIsoSurface();
	void buildIsoSurface_fromVoxelField();
	void buildIsoSurface_supportMesh();

	void isoSurfaceFieldComputingforToolPathGeneration();
	void transformVolumeMeshtoSurface();
	void surfaceMeshHeatMethodComp();

	void saveIsoLayer();


	/*------------------------------*/
	/*---- Fabrication checking ----*/

	void flipISOSurfaceOrder();

	void findingFabricationDirection();

	void computeSupportingStructure();
	void fabricationBuildSupportNode();
	void fabricationSelectSupportRegion();

	void fabrication_intput_support_tetmesh();
	void fabrication_generate_delete_useless_support_surface();
	void fabrication_Trimming_ConvexHull();
	void inputGeneratedCurveLayer_fabrication();
	void objtoofffile_fabrication();

	void fabrication_collisionChecking();

	void deselectAllinModelTree();
	void OnMeshoperationSubstraction();

private:
	Ui::MainWindow *ui;
	GLKLib *pGLK;

	GLKObList polygenMeshList;
	int voxelLayerNum;

	bool detectFlipLayerSetOrder(PolygenMesh* layerset);
	void flipISOSurfaceOrder(PolygenMesh* layerset);

private:
	void createActions();
	void createTreeView();
	void QTgetscreenshoot();
	void LoadPlatformMesh(QMeshPatch* model);
	
	// TRUE input nozzle, otherwise input platform
	PolygenMesh* _loadPlatformAndNozzle(bool nozzleinput);

	void _inputInstalledIsoSurfaceSet(PolygenMesh* isoSurfaceSet, PolygenMesh* tetModel);

	PolygenMesh* inputGeneratedCurveLayer();

	PolygenMesh *getSelectedPolygenMesh();

	QSignalMapper *signalMapper;
	QSignalMapper *geoField_signalMapper;

	QStandardItemModel *treeModel;

	isoSurface *SurfaceGenerate;
	VOXELSET *voxSet;
	FabricationProcess *fabProcess;

	//PolygenMesh *voxelMesh;
	//QuadTrglMesh *platformMesh;

	/*---------------------------*/
	/*----Toolpath generation----*/

private slots:
	void runHeatMethodCompFieldValue();
	void generateToolPathforMultiAxisPrinting();
	void generateToolPath_StressField();

	void generateToolPathforAllLayer_meshChecking();
	void generateToolPathforAllLayer_memoryReduce();

	void outputSingleToolPath();

	void directToolpathGeneration();
	//void changeViewMode();

private:
	void allLayerToolPathHeatMethodCompute(PolygenMesh* printLayerSet,
		bool accelerate, int omptime, int EachCore);
	PolygenMesh* inputGeneratedCurveLayer_toolpath();

	void toolpath_inputCurveLayer_memoryReduce(std::vector<std::string>& files);
	void toolpath_fieldCompSingleLayer_memoryReduce(QMeshPatch* layer);
	void toolpath_waypointGenerate_memoryReduce(QMeshPatch* layer);
	void toolpath_computeParameter(
		toolPathGeneration *layerToolPathComp, 
		int& boundaryNum, int& zigzagTPathNum, int& boundaryTPathNum,
		double& shrinkOffset, double& boundaryTPathOffset);

	void _buildFileNameSetbySorting(std::vector<std::string>& files, std::string fieldPath);

	toolPathGeneration *ToolPathComp;
	heatMethodField *heatField;

	int viewMode = 1; //1 means show all, -1 means show single

private:
	PolygenMesh* _buildPolygenMesh(mesh_type type, std::string name);
	PolygenMesh* _detectPolygenMesh(mesh_type type);
	void _updateFrameworkParameter();
	void _setParameter(
		double tensileRaio, double compressRatio, int layerNum,
		bool threeDimComp, int twoDimRotateAngle );
};

#endif // MAINWINDOW_H