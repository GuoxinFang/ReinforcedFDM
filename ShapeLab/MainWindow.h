#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include <QTimer>
#include <QLabel>
#include <omp.h>

#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"


#include "fileIO.h"
#include "meshOperator.h"

#include "PrincipleStressField.h"
#include "VectorField.h"
#include "ScalarField.h"
#include "SurfaceGuidanceField.h"
#include "toolpathgeneration.h"
#include "heatmethodfield.h"
#include "heatMethod.h"
#include "isoSurface.h"

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget* parent = 0);
	~MainWindow();

	//-------file IO-------//
private slots:
	void open();
	void save();

	void saveSelection();
	void readSelection();

	void exportMeshtoAbaqus(); // -- ABAQUS file IO

protected:
	void dragEnterEvent(QDragEnterEvent* event);
	void dropEvent(QDropEvent* event);

private:
	fileIO* fileIOObject;
	meshOperator* meshOperatorObject;
	PrincipleStressField* stressFieldComp;


private slots:
	void autoScroll();

	void signalNavigation(int flag);
	void updateTree();
	void mouseMoveEvent(QMouseEvent* event);
	void on_pushButton_clearAll_clicked();
	void on_treeView_clicked(const QModelIndex& index);
	void usingFaceNormalShade();
	void usingNodeNormalShade();
	void changeIsoLayerDisplay();
	
	void viewAllIsoLayerandOffsetDisplay();
	void compute_LayerThickness_and_Visulize();

	void drawSparseVectorField();


public slots:

	/*--------------------------------------------*/
	/* Principle Stress Field Computing Functions */

	void compPrincipleStressField();
	void changeTensileandCompressRegion();

	/*--------------------------------------------------------*/
	/* Guidence Field Computing (Vector Field + Scalar Field) */

	void compGuidenceField();

	void doComputeStressField_vectorFieldComp();
	void doComputeStressField_vectorFlipNormal();
	void doComputeStressField_vectorDeleteRegion();
	void doComputeStressField_scalarFieldComp();

	/*-------------------------------------*/
	/*---- ISO surface generation / IO ----*/

	void buildIsoSurface();

	void isoSurfaceFieldComputingforToolPathGeneration();

	void saveIsoLayer();
	void flipISOSurfaceOrder();
	void deselectAllinModelTree();

private:
	Ui::MainWindow *ui;
	GLKLib *pGLK;

	GLKObList polygenMeshList;
	int voxelLayerNum;

	bool detectFlipLayerSetOrder(PolygenMesh* layerset);
	void flipISOSurfaceOrder(PolygenMesh* layerset);

private:
	void createActions();
	void createMainFunctionActions();
	void createTreeView();
	void QTgetscreenshoot();
	
	// TRUE input nozzle, otherwise input platform
	PolygenMesh* _loadPlatformAndNozzle(bool nozzleinput);

	void _inputInstalledIsoSurfaceSet(PolygenMesh* isoSurfaceSet, PolygenMesh* tetModel);

	PolygenMesh *getSelectedPolygenMesh();

	QSignalMapper *signalMapper;
	QSignalMapper *geoField_signalMapper;

	QStandardItemModel *treeModel;

	isoSurface *SurfaceGenerate;

	/*---------------------------*/
	/*----Toolpath generation----*/

private slots:
	void runHeatMethodCompFieldValue();
	void generateToolPathforMultiAxisPrinting();
	void generateToolPath_StressField();

	void generateToolPathforAllLayer_meshChecking();

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