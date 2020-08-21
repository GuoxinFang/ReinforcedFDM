#include "stdafx.h"

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <QtDebug>
#include <QDesktopWidget>
#include <QCoreApplication>
#include <QMimeData>
#include <QTreeView>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QScreen>
#include <QInputDialog>
#include <QStyleFactory>

#include <fstream>
#include "dirent.h"
#include "alphanum.hpp"

#include "../GLKLib/GLKCameraTool.h"
#include "../GLKLib/InteractiveTool.h"
#include "../GLKLib/GLKMatrixLib.h"
#include "../GLKLib/GLKGeometry.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	/*This function is used to change the top-left point of UI*/
	move(250, 50);

	QApplication::setStyle(QStyleFactory::create("Fusion"));

	signalMapper = new QSignalMapper(this);
	geoField_signalMapper = new QSignalMapper(this);
	addToolBar(ui->toolBar); addToolBar(ui->navigationToolBar); addToolBar(ui->selectionToolBar);

	createTreeView(); createActions();

	/*OpenGL window in the middle of UI*/
	pGLK = new GLKLib();
	ui->horizontalLayout->addWidget(pGLK);
	ui->horizontalLayout->setMargin(0);
	pGLK->setFocus(); pGLK->clear_tools();
	pGLK->set_tool(new GLKCameraTool(pGLK, ORBITPAN));

	ui->IsoLayerIndex->setMaximum(ui->isoLayerNumber->value());

	fileIOObject = new fileIO();
	meshOperatorObject = new meshOperator();

}

MainWindow::~MainWindow()
{
	delete ui;
}


void MainWindow::createActions()
{
	//-------file IO-------//
	connect(ui->actionOpen, SIGNAL(triggered(bool)), this, SLOT(open()));
	connect(ui->actionSave, SIGNAL(triggered(bool)), this, SLOT(save()));

	/*Save and Read the selection region, with function input FEM stress for each node*/
	connect(ui->actionSaveSelection, SIGNAL(triggered(bool)), this, SLOT(saveSelection()));
	connect(ui->actionReadSelection, SIGNAL(triggered(bool)), this, SLOT(readSelection()));

	/*Transform .tet mesh to the format that abaqus can read*/
	connect(ui->actionExport_to_Abaqus_model, SIGNAL(triggered(bool)), this, SLOT(exportMeshtoAbaqus()));

	//-------Navigation-------// 
	connect(ui->actionFront, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionBack, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionTop, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionBottom, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionLeft, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionRight, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionIsometric, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionZoom_In, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionZoom_Out, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionZoom_All, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionZoom_Window, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->actionFront, 0);
	signalMapper->setMapping(ui->actionBack, 1);
	signalMapper->setMapping(ui->actionTop, 2);
	signalMapper->setMapping(ui->actionBottom, 3);
	signalMapper->setMapping(ui->actionLeft, 4);
	signalMapper->setMapping(ui->actionRight, 5);
	signalMapper->setMapping(ui->actionIsometric, 6);
	signalMapper->setMapping(ui->actionZoom_In, 7);
	signalMapper->setMapping(ui->actionZoom_Out, 8);
	signalMapper->setMapping(ui->actionZoom_All, 9);
	signalMapper->setMapping(ui->actionZoom_Window, 10);

	//-------Visualization-------//
	/*Basic draw triangle, edge and node. Addition with normal and shade.*/
	connect(ui->actionShade, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionMesh, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionProfile, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionFaceNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionNodeNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionFaceNormalShade, SIGNAL(triggered(bool)), this, SLOT(usingFaceNormalShade()));
	connect(ui->actionNodeNormalShade, SIGNAL(triggered(bool)), this, SLOT(usingNodeNormalShade()));

	signalMapper->setMapping(ui->actionShade, 20);
	signalMapper->setMapping(ui->actionMesh, 21);
	signalMapper->setMapping(ui->actionNode, 22);
	signalMapper->setMapping(ui->actionProfile, 23);
	signalMapper->setMapping(ui->actionFaceNormal, 24);
	signalMapper->setMapping(ui->actionNodeNormal, 25);
	ui->actionShade->setChecked(true);

	connect(ui->pushButton_deselectAllinModelTree, SIGNAL(released()), this, SLOT(deselectAllinModelTree()));

	/*Shift the element in origin coordinate (based on selected mesh)*/
	connect(ui->actionShifttoOrigin, SIGNAL(triggered(bool)), this, SLOT(shiftToOrigin()));

	//-------Selection-------//
	/*Flag used for Node: Selected, Fixed and Handle*/
	connect(ui->actionSelectNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectEdge, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectFace, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectFix, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectHandle, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->actionSelectNode, 30);
	signalMapper->setMapping(ui->actionSelectEdge, 31);
	signalMapper->setMapping(ui->actionSelectFace, 32);
	signalMapper->setMapping(ui->actionSelectFix, 33);
	signalMapper->setMapping(ui->actionSelectHandle, 34);

	connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(signalNavigation(int)));
	connect(ui->pushButton_clearAll, SIGNAL(released()), this, SLOT(on_pushButton_clearAll_clicked()));

	connect(ui->pushButton_tetraDelete, SIGNAL(released()), this, SLOT(OnMeshoperationSubstraction()));

	//--------------------------------------------------------//
	//------- Below are the function for SIGGRAPH Asia -------//

	/* input FEM result and pre-process the system */
	connect(ui->pushButton_compPrincipleStressField, SIGNAL(released()), this, SLOT(compPrincipleStressField()));
	connect(ui->pushButton_changeTensileandCompressRegion, SIGNAL(released()), this, SLOT(changeTensileandCompressRegion()));
	connect(ui->actionInputFEM, SIGNAL(triggered(bool)), this, SLOT(inputFEMResult()));

	/* Compute field for slicing */
	connect(ui->pushButton_CompGuideField, SIGNAL(released()), this, SLOT(compGuidenceField()));

	connect(ui->pushButton_CompVectorField, SIGNAL(released()), this, SLOT(doComputeStressField_vectorFieldComp()));
	connect(ui->pushButton_CompScalarField, SIGNAL(released()), this, SLOT(doComputeStressField_scalarFieldComp()));


	connect(ui->pushButtonBuildIsoSurface, SIGNAL(released()), this, SLOT(buildIsoSurface()));
	connect(ui->pushButton_fabricationSupportSurface, SIGNAL(released()), this, SLOT(buildIsoSurface_supportMesh()));

	connect(ui->pushButton_compFieldonIsoSurface, SIGNAL(released()), this, SLOT(isoSurfaceFieldComputingforToolPathGeneration()));

	

	connect(ui->pushButton_outputIsoLayer, SIGNAL(released()), this, SLOT(saveIsoLayer()));
	connect(ui->pushButton_directToolPathGeneration, SIGNAL(released()), this, SLOT(directToolpathGeneration()));
	connect(ui->pushButton_generateStressFieldToolPath, SIGNAL(released()), this, SLOT(generateToolPath_StressField()));
	connect(ui->pushButton_layerThickCompandDraw, SIGNAL(released()), this, SLOT(compute_LayerThickness_and_Visulize()));

	connect(ui->pushButton_scaleModelSize, SIGNAL(released()), this, SLOT(scalarModelSize()));

	//-------Display function-------//
	connect(ui->maximumValue, SIGNAL(valueChanged(int)), this, SLOT(changeColorValue()));
	connect(ui->minimumValue, SIGNAL(valueChanged(int)), this, SLOT(changeColorValue()));
	connect(ui->IsoLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(changeIsoLayerDisplay()));
	connect(ui->checkBox_spaseVectorField, SIGNAL(released()), this, SLOT(drawSparseVectorField()));

	/*ISO-surface Generation directly from tetrahedral mesh*/
	connect(ui->pushButton_volumetoSurface, SIGNAL(released()), this, SLOT(transformVolumeMeshtoSurface()));


	connect(ui->pushButton_VectorFieldFlipNormal, SIGNAL(released()), this, SLOT(doComputeStressField_vectorFlipNormal()));
	connect(ui->pushButton_VectorFieldDeleteRegion, SIGNAL(released()), this, SLOT(doComputeStressField_vectorDeleteRegion()));


	connect(ui->pushButton_isoFacefromVoxelLayer, SIGNAL(released()), this, SLOT(buildIsoSurface_fromVoxelField()));
	connect(ui->pushButton_viewallLayerandOffset, SIGNAL(released()), this, SLOT(viewAllIsoLayerandOffsetDisplay()));

	connect(ui->pushButton_changeOrder, SIGNAL(released()), this, SLOT(flipISOSurfaceOrder()));
	connect(ui->checkBox_drawScalarGradient, SIGNAL(released()), this, SLOT(drawScalarFieldGradient()));


	/*Fabrication checking function*/
	connect(ui->pushButton_fabricationDirection, SIGNAL(released()), this, SLOT(findingFabricationDirection()));
	connect(ui->pushButton_supportingStructureDetection, SIGNAL(released()), this, SLOT(computeSupportingStructure()));
	connect(ui->pushButton_buildSupportNode, SIGNAL(released()), this, SLOT(fabricationBuildSupportNode()));
	connect(ui->pushButton_selectSupportRegion, SIGNAL(released()), this, SLOT(fabricationSelectSupportRegion()));


	connect(ui->pushButton_fabricationSupportTetMesh, SIGNAL(released()), this, SLOT(fabrication_intput_support_tetmesh()));
	connect(ui->pushButton_fabricationEffectSupportSurfaceDetection, SIGNAL(released()), this, SLOT(fabrication_generate_delete_useless_support_surface()));

	connect(ui->pushButton_cutLayerbyConvexHull, SIGNAL(released()), this, SLOT(fabrication_Trimming_ConvexHull()));
	connect(ui->pushButton_InputIsoLayerwithSupport, SIGNAL(released()), this, SLOT(inputGeneratedCurveLayer_fabrication()));
	connect(ui->pushButton_objtooff, SIGNAL(released()), this, SLOT(objtoofffile_fabrication()));

	connect(ui->pushButton_checkCollision, SIGNAL(released()), this, SLOT(fabrication_collisionChecking()));

	/*Voxel generation and computing*/
	connect(ui->pushButton_GenerateVoxel, SIGNAL(released()), this, SLOT(generateVoxelfromPolygonMesh()));
	connect(ui->pushButton_voxelLayerGeneration, SIGNAL(released()), this, SLOT(doConvexPeelingOrderCompute()));
	connect(ui->pushButton_translateTetrahedraltoVoxel, SIGNAL(released()), this, SLOT(transformTetrahedralIsoValuetoVoxel()));
	connect(ui->spinBox_voxelLayer, SIGNAL(valueChanged(int)), this, SLOT(changeVoxelLayerDisplay()));
	connect(ui->pushButton_saveVoxelOrder, SIGNAL(released()), this, SLOT(saveVoxelLayerOrder()));
	connect(ui->pushButton_loadVoxelOrder, SIGNAL(released()), this, SLOT(loadVoxelLayerOrder()));

	//tool path generation
	connect(ui->pushButton_runHeatMethod, SIGNAL(released()), this, SLOT(runHeatMethodCompFieldValue()));
	connect(ui->pushButton_generateZigZagToolPath, SIGNAL(released()), this, SLOT(generateToolPathforMultiAxisPrinting()));
	connect(ui->pushButton_outputToolPath, SIGNAL(released()), this, SLOT(outputSingleToolPath()));

	connect(ui->pushButton_heatMethodTest, SIGNAL(released()), this, SLOT(surfaceMeshHeatMethodComp()));


	connect(ui->pushButton_toolpathGenerationChecking, SIGNAL(released()), this, SLOT(generateToolPathforAllLayer_meshChecking()));
	connect(ui->pushButton_allLayerToolpathGenerate, SIGNAL(released()), this, SLOT(generateToolPathforAllLayer_memoryReduce()));

	//connect(ui->pushButton_switchViewMode, SIGNAL(released()), this, SLOT(changeViewMode()));
}

void MainWindow::open()
{
	/* Currently support .obj and .tet file formate input*/
	QString filenameStr = QFileDialog::getOpenFileName(this, tr("Open File,"), "..//Model//VolumeMesh//", tr(""));
	if (filenameStr.size() < 2) return;
	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();
	QByteArray filenameArray = filenameStr.toLatin1();
	char* filename = filenameArray.data();

	// set polygen name
	std::string strFilename(filename);
	std::size_t foundStart = strFilename.find_last_of("/");
	std::size_t foundEnd = strFilename.find_last_of(".");
	std::string modelName;
	modelName = strFilename.substr(0, foundEnd);
	modelName = modelName.substr(foundStart + 1);

	if (QString::compare(fileSuffix, "obj") == 0) {
		PolygenMesh* polygenMesh = new PolygenMesh(SURFACE_MESH);
		polygenMesh->ImportOBJFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
	}

	else if (QString::compare(fileSuffix, "tet") == 0) {
		PolygenMesh* polygenMesh = new PolygenMesh(INIT_TET);
		std::cout << "Input tetrahedral mesh from: " << filename << std::endl;
		std::cout << "The model name is '" << modelName << "'" << std::endl;
		polygenMesh->ImportTETFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
		this->_updateFrameworkParameter();
	}

	updateTree();
	pGLK->refresh(true);
}

void MainWindow::save()
{
	/* Currently support saving the .obj file*/
	PolygenMesh* polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (!polygenMesh)
		return;
	QString filenameStr = QFileDialog::getSaveFileName(this, tr("OBJ File Export,"), "..", tr("OBJ(*.obj)"));
	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();

	if (QString::compare(fileSuffix, "obj") == 0) {
		QFile exportFile(filenameStr);
		if (exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
			QTextStream out(&exportFile);
			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
					QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
					double xx, yy, zz;
					node->GetCoord3D(xx, yy, zz);
					float r, g, b;
					node->GetColor(r, g, b);
					out << "v " << xx << " " << yy << " " << zz << " " << node->value1 << endl;
				}
				for (GLKPOSITION posFace = patch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
					QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(posFace);
					out << "f " << face->GetNodeRecordPtr(0)->GetIndexNo() << " " << face->GetNodeRecordPtr(1)->GetIndexNo() << " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
				}
			}
		}
		exportFile.close();
	}
}

void MainWindow::inputFEMResult()
{
	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	if (stressFieldComp->tetMesh == nullptr) 	stressFieldComp = new PrincipleStressField(tetMesh);

	stressFieldComp->InputFEMResult(initModel->getModelName());
	pGLK->refresh(true);
}

/*----------------------------------------------------------------------------------*/
//Visulization function (ISOSURFACE)

void MainWindow::changeIsoLayerDisplay() {
	bool single = ui->VisualSingleLayerButtom->isChecked();
	int displayLayerIndex = ui->IsoLayerIndex->value();

	bool changed = false;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

		int index = 0;

		if (polygenMesh->meshType == SUPPORT_LAYERS || polygenMesh->meshType == INIT_LAYERS) {

			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch* isoLayer = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				isoLayer->drawThisIsoLayer = false;

				if (single == true) {
					if (index == displayLayerIndex)
						isoLayer->drawThisIsoLayer = true;
				}
				else {
					if (index < displayLayerIndex)
						isoLayer->drawThisIsoLayer = true;
				}
				index++;
			}

		}


	}
	ui->LayerIndexLabel->setText(QString("No.%1 layer").arg(displayLayerIndex));
	pGLK->refresh(true);
}

void MainWindow::viewAllIsoLayerandOffsetDisplay() {

	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->meshType == INIT_LAYERS || polygenMesh->meshType == SUPPORT_LAYERS) {

			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch* isoLayer = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				isoLayer->drawThisIsoLayer = true;
			}

		}
	}

	pGLK->refresh(true);
}

void MainWindow::compute_LayerThickness_and_Visulize() {

	PolygenMesh* initLayer = this->_detectPolygenMesh(INIT_LAYERS);
	if (initLayer) {
		SurfaceGenerate->computeLayerThicknessForNodeAndDraw(initLayer, false);
		std::cout << " -- finsih computing the layer thickness!" << std::endl;
	}
	PolygenMesh* supportLayer = this->_detectPolygenMesh(SUPPORT_LAYERS);
	if (supportLayer) {
		SurfaceGenerate->computeLayerThicknessForNodeAndDraw(supportLayer, true);
		std::cout << " -- finsih computing the layer thickness!" << std::endl;
	}

	pGLK->refresh(true);
}























void MainWindow::flipISOSurfaceOrder() {

	PolygenMesh *polygenMesh = NULL;

	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh *mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);

		if (mesh->meshType == INIT_LAYERS || mesh->meshType == SUPPORT_LAYERS) {

			polygenMesh = mesh;

			//flip the layer order
			PolygenMesh* flipOrder = new PolygenMesh(INIT_LAYERS);

			for (GLKPOSITION posMesh = polygenMesh->meshList.GetTailPosition(); posMesh != nullptr;) {
				QMeshPatch *layer = (QMeshPatch*)polygenMesh->meshList.GetPrev(posMesh);
				flipOrder->meshList.AddTail(layer);
			}

			polygenMesh->meshList.RemoveAll();

			for (GLKPOSITION posMesh = flipOrder->meshList.GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch *layer = (QMeshPatch*)flipOrder->meshList.GetNext(posMesh);
				polygenMesh->meshList.AddTail(layer);
			}

			//change layer index
			int index = 0;
			for (GLKPOSITION posMesh = polygenMesh->meshList.GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch *layer = (QMeshPatch*)polygenMesh->meshList.GetNext(posMesh);
				layer->SetIndexNo(index + 1); index++;
			}

		}
	}


	pGLK->refresh(true);

	/*
	//switch the order
	for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
		for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
			double pp[3];
			node->GetCoord3D(pp);
			node->SetCoord3D(pp[0], -pp[1], pp[2]);
		}
	}

	//QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetAt(polygenMesh->GetMeshList().GetTailPosition()->prev);
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	double pc[3] = { 0 };
	for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
		double pp[3];
		node->GetCoord3D(pp);
		if (pp[1] < pc[1]) pc[1] = pp[1];
	}

	for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
		for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
			double pp[3];
			node->GetCoord3D(pp);
			node->SetCoord3D(pp[0], pp[1] - pc[1] + 0.5, pp[2]);
		}
	}
	*/
}

void MainWindow::flipISOSurfaceOrder(PolygenMesh* layerset)
{

	if (layerset->meshType != INIT_LAYERS && layerset->meshType != SUPPORT_LAYERS) {
		std::cout << "not iso-surface set mesh " << std::endl;
		return;
	}

	//flip the layer order
	PolygenMesh* flipOrder = new PolygenMesh(INIT_LAYERS);

	for (GLKPOSITION posMesh = layerset->meshList.GetTailPosition(); posMesh != nullptr;) {
		QMeshPatch *layer = (QMeshPatch*)layerset->meshList.GetPrev(posMesh);
		flipOrder->meshList.AddTail(layer);
	}

	layerset->meshList.RemoveAll();

	for (GLKPOSITION posMesh = flipOrder->meshList.GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *layer = (QMeshPatch*)flipOrder->meshList.GetNext(posMesh);
		layerset->meshList.AddTail(layer);
	}

	//change layer index
	int index = 0;
	for (GLKPOSITION posMesh = layerset->meshList.GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *layer = (QMeshPatch*)layerset->meshList.GetNext(posMesh);
		layer->SetIndexNo(index + 1); index++; //mesh index start from 0;
	}
	std::cout << "flip mesh order for this iso-surface set" << std::endl;

}


void MainWindow::saveSelection()
{
	PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (polygenMesh == NULL || polygenMesh->meshType != INIT_TET) { printf(" -- system contains no tet model, return! \n"); return; }

	fileIOObject->saveSelectedFixedHandleRegion(polygenMesh);
}

void MainWindow::readSelection()
{
	PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (polygenMesh == NULL || polygenMesh->meshType != INIT_TET) { printf(" -- system contains no tet model, return! \n"); return; }

	fileIOObject->inputFixedHandleRegion(polygenMesh);
	pGLK->refresh(true);
}


void MainWindow::exportMeshtoAbaqus()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh) polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (polygenMesh->meshType != INIT_TET) { QMessageBox::information(this, "Error", "Input is not a volume mesh!\n"); return; }

	fileIOObject->exportMeshtoAbaqusFEM(polygenMesh);
	// QMessageBox::information(this, "Information", "Finish output Abaqus file in same folder.\n");
}


void MainWindow::mouseMoveEvent(QMouseEvent *event)
{

}

void MainWindow::signalNavigation(int flag)
{
	if (flag <= 10)
		pGLK->setNavigation(flag);
	if (flag >= 20 && flag <= 25) {
		pGLK->setViewModel(flag - 20);
		switch (flag) {
		case 20:
			ui->actionShade->setChecked(pGLK->getViewModel(0));
			break;
		case 21:
			ui->actionMesh->setChecked(pGLK->getViewModel(1));
			break;
		case 22:
			ui->actionNode->setChecked(pGLK->getViewModel(2));
			break;
		case 23:
			ui->actionProfile->setChecked(pGLK->getViewModel(3));
			break;
		case 24:
			ui->actionFaceNormal->setChecked(pGLK->getViewModel(4));
			break;
		case 25:
			ui->actionNodeNormal->setChecked(pGLK->getViewModel(5));
			break;
		}
	}
	if (flag == 30 || flag == 31 || flag == 32 || flag == 33 || flag == 34) {
		InteractiveTool *tool;
		switch (flag) {
		case 30:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NODE, ui->boxDeselect->isChecked());
			break;
		case 31:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), EDGE, ui->boxDeselect->isChecked());
			break;
		case 32:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FACE, ui->boxDeselect->isChecked());
			break;
		case 33:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FIX, ui->boxDeselect->isChecked());
			break;
		case 34:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NHANDLE, ui->boxDeselect->isChecked());
			break;
		}
		pGLK->set_tool(tool);
	}
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
	if (event->mimeData()->hasUrls())
		event->acceptProposedAction();
}

void MainWindow::usingFaceNormalShade() {
	PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	polygenMesh->m_bVertexNormalShading = false;
	pGLK->refresh(true);
}

void MainWindow::usingNodeNormalShade() {
	PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	polygenMesh->m_bVertexNormalShading = true;
	pGLK->refresh(true);
}

void MainWindow::dropEvent(QDropEvent *event)
{
	QString filenameStr;
	foreach(const QUrl &url, event->mimeData()->urls())
		filenameStr = url.toLocalFile();
	QByteArray filenameArray = filenameStr.toLatin1();
	char *filename = filenameArray.data();

	PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);

	// set polygen name
	std::string strFilename(filename);
	std::size_t foundStart = strFilename.find_last_of("/");
	std::size_t foundEnd = strFilename.find_last_of(".");
	std::string modelName;
	modelName = strFilename.substr(0, foundEnd);
	modelName = modelName.substr(foundStart + 1);
	int i = 0;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
		std::string name = (polygen->getModelName()).substr(0, (polygen->getModelName()).find(' '));
		if (name == modelName)
			i++;
	}
	if (i > 0)
		modelName += " " + std::to_string(i);

	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();

	if (QString::compare(fileSuffix, "obj") == 0) {
		polygenMesh->ImportOBJFile(filename, modelName);
		ui->SystemDialog->insertPlainText("Finish input obj file.\n\n");
		polygenMesh->meshType = SURFACE_MESH;
	}
	else if (QString::compare(fileSuffix, "off") == 0) {
		polygenMesh->ImportOFFFile(filename, modelName);
		ui->SystemDialog->insertPlainText("Finish input off file.\n\n");
		polygenMesh->meshType = SURFACE_MESH;

	} //not sure if this can be used!
	else if (QString::compare(fileSuffix, "tet") == 0) {
		polygenMesh->ImportTETFile(filename, modelName);
		ui->SystemDialog->insertPlainText("Finish input tet file.\n\n");
		polygenMesh->meshType = INIT_TET;
	}
	else {
		QMessageBox::critical(this, "Input model fail", "Input file is not in a right format! \n (only .obj and .tet file is accepted)");
		return;
	}
	polygenMesh->m_bVertexNormalShading = false;
	polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
	pGLK->AddDisplayObj(polygenMesh, true);
	polygenMeshList.AddTail(polygenMesh);

	//shiftToOrigin();
	updateTree();

}

void MainWindow::_updateFrameworkParameter() {

	std::string modelName = this->_detectPolygenMesh(INIT_TET)->getModelName();

	if (modelName == "bunnyhead") this->_setParameter(0.1, 0.1, 100, false, 295);
	else if (modelName == "topopt_new") this->_setParameter(0.3, 0.3, 100, false, 210);
	else if (modelName == "CSquare") this->_setParameter(0.1, 0.1, 120, true, 0);
	else if (modelName == "YogaNew") this->_setParameter(0.1, 0.1, 160, true, 0);
	else if (modelName == "bridgeSmall") this->_setParameter(0.3, 0.0, 150, false, 180);
	else {
		std::cout << "This model is not found, using initial parameters!" << std::endl;
		this->_setParameter(0, 0, 0, true, 0); return;
	}

	std::cout << " -- Successfully input compute parameters for the given model -- " << std::endl;
}

void MainWindow::_setParameter(
	double tensileRaio, double compressRatio, int layerNum, bool threeDimComp, int twoDimRotateAngle){

	ui->tensileRegionRatio->setText(QString::number(tensileRaio));
	ui->compressRegionRatio->setText(QString::number(tensileRaio));
	ui->isoLayerNumber->setValue(layerNum);
	ui->checkBox_3DCompute->setChecked(threeDimComp);

	if (threeDimComp == false)
		ui->spinBox_ComputedAnlge->setValue(twoDimRotateAngle);
}



void MainWindow::autoScroll()
{
	QTextCursor cursor = ui->SystemDialog->textCursor();
	cursor.movePosition(QTextCursor::End);
	ui->SystemDialog->setTextCursor(cursor);
}

void MainWindow::createTreeView()
{
	treeModel = new QStandardItemModel();
	ui->treeView->setModel(treeModel);
	ui->treeView->setHeaderHidden(true);
	ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
	ui->treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui->treeView->expandAll();
}

void MainWindow::updateTree()
{
	treeModel->clear();
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		QString modelName = QString::fromStdString(polygenMesh->getModelName());
		QStandardItem *modelListItem = new QStandardItem(modelName);
		modelListItem->setCheckable(true);
		if (polygenMesh->bShow)
			modelListItem->setCheckState(Qt::Checked);
		else modelListItem->setCheckState(Qt::Unchecked);
		treeModel->appendRow(modelListItem);
	}

	//pGLK->refresh(true);
}

void MainWindow::deselectAllinModelTree() {
	int rows = treeModel->rowCount();
	int column = treeModel->columnCount();
	for (int i = 0; i < rows; ++i){
		for (int j = 0; j < column; ++j){
			QStandardItem* item = treeModel->item(i, j);
			item->setCheckState(Qt::Unchecked);
		}
	}

	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* mesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		mesh->bShow = false;
	}

	pGLK->refresh(true);

}

PolygenMesh *MainWindow::getSelectedPolygenMesh()
{
	if (!treeModel->hasChildren())
		return nullptr;
	QModelIndex index = ui->treeView->currentIndex();
	QString selectedModelName = index.data(Qt::DisplayRole).toString();
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		QString modelName = QString::fromStdString(polygenMesh->getModelName());
		if (QString::compare(selectedModelName, modelName) == 0)
			return polygenMesh;
	}
	return nullptr;
}

void MainWindow::on_pushButton_clearAll_clicked()
{
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		pGLK->DelDisplayObj(polygenMesh);
		/*if(polygenMesh->pglkIndex == 1) pGLK->DelDisplayObj(polygenMesh);
		else if (polygenMesh->pglkIndex == 2) pGLK1->DelDisplayObj(polygenMesh);*/
	}
	polygenMeshList.RemoveAll();
	pGLK->ClearDisplayObjList();
	pGLK->refresh();
	updateTree();

	/*int i = 0;
	for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr; i++){
	PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();
	if (i<2)
	continue;
	for (GLKPOSITION pos2=patch->GetFaceList().GetHeadPosition(); pos2!=nullptr;){
	QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(pos2);
	face->m_nIdentifiedPatchIndex = 0;
	}
	}
	pGLK->refresh(true);*/
}

void MainWindow::on_treeView_clicked(const QModelIndex &index)
{
	ui->treeView->currentIndex();
	QStandardItem *modelListItem = treeModel->itemFromIndex(index);
	ui->treeView->setCurrentIndex(index);
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (modelListItem->checkState() == Qt::Checked)
		polygenMesh->bShow = true;
	else
		polygenMesh->bShow = false;
	pGLK->refresh(true);
}

void MainWindow::changeColorValue()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	int min = ui->minimumValue->value();
	int max = ui->maximumValue->value();

	patch->drawValue[0] = patch->minStressValue * (1 - min * 0.01);
	patch->drawValue[1] = patch->maxStressValue * (1 - max * 0.01);

	ui->minValue->setText(QString("MinValue = %1").arg(patch->drawValue[0]));
	ui->maxValue->setText(QString("MaxValue = %1").arg(patch->drawValue[1]));

	pGLK->refresh(true);
}

void MainWindow::QTgetscreenshoot()
{
	QScreen *screen = QGuiApplication::primaryScreen();
	QString filePathName = "Screen-";
	filePathName += QDateTime::currentDateTime().toString("yyyy-MM-dd hh-mm-ss");
	filePathName += ".png"; //important to control the file type
	if (!screen->grabWindow(0).save(filePathName, "png")) cout << "Get screen shoot failed!!!" << endl;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

//Function summary in 07-05-2020 (important function!)

// Input installed isosurface inside given folder based on input tetrahedral mesh name
void MainWindow::_inputInstalledIsoSurfaceSet(PolygenMesh* isoSurfaceSet, PolygenMesh* tetModel) {
	printf(" -- Input INIT_LAYERS from folder\n\n");
	isoSurfaceSet = this->_buildPolygenMesh(INIT_LAYERS, "isoSurface");

	std::string folderPath = "..\\Model\\IsoSurface\\" + tetModel->getModelName();
	std::vector<std::string> layersFileSet;
	this->_buildFileNameSetbySorting(layersFileSet, folderPath);

	fileIOObject->inputInstalledIsoSurface(isoSurfaceSet, layersFileSet, folderPath);
}

// Sorting the file name (installed isosurface mesh) in given folder
void MainWindow::_buildFileNameSetbySorting(std::vector<std::string>& files, std::string fieldPath) {
	std::cout << fieldPath << std::endl;
	DIR* dp;
	struct dirent* ep;
	dp = opendir(fieldPath.c_str());

	if (dp == NULL) { perror("Couldn't open the directory"); return; }

	while (ep = readdir(dp))
	{
		if ((string(ep->d_name) != ".") && (string(ep->d_name) != ".."))
		{
			if (string(ep->d_name).find(".obj") == string::npos &&
				string(ep->d_name).find(".txt") == string::npos) continue;
			files.push_back(string(ep->d_name));
		}
	}
	(void)closedir(dp);

	cout << "There are " << files.size() << " files in the current directory." << endl;
	sort(files.begin(), files.end(), doj::alphanum_less<std::string>());

	//for (int i = 0; i < files.size(); i++) files[i] = fieldPath + fieldPath[i];
}

PolygenMesh* MainWindow::_loadPlatformAndNozzle(bool nozzleinput) {
	//-------------------------------------
	//read platform file and build model
	PolygenMesh* inputMesh;

	char filename[1024];
	if (nozzleinput == false) {
		sprintf(filename, "%s%s", "..\\Model\\", "platform.obj");
		inputMesh = this->_buildPolygenMesh(SURFACE_MESH, "Platform");
		printf("Input platform!\n");
	}
	else {
		sprintf(filename, "%s%s", "..\\Model\\", "nozzleCHull.obj");
		inputMesh = this->_buildPolygenMesh(SURFACE_MESH, "Nozzle");
		printf("Input nozzle!\n");
	}

	QMeshPatch* platform = new QMeshPatch;
	platform->SetIndexNo(inputMesh->GetMeshList().GetCount()); //index begin from 0
	inputMesh->GetMeshList().AddTail(platform);
	platform->inputOBJFile(filename, false);

	return inputMesh;
}
















////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////














































void MainWindow::doComputeStressField_vectorFieldComp() {

	if (ui->checkBox_3DCompute->isChecked() == false)
		GuideFieldComp->runFieldComputing_TenslieandCompressRegion();
	else
		GuideFieldComp->runFieldComputing_optVectorField();
	pGLK->refresh(true);

}

void MainWindow::doComputeStressField_vectorFlipNormal() {
	GuideFieldComp->runFieldComputing_FlipSelectedRegion();
	pGLK->refresh(true);
}

void MainWindow::doComputeStressField_vectorDeleteRegion() {

	GuideFieldComp->runFieldComputing_DeleteRegion();
	pGLK->refresh(true);

}

void MainWindow::doComputeStressField_scalarFieldComp() {

	GuideFieldComp->runFieldComputing_computeScalarField();

	PolygenMesh* initialModel = (PolygenMesh*)polygenMeshList.GetHead();

	fileIOObject->outputVectorFieldforRendering(initialModel);

	pGLK->refresh(true);

}

void MainWindow::drawScalarFieldGradient() {
	GuideFieldComp->visualGuideFieldGradient(ui->checkBox_drawScalarGradient->isChecked());
	pGLK->refresh(true);
}

void MainWindow::drawSparseVectorField() {

	GuideFieldComp->tetMesh->sparseVectorDraw =
		ui->checkBox_spaseVectorField->isChecked();

	pGLK->refresh(true);
}

void MainWindow::buildIsoSurface_fromVoxelField()
{
	//----------------------------------------------------------
	//Detect volume mesh in the system and build SFieldComp item
	PolygenMesh* polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh) {
		if (!polygenMeshList.GetHead()) {
			QMessageBox::warning(this, "Warning", "There's no model exist in the system!\n");
			return;
		}
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
		if (polygenMesh->meshType == SURFACE_MESH) {
			QMessageBox::warning(this, "Warning", "This is not a volume mesh\n");
			return;
		}
	}

	QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();
	SurfaceGenerate = new isoSurface(patch);

	//------------------------------------------------------------------------
	//Transform the voxel layer number to tetrahedral mesh node as field value
	VOXSetOperation::transformVoxeltoMeshField(voxSet, patch);

	//-----normalize the field value
	Eigen::VectorXd guideFieldNormalize(patch->GetNodeNumber());
	Eigen::VectorXd guideField(patch->GetNodeNumber());

	int iter = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		guideField(iter) = Node->guideFieldValue;
		iter++;
	}

	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < patch->GetNodeNumber(); i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i < patch->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	iter = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->guideFieldValue = guideFieldNormalize(iter);
		iter++;
	}

	patch->drawVectorField = true;

	std::cout << "Finish generate scalar field for the tetrahedra mesh by voxel" << std::endl;


	meshOperatorObject->compTetMeshVolumeMatrix(patch);

	GuideFieldComp = new GuidanceField(patch);
	GuideFieldComp->runFieldComputing_VoxelGuide();
	pGLK->refresh(true);

}


void MainWindow::changeTensileandCompressRegion() {
	stressFieldComp->DetermineCriticalTensileandCompressRegion(
		ui->tensileRegionRatio->text().toDouble(), ui->compressRegionRatio->text().toDouble());
	pGLK->refresh(true);
}

/*--------------------------------------------------*/
//IO function - save generated isosurface (ISOSURFACE)

void MainWindow::saveIsoLayer()
{
	PolygenMesh* initialModel = (PolygenMesh*)polygenMeshList.GetHead();
	if(initialModel == NULL || initialModel->meshType!= INIT_TET) { printf(" No tet model detected in the system! \n"); return; }
	
	/*-------------------------------
	build the folder to save isolayer
	-------------------------------*/

	std::string folderPath = "..\\Model\\IsoSurface\\" + initialModel->getModelName();
	
	//-- System command to build folder
	string command = "mkdir " + folderPath;
	string command_saveField = "mkdir " + folderPath + "\\Field";
	string command_support = "mkdir " + folderPath + "\\Support";
	string command_supportTrim = "mkdir " + folderPath + "\\Support_Trim";
	string command_supportSplit = "mkdir " + folderPath + "\\Support_Split";
	string command_layerFieldRender = "mkdir " + folderPath + "\\layerFieldRender";

	string command_toolpath = "mkdir " + folderPath + "\\toolpath";

	system(command.c_str()); system(command_saveField.c_str()); 
	system(command_support.c_str()); system(command_supportTrim.c_str());
	system(command_supportSplit.c_str()); system(command_toolpath.c_str());
	system(command_layerFieldRender.c_str());

	/*-------------------------------------------
	find the isosurface polygenmesh in the system
	-------------------------------------------*/

	PolygenMesh* isosurfaceSet = NULL;
	PolygenMesh* isosurfaceSet_support = NULL;
	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh *mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == SUPPORT_LAYERS) { isosurfaceSet_support = mesh; break; }
	}
	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh *mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == INIT_LAYERS) { isosurfaceSet = mesh; break; }
	}	
	
	if (isosurfaceSet == NULL) { printf(" No iso-surface detected in the system! \n"); return; }

	/*---------------
	Output isoSurface
	---------------*/

	if (isosurfaceSet != NULL && isosurfaceSet_support != NULL) {
		fileIOObject->updateInitialSurfaceName(isosurfaceSet, isosurfaceSet_support);
	}

	bool splitMode = ui->checkBox_outputILayer_splitMode->isChecked();
	bool singleOutputMode = ui->checkBox_outputSingleLayer->isChecked();
	bool offMode = ui->checkBox_outputILayer_OFFMode->isChecked();
	int maxLayerNum = ui->outputLayerIndexNum->value();

	//notice that the iso-surface toolpath stress field will also be installed.
	fileIOObject->minPrincipleStressValue = ((QMeshPatch*)initialModel->GetMeshList().GetHead())->minPrincipleStressValue;
	fileIOObject->maxPrincipleStressValue = ((QMeshPatch*)initialModel->GetMeshList().GetHead())->maxPrincipleStressValue;

	fileIOObject->outputISOSurfaceMesh(
		isosurfaceSet, splitMode, singleOutputMode, initialModel->getModelName(), maxLayerNum, offMode);

	if (isosurfaceSet_support != NULL) {
		fileIOObject->outputISOSurfaceMesh_support(isosurfaceSet_support, initialModel->getModelName());
	}
	/*----------------------------------------------------------------------------
	Output vector and scalr field for drawing (for SIGGRAPH Asia paper 2020-03-12)
	----------------------------------------------------------------------------*/

	fileIOObject->saveFieldforRendering(initialModel);
	
}

//##################################################################################//
/*----------------------------------------------------------------------------------*/
/*-----------------------Fabrication Checking Function------------------------------*/


void MainWindow::fabrication_collisionChecking() {

	// ## MAKE SURE this function run after detect the printing direction!
	PolygenMesh* tetMesh = this->_detectPolygenMesh(INIT_TET);
	QMeshPatch* initTet = (QMeshPatch*)tetMesh->GetMeshList().GetHead(); initTet->checkCollision = true;

	/* -- First input convex hull mesh of nozzle -- */
	PolygenMesh* printNozzleMesh = this->_loadPlatformAndNozzle(true); //true means input nozzle mesh
	QMeshPatch* printNozzle = (QMeshPatch*) printNozzleMesh->GetMeshList().GetHead();

	/* -- Check collision by moving nozzle to the position -- */
	if (fabProcess->runFabricationCollisionDetection(printNozzle)) {


		std::cout << "THIS Model includes Collision!" << std::endl;

		if (ui->checkBox_CollisionUpdateLayer->isChecked()) {
			// update the surface mesh
			//SFieldComp->computeVolumeMatrixTetrahedral();
			GuideFieldComp->adujstField_collisionRegion();

			PolygenMesh* isoSurfaceSet = this->_detectPolygenMesh(INIT_LAYERS);
			isoSurfaceSet->ClearAll();
			isoSurface * newSurfaceMesh = new isoSurface(initTet);
			
			ui->isoLayerNumber->setValue(120);
			
			newSurfaceMesh->generateIsoSurface(isoSurfaceSet, ui->isoLayerNumber->value());

			int index = 0;
			for (GLKPOSITION posMesh = isoSurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch* isoLayer = (QMeshPatch*)isoSurfaceSet->GetMeshList().GetNext(posMesh);
				int layerIndex = index; index++;

				GuideFieldComp->runIsoLayerFielCompute(isoLayer, layerIndex);
			}

			viewAllIsoLayerandOffsetDisplay();
		}	

		else this->fileIOObject->outputCollisionSurface(this->_detectPolygenMesh(INIT_LAYERS));

	}
	
	for (int i = 0; i < 5; i++) SurfaceGenerate->smoothingIsoSurface(fabProcess->isoSurfaceSet);

	pGLK->refresh(true);
}

bool MainWindow::detectFlipLayerSetOrder(PolygenMesh* layerset) {

	QMeshPatch* firstLayer = (QMeshPatch*)layerset->GetMeshList().GetHead();
	QMeshPatch* lastLayer = (QMeshPatch*)layerset->GetMeshList().GetTail();
	double fBondingBox[6], bBondingBox[6];
	firstLayer->ComputeBoundingBox(fBondingBox);
	lastLayer->ComputeBoundingBox(bBondingBox);
	if (fBondingBox[2] > bBondingBox[2])
		return true;
	else return false;

}

void MainWindow::computeSupportingStructure() {

	if( polygenMeshList.GetCount() > 0 && fabProcess->systemBuild )
		fabProcess->runSupportRegionCompute(false);

	else {
		PolygenMesh *tetModel = NULL;
		for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
			PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
			if (polygenMesh->meshType == INIT_TET) tetModel = polygenMesh;
		}

		PolygenMesh *printingLayer = NULL;

		printingLayer = inputGeneratedCurveLayer();

		fabProcess = new FabricationProcess(printingLayer, this->_loadPlatformAndNozzle(false), tetModel);
		fabProcess->systemBuild = true;
		
		fabProcess->runSupportRegionCompute(false);
	}

	pGLK->refresh(true);
}

void MainWindow::fabricationSelectSupportRegion() {

	PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();

	if (ui->checkBox_deselectSupportNode->isChecked()) {
		polygenMesh->supportRegionSelection = true;
		if (ui->checkBox_deselectSupportNode_byNode->isChecked())
			polygenMesh->supportRegionSelection_byInit = true;
		else polygenMesh->supportRegionSelection_byInit = false;
	}
	else polygenMesh->supportRegionSelection = false;

}

void MainWindow::fabricationBuildSupportNode() {

	/*#### supportNode includes two patch: support node + convex hull ####*/

	//----------------build the PolygenMesh----------------//

	PolygenMesh* supportRegion = NULL;

	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->getModelName() == "supportRegion")
			supportRegion = polygenMesh;
	}

	if (supportRegion == NULL) {
		supportRegion = new PolygenMesh(SUPPORT_REGION);
		supportRegion->setModelName("supportRegion");
		supportRegion->BuildGLList(supportRegion->m_bVertexNormalShading);

		pGLK->AddDisplayObj(supportRegion, true);
		polygenMeshList.AddTail(supportRegion);
		updateTree();
	}
	else supportRegion->ClearAll();
	
	//----------------build the support node----------------//

	fabProcess->computeSupportNode(supportRegion);
	
	//----------------build the convex hull----------------//

	fabProcess->computeSupportingConvexHull(supportRegion);

	//----------------output the convex hull and rotated tetrahedral mesh----------------//

	fabProcess->outputConvexHullandTetrahedralMesh(supportRegion);

	//----------------output support node----------------//
	if(ui->checkBox_saveSupportNode->isChecked()) {
		QMeshPatch* supportNodePatch = (QMeshPatch*)supportRegion->GetMeshList().GetHead();
		this->fileIOObject->outputSupportNodeMesh(supportNodePatch, this->_detectPolygenMesh(INIT_TET));
	}
	
	
	pGLK->refresh(true);

}

void MainWindow::fabrication_generate_delete_useless_support_surface() {

	//after this function, the non-important surface (without interact with support line) will be delete

	PolygenMesh *supportRegion; // support Region mesh
	PolygenMesh *supportIsoSurface;

	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

		if (polygenMesh->meshType == SUPPORT_LAYERS)
			supportIsoSurface = polygenMesh;
		else if (polygenMesh->meshType == SUPPORT_REGION)
			supportRegion = polygenMesh;

	}

	/* Detect interset and delect non-important support region */

	fabProcess->supportNodeBoxSize = ui->doubleSpinBox_supportNodeBoxSize->value();
	fabProcess->support_surface_delete_nonimportant_region(supportRegion, supportIsoSurface);
	printf("finish delete layer without support region.\n\n");

	/*Update the layer index of support-layer polygenmesh*/

	int layerIndex = 1;
	for (GLKPOSITION pos = supportIsoSurface->GetMeshList().GetHeadPosition(); pos != nullptr;) {
		QMeshPatch *singleLayer = (QMeshPatch*)supportIsoSurface->GetMeshList().GetNext(pos);
		if (singleLayer->GetNodeNumber() == 0) supportIsoSurface->GetMeshList().Remove(singleLayer);
		else {
			singleLayer->SetIndexNo(layerIndex); layerIndex++;
		}
	}
	printf("update layer index.\n\n");

	pGLK->refresh(true);
}

void MainWindow::fabrication_Trimming_ConvexHull() {

	/* cutting the mesh with convex-hull */

	// detect isosurface set
	PolygenMesh* supportIsoSurface = NULL;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

		if (polygenMesh->meshType == SUPPORT_LAYERS)
			supportIsoSurface = polygenMesh;

	}
	if (supportIsoSurface == NULL) {
		std::cout << "isosurface not exist in the system!" << std::endl; return;
	}

	//build system and input convex hull
	isoSurface *planeCutSurfaceMesh = new isoSurface(NULL);

	PolygenMesh *convexHull = new PolygenMesh(SURFACE_MESH);
	convexHull->setModelName("convexHull");

	QMeshPatch* convexHullSupport = new QMeshPatch;
	convexHullSupport->SetIndexNo(convexHull->GetMeshList().GetCount()); //index begin from 0
	convexHull->GetMeshList().AddTail(convexHullSupport);

	convexHull->m_bVertexNormalShading = false;
	convexHull->BuildGLList(convexHull->m_bVertexNormalShading);
	polygenMeshList.AddTail(convexHull);
	updateTree();
	pGLK->AddDisplayObj(convexHull, true);

	char filename[1024];
	sprintf(filename, "%s", "..\\Model\\IsoSurface\\mechancialbarSmall\\convex_hull.obj");
	convexHullSupport->inputOBJFile(filename, false); std::cout << endl;

	//begin cut
	double scaleRatio = 0.99;
	double center[3] = { 0 };
	for (GLKPOSITION pos = convexHullSupport->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode *node = (QMeshNode*)convexHullSupport->GetNodeList().GetNext(pos);
		double pp[3];  node->GetCoord3D(pp);
		for (int i = 0; i<3; i++) center[i] += pp[i];
	}
	for (int i = 0; i<3; i++) center[i] /= (double)convexHullSupport->GetNodeNumber();

	for (GLKPOSITION pos = convexHullSupport->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode *node = (QMeshNode*)convexHullSupport->GetNodeList().GetNext(pos);
		double pp[3];  node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++)
			pp[i] = (pp[i] - center[i])*scaleRatio + center[i];
		node->SetCoord3D(pp[0], pp[1], pp[2]);
	}

	std::cout << "in total " << convexHullSupport->GetFaceNumber() << " cutting time!" << std::endl;

	for (GLKPOSITION pos = supportIsoSurface->GetMeshList().GetHeadPosition(); pos != nullptr;) {
		QMeshPatch *singleLayer = (QMeshPatch*)supportIsoSurface->GetMeshList().GetNext(pos);
		if (singleLayer->includeSupportRegion == false) continue;

		std::vector<Eigen::Vector4d> planeEquation;
		for (GLKPOSITION Pos = convexHullSupport->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace *convexHullFace = (QMeshFace*)convexHullSupport->GetFaceList().GetNext(Pos);

			bool cutFace = true;
			Eigen::Vector4d thisPlaneEquation;
			convexHullFace->CalPlaneEquation(thisPlaneEquation(0), thisPlaneEquation(1), thisPlaneEquation(2), thisPlaneEquation(3));
			for (int i = 0; i < planeEquation.size(); i++) {
				double dotProduct = thisPlaneEquation(0)*planeEquation[i](0) +
					thisPlaneEquation(1)*planeEquation[i](1) + thisPlaneEquation(2)*planeEquation[i](2);
				if (fabs(dotProduct) > 0.999999 && fabs(thisPlaneEquation(3) - planeEquation[i](3) < 0.00001) ){
					cutFace = false; break;
				}
			}

			if (cutFace == false) {
				//std::cout << "No. " << convexHullFace->GetIndexNo() << " no need to cut!"<< std::endl;
				continue;
			}
			else {
				//std::cout << "No. " << convexHullFace->GetIndexNo() << std::endl;
				planeCutSurfaceMesh->planeCutSurfaceMesh_delete(singleLayer, convexHullFace);
				planeEquation.push_back(thisPlaneEquation);
			}

		}
		std::cout << "No. " << singleLayer->GetIndexNo() << "finished" <<std::endl;

		//break;

	}

	for (GLKPOSITION pos = supportIsoSurface->GetMeshList().GetHeadPosition(); pos != nullptr;) {
		QMeshPatch *singleLayer = (QMeshPatch*)supportIsoSurface->GetMeshList().GetNext(pos);
	
		int nodeIndex = 0;
		for (GLKPOSITION Pos = singleLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *node = (QMeshNode*)singleLayer->GetNodeList().GetNext(Pos);
			nodeIndex++; node->SetIndexNo(nodeIndex);
		}
	}


	printf("finish cutting the mesh.\n\n");
	pGLK->refresh(true);

}

void MainWindow::inputGeneratedCurveLayer_fabrication() {

	//-------------------------------------
	//detect mesh file
	printf("Start import isosurface with support layer.\n");
	DIR* dp;
	struct dirent* ep;
	vector<string> files;
	dp = opendir("..\\Model\\IsoSurface\\bunnyhead\\final"); //change this with the model name!

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			//cout << ep->d_name << endl;
			if ((string(ep->d_name) != ".") && (string(ep->d_name) != ".."))
			{
				files.push_back(string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	cout << "There are " << files.size() << " files in the current directory." << endl;

	PolygenMesh* sliceSet = new PolygenMesh(SUPPORT_LAYERS);
	sliceSet->setModelName("layer_with_support");

	//-------------------------------------
	//read slice files and build mesh_patches
	sort(files.begin(), files.end(), doj::alphanum_less<std::string>());

	char filename[1024];
	for (int i = 0; i < files.size(); i++)
	{
		//sprintf(filename, "%s%s", "..\\Model\\smoothness\\", files[i].data());
		sprintf(filename, "%s%s", "..\\Model\\IsoSurface\\bunnyhead\\final\\", files[i].data());
		if (files[i].find(".obj") == string::npos) continue;

		cout << "Input path file / name: " << filename << ", ";

		QMeshPatch* slice = new QMeshPatch;
		slice->drawThisIsoLayer = true;
		slice->SetIndexNo(sliceSet->GetMeshList().GetCount()); //index begin from 0
		sliceSet->GetMeshList().AddTail(slice);

		slice->inputOBJFile(filename, false); std::cout << endl;

		string supportFlag = files[i].substr(files[i].length() - 5, 1);
		std::cout << supportFlag << std::endl;
		if (supportFlag == "S") 		slice->includeSupportRegion = true;
		else 		slice->includeSupportRegion = false;

		//pre-computing the normal
		for (GLKPOSITION Pos = slice->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace *face = (QMeshFace*)slice->GetFaceList().GetNext(Pos);
			face->CalPlaneEquation();
		}
		for (GLKPOSITION Pos = slice->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)slice->GetNodeList().GetNext(Pos);
			Node->CalNormal();
		}

	}
	// Display
	sliceSet->BuildGLList(sliceSet->m_bVertexNormalShading);
	polygenMeshList.AddTail(sliceSet);
	pGLK->AddDisplayObj(sliceSet, true);
	updateTree();
	printf("-------Fabrication checking: Sliceset import successfully.\n\n");

	pGLK->refresh(true);
}

void MainWindow::objtoofffile_fabrication() {
	//-------------------------------------
	//detect mesh file
	printf("Start obj -> off\n");
	DIR* dp;
	struct dirent* ep;
	vector<string> files;
	dp = opendir("..\\Model\\IsoSurface\\layer_with_support"); //change this with the model name!

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			//cout << ep->d_name << endl;
			if ((string(ep->d_name) != ".") && (string(ep->d_name) != ".."))
			{
				files.push_back(string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	cout << "There are " << files.size() << " files in the current directory." << endl;


	//-------------------------------------
	//read slice files and build mesh_patches
	sort(files.begin(), files.end(), doj::alphanum_less<std::string>());

	char filename[1024];
	for (int i = 0; i < files.size(); i++)
	{
		//sprintf(filename, "%s%s", "..\\Model\\smoothness\\", files[i].data());
		sprintf(filename, "%s%s", "..\\Model\\IsoSurface\\layer_with_support\\", files[i].data());
		if (files[i].find(".obj") == string::npos) continue;

		cout << "Input path file / name: " << filename << ", ";

		QMeshPatch* slice = new QMeshPatch;


		slice->inputOBJFile(filename, false); std::cout << endl;

		std::string path = "..\\Model\\IsoSurface\\layer_with_support\\" + files[i] + ".off";
		ofstream nodeSelection(path);

		nodeSelection << "OFF" << std::endl;
		nodeSelection << slice->GetNodeNumber() << " " << slice->GetFaceNumber() << " 0" << std::endl;

		for (GLKPOSITION posNode = slice->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode *node = (QMeshNode*)slice->GetNodeList().GetNext(posNode);
			double pp[3];
			node->GetCoord3D(pp);
			nodeSelection << pp[0] << " " << pp[1] << " " << pp[2] << endl;
		}
		for (GLKPOSITION posFace = slice->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace *face = (QMeshFace*)slice->GetFaceList().GetNext(posFace);

			nodeSelection << "3 " << face->GetNodeRecordPtr(0)->GetIndexNo() - 1
				<< " " << face->GetNodeRecordPtr(1)->GetIndexNo() - 1
				<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() - 1 << endl;
		}
		nodeSelection.close();

		/*
		char filename[1024];
		for (int i = 0; i < files.size(); i++)
		{
			//sprintf(filename, "%s%s", "..\\Model\\smoothness\\", files[i].data());
			sprintf(filename, "%s%s", "..\\Model\\IsoSurface\\layer_with_support\\", files[i].data());
			if (files[i].find(".off") == string::npos) continue;

			cout << "Input path file / name: " << filename << ", ";

			QMeshPatch* slice = new QMeshPatch;


			slice->inputOFFFile(filename, false); std::cout << endl;

			std::string path = "..\\Model\\IsoSurface\\layer_with_support\\" + files[i] + ".obj";
			ofstream nodeSelection(path);


			for (GLKPOSITION posNode = slice->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
				QMeshNode *node = (QMeshNode*)slice->GetNodeList().GetNext(posNode);
				double pp[3];
				node->GetCoord3D(pp);
				nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << endl;
			}
			for (GLKPOSITION posFace = slice->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
				QMeshFace *face = (QMeshFace*)slice->GetFaceList().GetNext(posFace);
				nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
					<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
					<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
			}
			nodeSelection.close();
		}
		*/

	}

	pGLK->refresh(true);
}

void MainWindow::fabrication_intput_support_tetmesh() {

	printf(" \n\n ##----------------------------------------------------------## \n");
	printf(" -- Support tetrahedral mesh successfully inputed in the system! \n");

	PolygenMesh *initPolygen = (PolygenMesh *)polygenMeshList.GetHead();

	const char* modelName = initPolygen->getModelName().c_str();
	//-------------------------------------
	// input support tetmesh
	char filename[1024];
	//sprintf(filename, "%s%s%s", "..\\Model\\Volume_Mesh\\", "letterSfineMesh", "_supporting.tet");
	sprintf(filename, "%s%s%s", "..\\Model\\Volume_Mesh\\", modelName, "_supporting.tet");
	//sprintf(filename, "%s%s%s", "..\\Model\\Volume_Mesh\\", "mechancialbarSmall", "_supporting.tet");
	std::cout << filename << std::endl;

	PolygenMesh *supportTetMesh = new PolygenMesh(SUPPORT_TET);
	supportTetMesh->setModelName("support_Model");

	QMeshPatch *supportPatch = new QMeshPatch;
	supportPatch->SetIndexNo(supportTetMesh->GetMeshList().GetCount()); //index begin from 0
	supportTetMesh->GetMeshList().AddTail(supportPatch);
	supportPatch->inputTETFile(filename, false);

	//-------------------------------------
	// update the drawing
	supportTetMesh->m_bVertexNormalShading = false;
	supportTetMesh->BuildGLList(supportTetMesh->m_bVertexNormalShading);
	polygenMeshList.AddTail(supportTetMesh);
	updateTree();
	pGLK->AddDisplayObj(supportTetMesh, true);

	//--------------------------------------------------------
	// ## ---- perivious method, drag the mesh and then inputs	
	/*
	PolygenMesh *initPolygen = (PolygenMesh *)polygenMeshList.GetHead();
	PolygenMesh *supportPolygen = NULL;

	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

		if (polygenMesh->getModelName() == "mechancialbar_supporting")
			supportPolygen = polygenMesh;

	}

	if (supportPolygen == NULL) { cerr << "Support Tet not input!" << std::endl; return; }
	*/
	//--------------------------------------------------------

	//----------------------------------
	// link the topology and allign mesh
	fabProcess->support_tet_link_topology(
		(QMeshPatch*)initPolygen->GetMeshList().GetHead(), (QMeshPatch*)supportTetMesh->GetMeshList().GetHead());

	pGLK->refresh(true);

}

void MainWindow::buildIsoSurface_supportMesh() {

	QMeshPatch* supportTet = fabProcess->supportPatch;

	///////////////////////////////////////////////////
	/*compute volume matrix for this tetrahedral mesh*/
	//StressField* supportStressFieldComp = new StressField(supportTet);


	//supportStressFieldComp->computeVolumeMatrixTetrahedral();

	meshOperatorObject->compTetMeshVolumeMatrix(supportTet);

	///////////////////////////////////
	/*compute vector and scalar field*/
	GuidanceField* supportFieldCompute = new GuidanceField(supportTet);
	supportFieldCompute->runSupportTetFieldCompute();

	//////////////////////
	/*iso-surface slicer*/

	isoSurface* supportSurfaceGenerate = new isoSurface(supportTet);

	PolygenMesh* isoSurface_support = this->_buildPolygenMesh(SUPPORT_LAYERS, "isoSurface_support");

	/*PolygenMesh *isoSurface_support = new PolygenMesh(SUPPORT_LAYERS);
	isoSurface_support->setModelName("isoSurface_support");
	isoSurface_support->m_bVertexNormalShading = true;
	isoSurface_support->BuildGLList(isoSurface_support->m_bVertexNormalShading);
	pGLK->AddDisplayObj(isoSurface_support, true);
	polygenMeshList.AddTail(isoSurface_support);
	updateTree();*/

	/* ---- Generate iso-surface for all region ---- */
	supportSurfaceGenerate->generateIsoSurface_supportStructure(isoSurface_support, ui->isoLayerNumber->value());

	supportSurfaceGenerate->minLayerThickness = ui->doubleSpinBox_LayerThickMin->value();
	supportSurfaceGenerate->maxLayerThickness = ui->doubleSpinBox_LayerThickMax->value();
	for(int i=0;i<3;i++)
		bool iter = supportSurfaceGenerate->checkIsoSurfaceDistanceAndBuildNewLayer_supportStructure(isoSurface_support);
	//supportSurfaceGenerate->deleteCloseIsoSurface_supportLayer(isoSurface_support);

	//bool iter;
	//do {
	//	iter = SurfaceGenerate->checkIsoSurfaceDistance(isoSurface);
	//} while (iter == true);

	//SurfaceGenerate->deleteCloseIsoSurface(isoSurface);

	/* ---- Split the support region ---- */
	if (detectFlipLayerSetOrder(isoSurface_support)) flipISOSurfaceOrder(isoSurface_support);
	for (GLKPOSITION posMesh = isoSurface_support->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* singleLayer = (QMeshPatch*)isoSurface_support->GetMeshList().GetNext(posMesh);
		if (singleLayer->isInnerSlcingLayer == true) std::cout << "No." << singleLayer->GetIndexNo() << "is inner layer!" << std::endl;
	
	}

	supportSurfaceGenerate->splitSupportandModelSurface(isoSurface_support);

	for (int i = 0; i < 5; i++) supportSurfaceGenerate->smoothingIsoSurface(isoSurface_support);

	

	viewAllIsoLayerandOffsetDisplay();
	ui->IsoLayerIndex->setMaximum(isoSurface_support->GetMeshList().GetCount());

	pGLK->refresh(true);
	return;
	
	//----------------build the folder to save vector and sacalr field file----------------//
	PolygenMesh*  initialModel = (PolygenMesh*)polygenMeshList.GetHead();
	std::string folderPath = "..\\Model\\RenderFile\\" + initialModel->getModelName();
	string command;
	//command = "mkdir -p " + folderPath;
	command = "mkdir " + folderPath;
	system(command.c_str());


	//----------------output vector field----------------//
    // this part also need to compute the sacalr field value to visuilize the vector color
	PolygenMesh* polygenMesh;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->meshType == SUPPORT_TET) break;
	}
	QMeshPatch* tetModel = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string vectorFieldPath = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_vectorFieldSupportRegion.txt";
	ofstream vectorField(vectorFieldPath);

	for (GLKPOSITION posNode = tetModel->GetTetraList().GetHeadPosition(); posNode != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetModel->GetTetraList().GetNext(posNode);
		if (tet->tetSupportElement) continue;
		double pp[3];
		tet->CalCenterPos(pp[0], pp[1], pp[2]);
		double tetScalarField = 0;
		for (int i = 0; i < 4; i++)
			tetScalarField += tet->GetNodeRecordPtr(i + 1)->guideFieldValue;

		vectorField << pp[0] << "," << pp[1] << "," << pp[2] << ","
			<< tet->vectorField(0) << "," << tet->vectorField(1) << "," << tet->vectorField(2) << "," << tetScalarField / 4 << std::endl;
	}

	vectorField.close();

	std::string vectorFieldPath1 = "../Model/RenderFile/"
		+ initialModel->getModelName() + "/" + initialModel->getModelName() + "_vectorFieldOriginalRegion.txt";
	ofstream vectorField1(vectorFieldPath1);

	for (GLKPOSITION posNode = tetModel->GetTetraList().GetHeadPosition(); posNode != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetModel->GetTetraList().GetNext(posNode);
		if (tet->tetSupportElement == false) continue;
		double pp[3];
		tet->CalCenterPos(pp[0], pp[1], pp[2]);
		double tetScalarField = 0;
		for (int i = 0; i < 4; i++)
			tetScalarField += tet->GetNodeRecordPtr(i + 1)->guideFieldValue;

		vectorField1 << pp[0] << "," << pp[1] << "," << pp[2] << ","
			<< tet->vectorField(0) << "," << tet->vectorField(1) << "," << tet->vectorField(2) << "," << tetScalarField / 4 << std::endl;
	}

	vectorField1.close();

}

PolygenMesh* MainWindow::inputGeneratedCurveLayer() {

	printf("Start import Sliceset.\n");
	DIR* dp;
	struct dirent* ep;
	vector<string> files;
	dp = opendir("..\\Model\\IsoSurface\\keyring"); //change this with the model name!

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			//cout << ep->d_name << endl;
			if ((string(ep->d_name) != ".") && (string(ep->d_name) != ".."))
			{
				files.push_back(string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	cout << "There are " << files.size() << " files in the current directory." << endl;

	PolygenMesh* sliceSet = new PolygenMesh(INIT_LAYERS);
	sliceSet->setModelName("printingLayer");

	//-------------------------------------
	//read slice files and build mesh_patches
	sort(files.begin(), files.end(), doj::alphanum_less<std::string>());

	char filename[1024];
	for (int i = 0; i < files.size(); i++)
	{
		//sprintf(filename, "%s%s", "..\\Model\\smoothness\\", files[i].data());
		sprintf(filename, "%s%s", "..\\Model\\IsoSurface\\keyring\\", files[i].data());
		if (files[i].find(".off") == string::npos) continue;

		//cout << "Input path file / name: " << filename << ", ";

		QMeshPatch* slice = new QMeshPatch;
		slice->drawThisIsoLayer = true;
		slice->SetIndexNo(sliceSet->GetMeshList().GetCount()); //index begin from 0
		sliceSet->GetMeshList().AddTail(slice);
		//slice->inputOBJFile(filename, false); std::cout << endl;
		slice->inputOFFFile(filename, false); // std::cout << endl;

		//pre-computing the normal
		for (GLKPOSITION Pos = slice->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace *face = (QMeshFace*)slice->GetFaceList().GetNext(Pos);
			face->CalPlaneEquation();
		}
		for (GLKPOSITION Pos = slice->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)slice->GetNodeList().GetNext(Pos);
			Node->CalNormal();
		}

	}
	// Display
	sliceSet->BuildGLList(sliceSet->m_bVertexNormalShading);
	polygenMeshList.AddTail(sliceSet);
	pGLK->AddDisplayObj(sliceSet, true);
	updateTree();
	printf("-------Fabrication checking: Sliceset import successfully.\n\n");

	pGLK->refresh(true);
	return sliceSet;
}



//##################################################################################//
/*----------------------------------------------------------------------------------*/
/*----------Voxel representation and voxel-based curved layer generation------------*/

void MainWindow::generateVoxelfromPolygonMesh() {
	//------------------------------------------------------------------------------------------------
	//Get mesh inside the system, including the voxelization model and platform model
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh) {
		if (!polygenMeshList.GetHead()) {
			QMessageBox::warning(this, "Warning", "There's no model exist in the system!\n"); return;
		}
	}

	bool surfaceMeshDetected = false;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		/*if (polygenMesh->isSurfaceMesh == true) {
		surfaceMeshDetected = true;	break;
		}*/
	}
	/*if (!surfaceMeshDetected) {
	QMessageBox::warning(this, "Warning",
	"There's no surface mesh in the system! You should transfer volume mesh to surface face first!\n");
	return;
	}*/

	QMeshPatch *modelWaitforVoxelization = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	//QMeshPatch *platform = new QMeshPatch; bool platformDetected = false;
	//for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
	//	polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
	//	if (polygenMesh->getModelName() == "platform") {
	//		cout << endl << "platform detected from system!!" << endl;
	//		platform = (QMeshPatch*)polygenMesh->GetMeshList().GetHead(); platformDetected = true; break;}
	//}
	//if(platformDetected == false) {
	//	QMessageBox::warning(this, "Warning", "There's no platform in the system!\n"); return; }

	this->LoadPlatformMesh(modelWaitforVoxelization);
	QMeshPatch *platform = new QMeshPatch;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->getModelName() == "Printing Platform") {
			platform = (QMeshPatch*)polygenMesh->GetMeshList().GetHead(); break;
		}
	}

	//-------------------------------------------------------------------------------------------------
	//Transform QMeshpatch into QuadTrglMesh, align model and platform together.
	QuadTrglMesh *platformMesh = new QuadTrglMesh;
	platformMesh->TransformFromQMeshPatch(platform);

	QuadTrglMesh *mesh = new QuadTrglMesh;
	mesh->TransformFromQMeshPatch(modelWaitforVoxelization);

	//-------------------------------------------------------------------------------------------------
	//Build BSP tree and use it for voxelization
	BSPTREE *treePtr;		float boundingBox[6];
	long time = clock();
	BSPSolidOperation::BSPTreeConstructionFromBRep(mesh, treePtr, boundingBox);
	printf("-------------------------------------------------------------\n");
	printf("The construction of BSP solid is completed (taking %ld ms).\n", clock() - time);

	//-----------------generate BSP tree by chengkai's method--------------------
	//BSPTree *bsp_tree = new BSPTree();
	//bsp_tree->BSPTreeFromMesh(patch);
	//Eigen::MatrixXd V(patch->GetNodeNumber(), 3);
	//int index = 0;
	//for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
	//	Node->GetCoord3D(V(index, 0), V(index, 1), V(index, 2));
	//	index++;
	//}
	//static float voxel_size = 1; //this is the size of voxel
	//static int offset_size = 5; //this is the offset value
	//VoxelGrid *voxel_set = new VoxelGrid(V, voxel_size, offset_size);
	//voxel_set->Voxelization(bsp_tree);

	//-------------------------------------------------------------------------------------------------
	//ConstructVoxelSetFromBSPSolid and build new polygenmesh for voxel visualization
	time = clock();
	voxSet = new VOXELSET;
	float width = ui->doubleSpinBox_voxelWidth->value();
	voxSet = VOXSetOperation::ConstructVoxelSetFromBSPSolid(treePtr, boundingBox, width, false);
	printf("-------------------------------------------------------------\n");
	printf("Finish generate the VoxelSet (taking %ld ms).\n", clock() - time);

	//-------------------------------------------------------------------------------------------------
	//Expanding the voxel in case the tetrahedral mesh node cannot find connected voxel
	time = clock();
	for (int i = 0; i < ui->spinBox_expandingLayer->value(); i++)
		VOXSetOperation::expandingVoxelSet(voxSet);
	printf("-------------------------------------------------------------\n");
	printf("Expanding the VoxelSet (taking %ld ms).\n", clock() - time);

	//-------------------------------------------------------------------------------------------------
	//Build voxelMesh for visualization
	time = clock();
	PolygenMesh* voxelMesh = new PolygenMesh(VOXEL_MESH);

	VOXSetOperation::voxelVisualizationwithPlatform(voxelMesh, voxSet, platform);
	//VOXSetOperation::voxelVisualization(voxelMesh, voxSet);

	voxelMesh->setModelName("Voxel");
	voxelMesh->voxelSize = width;
	voxelMesh->m_bVertexNormalShading = false;
	voxelMesh->BuildGLList(voxelMesh->m_bVertexNormalShading);
	polygenMeshList.AddTail(voxelMesh);

	//only display voxel and platform after building them
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->meshType != VOXEL_MESH && polygenMesh->getModelName() != "Printing Platform")
			polygenMesh->bShow = false;
	}

	updateTree();
	pGLK->AddDisplayObj(voxelMesh, true);
	ui->pushButton_voxelLayerGeneration->setEnabled(true);

	printf("-------------------------------------------------------------\n");
	printf("Finish build the voxelMesh for visualization (taking %ld ms).\n", clock() - time);
	//-------------------------------------------------------------------------------------------------
	//Convex hull peeling order compute 
	//doConvexPeelingOrderCompute(platformMesh);
	//VOXSetOperation::layerInformationUpdate(voxelMesh, voxSet);
	//ui->spinBox_voxelLayer->setEnabled(true);
	//QMeshPatch* voxelPatch = (QMeshPatch*) voxelMesh->GetMeshList().GetHead();
	//voxelLayerNum = 0;
	//for (GLKPOSITION Pos = voxelPatch->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode *Node = (QMeshNode*)voxelPatch->GetNodeList().GetNext(Pos);
	//	if (Node->voxelLayerIndex > voxelLayerNum) voxelLayerNum = Node->voxelLayerIndex;
	//}
	//ui->spinBox_voxelLayer->setMaximum(voxelLayerNum);
}

void MainWindow::doConvexPeelingOrderCompute() {
	//-------------------------------------
	//Get platform mesh and build voxelMesh
	PolygenMesh* voxelMesh;

	QMeshPatch *platform = new QMeshPatch;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->getModelName() == "Printing Platform") {
			platform = (QMeshPatch*)polygenMesh->GetMeshList().GetHead(); break;
		}
	}
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		voxelMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (voxelMesh->getModelName() == "Voxel") {
			break;
		}
	}
	QuadTrglMesh *platformMesh = new QuadTrglMesh;
	platformMesh->TransformFromQMeshPatch(platform);

	//-------------------------------------
	//define few file location
	char resRoboAMFilename[1000];
	sprintf(resRoboAMFilename, "/Res/_test");
	char CCL_DEFAULT_FOLDER_LOCATION[1000];
	sprintf(CCL_DEFAULT_FOLDER_LOCATION, "Data\\");
	char CCL_DEFAULT_AMR_FILE[1000];
	sprintf(CCL_DEFAULT_AMR_FILE, "Data\\_output.amr");

	//-------------------------------------
	//run ComputeConvexPeelingOrder function
	int flag = ui->spinBox_VoxelGenerateMethod->value();
	int FabNum = 0;
	switch (flag) {
	case 1:
		FabNum = VOXSetOperation::ComputeConvexPeelingOrder(voxSet,
			1, // printing direction
			CCL_DEFAULT_AMR_FILE, platformMesh,
			CCL_DEFAULT_FOLDER_LOCATION, resRoboAMFilename);
		break;
	case 2:
		FabNum = VOXSetOperation::ComputeConvexPeelingGovenedConvexGrowingAMOrder(voxSet,
			1, // printing direction
			CCL_DEFAULT_AMR_FILE, platformMesh,
			CCL_DEFAULT_FOLDER_LOCATION, resRoboAMFilename);
		break;
	case 3: //-----convex-front growing method + convex hull without platform
		FabNum = VOXSetOperation::ComputeAdditiveManufacturingOrder(voxSet,
			1, // printing direction
			0,	// tolerance for the base layer
			CCL_DEFAULT_AMR_FILE, 2,
			NULL, //without checking the platformMesh for convex-hull!
				  //platformMesh,
			CCL_DEFAULT_FOLDER_LOCATION, resRoboAMFilename);
		break;
	case 4: //-----convex-front growing method + convex hull with platform
		FabNum = VOXSetOperation::ComputeAdditiveManufacturingOrder(voxSet,
			1, // printing direction
			0,	// tolerance for the base layer
			CCL_DEFAULT_AMR_FILE, 2,
			platformMesh,
			CCL_DEFAULT_FOLDER_LOCATION, resRoboAMFilename);
		break;
	case 5:
		//convex-front growing method (with shadow prevention, convex hull with platform)
		FabNum = VOXSetOperation::ComputeAdditiveManufacturingOrderwithShadowPrevention(voxSet,
			1, // printing direction
			0,	// tolerance for the base layer
			platformMesh);
		break;

	case 6:
		this->loadVoxelLayerOrder();
		VOXSetOperation::ComputeTetrahedralFieldGovenedGrowingAMOrder(voxSet, platformMesh,
			1 // printing direction
		);
		break;
	}

	VOXSetOperation::layerInformationUpdate(voxelMesh, voxSet);
	voxelMesh->voxelOrderComputed = true;
	ui->spinBox_voxelLayer->setEnabled(true);
	QMeshPatch* voxelPatch = (QMeshPatch*)voxelMesh->GetMeshList().GetHead();
	voxelLayerNum = 0;
	for (GLKPOSITION Pos = voxelPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelPatch->GetNodeList().GetNext(Pos);
		if (Node->voxelLayerIndex > voxelLayerNum) voxelLayerNum = Node->voxelLayerIndex;
	}
	ui->spinBox_voxelLayer->setMaximum(voxelLayerNum);

	pGLK->refresh(true);
}

void MainWindow::transformVolumeMeshtoSurface() {
	//-------------------------------------
	//get volume mesh 
	if (!polygenMeshList.GetHead()) return;
	PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch* volumeMesh = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();
	if (!volumeMesh->isVolume) {
		QMessageBox::warning(this, "Warning", "Selected mesh is not a volume mesh!\n");
		return;
	}
	//-------------------------------------
	//build surface mesh in the system
	PolygenMesh *surfaceMesh = new PolygenMesh(SURFACE_MESH);
	surfaceMesh->setModelName(polygenMesh->getModelName() + "_surfaceMesh");

	QMeshPatch *surface = new QMeshPatch;
	surface->SetIndexNo(surfaceMesh->GetMeshList().GetCount()); //index begin from 0
	surfaceMesh->GetMeshList().AddTail(surface);

	//build nodeNum and nodeTable
	int nodeNum = 0;
	for (GLKPOSITION Pos = volumeMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)volumeMesh->GetNodeList().GetNext(Pos);
		if (!Node->inner) nodeNum++;
	}
	float* nodeTable;
	nodeTable = (float *)malloc(sizeof(float)*nodeNum * 3);
	int index = 0;
	for (GLKPOSITION Pos = volumeMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)volumeMesh->GetNodeList().GetNext(Pos);
		if (!Node->inner) {
			double pp[3];
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
			Node->VolumetoSurfaceIndex = index; //VolumetoSurfaceIndex start from 0
			index++;
		}
	}

	//build faceNum and faceTable
	int faceNum = 0;
	for (GLKPOSITION Pos = volumeMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)volumeMesh->GetFaceList().GetNext(Pos);
		if (!face->inner) faceNum++;
	}
	unsigned int* faceTable;
	faceTable = (unsigned int *)malloc(sizeof(unsigned int)*faceNum * 3);
	index = 0;
	for (GLKPOSITION Pos = volumeMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *face = (QMeshFace*)volumeMesh->GetFaceList().GetNext(Pos);
		if (!face->inner) {
			for (int i = 0; i < 3; i++)
				faceTable[(index + 1) * 3 - 1 - i] = face->GetNodeRecordPtr(i)->VolumetoSurfaceIndex;
			index++;
		}
	}
	surface->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	surfaceMesh->m_bVertexNormalShading = false;
	surfaceMesh->BuildGLList(surfaceMesh->m_bVertexNormalShading);
	polygenMeshList.AddTail(surfaceMesh);
	updateTree();

	pGLK->AddDisplayObj(surfaceMesh, true);

	free(nodeTable);
	free(faceTable);
}

void MainWindow::LoadPlatformMesh(QMeshPatch* model) {
	//-------------------------------------
	//read platform file and build model
	char filename[1024];
	sprintf(filename, "%s%s", "..\\Model\\", "platform.obj");

	PolygenMesh *platformMesh = new PolygenMesh(SURFACE_MESH);
	platformMesh->setModelName("Printing Platform");

	QMeshPatch *platform = new QMeshPatch;
	platform->SetIndexNo(platformMesh->GetMeshList().GetCount()); //index begin from 0
	platformMesh->GetMeshList().AddTail(platform);
	platform->inputOBJFile(filename, false);

	//-------------------------------------
	//scale platform to match the size of model
	double pp[3];
	double xmax = -1e+8, xmin = 1e+8;
	double zmax = -1e+8, zmin = 1e+8;
	for (GLKPOSITION Pos = model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)model->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[0]>xmax) xmax = pp[0];
		if (pp[0]<xmin) xmin = pp[0];
		if (pp[2]>zmax) zmax = pp[2];
		if (pp[2]<zmin) zmin = pp[2];
	}
	double xscale = xmax - xmin;
	double zscale = zmax - zmin;
	double scale = MAX(xscale, zscale) / 75;

	scale = 1.2;
	if (scale > 1) {
		for (GLKPOSITION Pos = platform->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)platform->GetNodeList().GetNext(Pos);
			Node->GetCoord3D(pp[0], pp[1], pp[2]);
			//for (int i = 0; i < 3; i++) pp[i] *= scale * 1.1;

			//for (int i = 0; i < 3; i++) pp[i] *= scale * 2.5;
			for (int i = 0; i < 3; i++) pp[i] *= scale * 0.9;

			Node->SetCoord3D(pp[0], pp[1], pp[2]);
		}
	}

	//-------------------------------------
	//move platform to the bottom of model and locate it to the center
	double modelCentral[3] = { 0 };
	double bottomY = 100000.0;
	for (GLKPOSITION Pos = model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)model->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) modelCentral[i] += pp[i];
		if (pp[1] < bottomY) bottomY = pp[1];
	}
	for (int i = 0; i < 3; i++) modelCentral[i] = modelCentral[i] / model->GetNodeNumber();

	double platformCentral[3] = { 0 }; double upperHight = -100000.0;
	for (GLKPOSITION Pos = platform->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)platform->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) platformCentral[i] += pp[i];
		if (pp[1] > upperHight) upperHight = pp[1];
	}
	for (int i = 0; i < 3; i++) platformCentral[i] = platformCentral[i] / platform->GetNodeNumber();

	double Xshift = modelCentral[0] - platformCentral[0];
	double Yshift = bottomY - upperHight;
	double Zshift = modelCentral[2] - platformCentral[2];
	for (GLKPOSITION Pos = platform->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)platform->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		Node->SetCoord3D(pp[0] + Xshift, pp[1] + Yshift, pp[2] + Zshift);
	}

	//-------------------------------------
	//update the drawing
	platformMesh->m_bVertexNormalShading = false;
	platformMesh->BuildGLList(platformMesh->m_bVertexNormalShading);
	polygenMeshList.AddTail(platformMesh);
	updateTree();
	pGLK->AddDisplayObj(platformMesh, true);
	//pGLK->refresh(true);

	printf("Platform import successfully and centered!\n");
}

void MainWindow::transformTetrahedralIsoValuetoVoxel() {

	PolygenMesh *tetPolygenMesh;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		tetPolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (tetPolygenMesh->meshType == INIT_TET) break;
	}

	PolygenMesh *voxel;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		voxel = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (voxel->meshType == VOXEL_MESH) break;
	}

	std::cout << tetPolygenMesh->getModelName() << std::endl;

	QMeshPatch *tetPatch = (QMeshPatch*)tetPolygenMesh->GetMeshList().GetHead();
	QMeshPatch *voxelPatch = (QMeshPatch*)voxel->GetMeshList().GetHead();

	VOXSetOperation::transformMeshFieldValuetoVoxel(voxSet, tetPatch, voxelPatch);

	voxel->voxelOrderComputed = true;
	pGLK->refresh(true);

	printf("Successfully transfer the iso value from tetrahedral mesh to voxel layer!\n");
}

void MainWindow::saveVoxelLayerOrder() {
	//get voxel patch
	PolygenMesh *voxel;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		voxel = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (voxel->meshType == VOXEL_MESH) break;
	}
	QMeshPatch *voxelPatch = (QMeshPatch*)voxel->GetMeshList().GetHead();

	//save voxel layer to txt file
	ofstream voxelLayer("..\\Model\\VoxelLayer.txt");
	int index = 0;
	for (GLKPOSITION Pos = voxelPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelPatch->GetNodeList().GetNext(Pos);
		if (Node->isPlatformNode) continue;
		voxelLayer << index << ":" << Node->voxelLayerIndex << endl;
		index++;
	}
	voxelLayer.close();
}

void MainWindow::loadVoxelLayerOrder() {
	//get voxel patch
	PolygenMesh *voxel;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		voxel = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (voxel->meshType == VOXEL_MESH) break;
	}
	QMeshPatch *voxelPatch = (QMeshPatch*)voxel->GetMeshList().GetHead();

	//scan txt file;
	ifstream voxelLayer("..\\Model\\VoxelLayer.txt");
	if (!voxelLayer) cerr << "Sorry! We were unable to open the file!\n";

	vector<int> VoxelIndex(voxelPatch->GetNodeNumber()), VoxelLayer(voxelPatch->GetNodeNumber());
	int LineIndex = 0;
	std::string sss;
	while (getline(voxelLayer, sss)) {
		const char * c = sss.c_str();
		sscanf(c, "%d:%d", &VoxelIndex[LineIndex], &VoxelLayer[LineIndex]);
		LineIndex++;
	}
	voxelLayer.close();

	int index = 0;
	//save the index to node in voxelPatch
	for (GLKPOSITION Pos = voxelPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelPatch->GetNodeList().GetNext(Pos);
		if (Node->isPlatformNode) continue;
		Node->voxelLayerIndex = VoxelLayer[index] + 1;
		index++;
	}
	voxel->voxelOrderComputed = true;

	//save the index to voxel in the voxSet and update the layer information
	for (int i = 0; i < voxSet->nodeNum; i++) voxSet->nodeArray[i].stressFieldLayerIndex = VoxelLayer[i] + 1;

	pGLK->refresh(true);
	printf("\n Successfully input the layer index for all voxel. (reminder: this function only used for development!)\n");

}

void MainWindow::changeVoxelLayerDisplay() {
	int layerIndex = ui->spinBox_voxelLayer->value();
	PolygenMesh *voxelMesh;
	QMeshPatch *voxelPatch;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		voxelMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (voxelMesh->meshType == VOXEL_MESH) {
			voxelPatch = (QMeshPatch*)voxelMesh->GetMeshList().GetHead();
			break;
		}
	}
	for (GLKPOSITION Pos = voxelPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)voxelPatch->GetNodeList().GetNext(Pos);
		Node->isVoxelDraw = true;
		if (Node->voxelLayerIndex > layerIndex) Node->isVoxelDraw = false;
	}
	VOXSetOperation::voxelVisualization_layerUpdate(voxelMesh, voxSet);

	pGLK->refresh(true);
}


//##################################################################################//
/*----------------------------------------------------------------------------------*/
/*------------------------Tool-path Generation Function-----------------------------*/

//*-----single toolpath generation-----

void MainWindow::runHeatMethodCompFieldValue() {
	long time = clock();

	//Initialize the program
	PolygenMesh* pMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch* patch = (QMeshPatch*)pMesh->GetMeshList().GetHead();
	heatField = new heatMethodField(patch);
	ToolPathComp = new toolPathGeneration(patch);

	/*--------------------------------*/
	/*Generate boundary distance field*/

	//--Setp1: Initialize the geofield value for boundary node
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->selected = false;
		Node->geoFieldValue = 0;
	}

	//set boundary point as heat source
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		if (edge->IsBoundaryEdge() == true) {
			edge->GetStartPoint()->geoFieldValue = 1;
			edge->GetEndPoint()->geoFieldValue = 1;
			edge->GetStartPoint()->selected = true;
			edge->GetEndPoint()->selected = true;
		}
	}

	//--Step2: Compute the boundary distance field and transfer to node boundary Value
	heatField->runHeatMethod();

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->boundaryValue = Node->geoFieldValue;
	}
	//std::cout<<"-- ToolPath Generation: Finish computing the boundary field value!"<<std::endl<<std::endl;

	/*------------------------------*/
	/*Generate Zigzag distance field*/

	//--Step1: Initialize the geofield value of selection node by PCA direction
	heatField->planeCutSelection(NULL);

	//isoSurface *planeCutSurfaceMesh = new isoSurface(NULL);
	//planeCutSurfaceMesh->planeCutSurfaceMesh_delete(patch, NULL);
	//pGLK->refresh(true);

	//--Step2: Compute zigzag distance field and transfer to node zigzag field value
	heatField->runHeatMethod();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->zigzagValue = Node->geoFieldValue;
		//std::cout << Node->geoFieldValue << std::endl;
	}
	//std::cout<<"-- ToolPath Generation: Finish computing the zigzag field value!"<<std::endl<<std::endl;

	/*Update the mesh color*/
	patch->drawgeoField = true;
	pGLK->refresh(true);

	printf("The field value computing is completed (taking %f s).\n", (double(clock() - time)) / CLOCKS_PER_SEC);

}

void MainWindow::outputSingleToolPath()
{
	PolygenMesh *polygenMesh = NULL;
	bool findToolPath = false;
	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh *mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == TOOL_PATH) {
			polygenMesh = mesh;	break;
		}
	}
	if (polygenMesh == NULL) {
		std::cerr << "Didn't find toolpath mesh!" << std::endl; return;
	}

	for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

		char output_filename[256];
		strcpy(output_filename, "..\\Model\\toolpath\\");
		char filetype[64];
		strcpy(filetype, "01.txt");
		strcat(output_filename, filetype);
		ofstream TP(output_filename);

		for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
			if (!node->resampleChecked) continue;

			double pp[3]; node->GetCoord3D(pp);
			double n[3]; node->GetNormal(n[0], n[1], n[2]);

			//TP << pp[0] << " " << pp[2] << " " << pp[1] << " ";
			//TP << -n[0] << " " << -n[2] << " " << -n[1] << endl;

			TP << pp[0] << " " << pp[1] << " " << pp[2] << " ";
			TP << n[0] << " " << n[1] << " " << n[2] << endl;

		}

		TP.close();
		break;
	}
	std::cout << "Finish output single toolpath" << std::endl;
}

void MainWindow::generateToolPathforMultiAxisPrinting() {

	/*------------------*/
	//detect if the tool path polygenmesh exist
	PolygenMesh *toolPath = NULL; 
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr; ) {
		PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->meshType == TOOL_PATH) { toolPath = polygenMesh; break; }
	}

	if (toolPath != NULL) toolPath->ClearAll();	
	else { //add new polygenmesh inside the system
		toolPath = new PolygenMesh(TOOL_PATH); toolPath->setModelName("BoundaryToolpath");
		toolPath->BuildGLList(toolPath->m_bVertexNormalShading);
		pGLK->AddDisplayObj(toolPath, true); polygenMeshList.AddTail(toolPath);
		updateTree(); }


	/*-----------------*/
	/*Compute parameter*/

	int boundaryNum, zigzagTPathNum, boundaryTPathNum;
	double shrinkOffset, boundaryTPathOffset;

	toolpath_computeParameter(
		ToolPathComp, boundaryNum, zigzagTPathNum, boundaryTPathNum, shrinkOffset, boundaryTPathOffset);

	std::cout << "For the input mesh, computed [boundary# zigzag# shrink_offset] = "
		<< boundaryNum << ", " << zigzagTPathNum << ", " << shrinkOffset << std::endl;

	/*------------------*/
	/*Generate tool path*/
	long time = clock();

	if (boundaryNum <= 3 )
		ToolPathComp->generateBundaryToolPath(toolPath, boundaryTPathNum, boundaryTPathOffset);

	else {
		//****step1: generate zig-zag toolpath
		ToolPathComp->generateZigzagToolPath(toolPath, zigzagTPathNum);

		//****step2: generate boundary toolpath
		ToolPathComp->generateBundaryToolPath(toolPath, boundaryTPathNum, boundaryTPathOffset);
	}

	//****step3: resample all the trajectory
	ToolPathComp->resampling(toolPath);

	printf("Generate toolpath is completed (taking %f s).\n", (double(clock() - time)) / CLOCKS_PER_SEC);

	//build both (reminder to install the edge is connection!)-> reorder -> resample

	//ui->spinBox_regionNum->setMaximum(toolPath->GetMeshList().GetCount());
	for (GLKPOSITION pos = toolPath->GetMeshList().GetHeadPosition(); pos != nullptr; ) {
		QMeshPatch* patch = (QMeshPatch*)toolPath->GetMeshList().GetNext(pos);
		patch->toolPathDraw = true;
	}

	pGLK->refresh(true);

}

void MainWindow::generateToolPath_StressField() {

	if ((PolygenMesh*)polygenMeshList.GetCount() == 0) return;
	QMeshPatch* selectedSurface = (QMeshPatch*) ((PolygenMesh*)polygenMeshList.GetHead())->GetMeshList().GetHead();
	
	heatField = new heatMethodField(selectedSurface);
	ToolPathComp = new toolPathGeneration(selectedSurface);

	PolygenMesh* singleToolpath = this->_buildPolygenMesh(TOOL_PATH, "Layer toolpath");

	heatField->inputStressFieldValue();
	heatField->meshRefinement();

	///* ---- Generate boundary heat field ---- */
	heatField->compBoundaryHeatKernel();

	///* ---- Generate tool path ---- */
	int boundaryNum, zigzagTPathNum, boundaryTPathNum; double shrinkOffset, boundaryTPathOffset;
	toolpath_computeParameter(ToolPathComp, boundaryNum, zigzagTPathNum, boundaryTPathNum, shrinkOffset, boundaryTPathOffset);
	std::cout << "For the input mesh, computed [boundary# zigzag# shrink_offset] = "
		<< boundaryNum << ", " << zigzagTPathNum << ", " << shrinkOffset << std::endl;

	ToolPathComp->generateBundaryToolPath(singleToolpath, boundaryTPathNum, boundaryTPathOffset);
	printf(" -- generate boundary toolpah finished ");

	ToolPathComp->generateZigzagToolPath(singleToolpath, zigzagTPathNum);
	printf(" -- and zigzag toolpah ");

	ToolPathComp->resampling(singleToolpath);
	printf(" -- and resampling\n\n");

	pGLK->refresh(true);
	fileIOObject->outputToolpathRenderingFile(selectedSurface, false);

}


void MainWindow::directToolpathGeneration() {

	/*-------------------------------
	detect model inside system
	-------------------------------*/

	/* -- Initial Tet Model --*/
	PolygenMesh* initialModel = (PolygenMesh*)polygenMeshList.GetHead();
	if (initialModel == NULL || initialModel->meshType != INIT_TET) { printf(" No tet model detected in the system! \n"); return; }

	/* -- Initial Model isoSurface (support layer should also be included) --*/
	PolygenMesh* isosurfaceSet = NULL;
	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh* mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == INIT_LAYERS) { isosurfaceSet = mesh; break; }
	}
	if (isosurfaceSet == NULL)
		isosurfaceSet = this->_buildPolygenMesh(INIT_LAYERS, "isoSurface");
	isosurfaceSet->ClearAll();

	/*-------------------------------
	input splited isosurface
	-------------------------------*/

	std::string folderPath = "..\\Model\\IsoSurface\\" + initialModel->getModelName();
	std::vector<std::string> layersFileSet;
	std::vector<std::string> layersFileSet_support;
	std::vector<std::string> fieldFileSet;

	string stressFieldPath = folderPath + "\\Field";
	_buildFileNameSetbySorting(layersFileSet, folderPath);
	_buildFileNameSetbySorting(layersFileSet_support, folderPath + "\\Support_Trim");
	_buildFileNameSetbySorting(fieldFileSet, folderPath + "\\Field");

	fileIOObject->inputInstalledIsoSurface(
		isosurfaceSet, layersFileSet, fieldFileSet, folderPath);

	if (layersFileSet_support.size() > 0)
		fileIOObject->inputInstalledIsoSurface_support(
			isosurfaceSet, layersFileSet_support, folderPath, initialModel->getModelName());

	//for (GLKPOSITION posMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
	//	QMeshPatch* layer = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(posMesh);
	//	std::cout << layer->layerName << std::endl;
	//}

	ui->IsoLayerIndex->setMaximum(isosurfaceSet->GetMeshList().GetCount());

	int layerNum = isosurfaceSet->GetMeshList().GetCount();
	std::vector<string> layerName(layerNum);
	int layerIndexUpdate = 0;
	for (GLKPOSITION posMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(posMesh);
		layer->SetIndexNo(layerIndexUpdate); layerIndexUpdate++;
		layerName[layer->GetIndexNo()] = layer->layerName;
	}

	std::vector<PolygenMesh*> PolygenMeshSet(layerNum);
	for (int i = 0; i < layerNum; i++)
		PolygenMeshSet[i] = this->_buildPolygenMesh(TOOL_PATH, layerName[i]);
	string toolpath_dir = folderPath + "\\toolpath\\";

	long time = clock();

	int Core = 12;
#pragma omp parallel
	{
#pragma omp for  
		for (int omptime = 0; omptime < Core; omptime++) {
		
			for (GLKPOSITION posMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch* layer = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(posMesh);

				if (layer->GetIndexNo() % Core != omptime) continue;

				//if (layer->layerName != "5100") continue;

				//if (layer->includeSupportRegion == false) continue;
				std::cout << " ***** ----- Begin Process layer : " <<  layer->layerName << std::endl;

				PolygenMesh* singleToolpath = PolygenMeshSet[layer->GetIndexNo()];

				GuidanceField* GuideFieldComp_layer = new GuidanceField(layer);

				if (layer->includeSupportRegion == false) {
					GuideFieldComp_layer->scalarFieldCompute_isoSurface(layer);

				}
				else GuideFieldComp_layer->scalarFieldCompute_supportSurface(layer);
				delete GuideFieldComp_layer;

				//if (layer->layerName == "5100") {
				//	fileIOObject->outputToolpathRenderingFile(layer, false);
				//	fileIOObject->outputToolpathRenderingFile_field(layer);
				//}


				///* ---- Generate boundary heat field ---- */
				heatMethodField* heatField_layer = new heatMethodField(layer);
				heatField_layer->meshRefinement();
				heatField_layer->compBoundaryHeatKernel();
				delete heatField_layer;

				/*if (layer->layerName == "5100") 
					fileIOObject->outputToolpathRenderingFile(layer, true);*/
				
				toolPathGeneration* ToolPathComp_layer = new toolPathGeneration(layer);

				///* ---- Generate tool path ---- */
				int boundaryNum, zigzagTPathNum, boundaryTPathNum; double shrinkOffset, boundaryTPathOffset;
				toolpath_computeParameter(ToolPathComp_layer, boundaryNum, zigzagTPathNum, boundaryTPathNum, shrinkOffset, boundaryTPathOffset);
				std::cout << "For the input mesh, computed [boundary# zigzag# shrink_offset] = "
					<< boundaryNum << ", " << zigzagTPathNum << ", " << shrinkOffset << std::endl;

				if (layer->includeSupportRegion) {
					int layerIndex = stoi(layer->layerName.substr(0, layer->layerName.length() - 3));
					if (layerIndex % 2 == 0) {
						zigzagTPathNum /= 2;
						ToolPathComp_layer->maxConnectDist *= 2;
					}
					ToolPathComp_layer->toolpathOffset = 0.9;			
				}
				else {
					ToolPathComp_layer->toolpathOffset = 0.8;
				}

				ToolPathComp_layer->generateBundaryToolPath(singleToolpath, boundaryTPathNum, boundaryTPathOffset);

				ToolPathComp_layer->generateZigzagToolPath(singleToolpath, zigzagTPathNum);

				ToolPathComp_layer->resampling(singleToolpath);
				delete ToolPathComp_layer;

				// output toolpath
			
				char toolpathname[256];
				sprintf(toolpathname, "%s%s%s", toolpath_dir.data(), layer->layerName.data(), ".txt");
				std::cout << " ***** ----- Finish Process this layer, output: " << toolpathname << std::endl << std::endl;

				ofstream TP(toolpathname);

				for (GLKPOSITION posMesh = singleToolpath->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
					QMeshPatch* patch = (QMeshPatch*)singleToolpath->GetMeshList().GetNext(posMesh);
					for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
						QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
						if (!node->resampleChecked) continue;
						double pp[3]; node->GetCoord3D(pp); double n[3]; node->GetNormal(n[0], n[1], n[2]);

						TP << pp[0] << " " << pp[1] << " " << pp[2] << " ";
						TP << n[0] << " " << n[1] << " " << n[2] << endl;

					}
				}
				TP.close();
			}
		}
	}

	printf("computing toolpath taking %ld ms.\n", clock() - time);

	pGLK->refresh(true);

}

PolygenMesh* MainWindow::_buildPolygenMesh(mesh_type type, std::string name) {

	PolygenMesh* newMesh = new PolygenMesh(type);
	newMesh->setModelName(name);
	newMesh->BuildGLList(newMesh->m_bVertexNormalShading);
	pGLK->AddDisplayObj(newMesh, true); 
	polygenMeshList.AddTail(newMesh);
	updateTree();
	return newMesh;

}

PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type) {

	PolygenMesh* detectedMesh = NULL;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (thispolygenMesh->meshType == type) { 
			detectedMesh = thispolygenMesh; break; }
	}
	return detectedMesh;

}


//Checking if the mesh quality is good for toolpath generation, meanwhile build the mesh type
void MainWindow::generateToolPathforAllLayer_meshChecking() {

	std::vector<std::string> files;
	this->toolpath_inputCurveLayer_memoryReduce(files);

	// detect layer type
	std::vector<int> layerType(files.size());
	std::vector<int> initlayerNum;
	std::vector<int> supportlayerNum;


	for (int i = 0; i < files.size(); i++) {
		layerType[i] = -1;

		string supportFlag = files[i].substr(files[i].length() - 5, 1);

		if (supportFlag == "S") {
			string layerflag = files[i].substr(0, files[i].length() - 7);
			int layerNum = std::stoi(layerflag);
			bool exist = false;
			for (int j = 0; j < supportlayerNum.size(); j++) {
				if (layerNum == supportlayerNum[j]) {
					exist = true; break;
				}
			}
			if (exist == false) supportlayerNum.push_back(layerNum);
			layerType[i] = supportlayerNum.size() % 2 - 2; // -2 or -1		

		}
		else
		{
			string layerflag = files[i].substr(0, files[i].length() - 6);

			int layerNum = std::stoi(layerflag);
			bool exist = false;
			for (int j = 0; j < initlayerNum.size(); j++) {
				if (layerNum == initlayerNum[j]) {
					exist = true; break;
				}
			}
			if (exist == false) initlayerNum.push_back(layerNum);
			layerType[i] = initlayerNum.size() % 2;
		}
	}

	/*-----------------------------------
	first input all layer into the system
	------------------------------------*/

	PolygenMesh * sliceSet = new PolygenMesh(INIT_LAYERS);
	sliceSet->BuildGLList(sliceSet->m_bVertexNormalShading); sliceSet->setModelName("printingLayer");
	polygenMeshList.AddTail(sliceSet); pGLK->AddDisplayObj(sliceSet, true); updateTree();
	printf("\n------- ToolPath Generation: Sliceset import successfully.\n\n");

	for (int i = 0; i < files.size(); i++) {

		char filename[1024]; sprintf(filename, "%s%s", "..\\Model\\toolpath\\remeshed_layer\\", files[i].data());

		//QMeshPatch* layer = new QMeshPatch; layer->inputOFFFile(filename, false);
		QMeshPatch* layer = new QMeshPatch; layer->inputOBJFile(filename, false);
		layer->layerName = files[i].substr(0, files[i].length() - 4);
		layer->layerToolpathType = layerType[i]; // update the layer type
		layer->drawThisIsoLayer = true;
		sliceSet->GetMeshList().AddTail(layer);
		layer->SetIndexNo(i);
	}
	std::cout << "/******Finish input all the layers******/" << std::endl << std::endl;
	
	//PolygenMesh* sliceSet = this->inputGeneratedCurveLayer_toolpath();


	/*-----------------------------------
	CPU parallel computing - heat method
	------------------------------------*/
	int layerNum = sliceSet->GetMeshList().GetCount();
	std::cout << "CPU acceration: in total " << layerNum << " layer to compute." << std::endl << std::endl;

	int Core = 12;
	int EachCore = layerNum / Core + 1;

#pragma omp parallel
	{
#pragma omp for  
		for (int omptime = 0; omptime < Core; omptime++) {
			allLayerToolPathHeatMethodCompute(sliceSet, true, omptime, EachCore);
		}
	}
	std::cout << "/******Finish compute the heat field for all layer******/" << std::endl << std::endl;

	//check if all mesh quality is good
	for (GLKPOSITION posMesh = sliceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)sliceSet->GetMeshList().GetNext(posMesh);

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			if (Node->geoFieldValue != Node->geoFieldValue) {
				std::cout << layer->GetIndexNo() << " ERROR, remesh!" << std::endl;
				break;
			}
		}

	}

	pGLK->refresh(true);
}

//*-----multiple layer toolpath generation-----

void MainWindow::generateToolPathforAllLayer_memoryReduce() {

	std::vector<std::string> files;
	this->toolpath_inputCurveLayer_memoryReduce(files);

	// detect layer type
	std::vector<int> layerType(files.size()); 
	std::vector<int> initlayerNum;
	std::vector<int> supportlayerNum;


	for (int i = 0; i < files.size(); i++) {
		layerType[i] = -1;

		string supportFlag = files[i].substr(files[i].length() - 5, 1);

		if (supportFlag == "S") {
			string layerflag = files[i].substr(0, files[i].length() - 7);
			int layerNum = std::stoi(layerflag);
			bool exist = false;
			for (int j = 0; j < supportlayerNum.size(); j++) {
				if (layerNum == supportlayerNum[j]) {
					exist = true; break;
				}
			}
			if (exist == false) supportlayerNum.push_back(layerNum);
			layerType[i] = supportlayerNum.size() % 2 - 2; // -2 or -1		
		
		}
		else
		{
			string layerflag = files[i].substr(0, files[i].length() - 6);

			int layerNum = std::stoi(layerflag);
			bool exist = false;
			for (int j = 0; j < initlayerNum.size(); j++) {
				if (layerNum == initlayerNum[j]) {
					exist = true; break;
				}
			}
			if (exist == false) initlayerNum.push_back(layerNum);
			layerType[i] = initlayerNum.size() % 2;
		}
	}

	// for (int i = 0; i < files.size(); i++) std::cout << files[i] << ", layerType = " << layerType[i] << std::endl;

	if (ui->checkBox_cpuSpeedUpToolpathGeneration->isChecked()) {
#pragma omp parallel
		{
#pragma omp for  
			for (int i = 0; i < files.size(); i++) {

				char filename[1024]; sprintf(filename, "%s%s", "..\\Model\\toolpath\\remeshed_layer\\", files[i].data());

				//QMeshPatch* layer = new QMeshPatch; layer->inputOFFFile(filename, false);
				QMeshPatch* layer = new QMeshPatch; layer->inputOBJFile(filename, false);
				layer->layerName = files[i].substr(0, files[i].length() - 4);
				layer->layerToolpathType = layerType[i]; // update the layer type

				this->toolpath_fieldCompSingleLayer_memoryReduce(layer);

				this->toolpath_waypointGenerate_memoryReduce(layer);

				std::cout << "Finish the tool path computing for layer: " << files[i] << std::endl << std::endl;

				layer->ClearAll();
				delete layer;

			}
		}
	}
	else {
		for (int i = 0; i < files.size(); i++) {

			char filename[1024]; sprintf(filename, "%s%s", "..\\Model\\toolpath\\remeshed_layer\\", files[i].data());

			//QMeshPatch* layer = new QMeshPatch; layer->inputOFFFile(filename, false);
			QMeshPatch* layer = new QMeshPatch; layer->inputOBJFile(filename, false);

			layer->layerName = files[i].substr(0, files[i].length() - 4);
			layer->layerToolpathType = layerType[i]; // update the layer type

			this->toolpath_fieldCompSingleLayer_memoryReduce(layer);

			this->toolpath_waypointGenerate_memoryReduce(layer);

			std::cout << "Finish the tool path computing for layer: " << files[i] << std::endl << std::endl;

			layer->ClearAll();
			delete layer;

		}
	}


	updateTree();
	pGLK->refresh(true);
}

void MainWindow::toolpath_inputCurveLayer_memoryReduce(std::vector<std::string>& files) {

	printf("----- Start import all layers for toolpath generation\n\n");
	DIR* dp;
	struct dirent* ep;
	//std::vector<std::string> files;
	dp = opendir("..\\Model\\toolpath\\remeshed_layer");

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			//cout << ep->d_name << endl;
			if ((string(ep->d_name) != ".") && (string(ep->d_name) != ".."))
			{
				//if (string(ep->d_name).find(".off") == string::npos) continue;
				if (string(ep->d_name).find(".obj") == string::npos) continue;
				files.push_back(string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	cout << "There are " << files.size() << " files in the current directory." << endl;
	sort(files.begin(), files.end(), doj::alphanum_less<std::string>());

}

void MainWindow::toolpath_fieldCompSingleLayer_memoryReduce(QMeshPatch* layer) {

	heatMethodField* layerHeatField = new heatMethodField(layer);

	/*--------------------------------*/
	/*Generate boundary distance field*/

	//--Setp1: Initialize the geofield value for boundary node
	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
		Node->selected = false;
		Node->geoFieldValue = 0;
	}

	//set boundary point as heat source
	for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *edge = (QMeshEdge*)layer->GetEdgeList().GetNext(Pos);
		if (edge->IsBoundaryEdge() == true) {
			edge->GetStartPoint()->geoFieldValue = 1;
			edge->GetEndPoint()->geoFieldValue = 1;
			edge->GetStartPoint()->selected = true;
			edge->GetEndPoint()->selected = true;
		}
	}

	//--Step2: Compute the boundary distance field and transfer to node boundary Value
	layerHeatField->runHeatMethod();

	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
		Node->boundaryValue = Node->geoFieldValue;
	}

	/*------------------------------*/
	/*Generate Zigzag distance field*/

	// -- plane cut direction -- //
	double direction[3] = { 0.0 };

	//cross-section infill

	if (layer->layerToolpathType == -2) { direction[0] = 2.0; direction[2] = 1.0; } // --support
	else if(layer->layerToolpathType == -1) { direction[0] = -1.0; direction[2] = 2.0; } // --support
	else if (layer->layerToolpathType == 0) { direction[0] = 1.0; direction[2] = 1.0; } // --initial 
	else if (layer->layerToolpathType == 1) { direction[0] = 1.0; direction[2] = -1.0; } // --initial

	std::cout << layer->layerName << "," << layer->layerToolpathType << "," 
		<< direction[0] << "," << direction[1] << "," << direction[2] << std::endl;
	
	layerHeatField->planeCutSelection(direction);

	//--Step2: Compute zigzag distance field and transfer to node zigzag field value
	layerHeatField->runHeatMethod();
	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
		Node->zigzagValue = Node->geoFieldValue;
	}

	delete layerHeatField;

}

void MainWindow::toolpath_waypointGenerate_memoryReduce(QMeshPatch* layer) {

	//std::cout << layer->GetIndexNo() << endl << endl;

	toolPathGeneration *layerToolPathComp = new toolPathGeneration(layer);

	/*-----------------*/
	/*Compute parameter*/

	int boundaryNum, zigzagTPathNum, boundaryTPathNum;
	double shrinkOffset, boundaryTPathOffset;
	this->toolpath_computeParameter(
		layerToolPathComp, boundaryNum, zigzagTPathNum, boundaryTPathNum, shrinkOffset, boundaryTPathOffset);
	
	//make the support layer more sparse
	int scaleNum = 2;
	//if (layer->layerToolpathType == -1 && layer->layerToolpathType == -2 ) {
	if (layer->layerToolpathType == -2) {

		zigzagTPathNum /= scaleNum;
		layerToolPathComp->maxConnectDist *= scaleNum;
		layerToolPathComp->toolpathOffset = 0.9;
	}
	else {
		layerToolPathComp->toolpathOffset = 0.8;
	}

	std::cout << "For layer " << layer->layerName << ", the [boundary# zigzag# shrink_offset] = "
		<< boundaryNum << ", " << zigzagTPathNum << ", " << shrinkOffset << std::endl;

	PolygenMesh *toolPath = new PolygenMesh(TOOL_PATH);
	toolPath->setModelName(layer->layerName);
	toolPath->BuildGLList(toolPath->m_bVertexNormalShading);
	pGLK->AddDisplayObj(toolPath, true);
	polygenMeshList.AddTail(toolPath);

	/*------------------*/
	/*Generate tool path*/

	if (boundaryNum <= 4) {
		layerToolPathComp->generateBundaryToolPath(toolPath, boundaryTPathNum, boundaryTPathOffset);
	}
	else {
		std::cout << "normal case!" << std::endl;
		//****step2: generate boundary toolpath
		layerToolPathComp->generateBundaryToolPath(toolPath, boundaryTPathNum, boundaryTPathOffset);

		//****step1: generate zig-zag toolpath
		layerToolPathComp->generateZigzagToolPath(toolPath, zigzagTPathNum);

	}

	//****step3: resample all the trajectory
	layerToolPathComp->resampling(toolPath);

	//build both (reminder to install the edge is connection!)-> reorder -> resample

	//ui->spinBox_regionNum->setMaximum(toolPath->GetMeshList().GetCount());
	for (GLKPOSITION pos = toolPath->GetMeshList().GetHeadPosition(); pos != nullptr; ) {
		QMeshPatch* patch = (QMeshPatch*)toolPath->GetMeshList().GetNext(pos);
		patch->toolPathDraw = true;
	}

	//output toolpath
	char output_filename[256];
	sprintf(output_filename, "%s%s%s", "..\\Model\\toolpath\\waypoint\\", layer->layerName, ".txt");

	ofstream TP(output_filename);

	for (GLKPOSITION posMesh = toolPath->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *patch = (QMeshPatch*)toolPath->GetMeshList().GetNext(posMesh);

		for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
			if (!node->resampleChecked) continue;
			double pp[3]; node->GetCoord3D(pp); double n[3]; node->GetNormal(n[0], n[1], n[2]);

			TP << pp[0] << " " << pp[1] << " " << pp[2] << " ";
			TP << n[0] << " " << n[1] << " " << n[2] << endl;

		}
		//break;
	}
	TP.close();

	delete layerToolPathComp;

}

void MainWindow::toolpath_computeParameter(toolPathGeneration *layerToolPathComp,
	int& boundaryNum, int& zigzagTPathNum, int& boundaryTPathNum,
	double& shrinkOffset, double& boundaryTPathOffset) {

	/*-----------------------
	get parameter from system
	-----------------------*/

	layerToolPathComp->toolpathOffset = ui->doubleSpinBox_toolpathOffsetValue->value(); //offset value for the tool-path
	layerToolPathComp->boundaryGapDist = ui->doubleSpinBox_boundShrinkValue->value();
	layerToolPathComp->maxConnectDist = ui->doubleSpinBox_zigzagBoundConnectDistance->value();
	layerToolPathComp->maxGapDist = ui->doubleSpinBox_maxGapDistance->value();

	boundaryTPathNum = ui->spinBox_bToolPath_Num->value();

	/*--------------------------------------------------
	auto compute few parameter (based on toolpathOffset)
	--------------------------------------------------*/

	zigzagTPathNum = layerToolPathComp->autoComputeTPathNum(true);
	boundaryNum = layerToolPathComp->autoComputeTPathNum(false);

	/*if (boundaryNum == 0) {
		boundaryNum = 1; boundaryTPathOffset = 1;
	}*/

	//for the model contains narrow region, some special case should be avoided!
	boundaryTPathOffset = 1.0 / boundaryNum;

	if (boundaryNum <= 3) {
		//boundaryNum += 1;
		boundaryNum = 3; boundaryTPathNum = 3; 	boundaryTPathOffset = 1.0 / boundaryNum;

	}
		
	else if (boundaryNum <= boundaryTPathNum + 1) { boundaryNum += 3; boundaryTPathNum = 3; }
	//boundaryTPathOffset = 1.0 / boundaryNum;

	//if (boundaryNum <= boundaryTPathNum) boundaryNum += 3;

	//for the model with large gap, the shrink offset can be auto computed by this function
	/*double shrinkOffset = layerToolPathComp->autoComputeZigZagTPathOffestingValue(
	zigzagTPathNum, boundaryTPathNum, boundaryTPathOffset);*/
	shrinkOffset = 0.0;

	layerToolPathComp->minBoundaryFieldValue = 1 - (boundaryTPathNum + shrinkOffset) * boundaryTPathOffset;
}


void MainWindow::allLayerToolPathHeatMethodCompute(PolygenMesh* printLayerSet,
	bool accelerate, int omptime, int EachCore) {

	for (GLKPOSITION posMesh = printLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch *layer = (QMeshPatch*)printLayerSet->GetMeshList().GetNext(posMesh);

		if (accelerate) {
			if (layer->GetIndexNo() < omptime*EachCore) continue;
			else if (layer->GetIndexNo() > (1 + omptime)*EachCore) break;
		}

		heatMethodField* layerHeatField = new heatMethodField(layer);

		/*--------------------------------*/
		/*Generate boundary distance field*/

		//--Setp1: Initialize the geofield value for boundary node
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->selected = false;
			Node->geoFieldValue = 0;
		}

		//set boundary point as heat source
		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge *edge = (QMeshEdge*)layer->GetEdgeList().GetNext(Pos);
			if (edge->IsBoundaryEdge() == true) {
				edge->GetStartPoint()->geoFieldValue = 1;
				edge->GetEndPoint()->geoFieldValue = 1;
				edge->GetStartPoint()->selected = true;
				edge->GetEndPoint()->selected = true;
			}
		}

		//--Step2: Compute the boundary distance field and transfer to node boundary Value
		layerHeatField->runHeatMethod();

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->boundaryValue = Node->geoFieldValue;
		}

		/*------------------------------*/
		/*Generate Zigzag distance field*/

		//--Step1: Initialize the geofield value of selection node by PCA direction
		if (layer->GetIndexNo() % 2 == 0) {
			double norm[3] = { 1.0,0.0,1.0 };
			layerHeatField->planeCutSelection(norm);
		}
		else {
			double norm[3] = { 1.0,0.0,-1.0 };
			layerHeatField->planeCutSelection(norm);
		}
		//layerHeatField->planeCutSelection(NULL);


		//--Step2: Compute zigzag distance field and transfer to node zigzag field value
		layerHeatField->runHeatMethod();
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode *Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
			Node->zigzagValue = Node->geoFieldValue;
		}
		//std::cout<<"-- ToolPath Generation: Finish computing the zigzag field value!"<<std::endl<<std::endl;

		/*Update the mesh color*/
		layer->drawgeoField = true;

		delete layerHeatField;
		std::cout << "finish compute the heat field for Layer # " << layer->layerName << endl << endl;
	}
}




PolygenMesh* MainWindow::inputGeneratedCurveLayer_toolpath() {

	/**********************************************************
	Function written by Tianyu Zhang

	*Function : 1. Load .obj files into (PolygenMesh*)sliceSet
	*           2. Sort the .obj files with nature order
	*           3. Display all of .obj
	*Input    : .obj/.off files in "../Model/*.obj" (path)
	*Output   : Patches in (PolygenMesh*)sliceSet
	***********************************************************/

	printf("----- Start import all layers for toolpath generation\n\n");
	DIR* dp;
	struct dirent* ep;
	vector<string> files;
	dp = opendir("..\\Model\\toolpath\\remeshed_layer");

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			//cout << ep->d_name << endl;
			if ((string(ep->d_name) != ".") && (string(ep->d_name) != ".."))
			{
				files.push_back(string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	cout << "There are " << files.size() << " files in the current directory." << endl;

	PolygenMesh* sliceSet = new PolygenMesh(INIT_LAYERS);
	sliceSet->setModelName("printingLayer");

	//-------------------------------------
	//read slice files and build mesh_patches
	sort(files.begin(), files.end(), doj::alphanum_less<std::string>());

	char filename[1024];
	for (int i = 0; i < files.size(); i++)
	{
		sprintf(filename, "%s%s", "..\\Model\\toolpath\\remeshed_layer\\", files[i].data());
		cout << "Input path file / name: " << filename << ", ";

		//if (files[i].find(".off") == string::npos) continue;
		if (files[i].find(".obj") == string::npos) continue;

		QMeshPatch* slice = new QMeshPatch;
		slice->SetIndexNo(sliceSet->GetMeshList().GetCount()); //index begin from 0
		slice->layerName = files[i];
		sliceSet->GetMeshList().AddTail(slice);
		slice->inputOBJFile(filename, false); std::cout << endl;
		//slice->inputOFFFile(filename, false);  std::cout << endl;
		slice->drawThisIsoLayer = true;

	}
	// Display
	sliceSet->BuildGLList(sliceSet->m_bVertexNormalShading);
	polygenMeshList.AddTail(sliceSet);
	pGLK->AddDisplayObj(sliceSet, true);
	updateTree();
	printf("\n------- ToolPath Generation: Sliceset import successfully.\n\n");

	pGLK->refresh(true);

	return sliceSet;
}


//--this function is used for substraction tetrahdeal model from a closed surface mesh
void MainWindow::OnMeshoperationSubstraction()
{
	PolygenMesh* polygenMesh1;
	PolygenMesh* polygenMesh2;

	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygenMesh->meshType == INIT_TET) polygenMesh1 = polygenMesh;
		else polygenMesh2 = polygenMesh;
	}

	QMeshPatch* tetPatch = (QMeshPatch*)polygenMesh1->GetMeshList().GetHead();
	QMeshPatch* innerPatch = (QMeshPatch*)polygenMesh2->GetMeshList().GetHead();

	//build PQP
	QMeshFace** aFaces = new QMeshFace * [innerPatch->GetFaceNumber()];

	PQP_Model* PQPmodel = new PQP_Model;
	PQPmodel->BeginModel();
	for (GLKPOSITION pos3 = innerPatch->GetFaceList().GetHeadPosition(); pos3 != NULL; ) {
		QMeshFace* pFacet = (QMeshFace*)innerPatch->GetFaceList().GetNext(pos3);
		//if (pFacet->i_inner) continue;
		double v1[3], v2[3], v3[3];
		pFacet->GetNodeRecordPtr(0)->GetCoord3D(v1[0], v1[1], v1[2]);
		pFacet->GetNodeRecordPtr(1)->GetCoord3D(v2[0], v2[1], v2[2]);
		pFacet->GetNodeRecordPtr(2)->GetCoord3D(v3[0], v3[1], v3[2]);
		PQPmodel->AddTri(v1, v2, v3, pFacet->GetIndexNo());
		aFaces[pFacet->GetIndexNo() - 1] = pFacet;
		pFacet->CalPlaneEquation();
		pFacet->CalCenterPos();
	}
	PQPmodel->EndModel();

	printf("finish Create PQP\n");

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != NULL;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		//initialization tetra
		tetra->m_nIdentifiedPatchIndex = -1;
		double pp[3];
		tetra->CalCenterPos(pp[0], pp[1], pp[2]);
		PQP_DistanceResult dres; dres.last_tri = PQPmodel->last_tri;
		PQP_Distance(&dres, PQPmodel, pp, 0.0, 0.0);
		int tri_id = dres.last_tri->id - 1;
		QMeshFace* face = aFaces[tri_id];
		double v[3], D;
		face->GetPlaneEquation(v[0], v[1], v[2], D);
		double pp1[3];
		//face->GetCenterPos(pp1[0], pp1[1], pp1[2]);
		face->CalCenterPos(pp1[0], pp1[1], pp1[2]);

		for (int i = 0; i < 3; i++)
			pp[i] -= pp1[i];
		double dProduct = DOT(pp, v);
		if (dProduct < 0) {
			tetra->m_nIdentifiedPatchIndex = 1;
		}
	}

	delete[] aFaces;

	printf("finished PQP search\n");

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != NULL;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (tetra->m_nIdentifiedPatchIndex != 1) continue;
		bool in[4] = { 0 };
		for (int i = 1; i <= 4; i++) {
			QMeshFace* face = tetra->GetFaceRecordPtr(i);
			QMeshTetra* oTetra = face->GetLeftTetra();
			if (oTetra == tetra) oTetra = face->GetRightTetra();
			if (!oTetra) { tetra->m_nIdentifiedPatchIndex = 0; break; }
			if (oTetra->m_nIdentifiedPatchIndex != 1) in[i - 1] = true;
		}
		if (in[0] && in[1] && in[2] && in[3]) tetra->m_nIdentifiedPatchIndex = 0;
	}

	int count = 0;
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != NULL;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (tetra->m_nIdentifiedPatchIndex != 1) continue;
		tetPatch->deleteTetra(tetra);
		count++;
	}
	printf("remove tetras %d\n", count);

	for (GLKPOSITION Pos = tetPatch->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* edge = (QMeshEdge*)(tetPatch->GetEdgeList().GetNext(Pos));
		edge->inner = true;
	}
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos != NULL;) {
		QMeshFace* face = (QMeshFace*)(tetPatch->GetFaceList().GetNext(Pos));
		face->CalPlaneEquation();
		face->CalCenterPos();
		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL) {
			for (int i = 0; i < 3; i++) face->GetEdgeRecordPtr(i + 1)->inner = false;
		}
	}

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos != NULL;) {
		QMeshNode* node = (QMeshNode*)(tetPatch->GetNodeList().GetNext(Pos));
		if (node->i_inner) continue;
		node->CalNormal();
	}


	pGLK->refresh(true);

	//export tet mesh
	std::string path = "../Model/bunnyhollow.tet";

	double pp[3];

	//make sure the inner / outer is detect 
	for (GLKPOSITION posNode = tetPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(posNode);
		node->inner = true;
	}
	for (GLKPOSITION posFace = tetPatch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(posFace);
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
	for (GLKPOSITION posNode = tetPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(posNode);
		if (!node->inner) {
			node->outerIndex = outerNum;
			outerNum++;
		}
	}

	//----tet file output
	ofstream nodeSelection(path);
	nodeSelection << tetPatch->GetNodeNumber() << " vertices" << endl;
	nodeSelection << tetPatch->GetTetraNumber() << " tets" << endl;

	int index = 0;
	for (GLKPOSITION posNode = tetPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(posNode);
		node->SetIndexNo(index); index++;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		nodeSelection << pp[0] << " " << pp[1] << " " << pp[2] << endl;
	}
	for (GLKPOSITION posFace = tetPatch->GetTetraList().GetHeadPosition(); posFace != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(posFace);
		nodeSelection << "4 " << tet->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(2)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(3)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(4)->GetIndexNo() << endl;
	}
	nodeSelection.close();

}


void MainWindow::shiftToOrigin()
{
	PolygenMesh* polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	// shift along with the selected model
	double center[3] = { 0.0,0.0,0.0 };
	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		double xyz[3];
		node->GetCoord3D(xyz[0], xyz[1], xyz[2]);
		for (int i = 0; i < 3; i++)
			center[i] += xyz[i];
	}
	for (int i = 0; i < 3; i++)
		center[i] /= (double)patch->GetNodeNumber();

	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
		for (GLKPOSITION posMesh = polygen->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* meshPatch = (QMeshPatch*)polygen->GetMeshList().GetNext(posMesh);
			for (GLKPOSITION posNode = meshPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
				QMeshNode* node = (QMeshNode*)meshPatch->GetNodeList().GetNext(posNode);
				double xx, yy, zz;
				node->GetCoord3D(xx, yy, zz);
				node->SetCoord3D(xx - center[0], yy - center[1], zz - center[2]);
			}
		}
	}
	pGLK->refresh(true);
}

void MainWindow::scalarModelSize() {
	PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	//---------------------------------//
	// -- scalar the model

	/*double center[3] = { 0 };
	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		double pp[3];  node->GetCoord3D(pp);
		for (int i = 0; i<3; i++) center[i] += pp[i];
	}
	for (int i = 0; i<3; i++) center[i] /= (double) patch->GetNodeNumber();

	double minY = 99999.999;
	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		double pp[3];  node->GetCoord3D(pp); if (minY > pp[1]) minY = pp[1];
	}

	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		double pp[3];  node->GetCoord3D(pp);
			pp[0] = (pp[0] - center[0])*ui->doubleSpinBox_modelScaleFactor->value() + center[0];
			pp[2] = (pp[2] - center[2]) * ui->doubleSpinBox_modelScaleFactor->value() + center[2];
			pp[1] = (pp[1] - minY) * ui->doubleSpinBox_modelScaleFactor->value() + minY;

		node->SetCoord3D(pp[0], pp[1], pp[2]);
	}*/

	//---------------------------------//
	// -- rotate the model (in 2D)

	/*double theta = 0.0174532922222*(-295);

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		double pp[3];
		Node->GetCoord3D_last(pp[0], pp[1], pp[2]);
		double xnew = pp[0] * cos(theta) - pp[1] * sin(theta);
		double ynew = pp[0] * sin(theta) + pp[1] * cos(theta);
		Node->SetCoord3D(xnew, ynew, pp[2]);
	}
	pGLK->refresh(true);*/

	//---------------------------------//
	// -- rotate the model (in 3D)

	double theta = DEGREE_TO_ROTATE(230.0);
	double beta = DEGREE_TO_ROTATE(130.0);
	Eigen::Vector3d rotateDir = { cos(theta) * sin(beta), cos(beta),sin(theta) * sin(beta) };
	Eigen::Matrix3d rotationMatrix; Eigen::Vector3d initNorm = { 0, 1.0, 0 };
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(rotateDir, initNorm);

	for (GLKPOSITION posMesh = patch->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(posMesh);
		Eigen::Vector3d pp; node->GetCoord3D(pp(0), pp(1), pp(2));
		Eigen::Vector3d rotatedpp = rotationMatrix * pp;
		node->SetCoord3D(rotatedpp(0), rotatedpp(1), rotatedpp(2));
	}

	double lowerY = 99999.999;
	for (GLKPOSITION posMesh = patch->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(posMesh);
		Eigen::Vector3d pp; node->GetCoord3D(pp(0), pp(1), pp(2));
		if (lowerY > pp(1)); lowerY = pp(1);
	}

	for (GLKPOSITION posMesh = patch->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(posMesh);
		Eigen::Vector3d pp; node->GetCoord3D(pp(0), pp(1), pp(2));
		node->SetCoord3D(pp(0), pp(1) - lowerY + 20, pp(2));
	}

	//---------------------------------//

	std::string path = "../Model/scaledMesh.tet";
	ofstream tet_output(path);
	tet_output << patch->GetNodeNumber() << " vertices" << endl;
	tet_output << patch->GetTetraNumber() << " tets" << endl;

	int index = 0; double pp[3];
	for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
		node->SetIndexNo(index); index++;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		tet_output << pp[0] << " " << pp[1] << " " << pp[2] << endl;
	}
	for (GLKPOSITION posFace = patch->GetTetraList().GetHeadPosition(); posFace != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)patch->GetTetraList().GetNext(posFace);
		tet_output << "4 " << tet->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(2)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(3)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(4)->GetIndexNo() << endl;
	}
	tet_output.close();

	pGLK->refresh(true);

}
