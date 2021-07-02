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
	createMainFunctionActions();
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

	connect(ui->actionSaveSelection, SIGNAL(triggered(bool)), this, SLOT(saveSelection()));
	connect(ui->actionReadSelection, SIGNAL(triggered(bool)), this, SLOT(readSelection()));

	/* function for generate "inp" file as abaqus FEM input*/
	connect(ui->actionExport2Abaqus, SIGNAL(triggered(bool)), this, SLOT(exportMeshtoAbaqus()));
	/* input FEM result and pre-process the system */
	connect(ui->pushButton_changeTensileandCompressRegion, SIGNAL(released()), this, SLOT(changeTensileandCompressRegion()));

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

	/* Compute field for slicing */

	//-------Display function-------//
	
	connect(ui->IsoLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(changeIsoLayerDisplay()));
	connect(ui->checkBox_spaseVectorField, SIGNAL(released()), this, SLOT(drawSparseVectorField()));

	/*ISO-surface Generation directly from tetrahedral mesh*/
	connect(ui->pushButton_viewallLayerandOffset, SIGNAL(released()), this, SLOT(viewAllIsoLayerandOffsetDisplay()));
	connect(ui->pushButton_changeOrder, SIGNAL(released()), this, SLOT(flipISOSurfaceOrder()));

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
	PolygenMesh* polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh) polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (polygenMesh->meshType != INIT_TET) { QMessageBox::information(this, "Error", "Input is not a volume mesh!\n"); return; }

	fileIOObject->exportMeshtoAbaqusFEM(polygenMesh);
	// QMessageBox::information(this, "Information", "Finish output Abaqus file in same folder.\n");
}


void MainWindow::mouseMoveEvent(QMouseEvent* event)
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
		InteractiveTool* tool;
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

void MainWindow::dragEnterEvent(QDragEnterEvent* event)
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

void MainWindow::dropEvent(QDropEvent* event)
{
	QString filenameStr;
	foreach(const QUrl & url, event->mimeData()->urls())
		filenameStr = url.toLocalFile();
	QByteArray filenameArray = filenameStr.toLatin1();
	char* filename = filenameArray.data();

	PolygenMesh* polygenMesh = new PolygenMesh(UNDEFINED);

	// set polygen name
	std::string strFilename(filename);
	std::size_t foundStart = strFilename.find_last_of("/");
	std::size_t foundEnd = strFilename.find_last_of(".");
	std::string modelName;
	modelName = strFilename.substr(0, foundEnd);
	modelName = modelName.substr(foundStart + 1);
	int i = 0;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
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
		PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		QString modelName = QString::fromStdString(polygenMesh->getModelName());
		QStandardItem* modelListItem = new QStandardItem(modelName);
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
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < column; ++j) {
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

PolygenMesh* MainWindow::getSelectedPolygenMesh()
{
	if (!treeModel->hasChildren())
		return nullptr;
	QModelIndex index = ui->treeView->currentIndex();
	QString selectedModelName = index.data(Qt::DisplayRole).toString();
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		QString modelName = QString::fromStdString(polygenMesh->getModelName());
		if (QString::compare(selectedModelName, modelName) == 0)
			return polygenMesh;
	}
	return nullptr;
}

void MainWindow::on_pushButton_clearAll_clicked()
{
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		pGLK->DelDisplayObj(polygenMesh);
		/*if(polygenMesh->pglkIndex == 1) pGLK->DelDisplayObj(polygenMesh);
		else if (polygenMesh->pglkIndex == 2) pGLK1->DelDisplayObj(polygenMesh);*/
	}
	polygenMeshList.RemoveAll();
	pGLK->ClearDisplayObjList();
	pGLK->refresh();
	updateTree();
	pGLK->refresh(true);
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

void MainWindow::on_treeView_clicked(const QModelIndex& index)
{
	ui->treeView->currentIndex();
	QStandardItem* modelListItem = treeModel->itemFromIndex(index);
	ui->treeView->setCurrentIndex(index);
	PolygenMesh* polygenMesh = getSelectedPolygenMesh();
	if (modelListItem->checkState() == Qt::Checked)
		polygenMesh->bShow = true;
	else
		polygenMesh->bShow = false;
	pGLK->refresh(true);
}

void MainWindow::QTgetscreenshoot()
{
	QScreen* screen = QGuiApplication::primaryScreen();
	QString filePathName = "Screen-";
	filePathName += QDateTime::currentDateTime().toString("yyyy-MM-dd hh-mm-ss");
	filePathName += ".png"; //important to control the file type
	if (!screen->grabWindow(0).save(filePathName, "png")) cout << "Get screen shoot failed!!!" << endl;
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

}

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

void MainWindow::drawSparseVectorField() {

	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	tetMesh->sparseVectorDraw =
		ui->checkBox_spaseVectorField->isChecked();

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


/*----------------------------------------*/
/* Toolpath generation for initial layers */
/* Notice that this function should be call after the output the generated layer and split into single piece.*/

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

	int Core = omp_get_num_procs() - 2;
#pragma omp parallel
	{
#pragma omp for  
		for (int omptime = 0; omptime < Core; omptime++) {
		
			for (GLKPOSITION posMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch* layer = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(posMesh);

				if (layer->GetIndexNo() % Core != omptime) continue;

				//if (layer->GetIndexNo() > 20) break;

				//if (layer->layerName != "5100") continue;

				//if (layer->includeSupportRegion == false) continue;
				std::cout << " ***** ----- Begin Process layer : " <<  layer->layerName << std::endl;

				PolygenMesh* singleToolpath = PolygenMeshSet[layer->GetIndexNo()];

				SurfaceGuidanceField* GuideFieldComp_layer = new SurfaceGuidanceField();

				if (layer->includeSupportRegion == false) {
					if (!GuideFieldComp_layer->scalarFieldCompute_isoSurface(layer)) continue;
				}
				else GuideFieldComp_layer->scalarFieldCompute_supportSurface(layer);
				delete GuideFieldComp_layer;

			
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

				//ToolPathComp_layer->mergeFieldforCCF();


				//boundaryTPathNum = 10;
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

	layerToolPathComp->toolpathOffset = 0.8; // ui->doubleSpinBox_toolpathOffsetValue->value(); //offset value for the tool-path
	layerToolPathComp->boundaryGapDist = 0.1;  //ui->doubleSpinBox_boundShrinkValue->value();
	layerToolPathComp->maxConnectDist = 1.7; //ui->doubleSpinBox_zigzagBoundConnectDistance->value();
	layerToolPathComp->maxGapDist = 4.0; // ui->doubleSpinBox_maxGapDistance->value();

	boundaryTPathNum = 3; //ui->spinBox_bToolPath_Num->value();

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