#include "stdafx.h"
#include "MainWindow.h"
#include "ui_MainWindow.h"

using namespace std;

void MainWindow::createMainFunctionActions() {

	connect(ui->pushButton_compPrincipleStressField, SIGNAL(released()), this, SLOT(compPrincipleStressField()));

	connect(ui->pushButton_CompGuideField, SIGNAL(released()), this, SLOT(compGuidenceField()));

	connect(ui->pushButtonBuildIsoSurface, SIGNAL(released()), this, SLOT(buildIsoSurface()));

	connect(ui->pushButton_compFieldonIsoSurface, SIGNAL(released()), this, SLOT(isoSurfaceFieldComputingforToolPathGeneration()));

	connect(ui->pushButton_outputIsoLayer, SIGNAL(released()), this, SLOT(saveIsoLayer()));

	// step-by-step field compute

	connect(ui->pushButton_CompVectorField, SIGNAL(released()), this, SLOT(doComputeStressField_vectorFieldComp()));
	connect(ui->pushButton_CompScalarField, SIGNAL(released()), this, SLOT(doComputeStressField_scalarFieldComp()));

	connect(ui->pushButton_VectorFieldFlipNormal, SIGNAL(released()), this, SLOT(doComputeStressField_vectorFlipNormal()));
	connect(ui->pushButton_VectorFieldDeleteRegion, SIGNAL(released()), this, SLOT(doComputeStressField_vectorDeleteRegion()));

	// toolpath generation
	connect(ui->pushButton_directToolPathGeneration, SIGNAL(released()), this, SLOT(directToolpathGeneration()));

}

/*----------------------------------*/
/* Principle Stress Field Computing */

void MainWindow::compPrincipleStressField()
{
	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	if (ui->checkBox_3DCompute->isChecked()) tetMesh->spaceComp = true;

	//------------------------------------------------
	//  Input FEM result and compute principle stress

	stressFieldComp = new PrincipleStressField(tetMesh);
	stressFieldComp->InputFEMResult(initModel->getModelName());
	stressFieldComp->ComputeElementPrincipleStress();
	stressFieldComp->DetermineCriticalTensileandCompressRegion(
		ui->tensileRegionRatio->text().toDouble(), ui->compressRegionRatio->text().toDouble());

	meshOperatorObject->compTetMeshVolumeMatrix(tetMesh);

	fileIOObject->outputPrincipleStressValueforCriticalRegion(initModel);

	std::cout << "Finish input FEM result and propreces the system!" << std::endl << std::endl;

	pGLK->refresh(true);

	ui->pushButton_CompGuideField->setEnabled(true);

}

/*--------------------------*/
/* Guidence Field Computing */

/* auto generate the guidance field in one-go */
VectorField* vectorFieldComp;
ScalarField* scalarFieldComp;
void MainWindow::compGuidenceField()
{
	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	//--------------------------------------------------------------------
	//  Vector Field Computing

	vectorFieldComp = new VectorField(tetMesh, false);
	vectorFieldComp->initMeshVectorFieldCompute(false);

	std::cout << " Finish generate the vector field based on principle stress distribution! " << std::endl;

	//--------------------------------------------------------------------
	//  Scalar Field Computing

	scalarFieldComp = new ScalarField(tetMesh, false);
	scalarFieldComp->compScalarField_initMesh();

	pGLK->refresh(true);
}

/* manually generate the guidance field step-by-step */

void MainWindow::doComputeStressField_vectorFieldComp() {

	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	vectorFieldComp = new VectorField(tetMesh, false);
	vectorFieldComp->initMeshVectorFieldCompute(true);
	pGLK->refresh(true);
}

void MainWindow::doComputeStressField_vectorFlipNormal() {

	vectorFieldComp->flipSelectedRegion();
	pGLK->refresh(true);

}

void MainWindow::doComputeStressField_scalarFieldComp() {

	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	vectorFieldComp->fillNIERegionandSmooth(200, true);

	scalarFieldComp = new ScalarField(tetMesh, false);
	scalarFieldComp->compScalarField_initMesh();

	pGLK->refresh(true);
}

void MainWindow::doComputeStressField_vectorDeleteRegion() {

	vectorFieldComp->deleteSelectedRegion();

	//GuideFieldComp->runFieldComputing_DeleteRegion();
	pGLK->refresh(true);

}

/*----------------------------------------------------------------------------------*/
/* Iso-surface substraction based on scalar field - Initial curved layer generation */

void MainWindow::buildIsoSurface()
{
	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == nullptr) { std::cerr << "No mesh detected!" << std::endl; return; }
	QMeshPatch* model = (QMeshPatch*)initModel->GetMeshList().GetHead();

	PolygenMesh* initModelLayerSet = this->_buildPolygenMesh(INIT_LAYERS, "Init Curved Layer");

	//--------------------------------------------------------------------
	//  build iso surface from scalar field

	isoSurface* initModelSlicer = new isoSurface(model);
	initModelSlicer->generateIsoSurface(initModelLayerSet, ui->isoLayerNumber->value());

	// layer smoothness should be disabled in some case
	for (int i = 0; i < 5; i++) SurfaceGenerate->smoothingIsoSurface(initModelLayerSet);
	ui->IsoLayerIndex->setMaximum(initModelLayerSet->GetMeshList().GetCount());
	initModel->drawshade = false;
	
	viewAllIsoLayerandOffsetDisplay();
}

/*-------------------------------------------------------------------------------------------------------*/
/* Iso-surface field computing based on principle stress filed, used for directional toolpath generation */
/* Notice that this function should be call after the printing direction is determined                   */

void MainWindow::isoSurfaceFieldComputingforToolPathGeneration() {

	Eigen::Vector3d dir;
	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel->getModelName() == "bunnyhead") dir << 0.0, 1.0, 0.0;
	else if (initModel->getModelName() == "topopt_new") dir << 1.0, 0.0, 0.0;
	else dir << 1.0, 0.0, 0.0;

	//----Compute the vector field in every layer - for the toolpath generation!
	PolygenMesh* initModelLayerSet = this->_detectPolygenMesh(INIT_LAYERS);

	SurfaceGuidanceField* surfaceGuidFieldComp = new SurfaceGuidanceField();
	surfaceGuidFieldComp->inputDir = dir;
	int index = 0;
	for (GLKPOSITION posMesh = initModelLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoLayer = (QMeshPatch*)initModelLayerSet->GetMeshList().GetNext(posMesh);
		int layerIndex = index; index++;
		surfaceGuidFieldComp->runIsoLayerVectorFielCompute(isoLayer, layerIndex);
		std::cout << "Finish compute vector field in IsoLayer#" << layerIndex << std::endl;
	}

}


