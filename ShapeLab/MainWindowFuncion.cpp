#include "stdafx.h"

#include "MainWindow.h"
#include "ui_MainWindow.h"

using namespace std;

/*----------------------------------*/
/* Principle Stress Field Computing */

void MainWindow::compPrincipleStressField()
{
	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	if (ui->checkBox_3DCompute->isChecked()) tetMesh->spaceComp = true;

	//--------------------------------------------------------------------
	//  Input FEM result and compute principle stress

	stressFieldComp = new PrincipleStressField(tetMesh);
	stressFieldComp->InputFEMResult(initModel->getModelName());
	stressFieldComp->ComputeElementPrincipleStress();
	stressFieldComp->DetermineCriticalTensileandCompressRegion(
		ui->tensileRegionRatio->text().toDouble(), ui->compressRegionRatio->text().toDouble());

	meshOperatorObject->compTetMeshVolumeMatrix(tetMesh);

	fileIOObject->outputPrincipleStressValueforCriticalRegion(initModel);
	ui->pushButton_CompGuideField->setEnabled(true);

	std::cout << "Finish input FEM result and propreces the system!" << std::endl << std::endl;

	pGLK->refresh(true);
}

/*--------------------------*/
/* Guidence Field Computing */

void MainWindow::compGuidenceField()
{
	PolygenMesh* initModel = this->_detectPolygenMesh(INIT_TET);
	if (initModel == false) { std::cout << "NO mesh in the system!"; return; }
	QMeshPatch* tetMesh = (QMeshPatch*)initModel->GetMeshList().GetHead();

	//--------------------------------------------------------------------
	//  Vector Field Computing

	VectorField* vectorFieldComp = new VectorField(tetMesh, false);
	vectorFieldComp->initMeshVectorFieldCompute();

	std::cout << " Finish generate the vector field based on principle stress distribution! " << std::endl;

	//--------------------------------------------------------------------
	//  Scalar Field Computing

	/*ScalarField* scalarFieldComp = new ScalarField(tetMesh, false);
	scalarFieldComp->compScalarField_initMesh();*/

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

/*-----------------------------------------------------------------------------------------*/
/* Detect best printing direction (notice that this can be time-consuming without openmp!) */

void MainWindow::findingFabricationDirection() {

	//-------------------------------------------------------------------------------------------------
	//  Detect Mesh for fabrcation direction checking: initial tetrahedral mesh, iso-layer and platform

	PolygenMesh* tetModel = _detectPolygenMesh(INIT_TET);
	if (tetModel == NULL) { std::cout << " # init tetModel not found!" << std::endl; return; }

	PolygenMesh* isosurfaceSet = _detectPolygenMesh(INIT_LAYERS);
	
	// Notice that this is used for DEBUG purpose!
	if (isosurfaceSet == NULL) {
		this->_inputInstalledIsoSurfaceSet(isosurfaceSet, tetModel);
		if (isosurfaceSet->GetMeshList().GetCount() == 0) { 
			std::cout << " # NO isosurface installed!" << std::endl; return; }
	}

	//-------------------------------------------------------------------------------------------------
	//  Best printing direction detection

	FabricationDirectionDetection* fabDirDetectOperator =
		new FabricationDirectionDetection(tetModel, this->_loadPlatformAndNozzle(false), isosurfaceSet);

	bool threeDSpaceDetect = ui->checkBox_3DCompute->isChecked();
	bool runFabDirDetect = ui->checkBox_computeFabricationDirection->isChecked();
	double installed2DAngle = ui->spinBox_ComputedAnlge->value();

	long time = clock();

	if (!threeDSpaceDetect) fabDirDetectOperator->runFabDir2DCaseDetection(runFabDirDetect, installed2DAngle);
	
	printf("finding best fabrication direction takes %ld ms.\n", clock() - time);

	pGLK->refresh(true);

	return;


	if(threeDSpaceDetect) fabDirDetectOperator->runFabDir2DCaseDetection(runFabDirDetect, installed2DAngle);
	else fabDirDetectOperator->runFabDir2DCaseDetection( runFabDirDetect, installed2DAngle);

	fabProcess = new FabricationProcess(isosurfaceSet, this->_loadPlatformAndNozzle(false), tetModel);
	fabProcess->systemBuild = true;

	if (ui->checkBox_3DCompute->isChecked())
		fabProcess->runBestFabricationDirectionDetection(
			ui->checkBox_computeFabricationDirection->isChecked(),
			ui->spinBox_ComputedAnlgeAlpha->value(),
			ui->spinBox_ComputedAnlgeBeta->value(),
			ui->checkBox_3DrotateInverse->isChecked()
		);
	else
		fabProcess->runBestFabricationDirectionDetection(
			ui->checkBox_computeFabricationDirection->isChecked(), ui->spinBox_ComputedAnlge->value());

	if (detectFlipLayerSetOrder(isosurfaceSet)) flipISOSurfaceOrder(isosurfaceSet);
	printf("finding best fabrication direction takes %ld ms.\n", clock() - time);

}

/*-------------------------------------------------------------------------------------------------------*/
/* Iso-surface field computing based on principle stress filed, used for directional toolpath generation */
/* Notice that this function should be call after the printing direction is determined                   */

void MainWindow::isoSurfaceFieldComputingforToolPathGeneration() {

	//----Compute the vector field in every layer - for the toolpath generation!
	PolygenMesh* initModelLayerSet = this->_detectPolygenMesh(INIT_LAYERS);

	SurfaceGuidanceField* surfaceGuidFieldComp = new SurfaceGuidanceField();

	int index = 0;
	for (GLKPOSITION posMesh = initModelLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoLayer = (QMeshPatch*)initModelLayerSet->GetMeshList().GetNext(posMesh);
		int layerIndex = index; index++;
		surfaceGuidFieldComp->runIsoLayerVectorFielCompute(isoLayer, layerIndex);
		std::cout << "Finish compute vector field in IsoLayer#" << layerIndex << std::endl;
	}

}

void MainWindow::surfaceMeshHeatMethodComp() {

	PolygenMesh* initModelLayerSet = this->_detectPolygenMesh(SURFACE_MESH);
	QMeshPatch* model = (QMeshPatch*)initModelLayerSet->GetMeshList().GetHead();

	heatMethod* heatMethodOperator = new heatMethod(model);
	heatMethodOperator->compBoundaryHeatKernel();
	model->drawgeoField = true;

	pGLK->refresh(true);
}
