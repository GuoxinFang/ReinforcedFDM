#pragma once
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\PolygenMesh.h"


class fileIO
{

public:
	fileIO() {};
	~fileIO() {};
	
	void saveSelectedFixedHandleRegion(PolygenMesh* polygenMesh);
	bool inputFixedHandleRegion(PolygenMesh* polygenMesh);

	void exportMeshtoAbaqusFEM(PolygenMesh* polygenMesh);

	void outputISOSurfaceMesh(
		PolygenMesh* isosurfaceSet, bool splitMode, bool singleOutputMode, 
		std::string modelName, int maxLayerNum, bool offMode);

	void outputISOSurfaceMesh_support(PolygenMesh* isosurfaceSet_support, std::string modelName);

	void updateInitialSurfaceName(PolygenMesh* isosurfaceSet, PolygenMesh* isosurfaceSet_support);

	void saveFieldforRendering(PolygenMesh* initialModel);

	void inputInstalledIsoSurface(
		PolygenMesh* isosurfaceSet, std::vector<std::string>& layersFiles, std::string folderPath);

	void inputInstalledIsoSurface(
		PolygenMesh* isosurfaceSet, std::vector<std::string>& layersFiles, std::vector<std::string>& fieldFiles, std::string folderPath);

	void inputInstalledIsoSurface_support(
		PolygenMesh* isosurfaceSet, std::vector<std::string>& layersFiles, std::string folderPath, std::string modelName);

	void outputVectorFieldforRendering(PolygenMesh* initialModel);

	void outputPrincipleStressValueforCriticalRegion(PolygenMesh* initialModel);

	void outputCollisionSurface(PolygenMesh* isosurfaceSet);
	// function for save iso surface

	void outputSupportNodeMesh(QMeshPatch* supportNodePatch, PolygenMesh* initTetMesh);

	void outputToolpathRenderingFile(QMeshPatch* selectedSurface, bool isBoundaryField);
	void outputToolpathRenderingFile_field(QMeshPatch* selectedSurface);

	double minPrincipleStressValue, maxPrincipleStressValue;

private:

	bool _splitSingleSurfacePatch_detect(QMeshPatch* isoSurface);
	void _splitSingleSurfacePatch_flooding(QMeshPatch* isoSurface, int index);
	void _splitSingleSurfacePatch_splitmesh(
		QMeshPatch* isoSurface, std::vector<QMeshPatch*>& isoSurfaceSet_splited);
	void _outputSingleISOSurface(
		bool offMode, QMeshPatch* isoSurface, std::string path, bool split, bool support_split_final);
	void _outputISOSurfaceToolpathStressField(
		QMeshPatch* isosurface, std::string fieldPath, std::string fieldRenderPath);
	double _supportLayerAera(QMeshPatch* isoSurface_support);

	void _outputSingleCollisionSurface(QMeshPatch* layer);
	void _changeValueToColor(double maxValue, double minValue, double Value,
		float& nRed, float& nGreen, float& nBlue);
};