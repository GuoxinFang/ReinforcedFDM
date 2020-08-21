#pragma once
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\PolygenMesh.h"

class QMeshPatch;
class PolygenMesh;
class GLKMatrix;

class GuidanceField
{
public:
	GuidanceField(QMeshPatch *mesh);
	~GuidanceField();
	QMeshPatch* tetMesh;

	void runFieldComputing();
	void runFieldComputing_VoxelGuide();
	void runSupportTetFieldCompute();
	void runIsoLayerFielCompute(QMeshPatch* isoSurface, int layerIndex);

	void runFieldComputing_TenslieandCompressRegion();
	void runFieldComputing_DeleteRegion();
	void runFieldComputing_computeScalarField();
	void runFieldComputing_FlipSelectedRegion();

	void adujstField_collisionRegion();

	void LocalGlobalMethod_Basic();
	void LocalGlobalMethod_Laplacian();

	void visualGuideFieldGradient(bool scalar);

	void scalarFieldCompute_isoSurface(QMeshPatch* isoSurface);
	void scalarFieldCompute_supportSurface(QMeshPatch* layer_support);

	void runFieldComputing_optVectorField_oneShoot();
	void runFieldComputing_optVectorField();



	double vectorFieldLaplacianWeight;
	double laplacianWeight;

private:
	int selectedTetraNum;

	void ClearALL();

	void initialIndex();
	void compVectorField();
	void compGuideField();

	//runSupportTetFieldCompute function

	void supportTetFieldCompute_set_bottom_constrain();
	void expandConnectRegion_withFieldConstrain();
	void supportTetFieldCompute_fill_NIE_by_laplacian(int itertime);
	void supportTetFieldCompute_compute_GuideField_support_by_hard_constrain();
	void supportTetFieldCompute_compute_GuideField_support_by_soft_constrain();

	void supportTetFieldCompute_normalize_guideField(bool support, int initNodeNum);

	//--------------------------------//

	void bary_tet(QMeshTetra* Tetra, Eigen::Vector4d& coordinate);
	void bary_tet(QMeshTetra* Tetra, Eigen::Vector4d& coordinate, Eigen::Vector3d& CenterP);

	void compVectorFieldwithLocalLaplacian();

	void compVectorFieldwithLocalLaplacian_neighborTetLaplacian(int eleType, QMeshTetra* Tetra);
	void detect_Neighbor_Tetrahedral_by_Node(
		std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, bool checkProcess, int ele_type, bool face_node);

	void orientation_Estimation_by_flooding(int type, int index);

	void VectorField_initialGuess_Tensile_Compress_Computing();
	void clear_small_neighbor_region();
	void compress_region_flooding_compute_orientation();
	int compRegionNum, tensileRegionNum;

	void smooth_compress_and_tensile_region_vector_field();
	void fill_vector_field_and_smooth(int iterTime, bool globalSmooth);

	void _fieldRegionDirOptimization();
	double _compVectorLaplacianEnergyValue();

	Eigen::SparseMatrix<double> Parameter;
	int nNum, tNum;

	int selectedInfluenceNum;
	int compressEleNum;
	int tensileEleNum;

	int compCeleNum; // compress element used in computation.
	int compTeleNum; // tensile element used in computation.

	//--------------------------------//
	// Field computing in ISO Layer
	bool initialize_isoSurface(QMeshPatch* isoSurface);
	bool fieldSmooth_isoSurface(QMeshPatch* isoSurface);
	void fieldFulfillLaplacian_isoSurface(QMeshPatch* isoSurface);
	void fieldFulfillLaplacian_isoSurface(QMeshPatch* isoSurface, int iter, int type);

	//--------------------------------//
	// Function for new method 04-26-2020
	double vectorDirRange;
	void _vectorField_segmentCriticalRegion();
	void _vectorField_processSegementRegion();
	void _region_detection_by_flooding(int type, int index);

	bool _regionFloodingStop(int type);
	void _detectNeiborTetbyNode_vectorConstrain(
		std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, int ele_type);

	void _smoothRegionVector(int regionIndex);
	int minEleNum = 50;

	//-------------------------------------------------//
	// Function for auto field dir compute May-16-2020 //
	Eigen::SimplicialLDLT <Eigen::SparseMatrix<double>> vectorFieldSolver;

	void _optimizeCriticalRegionVectorDir();
	void _buildVectorFieldCompSystem(
		int NIETetNum, int NIEFaceNum, Eigen::SparseMatrix<double>& A);
	void _fillBandCompute(int NIETetNum, int NIEFaceNum, Eigen::SparseMatrix<double>& A);
	void _changeRegionFieldDir(int regionIndex);

}; 
