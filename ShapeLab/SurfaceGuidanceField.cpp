#include "stdafx.h"
#include "SurfaceGuidanceField.h"

using namespace std;
using namespace Eigen;

void SurfaceGuidanceField::runIsoLayerVectorFielCompute(QMeshPatch* isoSurface, int layerIndex) {

	/* Compute the vector field of each iso-surface */

	// If the isosurface contains no critical region, vector field direction will be given directly.
	if (this->_initializeIsoSurface(isoSurface) == false) {
		isoSurface->stressFieldToolpathControl = false;

		Vector3d vectorDir = Eigen::Vector3d::Zero();
		if (layerIndex % 2 == 0) vectorDir << 1.0, 0.0, 1.0;
		else vectorDir << 1.0, 0.0, -1.0;
		vectorDir = vectorDir.normalized();

		// project vector field to the plane defined by triangle face
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			Face->principleStressDir = vectorDir;
			this->_projectVectortoSurface(Face);
		}

	}

	// Otherwise directional vector field based on stress distribution will be computed
	else {
		isoSurface->stressFieldToolpathControl = true;
		this->_SurfaceVectorFieldOrientationDetection(isoSurface);
		this->_surfaceVectorFieldFillNIERegion(isoSurface);
	}

}

bool SurfaceGuidanceField::_SurfaceVectorFieldOrientationDetection(QMeshPatch* isoSurface) {

	Eigen::Vector3d planeStressDir = Eigen::Vector3d::Zero();
	planeStressDir << 0.0, 1.0, 0.0; //-- yoga model
	//planeStressDir << 1.0, 0.0, 1.0; //-- bridge model
	//planeStressDir << 1.0, 0.0, 0.0; //-- suitable for topopt model
	//planeStressDir << 1.0, 0.0, 1.0; //-- CSquare Model, after collisiondetection
	//planeStressDir << 0.0, 0.0, 1.0; //-- loop model
	//planeStressDir << 1.0, 0.0, 0.0; //-- shell model

	planeStressDir = this->inputDir;
	planeStressDir = planeStressDir.normalized();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->isNIEface) continue;
		if (Face->principleStressDir.dot(planeStressDir) < 0) {
			Face->principleStressDir = -Face->principleStressDir;
		}
	}

	return true;
}

bool SurfaceGuidanceField::_initializeIsoSurface(QMeshPatch* isoSurface) {

	//project the principle stress normal
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->principleStressDir.norm() < 0.01) Face->isNIEface = true;
		else this->_projectVectortoSurface(Face);	
	}

	Eigen::Vector3d planeStressDir = Eigen::Vector3d::Zero();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->isNIEface == false) {
			planeStressDir = Face->principleStressDir; break;
		}
	}

	if (planeStressDir.norm() < 0.01) return false;
	else return true;
}

void SurfaceGuidanceField::_projectVectortoSurface(QMeshFace* Face) {
	Eigen::Vector3d faceNorm; double D;
	Face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
	faceNorm = faceNorm.normalized();
	double dotProduct = Face->principleStressDir.dot(faceNorm);
	Eigen::Vector3d planeNorm = Face->principleStressDir - dotProduct * faceNorm;
	//Face->principleStressDir = planeNorm.normalized().cross(faceNorm);
	Face->principleStressDir = planeNorm.normalized();
}

void SurfaceGuidanceField::_surfaceVectorFieldFillNIERegion(QMeshPatch* isoSurface) {

	_surfaceVectorFieldFulfill(isoSurface, 15, 1); // smooth important region
	_surfaceVectorFieldFulfill(isoSurface, 15, 2); // fullfill NIE region

	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		this->_projectVectortoSurface(Face);
	}

	_surfaceVectorFieldFulfill(isoSurface, 10, 0); // smooth all region
	//fieldFulfillLaplacian_isoSurface(isoSurface, 100, 0); // csquare region

	//fulfill the rest region
	Eigen::Vector3d averageDir = Eigen::Vector3d::Zero();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		averageDir += Face->principleStressDir;
	}
	averageDir = averageDir.normalized();
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		if (Face->principleStressDir.norm() > 0.01) continue;
		else {
			Face->principleStressDir = averageDir;
			this->_projectVectortoSurface(Face);
		}
	}
	_surfaceVectorFieldFulfill(isoSurface, 15, 3); // global smmothness

}

void SurfaceGuidanceField::_surfaceVectorFieldFulfill(QMeshPatch* isoSurface, int iter, int type) {

	for (int iterTime = 0; iterTime < iter; iterTime++) {
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			Vector3d averageField = Face->principleStressDir;

			if (type == 1 && Face->isNIEface == true) continue;
			if (type == 2 && Face->isNIEface == false) continue;

			for (int i = 0; i < 3; i++) {
				QMeshEdge* Edge = Face->GetEdgeRecordPtr(i + 1);
				if (Edge->IsBoundaryEdge()) continue;
				else {
					if (Face == Edge->GetLeftFace()) averageField += Edge->GetRightFace()->principleStressDir;
					else  averageField += Edge->GetLeftFace()->principleStressDir;
				}
			}
			Face->principleStressDir = averageField.normalized();

		}
	}
}


bool SurfaceGuidanceField::scalarFieldCompute_isoSurface(QMeshPatch* isoSurface) {

	int index = 0;
	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index); index++;
	}

	// ---- method 1: apply laplacian to all the element (including constrained)

	Eigen::SparseMatrix<double> Parameter(3 * isoSurface->GetFaceNumber(), isoSurface->GetNodeNumber()); //A

	Eigen::VectorXd guideField(isoSurface->GetNodeNumber()); //x

	Eigen::VectorXd b(3 * isoSurface->GetFaceNumber()); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	index = 0;
	for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index); index++;
		double weight = 1.0;
		if (Face->isNIEface == false) weight = 3.0;
		double faceAera = Face->CalArea();

		Eigen::Matrix3d faceMatrixPara, faceNodeCoord;
		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetCoord3D(faceNodeCoord(i, 0), faceNodeCoord(i, 1), faceNodeCoord(i, 2));
		}
		faceMatrixPara.row(0) = (faceNodeCoord.row(2) - faceNodeCoord.row(1)) / (2 * faceAera);
		faceMatrixPara.row(1) = (faceNodeCoord.row(0) - faceNodeCoord.row(2)) / (2 * faceAera);
		faceMatrixPara.row(2) = (faceNodeCoord.row(1) - faceNodeCoord.row(0)) / (2 * faceAera);

		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 3; i++) {
				QMeshNode* Node = Face->GetNodeRecordPtr(i);
				ParaTriplet.push_back(Eigen::Triplet<double>(
					Face->GetIndexNo() * 3 + j, Node->GetIndexNo(), faceMatrixPara(i, j) * weight)); // infill A
			}
		}

		for (int i = 0; i < 3; i++) b(Face->GetIndexNo() * 3 + i) = Face->principleStressDir(i) * weight; // infill B
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(isoSurface->GetNodeNumber(), isoSurface->GetNodeNumber());
	ATA = Parameter.transpose() * Parameter;
	Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

	//Solver.compute(ATA);
	Solver.analyzePattern(ATA);
	Solver.factorize(ATA);
	if (Solver.info() != Eigen::Success) { 
		cout << "this layer has error computing scalar field !" << endl; return false; 
	}

	Eigen::VectorXd ATb(isoSurface->GetNodeNumber());
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);

	Eigen::VectorXd guideFieldNormalize(isoSurface->GetNodeNumber());
	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < isoSurface->GetNodeNumber(); i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i < isoSurface->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
		//Node->guideFieldValue = guideField(Node->GetIndexNo());
		Node->guideFieldValue = guideFieldNormalize(Node->GetIndexNo());
		Node->zigzagValue = guideFieldNormalize(Node->GetIndexNo());

	}
	isoSurface->isoSurfaceGuideFieldComputed = true;
	return true;
}

void SurfaceGuidanceField::scalarFieldCompute_supportSurface(QMeshPatch* layer_support) {

	int layerIndex = stoi(layer_support->layerName.substr(0, layer_support->layerName.length() - 3));

	Eigen::Vector3d planeStressDir = Eigen::Vector3d::Zero();
	if (layerIndex % 2 == 0) planeStressDir << 1.0, 0.0, 1.0;
	else planeStressDir << 1.0, 0.0, -1.0;
	planeStressDir = planeStressDir.normalized();

	for (GLKPOSITION Pos = layer_support->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer_support->GetFaceList().GetNext(Pos);
		Face->principleStressDir = planeStressDir;

		Eigen::Vector3d faceNorm; double D;
		Face->CalPlaneEquation(faceNorm(0), faceNorm(1), faceNorm(2), D);
		faceNorm = faceNorm.normalized();
		double dotProduct = Face->principleStressDir.dot(faceNorm);
		Eigen::Vector3d planeNorm = Face->principleStressDir - dotProduct * faceNorm;
		Face->principleStressDir = planeNorm.normalized();

	}

	this->scalarFieldCompute_isoSurface(layer_support);
}
