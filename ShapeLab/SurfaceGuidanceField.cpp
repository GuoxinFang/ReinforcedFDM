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
	planeStressDir << 0.0, 1.0, 0.0; //-- yoga model?
	//planeStressDir << 1.0, 0.0, 1.0; //-- bridge model?
	//planeStressDir << 1.0, 0.0, 0.0; //-- suitable for topopt model!
	//planeStressDir << 1.0, 0.0, 1.0; //-- CSquare Model, after collisiondetection

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
