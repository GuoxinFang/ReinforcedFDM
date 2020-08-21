#include "meshOperator.h"

using namespace std;
using namespace Eigen;

void meshOperator::compTetMeshVolumeMatrix(QMeshPatch* tetMesh) {
	
	//-- This function calculate the volume matrix 
	//   for each tetrahedral elements and installed in formate
	/* [   b1 c1 d1
	       b2 c2 d2
	       b3 c3 d3
	       b4 c4 d4   ] */

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		Eigen::MatrixXd VolumeMatrix(4, 3);

		Vector3d aa, bb, cc, dd, pp;
		Tet->CalCenterPos(pp(0), pp(1), pp(2));

		Tet->GetNodeRecordPtr(1)->GetCoord3D(aa(0), aa(1), aa(2));
		Tet->GetNodeRecordPtr(2)->GetCoord3D(bb(0), bb(1), bb(2));
		Tet->GetNodeRecordPtr(3)->GetCoord3D(cc(0), cc(1), cc(2));
		Tet->GetNodeRecordPtr(4)->GetCoord3D(dd(0), dd(1), dd(2));

		Vector3d vap = pp - aa;
		Vector3d vbp = pp - bb;

		Vector3d vab = bb - aa;
		Vector3d vac = cc - aa;
		Vector3d vad = dd - aa;

		Vector3d vbc = cc - bb;
		Vector3d vbd = dd - bb;

		Vector3d bd_bc = vbd.cross(vbc);
		Vector3d ac_ad = vac.cross(vad);
		Vector3d ad_ab = vad.cross(vab);
		Vector3d ab_ac = vab.cross(vac);
		double volumeTet = Tet->CalVolume() * 6;

		VolumeMatrix.row(0) = bd_bc / volumeTet;
		VolumeMatrix.row(1) = ac_ad / volumeTet;
		VolumeMatrix.row(2) = ad_ab / volumeTet;
		VolumeMatrix.row(3) = ab_ac / volumeTet;

		Tet->VolumeMatrix = VolumeMatrix;

	}
}
