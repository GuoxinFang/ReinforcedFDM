// PMBody.cpp: implementation of the PMBody class.
//
//////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_DEPRECATE

#include <math.h>
#include <memory.h>
#include <fstream>
#include <omp.h>

#include "PolygenMesh.h"

#include "../Library/QHull/qhull_a.h"

#include <QDebug>
using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PolygenMesh::PolygenMesh(mesh_type type)
{
    ClearAll();
    m_drawListID=-1;
    m_bVertexNormalShading=false;
    isTransparent = false;
    m_drawListNumber = 6;
	meshType = type;
}

PolygenMesh::~PolygenMesh()
{
    ClearAll();
    if (m_drawListID!=-1) glDeleteLists(m_drawListID, m_drawListNumber);
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

void PolygenMesh::CompBoundingBox(double boundingBox[])
{
    GLKPOSITION PosMesh;
    GLKPOSITION Pos;
    double xx,yy,zz;

    boundingBox[0]=boundingBox[2]=boundingBox[4]=1.0e+32;
    boundingBox[1]=boundingBox[3]=boundingBox[5]=-1.0e+32;

    for(PosMesh=meshList.GetHeadPosition();PosMesh!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(PosMesh));
        for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
            QMeshNode *node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
            node->GetCoord3D(xx,yy,zz);
            if (xx<boundingBox[0]) boundingBox[0]=xx;
            if (xx>boundingBox[1]) boundingBox[1]=xx;
            if (yy<boundingBox[2]) boundingBox[2]=yy;
            if (yy>boundingBox[3]) boundingBox[3]=yy;
            if (zz<boundingBox[4]) boundingBox[4]=zz;
            if (zz>boundingBox[5]) boundingBox[5]=zz;
        }
    }
}

void PolygenMesh::DeleteGLList()
{
    if (m_drawListID!=-1) {
        glDeleteLists(m_drawListID, m_drawListNumber);
        m_drawListID=-1;
    }
}

void PolygenMesh::BuildGLList(bool bVertexNormalShading)
{
    if (m_drawListID!=-1) glDeleteLists(m_drawListID, m_drawListNumber);
    m_drawListID = glGenLists(m_drawListNumber);

    _buildDrawShadeList(bVertexNormalShading);
    _buildDrawMeshList();
    _buildDrawNodeList();
    _buildDrawProfileList();
    _buildDrawFaceNormalList();
    _buildDrawNodeNormalList();
    computeRange();
}

void PolygenMesh::_buildDrawShadeList(bool bVertexNormalShading)
{
	if (drawshade == false) return;
	if (this->meshType == TOOL_PATH || this->meshType == UNDEFINED) return;

	glNewList(m_drawListID, GL_COMPILE);

	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);

	drawOriginalCoordinate();

	isTransparent = false;
	if (isTransparent) {
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	/*------------- Draw VOXEL -------------*/

	if (this->meshType == VOXEL_MESH) {
		this->drawVoxel();
	}

	/*------------- Draw SURFACE_MESH -------------*/

	if (this->meshType == SURFACE_MESH) {
		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));
			float rr, gg, bb;

			glBegin(GL_TRIANGLES);
			for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
				QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

				glColor3f(0.8, 0.8, 0.8);

				if (face->selected) glColor3f(0.6, 0.6, 1.0);
				else if (face->isFixedDraw) glColor3f(0.3, 0.8, 1.0);
				else if (face->isHandleDraw) glColor3f(0.8, 0.3, 1.0);

				//heat method for tool-path generation, visulize the heat field
				if (mesh->drawgeoField == true)
				{
					double field = 0;
					//for (int i = 0; i < 3; i++) field += face->GetNodeRecordPtr(i)->geoFieldValue;
					for (int i = 0; i < 3; i++) field += face->GetNodeRecordPtr(i)->boundaryValue;

					field /= 3;
					//_changeValueToColor(-10,10, Tetra->eleStress[0], rr, gg, bb);
					_changeValueToColor(1, 0, field, rr, gg, bb);
					glColor3f(rr, gg, bb);
				}
				if (face->isIntersetwithPlane) glColor3f(0.6, 0.6, 1.0);
				//----------------------------------------------------------//


				this->drawSingleFace(face);

			}
			glEnd();
		}
	}

	/*------------- Draw VOLUME MESH (INIT_TET & SUPPORT_TET) -------------*/

	if (this->meshType == INIT_TET || this->meshType == SUPPORT_TET) {
		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));
			float rr, gg, bb;

			glBegin(GL_TRIANGLES);
			for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
				QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

				if (face->inner == true) continue; // the inner face is not visulize

				//-- below are the color change to draw field or selection --//
				glColor3f(0.8, 0.8, 0.8);

				if (face->selected) glColor3f(0.6, 0.6, 1.0);
				else if (face->isFixedDraw) glColor3f(0.3, 0.8, 1.0);
				else if (face->isHandleDraw) glColor3f(0.8, 0.3, 1.0);

				if (mesh->drawStressField == true) //FEM stress distribution result
				{
					QMeshTetra* Tetra;
					if (face->GetLeftTetra() == NULL) Tetra = face->GetRightTetra();
					else Tetra = face->GetLeftTetra();
					//_changeValueToColor(-10,10, Tetra->eleStress[0], rr, gg, bb);
					_changeValueToColor(mesh->drawValue[1], mesh->drawValue[0], Tetra->eleStress[0], rr, gg, bb);
					glColor3f(rr, gg, bb);
				}
				//----------------------------------------------------------//

				this->drawSingleFace(face);

			}
			glEnd();
		}
	}

	/*------------- Draw SUPPORT REGION -------------*/

	if (this->meshType == SUPPORT_REGION) {

		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));
			if (mesh->isSupport) continue;
			else if (mesh->isSupportConvexHull) {

				glBegin(GL_TRIANGLES);
				for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
					QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

					glColor3f(0.8, 0.8, 0.8);

					this->drawSingleFace(face);
				}
				glEnd();

			}
		}

	}

	/*------------- Draw CURVED LAYERS -------------*/

	if (this->meshType == INIT_LAYERS || this->meshType == SUPPORT_LAYERS) {


		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

			if (mesh->drawThisIsoLayer == false) continue;
			float rr, gg, bb;

			glBegin(GL_TRIANGLES);
			for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
				QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

				if (mesh->includeSupportRegion) {
					if (face->importantSupportFace == false) continue;
					rr = 0.8, gg = 0.5, bb = 0.5;
				}
				else _changeValueToColor(mesh->GetIndexNo(), rr, gg, bb);

				if (mesh->drawgeoField == true)
				{
					double field = 0;
					for (int i = 0; i < 3; i++) field += face->GetNodeRecordPtr(i)->geoFieldValue;
					//for (int i = 0; i < 3; i++) field += face->GetNodeRecordPtr(i)->boundaryValue;

					field /= 3;
					//_changeValueToColor(-10,10, Tetra->eleStress[0], rr, gg, bb);
					_changeValueToColor(1, 0, field, rr, gg, bb);
				}

				glColor3f(rr, gg, bb);
				this->drawSingleFace(face);

			}
			glEnd();

		}
	}

	if (isTransparent) {
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
	}

	glEndList();
}

void PolygenMesh::_buildDrawMeshList()
{
	if (meshList.GetCount() == 0) return;
	if (this->meshType == VOXEL_MESH || this->meshType == UNDEFINED) return;

    float rr, gg, bb;

    glNewList(m_drawListID+1, GL_COMPILE);
    glDisable(GL_LIGHTING);
    glLineWidth(0.5);

	/*------------- Draw CURVED LAYERS -------------*/

	if (this->meshType == INIT_LAYERS || this->meshType == SUPPORT_LAYERS) {

		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch *mesh = (QMeshPatch *)(meshList.GetNext(Pos));

			if (mesh->drawThisIsoLayer == false) continue;

			glBegin(GL_LINES);


			for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
				QMeshEdge * edge = (QMeshEdge *)((mesh->GetEdgeList()).GetNext(PosEdge));
				rr = 0.2; gg = 0.2; bb = 0.2;

				//if (edge->IsBoundaryEdge() == false) continue; // not draw boundary edge
				if (mesh->includeSupportRegion){
					if (edge->importantSupportEdge == false) continue;
				}

				// #### DEBUG USAGE - split the 4-edge polygon when build the iso-surface
				/*if (edge->isMiddleEdge == true) { rr = 0.9; gg = 0.2; bb = 0.2;}
				else if (edge->isMiddleEdge1 == true) { rr = 0.2; gg = 0.9; bb = 0.2;}*/

				glColor3f(rr, gg, bb);

				this->drawSingleEdge(edge);
			}

			glEnd();
		}

	}

	/*------------- Draw SUPPORT_REGION -------------*/

	if (this->meshType == SUPPORT_REGION) {

		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch *mesh = (QMeshPatch *)(meshList.GetNext(Pos));
			if (mesh->isSupport) continue;
			else if (mesh->isSupportConvexHull) {

				glBegin(GL_LINES);

				rr = 0.2; gg = 0.2; bb = 0.2;
				for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
					QMeshEdge * edge = (QMeshEdge *)((mesh->GetEdgeList()).GetNext(PosEdge));
					glColor3f(rr, gg, bb);
					this->drawSingleEdge(edge);
				}

				glEnd();

			}
		}

	}

	/*------------- Draw VOLUME/SURFACE MESH -------------*/

	if (this->meshType == SURFACE_MESH || this->meshType == INIT_TET || this->meshType == SUPPORT_TET) {
		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch *mesh = (QMeshPatch *)(meshList.GetNext(Pos));

			glBegin(GL_LINES);

			rr = 0.2; gg = 0.2; bb = 0.2;

			for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
				QMeshEdge * edge = (QMeshEdge *)((mesh->GetEdgeList()).GetNext(PosEdge));

				if (edge->inner && (this->meshType == INIT_TET || this->meshType == SUPPORT_TET)) continue;
				
				glColor3f(rr, gg, bb);
				if (edge->selected) glColor3f(0.6, 0.6, 1.0);

				this->drawSingleEdge(edge);
			}

			glEnd();			
			
		}
	}

	/*------------- Draw VOLUME/SURFACE MESH -------------*/
	if (this->meshType == TOOL_PATH) {
		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch *mesh = (QMeshPatch *)(meshList.GetNext(Pos));

			glBegin(GL_LINES);

			rr = 0.2; gg = 0.2; bb = 0.2;

			for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
				QMeshEdge * edge = (QMeshEdge *)((mesh->GetEdgeList()).GetNext(PosEdge));

				if (edge->isInsideBoundaryTPathRegion) continue;

				_changeValueToColor(mesh->GetIndexNo(), rr, gg, bb);
				glColor3f(rr, gg, bb);

				this->drawSingleEdge(edge);
			}

			glEnd();

		}
	}

	glEndList();

}

void PolygenMesh::_buildDrawNodeList()
{
	if (meshList.GetCount() == 0) return;
	if (this->meshType == UNDEFINED) return;

    glNewList(m_drawListID+2, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glEnable(GL_POINT_SMOOTH);
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glPointSize(4.0);

	float rr, gg, bb;

	/*------------- Draw VOXEL -------------*/

	if (this->meshType == VOXEL_MESH) {
		this->drawVoxelCenter();
	}

	/*------------- Draw SUPPORT REGION -------------*/

	if (this->meshType == SUPPORT_REGION) {
		glBegin(GL_POINTS);
		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch *mesh = (QMeshPatch *)(meshList.GetNext(Pos));
			if (mesh->isSupport) {
				glColor3f(0.2, 0.2, 0.8);
				for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
					QMeshNode * node = (QMeshNode *)((mesh->GetNodeList()).GetNext(PosNode));
					drawSingleNode(node);			
				}	
			}
		}
		glEnd();
	}

	/*------------- Draw NORMAL CASE / TOOL-PATH -------------*/

	if (this->meshType == INIT_TET || this->meshType == SUPPORT_TET || this->meshType == SURFACE_MESH || this->meshType == TOOL_PATH) {
		glBegin(GL_POINTS);
		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch *mesh = (QMeshPatch *)(meshList.GetNext(Pos));
			for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
				QMeshNode * node = (QMeshNode *)((mesh->GetNodeList()).GetNext(PosNode));

				if (!node->resampleChecked && this->meshType == TOOL_PATH) continue;

				rr = 0.1; gg = 0.1; bb = 0.1;

				if (node->selected) { rr = 0.2; gg = 0.6; bb = 1.0; }
				else if (node->isFixed) { rr = 0.3; gg = 0.8; bb = 1.0; }
				else if (node->isHandle) { rr = 0.8; gg = 0.3; bb = 1.0; }

				_changeValueToColor(1, 0, node->scalarField, rr, gg, bb); // draw guidance field
				node->SetColor(rr, gg, bb); //install node color to node for rendering
				
				if (node->planeCutNewNode) { rr = 0.6; gg = 0.6; bb = 1.0; }

				glColor3f(rr, gg, bb);

				drawSingleNode(node);
			}
		}
		glEnd();
	}

	/*------------- Draw CURVED PRINTING LAYERS -------------*/

	if (this->meshType == INIT_LAYERS || this->meshType == SUPPORT_LAYERS) {
		for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
			QMeshPatch *mesh = (QMeshPatch *)(meshList.GetNext(Pos));

			if (mesh->drawThisIsoLayer == false) continue;

			glBegin(GL_POINTS);

			for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
				QMeshNode * node = (QMeshNode *)((mesh->GetNodeList()).GetNext(PosNode));

				rr = 0.1; gg = 0.1; bb = 0.1;

				if (mesh->includeSupportRegion) {
					if (node->importantSupportNode == false) continue;
				}

				if (mesh->drawLayerThickness) {
					//_changeValueToColor(this->maxLayerThickness, this->minLayerThickness
					_changeValueToColor(0.85, 0.2, node->layerThickDistance, rr, gg, bb);
				}

				if (node->supportNode) { rr = 0.8; gg = 0.1; bb = 0.1; }
				if (node->isCollisionHappenNode) { rr = 0.8; gg = 0.1; bb = 0.1; }

				if(this->meshType == INIT_LAYERS && mesh->isoSurfaceGuideFieldComputed == true)
					_changeValueToColor(1, 0, node->guideFieldValue, rr, gg, bb); // draw guidance field
				
				if (node->isShadowNode) { rr = 1.0; gg = 0.1; bb = 0.0; }

				glColor3f(rr, gg, bb);

				drawSingleNode(node);
			}
			glEnd();

		}
	}

	glEndList();

}

void PolygenMesh::_buildDrawProfileList()
{
	/*This function is used to draw vector field for:
	 (1) FEM simulation principle stress vector field (2) scalar field gradient (initial guess) */

	if (meshList.GetCount() == 0) return;

	glNewList(m_drawListID + 3, GL_COMPILE);
	//glDisable(GL_LIGHTING);

	if (this->meshType == INIT_TET || this->meshType == SUPPORT_TET) {
		for (GLKPOSITION PosMesh = meshList.GetHeadPosition(); PosMesh != NULL;) {
			QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(PosMesh));

			double edgeLength = 0;
			for (GLKPOSITION Pos_stressE = mesh->GetEdgeList().GetHeadPosition(); Pos_stressE != NULL;) {
				QMeshEdge* Edge = (QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos_stressE));
				edgeLength += Edge->CalLength();
			}
			edgeLength /= mesh->GetEdgeNumber(); // average edge length

			double normLength = 0;
			for (GLKPOSITION Pos_stressT = mesh->GetTetraList().GetHeadPosition(); Pos_stressT != NULL;) {
				QMeshTetra* Tetra = (QMeshTetra*)(mesh->GetTetraList().GetNext(Pos_stressT));
				normLength += fabs(Tetra->sigma_max);
			}
			normLength /= mesh->GetTetraNumber(); // average stress-field normal length
			double scale = edgeLength / normLength;

			glLineWidth(0.5);

			for (GLKPOSITION Pos = mesh->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* Tet = (QMeshTetra*)mesh->GetTetraList().GetNext(Pos);

				double x, y, z;
				Tet->CalCenterPos(x, y, z);

				double n[3], length; float rr, gg, bb;

				/*--------Draw vector field--------*/

				if (mesh->drawVectorField) {
					length = edgeLength;

					//spase drawing vector field
					if (mesh->sparseVectorDraw && Tet->GetIndexNo() % 10 != 0) continue;
					if (Tet->vectorField.norm() < 0.01) continue;

					for (int i = 0; i < 3; i++) n[i] = Tet->vectorField(i);
					if (mesh->drawScalarFieldGradient) {
						for (int i = 0; i < 3; i++) n[i] = Tet->scalarFieldGradient(i);
					}

					if (!Tet->isTensileorCompressSelect) {
						if (Tet->GetIndexNo() % 5 != 0) continue; rr = 0.2, gg = 0.2, bb = 0.2;
					} //low-stress NIE region
					else {
						if (Tet->sigma_max < 0) { rr = 0.2, gg = 0.2, bb = 0.9; } //Compress region
						else { rr = 0.9, gg = 0.2, bb = 0.2; } //tensile region
						
						if (Tet->floodingRegionIndex <= 0) 
							_changeValueToColor(10 - ((int)fabs(Tet->floodingRegionIndex)) % 10, rr, gg, bb);
						else _changeValueToColor(Tet->floodingRegionIndex, rr, gg, bb);


					}

					if (mesh->checkCollision) {
						if (Tet->collisionTetra) { rr = 0.2, gg = 0.9, bb = 0.2; } // collision tetra 
						else { rr = 0.2, gg = 0.2, bb = 0.2; }
					}
					
				}

				/*--------Draw the principle stress vector--------*/

				else if (mesh->drawPrincipleStressField) {

					length = 0.5 * scale * (Tet->sigma_max);
					length = edgeLength;

					for (int i = 0; i < 3; i++) 
						n[i] = Tet->tau_max(i);
						//n[i] = Tet->third_Principle_Stress_Vector(i);

					_changeValueToColor(mesh->maxPrincipleStressValue, mesh->minPrincipleStressValue, Tet->sigma_max, rr, gg, bb);
					//_changeValueToColor(mesh->minPrincipleStressValue, mesh->minPrincipleStressValue, Tet->principleStress, rr, gg, bb);

					Tet->principleStressColor[0] = rr;
					Tet->principleStressColor[1] = gg;
					Tet->principleStressColor[2] = bb;

					if (mesh->sparseVectorDraw && Tet->GetIndexNo() % 10 != 0) continue;

					if (Tet->isSmallInfluence == true) continue;
					if (Tet->isTensileorCompressSelect == false) continue;
				}

				//if(Tet->flooding_Processed)  { rr = 0.1, gg = 0.1, bb = 0.1; }

				//---- Draw Array (BODY) ----//
				glColor3f(rr, gg, bb);

				glBegin(GL_LINES);
				glVertex3d(x, y, z); glVertex3d(x + n[0] * length, y + n[1] * length, z + n[2] * length);
				glEnd();

				//---- Draw Array (TIP) ----//

				//this->drawSingleArrayTip(x, y, z, x + n[0] * length, y + n[1] * length, z + n[2] * length);
				double endPoint[3] = { x + n[0] * length, y + n[1] * length, z + n[2] * length };
				drawSingleArrayTip(endPoint, n, length);

			}

			continue;
		}
	}

	if (this->meshType == INIT_LAYERS) {
		for (GLKPOSITION PosMesh = meshList.GetHeadPosition(); PosMesh != NULL;) {
			QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(PosMesh));
			//std::cout << mesh->layerName << std::endl;
			if (mesh->drawThisIsoLayer == false) continue;
			for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)mesh->GetFaceList().GetNext(Pos);

				double x, y, z; Face->CalCenterPos(x, y, z);
				double n[3]; 
				for (int i = 0; i < 3; i++) {
					n[i] = Face->principleStressDir(i);
				}
				if (Face->principleStressDir.norm() < 0.01) continue;

				double length = 2;
				float rr, gg, bb; rr = 0.2, gg = 0.2, bb = 0.2;

				//---- Draw Array (BODY) ----//
				glColor3f(Face->principleStressColor[0], Face->principleStressColor[1], Face->principleStressColor[2]);

				glBegin(GL_LINES);
				glVertex3d(x, y, z); glVertex3d(x + n[0] * length, y + n[1] * length, z + n[2] * length);
				glEnd();

				//---- Draw Array (TIP) ----//

				//this->drawSingleArrayTip(x, y, z, x + n[0] * length, y + n[1] * length, z + n[2] * length);
				double endPoint[3] = { x + n[0] * length, y + n[1] * length, z + n[2] * length };
				drawSingleArrayTip(endPoint, n, length);
			}
		}
	}
	
	glEndList();
}

void PolygenMesh::_buildDrawFaceNormalList()
{
    if (meshList.GetCount()==0) return;
	if (this->meshType == VOXEL_MESH || this->meshType == UNDEFINED || this->meshType == SUPPORT_REGION) return;

    glNewList(m_drawListID+4, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glColor3f(0.5, 0.0, 0.5);
	double x, y, z, nx, ny, nz;

    glLineWidth(1.0);
    glBegin(GL_LINES);
    for(GLKPOSITION meshPos=meshList.GetHeadPosition();meshPos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(meshPos));
		if ((this->meshType == INIT_LAYERS || this->meshType == SUPPORT_LAYERS) && mesh->drawThisIsoLayer == false) continue;
		if (mesh->GetEdgeNumber() == 0) continue;

		double length = 0;
		for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
			QMeshEdge *edge = (QMeshEdge *)(mesh->GetEdgeList().GetNext(Pos));
			length += edge->CalLength();
		}
		length /= mesh->GetEdgeNumber();

        for(GLKPOSITION Pos=mesh->GetFaceList().GetHeadPosition();Pos!=NULL;){
            QMeshFace *face=(QMeshFace *)(mesh->GetFaceList().GetNext(Pos));
			if (face->GetLeftTetra() != nullptr && face->GetRightTetra() != nullptr) continue;
            face->CalCenterPos(x, y, z);
            face->CalPlaneEquation();
            face->GetNormal(nx,ny,nz);
            glVertex3d(x, y, z);
            glVertex3d(x+nx*length, y+ny*length, z+nz*length);
        }
    }
    glEnd();
    glEndList();
}

void PolygenMesh::_buildDrawNodeNormalList()
{

    if (meshList.GetCount()==0) return;
	if (this->meshType == VOXEL_MESH || this->meshType == UNDEFINED || this->meshType == SUPPORT_REGION) return;

    glNewList(m_drawListID+5, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glColor3f(0.0, 0.5, 0.0);

    glLineWidth(1.0);
    glBegin(GL_LINES);

    for(GLKPOSITION meshPos=meshList.GetHeadPosition();meshPos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(meshPos));
        QMeshEdge *edge = (QMeshEdge*)mesh->GetEdgeList().GetHead();
		if (edge == NULL) break;
        double length = edge->CalLength();
		length = 1.0;

        for(GLKPOSITION Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;){
            QMeshNode *node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
			double x, y, z, nx, ny, nz;
            node->GetCoord3D(x, y, z);
            double n[3];

			if (node->resampleChecked == false && this->meshType == TOOL_PATH) continue;

			if (node->normalSet && this->meshType == TOOL_PATH) 
				node->GetNormal(n[0], n[1], n[2]); // toolpath node already given	
			else node->CalNormal(n);

            glVertex3d(x, y, z);
            glVertex3d(x+n[0]*length, y+n[1]*length, z+n[2]*length);
        }
    }

    glEnd();
    glEndList();
}


void PolygenMesh::drawSingleFace(QMeshFace* face) {

	double xx, yy, zz, dd;

	if (m_bVertexNormalShading == false) {
		face->CalPlaneEquation();
		face->GetPlaneEquation(xx, yy, zz, dd);
		glNormal3d(xx, yy, zz);
	}

	for (int i = 0; i < 3; i++) {
		QMeshNode* node = face->GetNodeRecordPtr(i);

		if (m_bVertexNormalShading) {
			double normal[3];
			node->CalNormal(normal); glNormal3dv(normal);
		}
		
		node->GetCoord3D(xx, yy, zz);
		glVertex3d(xx, yy, zz);
	}
}

void PolygenMesh::drawSingleEdge(QMeshEdge* edge) {

	double xx, yy, zz;
	edge->GetStartPoint()->GetCoord3D(xx, yy, zz);
	glVertex3d(xx, yy, zz);
	edge->GetEndPoint()->GetCoord3D(xx, yy, zz);
	glVertex3d(xx, yy, zz);

}

void PolygenMesh::drawSingleNode(QMeshNode* node) {

	double nx, ny, nz, xx, yy, zz;
	node->GetNormal(nx, ny, nz);
	node->GetCoord3D(xx, yy, zz);
	glNormal3d(nx, ny, nz);
	glVertex3d(xx, yy, zz);

}




void PolygenMesh::drawVoxel() {

	for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL;) {
		QMeshPatch* mesh = (QMeshPatch *)(meshList.GetNext(Pos));

		glBegin(GL_QUADS);
		//glColor3f(153.0f / 255.0f, 217.0f / 255.0f, 234.0f / 255.0f);
		glColor3f(0.7f, 0.7f, 0.7f);

		GLKPOSITION PosVoxel;
		double xx, yy, zz, ww; float rgb[3];
		ww = this->voxelSize*0.5;

		//long time = clock();
		for (PosVoxel = (mesh->GetNodeList()).GetHeadPosition(); PosVoxel != NULL;)
		{
			QMeshNode* Voxel = (QMeshNode *)(mesh->GetNodeList().GetNext(PosVoxel));
			if (!Voxel->isBoundaryVoxelNode) continue;
			if (voxelOrderComputed == true) {
				_changeValueToColor(Voxel->voxelLayerIndex, rgb[0], rgb[1], rgb[2]);
				glColor3fv(rgb);
			}

			Voxel->GetCoord3D(xx, yy, zz);
			//-------------------------------------------------------------------------------------------
			//	For faces along x-direction
			if (Voxel->voxelFlags[1] == true) {
				glNormal3f(-1.0f, 0.0f, 0.0f);
				glVertex3d(xx - ww, yy - ww, zz - ww);	glVertex3d(xx - ww, yy - ww, zz + ww);
				glVertex3d(xx - ww, yy + ww, zz + ww);	glVertex3d(xx - ww, yy + ww, zz - ww);
			}
			if (Voxel->voxelFlags[0] == true) {
				glNormal3f(1.0f, 0.0f, 0.0f);
				glVertex3d(xx + ww, yy - ww, zz - ww);	glVertex3d(xx + ww, yy + ww, zz - ww);
				glVertex3d(xx + ww, yy + ww, zz + ww);	glVertex3d(xx + ww, yy - ww, zz + ww);
			}
			//-------------------------------------------------------------------------------------------
			//	For faces along y-direction
			if (Voxel->voxelFlags[3] == true) {
				glNormal3f(0.0f, -1.0f, 0.0f);
				glVertex3d(xx - ww, yy - ww, zz - ww);	glVertex3d(xx + ww, yy - ww, zz - ww);
				glVertex3d(xx + ww, yy - ww, zz + ww);	glVertex3d(xx - ww, yy - ww, zz + ww);
			}
			if (Voxel->voxelFlags[2] == true) {
				glNormal3f(0.0f, 1.0f, 0.0f);
				glVertex3d(xx - ww, yy + ww, zz - ww);	glVertex3d(xx - ww, yy + ww, zz + ww);
				glVertex3d(xx + ww, yy + ww, zz + ww);	glVertex3d(xx + ww, yy + ww, zz - ww);
			}
			//-------------------------------------------------------------------------------------------
			//	For faces along z-direction
			if (Voxel->voxelFlags[5] == true) {
				glNormal3f(0.0f, 0.0f, -1.0f);
				glVertex3d(xx - ww, yy - ww, zz - ww);	glVertex3d(xx - ww, yy + ww, zz - ww);
				glVertex3d(xx + ww, yy + ww, zz - ww);	glVertex3d(xx + ww, yy - ww, zz - ww);
			}
			if (Voxel->voxelFlags[4] == true) {
				glNormal3f(0.0f, 0.0f, 1.0f);
				glVertex3d(xx - ww, yy - ww, zz + ww);	glVertex3d(xx + ww, yy - ww, zz + ww);
				glVertex3d(xx + ww, yy + ww, zz + ww);	glVertex3d(xx - ww, yy + ww, zz + ww);
			}
		}

		glEnd();
		//printf("draw face (taking %ld ms).\n", clock() - time);


		//-------------------------------------------------------------------------------------------
		//	Build convex hull
		int index = 0;				int meshNodeNum = 0;
		double *pntArray;

		//int indexVoxelNotDraw = 0;
		int indexVoxelDraw = 0;
		int platformNodeNum = 0;

		for (PosVoxel = (mesh->GetNodeList()).GetHeadPosition(); PosVoxel != NULL;)
		{
			QMeshNode* Voxel = (QMeshNode *)(mesh->GetNodeList().GetNext(PosVoxel));
			/*if (Voxel->isVoxelDraw == false) {
			indexVoxelNotDraw++; continue;
			}*/
			if (Voxel->isBoundaryVoxelNode) indexVoxelDraw++;
			else if (Voxel->isPlatformNode) platformNodeNum++;
		}
		//-------------------------------------------------------------------------------------------
		//	Step 1: preparation, build the nodeArray.
		//pntArray = (double*)malloc(sizeof(double) * 3 * (mesh->GetNodeNumber() - indexVoxelNotDraw));

		pntArray = (double*)malloc(sizeof(double) * 3 * (indexVoxelDraw + platformNodeNum));
		//-------------------------------------------------------------------------------------------	
		for (PosVoxel = (mesh->GetNodeList()).GetHeadPosition(); PosVoxel != NULL;)
		{
			QMeshNode* Voxel = (QMeshNode *)(mesh->GetNodeList().GetNext(PosVoxel));
			//if (Voxel->isVoxelDraw == false) continue;
			if (Voxel->isBoundaryVoxelNode) {
				Voxel->GetCoord3D(pntArray[index * 3], pntArray[index * 3 + 1], pntArray[index * 3 + 2]);
				index++;
			}
			else if (Voxel->isPlatformNode) {
				Voxel->GetCoord3D(pntArray[index * 3], pntArray[index * 3 + 1], pntArray[index * 3 + 2]);
				index++;
			}
			else continue;
		}

		//-------------------------------------------------------------------------------------------
		//	Step 2: computing the convex-hull
		qh_init_A(stdin, stdout, stderr, 0, NULL);
		//qh_initflags("QbB Qs Pp FA Qt");
		qh_initflags("Qt Qx");
		//qh_init_B(pntArray, mesh->GetNodeNumber() + meshNodeNum - indexVoxelNotDraw, 3, false);
		qh_init_B(pntArray, indexVoxelDraw + platformNodeNum, 3, false);

		qh_qhull();
		qh_check_output();
		qh_triangulate();
		if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

		//-------------------------------------------------------------------------------------------
		//	Step 3: extract the result of convex-hull
		int faceNum = qh_qh.num_facets;
		//printf("Convex-Hull: %d faces\n",faceNum);
		//-------------------------------------------------------------------------------------
		if ((qh QHULLfinished) && faceNum > 0) {
			glColor3f(0, 0, 0);
			facetT *facet;		vertexT *vertex, **vertexp;
			FORALLfacets{
				glBegin(GL_LINE_LOOP);
			FOREACHvertex_(facet->vertices) {
				glVertex3dv(vertex->point);
			}
			glEnd();
			}
		}

		//-------------------------------------------------------------------------------------------
		//	Step 4: free the memory
		int curlong, totlong;
		qh_freeqhull(false);
		qh_memfreeshort(&curlong, &totlong);
		if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
		//-------------------------------------------------------------------------------------
		free(pntArray);
	}

}

void PolygenMesh::drawVoxelCenter() {
	glNewList(m_drawListID + 2, GL_COMPILE);


	for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL;) {
		QMeshPatch* mesh = (QMeshPatch *)(meshList.GetNext(Pos));
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glPointSize(5.0);
		glBegin(GL_POINTS);
		glColor3f(0.2f, 0.2f, 0.2f);

		GLKPOSITION PosVoxel;
		double xx, yy, zz, ww; float rgb[3];
		ww = this->voxelSize*0.5;

		//long time = clock();
		for (PosVoxel = (mesh->GetNodeList()).GetHeadPosition(); PosVoxel != NULL;)
		{
			QMeshNode* Voxel = (QMeshNode *)(mesh->GetNodeList().GetNext(PosVoxel));
			if (!Voxel->isBoundaryVoxelNode) continue;
			Voxel->GetCoord3D(xx, yy, zz);
			glVertex3d(xx, yy, zz);
		}
		glEnd();
	}

	glEndList();
}

void PolygenMesh::drawOriginalCoordinate() {
	double axisLeng = 30.0;
	glLineWidth(2.0);
	glBegin(GL_LINES);

	// X-axis - Red Color
	glColor3f(1.0, 0.0, 0.0); glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(axisLeng, 0.0, 0.0);

	// Y-axis - green Color
	glColor3f(0.0, 1.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, axisLeng, 0.0);

	// Z-axis - black Color
	glColor3f(0.0, 0.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, axisLeng);

	//glColor3f(1, 0.1, 0.1);glVertex3d(0.0, 0.0, 0.0);
	//glVertex3d(-7.72805,- 11.4536,- 3.48036); //voxel searching debug

	glEnd();

	// Draw ground
	//GLfloat fExtent = 46.0f;
	//GLfloat fStep = 1.0f;
	//GLfloat y = 0.0f;
	//GLint iLine;

	//glColor4f(0, 0, 1, 0);
	//glPushMatrix();
	//glBegin(GL_LINES);
	//for (iLine = -fExtent; iLine <= fExtent; iLine += fStep)
	//{
	//	glVertex3f(iLine, y, fExtent);    // Draw Z lines
	//	glVertex3f(iLine, y, -fExtent);

	//	glVertex3f(fExtent, y, iLine);
	//	glVertex3f(-fExtent, y, iLine);
	//}
	//glEnd();
	//glPopMatrix();
}

void PolygenMesh::drawSingleArrayTip(double pp[3], double dir[3], double arrowLength) {

	double bone_hight = 0.3 * arrowLength;
	double bone_radius = 0.06 * arrowLength;

	Eigen::Vector3d endPP = { pp[0],pp[1],pp[2] };

	Eigen::Vector3d A = { dir[0],dir[1],dir[2] };
	Eigen::Vector3d B = { 0, 1.0, 0 };

	Eigen::Matrix3d rotationMatrix;
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(B, A);

	Eigen::Vector3d pp1 = { bone_radius * sin(0) , 0.0 , bone_radius * cos(0) };
	Eigen::Vector3d pp2 = { bone_radius * sin(120 * 3.14 / 180) , 0.0 , bone_radius * cos(120 * 3.14 / 180) };
	Eigen::Vector3d pp3 = { bone_radius * sin(240 * 3.14 / 180) , 0.0 , bone_radius * cos(240 * 3.14 / 180) };
	Eigen::Vector3d ppCenter = { 0.0, bone_hight, 0.0 };

	pp1 = rotationMatrix * pp1 + endPP;
	pp2 = rotationMatrix * pp2 + endPP;
	pp3 = rotationMatrix * pp3 + endPP;
	ppCenter = rotationMatrix * ppCenter + endPP;

	glBegin(GL_TRIANGLES);

		glColor3f(0.9, 0.1, 0.1);

		glVertex3d(pp1(0), pp1(1), pp1(2));
		glVertex3d(pp2(0), pp2(1), pp2(2));
		glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

		glVertex3d(pp2(0), pp2(1), pp2(2));
		glVertex3d(pp3(0), pp3(1), pp3(2));
		glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

		glVertex3d(pp3(0), pp3(1), pp3(2));
		glVertex3d(pp1(0), pp1(1), pp1(2));
		glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

	glEnd();

	//glBegin(GL_QUAD_STRIP);
	//for (int i = 0; i <= 360; i += 60) {
	//	double p = i * 3.14 / 180;
	//	glColor3f(0.9, 0.1, 0.1);

	//	Eigen::Vector3d pp1 = { bone_radius * sin(p) , 0.0 , bone_radius * cos(p) };
	//	Eigen::Vector3d pp2 = { 0.0, bone_hight, 0.0 };

	//	pp1 = rotationMatrix * pp1  + endPP;
	//	pp2 = rotationMatrix * pp2 + endPP;

	//	glVertex3d(pp1(0), pp1(1), pp1(2));
	//	glVertex3d(pp2(0), pp2(1), pp2(2));
	//	/*glVertex3d(bone_radius * sin(p) * dir[0] + pp[0], bone_radius * cos(p) * dir[1] + pp[1], pp[2]);
	//	glVertex3d(pp[0], bone_hight * dir[1] + pp[1], pp[2]);*/
	//}
	//glEnd();

}

void PolygenMesh::drawSingleArrayTip(float x0, float y0, float z0, float x1, float y1, float z1)
{
    /* https://www.cnblogs.com/MakeView660/p/10436685.html */
	/* https://blog.csdn.net/ryfdizuo/article/details/6548257 */

	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);
	static GLUquadricObj *  quad_obj = NULL;
	if (quad_obj == NULL)
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle(quad_obj, GLU_FILL);
	gluQuadricNormals(quad_obj, GLU_SMOOTH);
	glPushMatrix();
	// move to initial point
	glTranslated(x0, y0, z0);
	// compute length
	double  length;
	length = sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);
	if (length < 0.0001) {
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;
	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;
	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;
	length = sqrt(side_x*side_x + side_y*side_y + side_z*side_z);
	if (length < 0.0001) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;
	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;
	// compute transfer matrix
	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
		up_x,   up_y,   up_z,   0.0,
		dir_x,  dir_y,  dir_z,  0.0,
		0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd(m);
	
	//parameters
	GLdouble radius = 0.1;		
	GLdouble slices = 8.0;		
	GLdouble stack = 3.0;		
	//gluCylinder(quad_obj, radius, radius, bone_length, slices, stack);

	GLdouble bone_hight = 0.3 * bone_length;
	GLdouble bone_radius = 0.06 * bone_length;

	glBegin(GL_QUAD_STRIP);//draw bone
	int i = 0;
	for (i = 0; i <= 360; i += 60)
	{
		float p = i * 3.14 / 180;
		glColor3f(0.9, 0.1, 0.1);

		glVertex3f(bone_radius*sin(p), bone_radius*cos(p), 0.0f);
		glVertex3f(0, 0, bone_hight);
	}
	glEnd();

	glPopMatrix();
}



void PolygenMesh::drawShade()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID);
}

void PolygenMesh::drawMesh()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+1);
}

void PolygenMesh::drawNode()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+2);
}

void PolygenMesh::drawProfile()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+3);
}

void PolygenMesh::drawFaceNormal()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+4);
}

void PolygenMesh::drawNodeNormal()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+5);
}

void PolygenMesh::ClearAll()
{
    GLKPOSITION Pos;

    for(Pos=meshList.GetHeadPosition();Pos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(Pos));
		mesh->ClearAll();
        delete mesh;
    }
    meshList.RemoveAll();
}

void PolygenMesh::computeRange()
{
	if (this->meshType == VOXEL_MESH) return;
	double range=0.0,ll,xx,yy,zz;
    GLKPOSITION Pos;
    GLKPOSITION PosNode;

    for(Pos=meshList.GetHeadPosition();Pos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(Pos));
        for(PosNode=(mesh->GetNodeList()).GetHeadPosition();PosNode!=NULL;) {
            QMeshNode *node=(QMeshNode *)((mesh->GetNodeList()).GetNext(PosNode));

            node->GetCoord3D(xx,yy,zz);
            ll=xx*xx+yy*yy+zz*zz;

            if (ll>range) range=ll;
        }
    }

    m_range=(float)(sqrt(range));
}

void PolygenMesh::_changeValueToColor(double maxValue, double minValue, double Value,
                                 float & nRed, float & nGreen, float & nBlue)
{
//	Value=fabs(Value);

    if (Value<=minValue)
    {
        nRed=0.0;
        nGreen=0.0;
        nBlue=0.0;
        return;
    }

    if ((maxValue-minValue)<0.000000000001)
    {
        nRed=0.0;
        nGreen=0.0;
        nBlue=1.0;
        return;
    }

    double temp=(Value-minValue)/(maxValue-minValue);

//    nRed=(float)(1.0-temp);	nGreen=(float)(1.0-temp); nBlue=(float)(1.0-temp);	return;

    if (temp>0.75)
    {
        nRed=1;
        nGreen=(float)(1.0-(temp-0.75)/0.25);
        if (nGreen<0) nGreen=0.0f;
        nBlue=0;
        return;
    }
    if (temp>0.5)
    {
        nRed=(float)((temp-0.5)/0.25);
        nGreen=1;
        nBlue=0;
        return;
    }
    if (temp>0.25)
    {
        nRed=0;
        nGreen=1;
        nBlue=(float)(1.0-(temp-0.25)/0.25);
        return;
    }
    else
    {
        nRed=0;
        nGreen=(float)(temp/0.25);
        nBlue=1;
    }

//    double t1,t2,t3;
//    t1=0.75;
//    t2=0.5;
//    t3=0.25;
//    if (temp>t1)
//    {
//        nRed=1;
//        nGreen=0.8-(float)(temp-t1)/(1-t1)*0.42;
//        if (nGreen<0.38) nGreen=0.38f;
//        nBlue=0.62-(float)(temp-t1)/(1-t1)*0.4;
//        if (nBlue<0.22) nBlue=0.22;
//        return;
//    }
//    if (temp>t2)
//    {
//        nRed=1;
//        nGreen=1.0-(float)(temp-t2)/(t1-t2)*0.2;
//        if (nGreen<0.8) nGreen=0.8f;
//        nBlue=0.75-(float)(temp-t2)/(t1-t2)*0.13;
//        if (nBlue<0.62) nBlue=0.62f;
//        return;
//    }
//    if (temp>t3)
//    {
//        nRed=(float)(temp-t3)/(t2-t3)*0.31+0.69;
//        if (nRed>1.0) nRed=1.0f;
//        nGreen=(float)(temp-t3)/(t2-t3)*0.09+0.91;
//        if (nGreen>1.0) nGreen=1.0f;
//        nBlue=0.95-(float)(temp-t3)/(t2-t3)*0.2;
//        if (nBlue<0.75) nBlue=0.75f;
//        return;
//    }
//    else
//    {
//        nRed=(float)temp/t3*0.47+0.22;
//        if (nRed>0.69) nRed=0.69f;
//        nGreen=(float)temp/t3*0.53+0.38;
//        if (nGreen>0.91) nGreen=0.91f;
//        nBlue=1.0-(float)temp/t3*0.05;
//        if (nBlue<0.95) nBlue=0.95f;
//        return;
//    }
}

void PolygenMesh::ImportOBJFile(char *filename, std::string modelName)
{
    QMeshPatch *newMesh = new QMeshPatch;
    if (newMesh->inputOBJFile(filename,false)){
        meshList.AddTail(newMesh);
        computeRange();
        setModelName(modelName);
    }
    else
        delete newMesh;
}

void PolygenMesh::ImportOFFFile(char *filename, std::string modelName)
{
	QMeshPatch *newMesh = new QMeshPatch;
	if (newMesh->inputOFFFile(filename, false)) {
		meshList.AddTail(newMesh);
		computeRange();
		setModelName(modelName);
	}
	else
		delete newMesh;
}

void PolygenMesh::ImportTETFile(char *filename, std::string modelName)
{
	QMeshPatch *newMesh = new QMeshPatch;
	if (newMesh->inputTETFile(filename, false)) {
		meshList.AddTail(newMesh);
		computeRange();
		setModelName(modelName);
	}
	else
		delete newMesh;
}

void PolygenMesh::_changeValueToColor(int nType, float & nRed, float & nGreen, float & nBlue)
{
	float color[][3] = {
		{ 220,20,60 },
		{ 107,200,35 },
		{ 30,144,255 },
		{ 255,105,180 },
		{ 244,164,96 },
		{ 176,196,222 },
		{ 255,100,70 },
		{ 128,255,128 },
		{ 128,128,255 },
		{ 255,255,128 },
		{ 0,128,0 },
		{ 255,128,255 },
		{ 255,214,202 },
		{ 128,128,192 },
		{ 255,165,0 }, //orange
		{ 255,128,192 },
		//		{39, 64, 139},//RoyalBlue
		{ 128,128,64 },
		{ 0,255,255 },
		{ 238,130,238 },//violet
		{ 220,220,220 },//gainsboro
		{ 188, 143, 143 }, // rosy brown
		{ 46, 139, 87 },//sea green
		{ 210, 105, 30 },//chocolate
		{ 237, 150, 100 },
		{ 100, 149, 237 },//cornflower blue
		{ 243, 20, 100 },
		// 26th
		{ 0,0,0 }
	};

	//	printf("%d ",nType);
	nRed = color[nType % 25][0] / 255.0f;
	nGreen = color[nType % 25][1] / 255.0f;
	nBlue = color[nType % 25][2] / 255.0f;
}


