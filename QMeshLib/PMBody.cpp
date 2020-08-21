#define _CRT_SECURE_NO_DEPRECATE

//#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <math.h>
#include <iostream>

#include "PMBody.h"
#include "../GLKLib/GLKGeometry.h"
#include "../Library/QHull/qhull_a.h"

PMBody::PMBody(void)
{
	m_drawShadingListID = -1;
	m_drawMeshListID = -1;
}

PMBody::~PMBody(void)
{
	ClearAll();
	if (m_drawShadingListID != -1) { glDeleteLists(m_drawShadingListID, 1); m_drawShadingListID = -1; }
	if (m_drawMeshListID != -1) { glDeleteLists(m_drawMeshListID, 1); m_drawMeshListID = -1; }
}

void PMBody::DeleteGLList(bool bShadeOrMesh)
{
	//if (m_drawListID!=-1) {glDeleteLists(m_drawListID, 2);	m_drawListID=-1;}
	if (bShadeOrMesh && m_drawShadingListID != -1) { glDeleteLists(m_drawShadingListID, 1); m_drawShadingListID = -1; }
	if (!bShadeOrMesh && m_drawMeshListID != -1) { glDeleteLists(m_drawMeshListID, 1); m_drawMeshListID = -1; }
}

void PMBody::BuildGLList(bool bShadeOrMesh)
{
	DeleteGLList(bShadeOrMesh);

	if (bShadeOrMesh) {
		m_drawShadingListID = glGenLists(1);
		_buildDrawShadeList();
		printf("PMBody Shading GL_ID = %d\n", m_drawShadingListID);
	}
	if (!bShadeOrMesh) {
		m_drawMeshListID = glGenLists(1);
		_buildDrawMeshList();
		printf("PMBody Shading GL_ID = %d\n", m_drawMeshListID);
	}
}

void PMBody::drawShade()
{
	if (meshList.IsEmpty()) { DeleteGLList(true); DeleteGLList(false); return; }
	glCallList(m_drawShadingListID);
}

void PMBody::drawMesh()
{
	if (meshList.IsEmpty()) { DeleteGLList(true); DeleteGLList(false); return; }
	glCallList(m_drawMeshListID);
}

void PMBody::drawProfile() {}
void PMBody::drawPreMesh() {}
void PMBody::drawHighLight() {}

void PMBody::drawBox(float xx, float yy, float zz, float r)
{
	glBegin(GL_QUADS);

	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(xx - r, yy - r, zz - r);
	glVertex3f(xx - r, yy + r, zz - r);
	glVertex3f(xx + r, yy + r, zz - r);
	glVertex3f(xx + r, yy - r, zz - r);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(xx - r, yy - r, zz + r);
	glVertex3f(xx + r, yy - r, zz + r);
	glVertex3f(xx + r, yy + r, zz + r);
	glVertex3f(xx - r, yy + r, zz + r);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(xx - r, yy - r, zz - r);
	glVertex3f(xx - r, yy - r, zz + r);
	glVertex3f(xx - r, yy + r, zz + r);
	glVertex3f(xx - r, yy + r, zz - r);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(xx + r, yy - r, zz - r);
	glVertex3f(xx + r, yy + r, zz - r);
	glVertex3f(xx + r, yy + r, zz + r);
	glVertex3f(xx + r, yy - r, zz + r);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(xx - r, yy - r, zz - r);
	glVertex3f(xx + r, yy - r, zz - r);
	glVertex3f(xx + r, yy - r, zz + r);
	glVertex3f(xx - r, yy - r, zz + r);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(xx - r, yy + r, zz - r);
	glVertex3f(xx - r, yy + r, zz + r);
	glVertex3f(xx + r, yy + r, zz + r);
	glVertex3f(xx + r, yy + r, zz - r);

	glEnd();
}

void PMBody::ClearAll()
{
	GLKPOSITION Pos;

	for (Pos = meshList.GetHeadPosition(); Pos != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(Pos));
		delete mesh;
	}
	meshList.RemoveAll();
}

void PMBody::FlipModel(short nDir)
{
	GLKPOSITION PosMesh;
	float pos[3];	int i, nodeNum;

	for (PosMesh = meshList.GetHeadPosition(); PosMesh != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(PosMesh));
		nodeNum = mesh->GetNodeNumber();
		for (i = 0; i<nodeNum; i++) { mesh->GetNodePos(i + 1, pos);	pos[nDir] = -pos[nDir];	mesh->SetNodePos(i + 1, pos); }
	}
}

void PMBody::Scaling(float ratio)
{
	GLKPOSITION PosMesh;
	float pos[3];	int i, nodeNum;

	for (PosMesh = meshList.GetHeadPosition(); PosMesh != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(PosMesh));
		nodeNum = mesh->GetNodeNumber();
		for (i = 0; i<nodeNum; i++) {
			mesh->GetNodePos(i + 1, pos);
			pos[0] = pos[0] * ratio; pos[1] = pos[1] * ratio; pos[2] = pos[2] * ratio;
			mesh->SetNodePos(i + 1, pos);
		}
	}
}

void PMBody::CompBoundingBox(float boundingBox[])
{
	GLKPOSITION PosMesh;
	float pos[3];	int i, nodeNum;

	boundingBox[0] = boundingBox[2] = boundingBox[4] = 1.0e+16;
	boundingBox[1] = boundingBox[3] = boundingBox[5] = -1.0e+16;

	for (PosMesh = meshList.GetHeadPosition(); PosMesh != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(PosMesh));
		nodeNum = mesh->GetNodeNumber();
		for (i = 0; i<nodeNum; i++) {
			mesh->GetNodePos(i + 1, pos);
			if (pos[0]<boundingBox[0]) boundingBox[0] = pos[0];
			if (pos[0]>boundingBox[1]) boundingBox[1] = pos[0];
			if (pos[1]<boundingBox[2]) boundingBox[2] = pos[1];
			if (pos[1]>boundingBox[3]) boundingBox[3] = pos[1];
			if (pos[2]<boundingBox[4]) boundingBox[4] = pos[2];
			if (pos[2]>boundingBox[5]) boundingBox[5] = pos[2];
		}
	}
}

void PMBody::Translation(float dx, float dy, float dz)
{
	float pos[3];	int i, nodeNum;
	GLKPOSITION Pos;

	for (Pos = meshList.GetHeadPosition(); Pos != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(Pos));
		nodeNum = mesh->GetNodeNumber();
		for (i = 0; i<nodeNum; i++) {
			mesh->GetNodePos(i + 1, pos);
			pos[0] += dx;	pos[1] += dy;	pos[2] += dz;
			mesh->SetNodePos(i + 1, pos);
		}
	}
}

void PMBody::Rotation(float axisVec[], float angle/*in degree*/)
{
	float pos[3], posNew[3];	int i, nodeNum;
	GLKPOSITION Pos;

	//-------------------------------------------------------------
	// Build rotation matrix
	float rMat[3][3];
	double dd, ax, ay, az, cs, sn;
	cs = cos(DEGREE_TO_ROTATE((double)angle));	sn = sin(DEGREE_TO_ROTATE((double)angle));
	dd = sqrt(((double)(axisVec[0]))*((double)(axisVec[0])) + ((double)(axisVec[1]))*((double)(axisVec[1])) + ((double)(axisVec[2]))*((double)(axisVec[2])));
	ax = ((double)(axisVec[0])) / dd;	ay = ((double)(axisVec[1])) / dd;	az = ((double)(axisVec[2])) / dd;
	rMat[0][0] = (float)(cs + (1.0 - cs)*ax*ax);		rMat[0][1] = (float)((1.0 - cs)*ax*ay - sn*az);	rMat[0][2] = (float)((1.0 - cs)*ax*az + sn*ay);
	rMat[1][0] = (float)((1.0 - cs)*ax*ay + sn*az);	rMat[1][1] = (float)(cs + (1.0 - cs)*ay*ay);		rMat[1][2] = (float)((1.0 - cs)*ay*az - sn*ax);
	rMat[2][0] = (float)((1.0 - cs)*ax*az - sn*ay);	rMat[2][1] = (float)((1.0 - cs)*ay*az + sn*ax);	rMat[2][2] = (float)(cs + (1.0 - cs)*az*az);

	//-------------------------------------------------------------
	// Rotate the vertices
	for (Pos = meshList.GetHeadPosition(); Pos != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(Pos));
		nodeNum = mesh->GetNodeNumber();
		for (i = 0; i<nodeNum; i++) {
			mesh->GetNodePos(i + 1, pos);
			posNew[0] = rMat[0][0] * pos[0] + rMat[0][1] * pos[1] + rMat[0][2] * pos[2];
			posNew[1] = rMat[1][0] * pos[0] + rMat[1][1] * pos[1] + rMat[1][2] * pos[2];
			posNew[2] = rMat[2][0] * pos[0] + rMat[2][1] * pos[1] + rMat[2][2] * pos[2];
			mesh->SetNodePos(i + 1, posNew);
		}
	}
}

void PMBody::_buildDrawShadeList()
{
	GLKPOSITION Pos;
	float pos[3], nv[3], rgb[3];		int i, meshIndex, faceNum;
	UINT ver[4];
	//	float *vertexArray,*normalArray;

	glNewList(m_drawShadingListID, GL_COMPILE);
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	meshIndex = 0;
	for (Pos = meshList.GetHeadPosition(); Pos != NULL; meshIndex++) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(Pos));
		faceNum = mesh->GetFaceNumber();

		_changeValueToColor(meshIndex, rgb[0], rgb[1], rgb[2]);
		//		rgb[0]=202.0f/255.0f;
		//		rgb[1]=150.0f/255.0f;
		//		rgb[2]=53.0f/255.0f;
		glColor3fv(rgb);

		/*		vertexArray=new float[faceNum*3*3];
		normalArray=new float[faceNum*3*3];
		for(i=0;i<faceNum;i++) {
		mesh->GetFaceNodes(i+1,ver[0],ver[1],ver[2],ver[3]);
		mesh->CompNormal(i+1,nv);
		normalArray[i*9]=nv[0];		normalArray[i*9+1]=nv[1];	normalArray[i*9+2]=nv[2];
		normalArray[i*9+3]=nv[0];	normalArray[i*9+4]=nv[1];	normalArray[i*9+5]=nv[2];
		normalArray[i*9+6]=nv[0];	normalArray[i*9+7]=nv[1];	normalArray[i*9+8]=nv[2];
		mesh->GetNodePos(ver[0],&(vertexArray[i*9]));
		mesh->GetNodePos(ver[1],&(vertexArray[i*9+3]));
		mesh->GetNodePos(ver[2],&(vertexArray[i*9+6]));
		}
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glVertexPointer( 3, GL_FLOAT, 0, vertexArray );
		glNormalPointer( GL_FLOAT, 0, normalArray );
		glDrawArrays( GL_TRIANGLES, 0, faceNum*3 );
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState( GL_VERTEX_ARRAY );
		delete vertexArray;		delete normalArray;*/

		for (i = 0; i<faceNum; i++) {
			mesh->GetFaceNodes(i + 1, ver[0], ver[1], ver[2], ver[3]);
			mesh->CompNormal(i + 1, nv);

			if (mesh->IsQuadFace(i + 1)) glColor3f(1, 1, 0);

			glNormal3fv(nv);
			mesh->GetNodePos(ver[0], pos);
			glVertex3fv(pos);
			mesh->GetNodePos(ver[1], pos);
			glVertex3fv(pos);
			mesh->GetNodePos(ver[2], pos);
			glVertex3fv(pos);

			if (!(mesh->IsQuadFace(i + 1))) continue;

			glNormal3fv(nv);
			mesh->GetNodePos(ver[0], pos);
			glVertex3fv(pos);
			mesh->GetNodePos(ver[2], pos);
			glVertex3fv(pos);
			mesh->GetNodePos(ver[3], pos);
			glVertex3fv(pos);

			glColor3fv(rgb);
			//			printf("(%d,%d,%d,%d) ",ver[0],ver[1],ver[2],ver[3]);
		}
		//for(i=0;i<mesh->GetNodeNumber();i++) {
		//	mesh->GetNodePos(i,pos);
		//	glVertex3fv(pos);
		//	pos[1]+=0.01;
		//	glVertex3fv(pos);
		//	pos[2]+=0.01;
		//	glVertex3fv(pos);
		//}
	}
	glEnd();
	glEndList();
}

void PMBody::_buildDrawMeshList()
{
	if (meshList.GetCount() == 0) return;

	glNewList(m_drawMeshListID, GL_COMPILE);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0);

	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glColor3f(0.0, 0.0, 0.0);
	glEnable(GL_POLYGON_OFFSET_LINE);
	glPolygonOffset(1.0, 1.0);

	GLKPOSITION Pos;
	float pos[3];	int i, j, edgeNum, faceNum;
	UINT ver[4];

	glBegin(GL_LINES);
	for (Pos = meshList.GetHeadPosition(); Pos != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(Pos));
		faceNum = mesh->GetFaceNumber();

		for (i = 0; i<faceNum; i++) {
			mesh->GetFaceNodes(i + 1, ver[0], ver[1], ver[2], ver[3]);
			if (mesh->IsQuadFace(i + 1)) edgeNum = 4; else edgeNum = 3;
			for (j = 0; j<edgeNum; j++) {
				if (ver[j]>ver[(j + 1) % edgeNum]) continue;	// to avoid the duplicate of drawing
				mesh->GetNodePos(ver[j], pos);
				glVertex3fv(pos);
				mesh->GetNodePos(ver[(j + 1) % edgeNum], pos);
				glVertex3fv(pos);
			}
		}
	}
	glEnd();

	glEndList();
}

void PMBody::_changeValueToColor(int nType, float & nRed, float & nGreen, float & nBlue)
{
	float color[][3] = {
		{ 255,255,255 },
		{ 255,0,128 },
		{ 0,255,255 },
		{ 128,255,0 },
		{ 128,128,64 },
		{ 255,0,0 },{ 0,255,0 },{ 0,0,255 },
		{ 128,128,192 },
		{ 255,255,128 },
		{ 255,128,0 },
		{ 255,128,255 },
		{ 255,214,202 },
		{ 128,128,192 },
		{ 255,165,0 }, //orange
		{ 255,128,192 },
		{ 128,128,64 },
		{ 0,255,255 },
		{ 238,130,238 },//violet
		{ 220,220,220 },//gainsboro
		{ 188, 143, 143 }, // rosy brown 
		{ 46, 139, 87 },//sea green
		{ 210, 105, 30 },//chocolate
		{ 100, 149, 237 }//cornflower blue 
	};

	nRed = color[nType % 22][0] / 255.0f;
	nGreen = color[nType % 22][1] / 255.0f;
	nBlue = color[nType % 22][2] / 255.0f;
}

void PMBody::computeRange()
{
	double range = 0.0, ll;	float pos[3];	int i, nodeNum;
	GLKPOSITION Pos;

	for (Pos = meshList.GetHeadPosition(); Pos != NULL;) {
		QuadTrglMesh *mesh = (QuadTrglMesh *)(meshList.GetNext(Pos));
		nodeNum = mesh->GetNodeNumber();
		for (i = 0; i<nodeNum; i++) {
			mesh->GetNodePos(i + 1, pos);
			ll = pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2];

			if (ll>range) range = ll;
		}
	}

	m_range = (float)(sqrt(range));
}

QuadTrglMesh::QuadTrglMesh(void)
{
	m_nodeNum = m_faceNum = 0;
}

QuadTrglMesh::~QuadTrglMesh(void)
{
	ClearAll();
}

void QuadTrglMesh::ClearAll()
{
	if (m_nodeNum != 0) free(m_nodeTable);
	if (m_faceNum != 0) free(m_faceTable);
}

void QuadTrglMesh::MallocMemory(int nodeNum, int faceNum)
{
	ClearAll();

	m_nodeNum = nodeNum;
	if (nodeNum>0) m_nodeTable = (float*)malloc(m_nodeNum * 3 * sizeof(float));

	m_faceNum = faceNum;
	if (faceNum>0) m_faceTable = (UINT*)malloc(m_faceNum * 4 * sizeof(UINT));
}

void QuadTrglMesh::SetNodePos(int index/*starting from 1*/, float pos[])
{
	m_nodeTable[(index - 1) * 3] = pos[0];
	m_nodeTable[(index - 1) * 3 + 1] = pos[1];
	m_nodeTable[(index - 1) * 3 + 2] = pos[2];
}

void QuadTrglMesh::SetFaceNodes(int index/*starting from 1*/,
	UINT verIndex1, UINT verIndex2,
	UINT verIndex3, UINT verIndex4)
{
	m_faceTable[(index - 1) * 4] = verIndex1;
	m_faceTable[(index - 1) * 4 + 1] = verIndex2;
	m_faceTable[(index - 1) * 4 + 2] = verIndex3;
	m_faceTable[(index - 1) * 4 + 3] = verIndex4;
}

int QuadTrglMesh::GetFaceNumber()
{
	return m_faceNum;
}

int QuadTrglMesh::GetNodeNumber()
{
	return m_nodeNum;
}

bool QuadTrglMesh::IsQuadFace(int index/*starting from 1*/)
{
	return (m_faceTable[(index - 1) * 4 + 3] != 0);
}

void QuadTrglMesh::GetFaceNodes(int index/*starting from 1*/,
	UINT &verIndex1, UINT &verIndex2,
	UINT &verIndex3, UINT &verIndex4)
{
	verIndex1 = m_faceTable[(index - 1) * 4];
	verIndex2 = m_faceTable[(index - 1) * 4 + 1];
	verIndex3 = m_faceTable[(index - 1) * 4 + 2];
	verIndex4 = m_faceTable[(index - 1) * 4 + 3];
}

void QuadTrglMesh::GetNodePos(int index/*starting from 1*/, float pos[])
{
	pos[0] = m_nodeTable[(index - 1) * 3];
	pos[1] = m_nodeTable[(index - 1) * 3 + 1];
	pos[2] = m_nodeTable[(index - 1) * 3 + 2];
}

void QuadTrglMesh::CompBoundingBox(float boundingBox[])
{
	float pos[3];	int i, nodeNum;

	boundingBox[0] = boundingBox[2] = boundingBox[4] = 1.0e+16;
	boundingBox[1] = boundingBox[3] = boundingBox[5] = -1.0e+16;

	nodeNum = GetNodeNumber();
	for (i = 0; i<nodeNum; i++) {
		GetNodePos(i + 1, pos);
		if (pos[0]<boundingBox[0]) boundingBox[0] = pos[0];
		if (pos[0]>boundingBox[1]) boundingBox[1] = pos[0];
		if (pos[1]<boundingBox[2]) boundingBox[2] = pos[1];
		if (pos[1]>boundingBox[3]) boundingBox[3] = pos[1];
		if (pos[2]<boundingBox[4]) boundingBox[4] = pos[2];
		if (pos[2]>boundingBox[5]) boundingBox[5] = pos[2];
	}
}

void QuadTrglMesh::CompNormal(int faceIndex/*starting from 1*/, float nv[])
{
	GLKGeometry geo;
	float p[3][3];	double nnv[3], aa, bb, cc, dd;
	int i, edgeNum;	UINT ver[4];

	if (IsQuadFace(faceIndex)) edgeNum = 4; else edgeNum = 3;
	GetFaceNodes(faceIndex, ver[0], ver[1], ver[2], ver[3]);
	nnv[0] = nnv[1] = nnv[2] = 0.0;

	for (i = 0; i<edgeNum; i++) {
		GetNodePos(ver[i], p[0]);
		GetNodePos(ver[(i + 1) % edgeNum], p[1]);
		GetNodePos(ver[(i + 2) % edgeNum], p[2]);
		geo.CalPlaneEquation(p[0], p[1], p[2], aa, bb, cc, dd);
		nnv[0] += aa;		nnv[1] += bb;		nnv[2] += cc;
	}
	dd = nnv[0] * nnv[0] + nnv[1] * nnv[1] + nnv[2] * nnv[2];	if (dd<1.0e-10) dd = 1.0;
	dd = sqrt(dd);
	nv[0] = (float)(nnv[0] / dd);
	nv[1] = (float)(nnv[1] / dd);
	nv[2] = (float)(nnv[2] / dd);
}

bool QuadTrglMesh::OutputOBJFile(char *filename)
{
	FILE *fp;

	fp = fopen(filename, "w");
	if (!fp)
	{
		printf("===============================================\n");
		printf("Can not open the data file - OBJ File Export!\n");
		printf("===============================================\n");
		return false;
	}

	fprintf(fp, "# The units used in this file are meters.\n");
	for (int i = 0; i<m_nodeNum; i++) {
		fprintf(fp, "v %.12f %.12f %.12f\n",
			m_nodeTable[i * 3], m_nodeTable[i * 3 + 1], m_nodeTable[i * 3 + 2]);
	}

	fprintf(fp, "\n");

	for (int i = 0; i<m_faceNum; i++) {
		if (IsQuadFace(i + 1)) {
			fprintf(fp, "f %u %u %u %u\n",
				m_faceTable[i * 4], m_faceTable[i * 4 + 1], m_faceTable[i * 4 + 2], m_faceTable[i * 4 + 3]);
		}
		else {
			fprintf(fp, "f %u %u %u\n",
				m_faceTable[i * 4], m_faceTable[i * 4 + 1], m_faceTable[i * 4 + 2]);
		}
	}

	fclose(fp);

	return true;
}

bool QuadTrglMesh::InputMEBFile(char *filename)
{
	FILE *fp;	int nodeNum, faceNum;

	fp = fopen(filename, "rb");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - MEB File Export!\n");
		printf("===============================================\n");
		return false;
	}

	fread(&nodeNum, sizeof(int), 1, fp);
	fread(&faceNum, sizeof(int), 1, fp);
	if (nodeNum <= 0 || faceNum <= 0) {
		printf("===============================================\n");
		printf("MEB File Import ERROR:  nodeNum=%u  faceNum=%u!\n", nodeNum, faceNum);
		printf("===============================================\n");
		fclose(fp);		return false;
	}

	//---------------------------------------------------------------
	MallocMemory(nodeNum, faceNum);
	fread(m_nodeTable, sizeof(float), m_nodeNum * 3, fp);
	fread(m_faceTable, sizeof(UINT), m_faceNum * 4, fp);

	fclose(fp);

	printf("-----------------------------------------------------\n");
	printf("Face number: %d\n", m_faceNum);
	printf("Node number: %d\n", m_nodeNum);

	return true;
}

bool QuadTrglMesh::OutputMEBFile(char *filename)
{
	FILE *fp;

	fp = fopen(filename, "wb");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - MEB File Export!\n");
		printf("===============================================\n");
		return false;
	}

	fwrite(&m_nodeNum, sizeof(int), 1, fp);
	fwrite(&m_faceNum, sizeof(int), 1, fp);

	fwrite(m_nodeTable, sizeof(float), m_nodeNum * 3, fp);
	fwrite(m_faceTable, sizeof(UINT), m_faceNum * 4, fp);

	fclose(fp);

	return true;
}

void QuadTrglMesh::TransformFromQMeshPatch(QMeshPatch* Patch) 
{
	int nodeNum = Patch->GetNodeNumber();
	int faceNum = Patch->GetFaceNumber();
	MallocMemory(nodeNum, faceNum);

	int nodeIndex = 1, faceIndex = 1;
	double pos[3]; float fpos[3];
	for (GLKPOSITION Pos = Patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)Patch->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pos[0], pos[1], pos[2]);
		for (int i = 0; i < 3; i++) fpos[i] = float(pos[i]);
		SetNodePos(nodeIndex, fpos);
		nodeIndex++;
	}
	int ver[4] = { 0 };
	for (GLKPOSITION Pos = Patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)Patch->GetFaceList().GetNext(Pos);
		for (int i = 0; i < 3; i++) ver[i] = (Face->GetNodeRecordPtr(i)->GetIndexNo());
		SetFaceNodes(faceIndex, ver[0], ver[1], ver[2], ver[3]);
		faceIndex++;
	}
}

bool QuadTrglMesh::InputOBJFile(char *filename)
{
	FILE *fp;
	char fields[4][255];
	UINT ver[4];
	char linebuf[256], buf[100];
	float pos[3];
	int i, nodeNum, faceNum;

	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - OBJ File Import (%s)!\n", filename);
		printf("===============================================\n");
		return false;
	}

	//---------------------------------------------------------------
	//	Analysis of OBJ file
	nodeNum = faceNum = 0;
	while (!feof(fp)) {
		sprintf(buf, "");
		sprintf(linebuf, "");
		fgets(linebuf, 255, fp);
		sscanf(linebuf, "%s", buf);

		if ((strlen(buf) == 1) && (buf[0] == 'v')) nodeNum++;
		if ((strlen(buf) == 1) && (buf[0] == 'f')) {
			char seps[] = " \n";
			//char seps2[]="/";
			char *token;
			char linebuf2[255];

			strcpy(linebuf2, linebuf);

			int num = 0;
			token = strtok(linebuf, seps);
			while (token != NULL) { token = strtok(NULL, seps); num++; }
			num = num - 1;

			if (num == 3 || num == 4) {
				faceNum++;
			}
			else if (num>4) {
				fclose(fp);
				printf("Warning: face with more than 4 sides is found, which cannot be supported by this program!\n");
				return false;	// cannot support mesh with more than 4 sides
			}
		}
	}
	fclose(fp);
	if (nodeNum == 0 || faceNum == 0) return false;

	//---------------------------------------------------------------
	//	Import of OBJ file
	MallocMemory(nodeNum, faceNum);
	//---------------------------------------------------------------
	fp = fopen(filename, "r");
	int nodeIndex = 1, faceIndex = 1;
	while (!feof(fp)) {
		sprintf(buf, "");
		sprintf(linebuf, "");
		fgets(linebuf, 255, fp);
		sscanf(linebuf, "%s", buf);

		if ((strlen(buf) == 1) && (buf[0] == 'v')) {
			sscanf(linebuf, "%s %f %f %f \n", buf, &(pos[0]), &(pos[1]), &(pos[2]));
			//pos[0]=pos[0]*0.01f;
			//pos[1]=pos[1]*0.01f;
			//pos[2]=pos[2]*0.01f;
			SetNodePos(nodeIndex, pos);	nodeIndex++;
		}
		if ((strlen(buf) == 1) && (buf[0] == 'f')) {
			char seps[] = " \n";
			char seps2[] = "/";
			char *token;
			char linebuf2[255];

			strcpy(linebuf2, linebuf);

			int num = 0;
			token = strtok(linebuf, seps);
			while (token != NULL) { token = strtok(NULL, seps); num++; }
			num = num - 1;

			if (num<3) continue;

			if (num>4) {
			}
			else {
				token = strtok(linebuf2, seps);
				for (i = 0; i<num; i++) { token = strtok(NULL, seps); strcpy(fields[i], token); }
				for (i = 0; i<num; i++) { token = strtok(fields[i], seps2); ver[i] = (UINT)(atoi(token)); }
				if (num == 3) ver[3] = 0;

				SetFaceNodes(faceIndex, ver[0], ver[1], ver[2], ver[3]);
				faceIndex++;
			}
		}
	}
	fclose(fp);

	printf("-----------------------------------------------------\n");
	printf("Face number: %d\n", m_faceNum);
	printf("Node number: %d\n", m_nodeNum);

	return true;
}

//----------------------------------------------------------------------------------------
//	The following function generate a new polygonal mesh by the convex hull of the input model
QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)
{
	int i, index;		QuadTrglMesh *outputMesh = NULL;
	double center[3], vec[3][3], dir[3], v1[3], v2[3];
	facetT *facet;
	vertexT *vertex, **vertexp;

	//-------------------------------------------------------------------------------------
	//	Step 1: initialization
	int pntNum = inputMesh->GetNodeNumber();
	double *pntArray = (double*)malloc(sizeof(double) * 3 * pntNum);
	for (i = 0; i<3 * pntNum; i++) pntArray[i] = (inputMesh->GetNodeArrayPtr())[i];

	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	//qh_initflags("QbB Qs Pp FA Qt");
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	printf("Convex-Hull: %d faces with %d vertices\n", faceNum, nodeNum);
	//-------------------------------------------------------------------------------------
	if (faceNum>0 && nodeNum>0) {
		outputMesh = new QuadTrglMesh;
		outputMesh->MallocMemory(nodeNum, faceNum);
		//---------------------------------------------------------------------------------
		center[0] = center[1] = center[2] = 0.0;
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		(outputMesh->GetNodeTablePtr())[index * 3] = (float)(vertex->point[0]);
		(outputMesh->GetNodeTablePtr())[index * 3 + 1] = (float)(vertex->point[1]);
		(outputMesh->GetNodeTablePtr())[index * 3 + 2] = (float)(vertex->point[2]);
		center[0] += vertex->point[0];
		center[1] += vertex->point[1];
		center[2] += vertex->point[2];
		index++;
		}
		center[0] = center[0] / (double)nodeNum;
		center[1] = center[1] / (double)nodeNum;
		center[2] = center[2] / (double)nodeNum;
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			i = 0;
		FOREACHvertex_(facet->vertices) {
			(outputMesh->GetFaceTablePtr())[index * 4 + i] = vertex->id + 1;
			SET(vec[i],vertex->point);
			i++;	if (i>3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}
		(outputMesh->GetFaceTablePtr())[index * 4 + 3] = 0;

		//-----------------------------------------------------------------------------
		//	Check the direction and the magnitude of a normal vector
		//double dir[3]
		//SUB(dir,vec[0],center);
		//if (DOT(dir,facet->normal)>=0.0) printf("*"); else printf("-");
		//printf("%lf norm=%lf\n",(DOT(center,facet->normal)+facet->offset),DOT(facet->normal,facet->normal));
		//	It has been verified all normal[] vectors are pointing outwards and are unit-vectors.

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		//printf("dot=%lf\n",DOT(dir,facet->normal));
		if (DOT(dir,facet->normal)<0) {
			UINT temp = (outputMesh->GetFaceTablePtr())[index * 4];
			(outputMesh->GetFaceTablePtr())[index * 4] = (outputMesh->GetFaceTablePtr())[index * 4 + 2];
			(outputMesh->GetFaceTablePtr())[index * 4 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);

	return outputMesh;
}
