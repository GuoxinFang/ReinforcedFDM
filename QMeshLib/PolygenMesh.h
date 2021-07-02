#ifndef POLYGENMESH_H
#define POLYGENMESH_H

#include <string>

#include "../GLKLib/GLKLib.h"
#include "../GLKLib/GLKObList.h"

#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshTetra.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

typedef enum mesh_type {
	//for SIGGRAPH paper
	INIT_TET, INIT_LAYERS, 
	SUPPORT_REGION, SUPPORT_TET, SUPPORT_LAYERS,
	VOXEL_MESH, TOOL_PATH,
	//normal case
	SURFACE_MESH, UNDEFINED
};

class PolygenMesh : public GLKEntity
{
public:
	PolygenMesh(mesh_type type);
    ~PolygenMesh();

	mesh_type meshType;

    void ImportOBJFile(char *filename, std::string modelName);
	void ImportOFFFile(char *filename, std::string modelName);
	void ImportTETFile(char *filename, std::string modelName);

    virtual void DeleteGLList();
    virtual void BuildGLList(bool bVertexNormalShading);

    virtual void drawShade();
    virtual void drawMesh();
    virtual void drawNode();
    virtual void drawProfile();
    virtual void drawFaceNormal();
    virtual void drawNodeNormal();
    virtual float getRange() {return m_range;}

    //void drawBox(float xx, float yy, float zz, float r);

    void ClearAll();
    void computeRange();
    GLKObList &GetMeshList() {return meshList;};
    void CompBoundingBox(double boundingBox[]);

    int m_materialTypeNum;
    bool m_bVertexNormalShading;

	// only select support node will open this flag for the first polygenMesh (03-13-2020)
	bool supportRegionSelection = false; 
    bool supportRegionSelection_byInit;

public:
    GLKObList meshList;
    float m_range;
    int m_drawListID;
    int m_drawListNumber;

	// -- Draw Mesh Function

    void _buildDrawShadeList(bool bVertexNormalShading);
	bool drawshade = true;

    void _buildDrawMeshList();
    void _buildDrawNodeList();
    void _buildDrawProfileList();
    void _buildDrawFaceNormalList();
    void _buildDrawNodeNormalList();

	// -- single element disply function

	void drawSingleFace(QMeshFace* face);
	void drawSingleEdge(QMeshEdge* edge);
	void drawSingleNode(QMeshNode* node);
	void drawSingleArrayTip(float x0, float y0, float z0, float x1, float y1, float z1);
    void drawSingleArrayTip(double pp[3], double dir[3], double arrowLength);

	void drawOriginalCoordinate();
	void drawVoxel();
	void drawVoxelCenter();

    void _changeValueToColor(int nType, float & nRed, float & nGreen, float & nBlue);
    void _changeValueToColor(double maxValue, double minValue, double Value,
                                 float & nRed, float & nGreen, float & nBlue);
    void _changeValueToColorGray(double maxValue, double minValue, double Value,
        float& nRed, float& nGreen, float& nBlue);

    void setModelName(std::string name) {modelName=name;};
	std::string getModelName() { return modelName; };

	void setTransparent() { isTransparent = true; };


	//used for isosurface
	double maxLayerThickness, minLayerThickness;

	//used for voxel
	double voxelSize;
	bool voxelOrderComputed = false;


private:
    std::string modelName;
    bool isTransparent;
};

#endif // POLYGENMESH_H

