// Created by Prof.Charlie Wang on 6/10/15.
// Modified by daichengkai on 29-4-17.
// Modified by Guoxin Fang on 11-06-19

#pragma once

#ifndef VOXELFAB_BSP_TREE_H
#define VOXELFAB_BSP_TREE_H

#include <iostream>
#include "../Library/Eigen/Eigen"
#include <vector>
#include <QMeshPatch.h>

struct BBox {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector3d min;
	Eigen::Vector3d max;
};

class BSPTreeNode {
public:
	BSPTreeNode(void) {
		leftChild = rightChild = NULL;
		bSolid = false;
	};

	virtual ~BSPTreeNode(void) {
		if (leftChild != NULL) delete leftChild;
		if (rightChild != NULL) delete rightChild;
	};

	Eigen::Vector4d plane;        // the normal vector and the pnt vector of the plane - and left subspace is above this plane and right subspace is below the plane
	BSPTreeNode *leftChild, *rightChild;
	bool bSolid;    // true - this is a solid cell and its left/right child must be NULL
					// false - if (leftChild==NULL && rightChild==NULL), this is an empty cell; otherwise, this is a fuzzy cell.

	bool IsEmpty() { if ((!bSolid) && (leftChild == NULL) && (rightChild == NULL)) return true; else return false; };

	bool IsLeaveNode() { return (leftChild == NULL && rightChild == NULL); }


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class TriangleForBSP {
public:
	TriangleForBSP(void) {};

	virtual ~TriangleForBSP(void) {};

	std::vector<Eigen::Vector3d> vertices;
	Eigen::Vector4d plane;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class BSPTree {
public:
	BSPTree(void) {
		tree_root = new BSPTreeNode;
		tree_root->bSolid = false;
	};

	virtual ~BSPTree(void) { delete tree_root; };

	void BSPTreeFromMesh(QMeshPatch *mesh, int max_level_allowed = -1, bool orthogonal_clipping = true);

	bool IsInside(const Eigen::Vector3d &point);

	void MeshTrimming(QMeshPatch *mesh);

	void test();

private:
	bool ComputePlaneEquation(Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, Eigen::Vector4d &plane);

	bool ComputeBoundingBox(BBox &bbox, std::vector<TriangleForBSP> &triangles);

	void ConstructBSPNode(BSPTreeNode *tree_node, int level, Eigen::Vector4d last_clipping_plane,
		std::vector<TriangleForBSP> triangles, int max_level_allowed, bool orthogonal_clipping);

	double TriangleArea(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c);

	bool SplittingTriangle(TriangleForBSP &triangle, Eigen::Vector4d &plane, short &nStatus,
		std::vector<TriangleForBSP> &new_triangles, std::vector<bool> &positions);

	void ClippingPolyByPlane(std::vector<TriangleForBSP> &mesh, Eigen::Vector4d &plane, std::vector<TriangleForBSP> &upper_faces,
		std::vector<TriangleForBSP> &lower_faces);

	void ClippingPolyByBSPTree(std::vector<TriangleForBSP> mesh, BSPTreeNode *tree_node, std::vector<TriangleForBSP> &remesh);

	BSPTreeNode *tree_root;

	bool IsInside(const Eigen::Vector3d &point, BSPTreeNode *node);

	std::vector<TriangleForBSP> bsp_triangles;

	void Traverse(BSPTreeNode *node);

};

#endif //VOXELFAB_BSP_TREE_H
