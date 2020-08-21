#pragma once

#ifndef VOXELFAB_BST_H
#define VOXELFAB_BST_H

#include <algorithm>
#include <iostream>
#include <vector>
#include "../Library/Eigen/Eigen"
#include "../ShapeLab/MainWindow.h"

class TreeNode {
public:
	TreeNode(void) {
		leftChild = rightChild = NULL;
	};

	virtual ~TreeNode(void) {
		if (leftChild != NULL) delete leftChild;
		if (rightChild != NULL) delete rightChild;
	};

	int index;
	TreeNode *leftChild, *rightChild;
	TreeNode *opposite;

	TreeNode *parent;
	std::vector<int> sets;

	bool IsLeaveNode() { return (leftChild == NULL) && (rightChild == NULL); }
	bool IsRootNode() { return parent == NULL; }
};

class BSTree {
public:
	BSTree(void) { tree_root = new TreeNode; };
	~BSTree(void) { delete tree_root; };

	void BSTreeFromVoxelGrid(VOXELSET *voxSet, GLKArray *checkVoxelSet, 
		GLKArray *safeVoxelSet, CONVEXHULLSET *currentConvexFront);
	void BSTree::GetSafeVoxel(GLKArray *nextFront);

private:
	TreeNode *tree_root;
	std::vector<int> safe_indices_;
	std::vector<bool> visited_flag;

	int level = 0;
	int count = 0;

	void Traverse(TreeNode *node, GLKArray *checkVoxelSet, 
		VOXELSET *voxSet, CONVEXHULLSET *currentConvexFront);
	bool checkifCurrentSetisShadow(std::vector<int> &indices, 
		GLKArray *checkVoxelSet, VOXELSET *voxSet, CONVEXHULLSET *currentConvexFront);
	bool seperateVoxelSetHalfbyPCA(VOXELSET *voxSet, GLKArray *checkVoxelSet,
		std::vector<int> &indices, std::vector<int> &left_index, std::vector<int> &right_index);
};

#endif //VOXELFAB_BST_H
