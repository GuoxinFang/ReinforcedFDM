#include "stdafx.h"
#include "BSTree.h"

void BSTree::BSTreeFromVoxelGrid(VOXELSET *voxSet, GLKArray *checkVoxelSet,
	GLKArray *safeVoxelSet, CONVEXHULLSET *currentConvexFront)
{

	visited_flag = std::vector<bool>(checkVoxelSet->GetSize(), false);

	//build the root node
	std::vector<int> sets(checkVoxelSet->GetSize()); 
	for (int i = 0; i < checkVoxelSet->GetSize(); i++) sets[i] = i;
	//tree_root->parent = NULL;
	tree_root->sets = sets; // The sets install all the index of voxel.
	Traverse(tree_root, checkVoxelSet, voxSet, currentConvexFront);
}

void BSTree::Traverse(TreeNode *node, GLKArray *checkVoxelSet, 
	VOXELSET *voxSet, CONVEXHULLSET *currentConvexFront)
{
	count++;
	if (!node) return;
	std::vector<int> indices;

	for (int i = 0; i < safe_indices_.size(); ++i) indices.push_back(safe_indices_[i]);
	for (int i = 0; i < node->sets.size(); ++i) indices.push_back(node->sets[i]);

	// check the voxel set for this part will generate shadow node or not
	bool checked = this->checkifCurrentSetisShadow(indices, checkVoxelSet, voxSet, currentConvexFront);

	//if (node->sets.size()<2) count++; //this node cannot be devided anymore

	if (!checked) {
		for (int i = 0; i < node->sets.size(); ++i) safe_indices_.push_back(node->sets[i]);
		for (int i = 0; i<node->sets.size(); ++i) visited_flag[node->sets[i]] = true;
		//std::cout << "Count = " << count << "----This part is safe! Contains " << node->sets.size() << std::endl;
		return;
	}
	else {
		//std::cout << "Count = " << count << "----is not safe! Contains " << node->sets.size() << std::endl;
		node->leftChild = new TreeNode;
		node->rightChild = new TreeNode;

		if (node->sets.size()<2) {
			node->leftChild = NULL;
			node->rightChild = NULL;
		}
		else {
			std::vector<int> left_indices;
			std::vector<int> right_indices;
			seperateVoxelSetHalfbyPCA(voxSet, checkVoxelSet, node->sets, left_indices, right_indices);
			//std::cout << "Left indices number = " << left_indices.size() << ", Right indices number = " << right_indices.size() << std::endl;

			node->leftChild->sets = left_indices;
			node->rightChild->sets = right_indices;
		}
		this->Traverse(node->leftChild, checkVoxelSet, voxSet, currentConvexFront);
		this->Traverse(node->rightChild, checkVoxelSet, voxSet, currentConvexFront);
	}
	return;
}

bool BSTree::seperateVoxelSetHalfbyPCA(VOXELSET *voxSet, GLKArray *checkVoxelSet, std::vector<int> &indices, 
	std::vector<int> &left_index, std::vector<int> &right_index)
{
	left_index.clear(); // (pos[0],pos[1],pos[2]),(...),(...)
	right_index.clear();

	//Eigen::MatrixXd voxelPoints(indices.size(), 3);
	//for (int i = 0; i < indices.size(); i++) {
	//	VOXELSETNode *currentNode = (VOXELSETNode *)checkVoxelSet->GetAt(indices[i]);
	//	voxelPoints(i, 0) = voxSet->origin[0] + ((float)(currentNode->posIndex[0]) + 0.5f)*(voxSet->width);
	//	voxelPoints(i, 1) = voxSet->origin[1] + ((float)(currentNode->posIndex[1]) + 0.5f)*(voxSet->width);
	//	voxelPoints(i, 2) = voxSet->origin[2] + ((float)(currentNode->posIndex[2]) + 0.5f)*(voxSet->width);
	//}

	//Eigen::Vector3d centroid;
	//Eigen::Matrix3d covariance;
	//Eigen::MatrixXd eigenvectors;
	//Eigen::VectorXd eigenvalues;

	//centroid.setZero();
	//for (int i = 0; i < voxelPoints.rows(); i++) { for (int j = 0; j < 3; j++)  centroid(j) += voxelPoints(i, j); }
	//centroid /= voxelPoints.rows();

	//covariance.setZero();

	//for (unsigned int i = 0; i < voxelPoints.rows(); ++i) {
	//	Eigen::Vector3d pt;
	//	for (int j = 0; j < 3; j++)	pt[j] = voxelPoints(i, j) - centroid[j];

	//	covariance(1, 1) += pt.y() * pt.y(); //the non X parts
	//	covariance(1, 2) += pt.y() * pt.z();
	//	covariance(2, 2) += pt.z() * pt.z();

	//	pt *= pt.x();
	//	covariance(0, 0) += pt.x(); //the X related parts
	//	covariance(0, 1) += pt.y();
	//	covariance(0, 2) += pt.z();
	//}

	//covariance(1, 0) = covariance(0, 1); covariance(2, 0) = covariance(0, 2); covariance(2, 1) = covariance(1, 2);
	///* normalize  */
	//covariance /= voxelPoints.rows();

	//// Calculate eigenvectors
	//Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance, Eigen::ComputeEigenvectors);
	//Eigen::Matrix3d eigenvectorsMatrix = eigenSolver.eigenvectors();
	//eigenvectorsMatrix.col(2) = eigenvectorsMatrix.col(0).cross(eigenvectorsMatrix.col(1));

	///* Create the inverse of the transformation matrix and reproject the point cloud, saving it in a different array
	//* if reprojectInput is false or in the same array otherwise */
	//Eigen::Matrix4d transformationMatrixInv = Eigen::Matrix4d::Identity();
	//transformationMatrixInv.block<3, 3>(0, 0) = eigenvectorsMatrix.transpose();
	//transformationMatrixInv.block<3, 1>(0, 3) = -1.0F * (transformationMatrixInv.block<3, 3>(0, 0) * centroid);
	//Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	//transformationMatrix.block<3, 3>(0, 0) = eigenvectorsMatrix;
	//transformationMatrix.block<3, 1>(0, 3) = centroid;
	//Eigen::MatrixXd reproject_points;
	//reproject_points.resize(voxelPoints.rows(), 3);
	//for (int i = 0; i < voxelPoints.rows(); i++) {
	//	double x = voxelPoints(i, 0);
	//	double y = voxelPoints(i, 1);
	//	double z = voxelPoints(i, 2);

	//	double r_x = transformationMatrixInv(0, 0) * x + transformationMatrixInv(0, 1) * y +
	//		transformationMatrixInv(0, 2) * z + transformationMatrixInv(0, 3);
	//	double r_y = transformationMatrixInv(1, 0) * x + transformationMatrixInv(1, 1) * y +
	//		transformationMatrixInv(1, 2) * z + transformationMatrixInv(1, 3);
	//	double r_z = transformationMatrixInv(2, 0) * x + transformationMatrixInv(2, 1) * y +
	//		transformationMatrixInv(2, 2) * z + transformationMatrixInv(2, 3);

	//	reproject_points.row(i) = Eigen::RowVector3d(r_x, r_y, r_z);
	//}

	//// Calculate minimum and maximum points of the bounding box and the center
	//double min[3], max[3], center[3];
	//// finding min/max on x,y,z axis
	//for (int i = 0; i < 3; i++) {
	//	min[i] = DBL_MAX; max[i] = -DBL_MAX;
	//}

	//for (int i = 0; i < reproject_points.rows(); i++) {
	//	for (int j = 0; j < 3; j++) {
	//		if (reproject_points(i, j) < min[j]) min[j] = reproject_points(i, j);
	//		if (reproject_points(i, j) < max[j]) max[j] = reproject_points(i, j);
	//	}
	//}

	//double centerZ = (min[2] + max[2]) / 2;

	//for (int i = 0; i < reproject_points.rows(); i++) {
	//	if (reproject_points(i, 2) <= centerZ) left_index.push_back(indices[i]);
	//	else right_index.push_back(indices[i]);
	//}

	for (int i = 0; i < indices.size(); i++) {
		if (i % 2 == 0) left_index.push_back(indices[i]);
		else right_index.push_back(indices[i]);
	}

	return true;
}

bool BSTree::checkifCurrentSetisShadow(std::vector<int> &indices, GLKArray *checkVoxelSet, VOXELSET *voxSet, CONVEXHULLSET *currentConvexFront) {
	int temp = indices.size();
	GLKArray *shadowSafeVoxel = new GLKArray(temp, temp, 0);
	double distTolerance = 0.86602540378*(voxSet->width);

	for (int i = 0; i < temp; i++) {
		shadowSafeVoxel->Add(checkVoxelSet->GetAt(indices[i]));
	}
	int shadowPointNum_thisVoxel = VOXSetOperation::checkifVoxelSetgetShadowed(voxSet, shadowSafeVoxel, currentConvexFront, NULL, distTolerance);

	shadowSafeVoxel->RemoveAll();
	delete shadowSafeVoxel;

	if (shadowPointNum_thisVoxel > 0)
		return true;
	else
		return false;
}

void BSTree::GetSafeVoxel(GLKArray *nextFront) {
	int temp = nextFront->GetSize();
	int safeNum = safe_indices_.size();
	GLKArray *shadowSafeVoxelSet = new GLKArray(temp, temp, 0);
	for (int i = 0; i < safe_indices_.size(); i++) {
		shadowSafeVoxelSet->Add(nextFront->GetAt(safe_indices_[i]));
	}
	nextFront->RemoveAll();
	for (int i = 0; i < safe_indices_.size(); i++) {
		nextFront->Add(shadowSafeVoxelSet->GetAt(i));
	}
	//shadowSafeVoxelSet->RemoveAll();
	//delete shadowSafeVoxelSet;
}