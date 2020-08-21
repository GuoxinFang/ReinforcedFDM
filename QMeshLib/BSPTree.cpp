// Created by Prof.Charlie Wang on 6/10/15.
// Created and modified by daichengkai on 29-4-17.
// Modified by Guoxin Fang on 11-06-19
// Noticed that this is not used in the project.

#include "BSPTree.h"

void BSPTree::BSPTreeFromMesh(QMeshPatch *mesh, int max_level_allowed, bool orthogonal_clipping) {
	
	for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)mesh->GetFaceList().GetNext(Pos);

		Eigen::Vector3d p1, p2, p3;
		Face->GetNodeRecordPtr(0)->GetCoord3D(p1(0), p1(1), p1(2));
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2(0), p2(1), p2(2));
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3(0), p3(1), p3(2));

		Eigen::Vector4d plane;
		if (this->ComputePlaneEquation(p1, p2, p3, plane)) {
			TriangleForBSP tri_face;
			tri_face.vertices.clear();
			tri_face.vertices.push_back(p1);
			tri_face.vertices.push_back(p2);
			tri_face.vertices.push_back(p3);
			tri_face.plane = plane;
			bsp_triangles.push_back(tri_face);
		}
	}

	Eigen::Vector4d last_clipping_plane(1.0, 0.0, 0.0, 0.0);
	this->ConstructBSPNode(tree_root, 0, last_clipping_plane, bsp_triangles, max_level_allowed, orthogonal_clipping);
	std::cout << "Finish build the BSP tree!" << std::endl;
}

bool BSPTree::ComputePlaneEquation(Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, Eigen::Vector4d &plane) {

	Eigen::Vector3d v1, v2;
	v1 = p2 - p1;
	v2 = p3 - p1;

	Eigen::Vector3d normal;
	normal = v1.cross(v2);
	if (normal.norm() < 1.0e-20)
		return false;
	normal.normalize();

	double aa, bb, cc, dd;
	aa = normal(0);
	bb = normal(1);
	cc = normal(2);
	dd = -(p1.dot(normal));

	plane = Eigen::Vector4d(aa, bb, cc, dd);

	return true;

}

bool BSPTree::ComputeBoundingBox(BBox &bbox, std::vector<TriangleForBSP> &triangles) {

	Eigen::MatrixXd vertices;
	vertices.resize(triangles.size() * 3, 3);
	for (size_t i = 0; i < triangles.size(); ++i) {
		vertices.row(i * 3) = triangles[i].vertices[0];
		vertices.row(i * 3 + 1) = triangles[i].vertices[1];
		vertices.row(i * 3 + 2) = triangles[i].vertices[2];
	}
	bbox.min = vertices.colwise().minCoeff();
	bbox.max = vertices.colwise().maxCoeff();

	return true;
}

double BSPTree::TriangleArea(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c) {

	Eigen::Vector3d vBA = b - a;
	Eigen::Vector3d vCA = c - a;
	Eigen::Vector3d crossPdk = vBA.cross(vCA);
	double area = crossPdk.norm() * 0.5f;
	return area;
}


void BSPTree::ConstructBSPNode(BSPTreeNode *tree_node, int level, Eigen::Vector4d last_clipping_plane,
	std::vector<TriangleForBSP> triangles, int max_level_allowed, bool orthogonal_clipping) {

	//    std::cout << "level " << level << std::endl;
	//    std::cout << "num " << triangles.size() << std::endl;
	//    std::cout << "last plane " << last_clipping_plane(0) << " " << last_clipping_plane(1) << " "
	//              << last_clipping_plane(2) << " " << last_clipping_plane(3) << std::endl;

	bool orthogonal_cutting = false;
	const int levelThreshold = 12;
	const int trglNumThreshold = 30;
	bool bClippingByTrglFace = false;


	double eps = 1.0e-5;
	std::vector<TriangleForBSP> upper_triangles;
	std::vector<TriangleForBSP> lower_triangles;
	double a, b, c, d;
	a = b = c = d = 0.0;
	a = 1.0;
	//------------------------------------------------------------------------------------------------
	//	Step 1: determine the cutting plane
	if (max_level_allowed > 0 && level >= max_level_allowed) {
		std::cout << "Limited Max Level" << max_level_allowed << " is reached!\n" << std::endl;
		tree_node->bSolid = true;
		return;
	}
	if (orthogonal_clipping && (level < levelThreshold) && (triangles.size() > trglNumThreshold)) {
		BBox bbox;
		this->ComputeBoundingBox(bbox, triangles);
		Eigen::Vector3d center_point = 0.5 * (bbox.max + bbox.min);
		Eigen::Vector3d normal = Eigen::Vector3d::Zero();
		normal(level % 3) = 1.0;
		a = normal(0);
		b = normal(1);
		c = normal(2);
		d = -(center_point.dot(normal));
		tree_node->plane = Eigen::Vector4d(a, b, c, d);
		last_clipping_plane = Eigen::Vector4d(a, b, c, d);
		orthogonal_cutting = true;
		if ((fabs(bbox.max(0) - bbox.min(0)) < 1.0e-4) ||
			(fabs(bbox.max(1) - bbox.min(1)) < 1.0e-4) ||
			(fabs(bbox.max(2) - bbox.min(2)) < 1.0e-4))
			orthogonal_cutting = false;
	}
	size_t st = -1;
	if (!orthogonal_cutting) {
		//--------------------------------------------------------------------------------------------
		//	Search the clipping plane which is most perpendicular to last clipping plane
		double dot, minDot = 1.0e+10f;
		for (size_t i = 0; i < triangles.size(); i++) {
			Eigen::Vector3d current_normal = triangles[i].plane.segment(0, 3);
			Eigen::Vector3d last_clipping_plane_normal = last_clipping_plane.segment(0, 3);
			dot = current_normal.dot(last_clipping_plane_normal);
			if (dot < minDot) {
				minDot = dot;
				st = i;
			}

		}

		if (st >= 0) {
			tree_node->plane = triangles[st].plane;
			last_clipping_plane = triangles[st].plane;
			bClippingByTrglFace = true;
		}
		else
			tree_node->plane = Eigen::Vector4d(a, b, c, d);

	}
	//------------------------------------------------------------------------------------------------
	//	Step 2: splitting the triangles into two groups
	double pos_area, neg_area;
	pos_area = neg_area = 0.0;

	for (size_t i = 0; i < triangles.size(); i++) {

		//--------------------------------------------------------------------------------------------
		//	filtering unnecessary triangles
		if (i == st) {
			pos_area += TriangleArea(triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2]);
			continue;
		}

		for (size_t j = 0; j < 3; ++j) {
			Eigen::Vector3d edge = triangles[i].vertices[j % 3] - triangles[i].vertices[(j + 1) % 3];
			double edge_length = edge.norm();
			if (edge_length < eps) {
				continue;
			}
		}

		std::vector<TriangleForBSP> new_triangles;
		std::vector<bool> positions;
		short nState;

		if (this->SplittingTriangle(triangles[i], last_clipping_plane, nState, new_triangles, positions)) {
			if (new_triangles.size()) {
				for (size_t i = 0; i < positions.size(); ++i) {
					if (positions[i] == true)
						upper_triangles.push_back(new_triangles[i]);
					else
						lower_triangles.push_back(new_triangles[i]);
				}
			}
		}
		else {

			if (nState == 0) {
				pos_area += TriangleArea(triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2]);
			}
			else if (nState == 1) {
				lower_triangles.push_back(triangles[i]);
			}
			else if (nState == 2) {
				upper_triangles.push_back(triangles[i]);
			}
			else {
				neg_area += TriangleArea(triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2]);
			}
		}
	}

	if ((neg_area - pos_area) > 0.5 * pos_area && (!orthogonal_clipping)) {
		upper_triangles.swap(lower_triangles);
		tree_node->plane = -tree_node->plane;
	}

	//    std::cout << "upper and lower" << std::endl;
	//    std::cout << upper_triangles.size() << " " << lower_triangles.size() << std::endl;

	//------------------------------------------------------------------------------------------------
	//	Step 4: construct the children BSP-tree node

	//------------------------------------------------------------------------------------------------
	//	for left-child
	tree_node->leftChild = new BSPTreeNode;
	tree_node->leftChild->bSolid = false;
	if (upper_triangles.size() > 0)
		ConstructBSPNode(tree_node->leftChild, level + 1, tree_node->plane, upper_triangles, max_level_allowed,
			orthogonal_clipping);

	//	for right-child
	tree_node->rightChild = new BSPTreeNode;
	tree_node->rightChild->bSolid = false;
	if (lower_triangles.size() > 0)
		ConstructBSPNode(tree_node->rightChild, level + 1, tree_node->plane, lower_triangles, max_level_allowed,
			orthogonal_clipping);
	else {
		if (bClippingByTrglFace) {
			tree_node->rightChild->bSolid = true;
			tree_node->rightChild->plane = bsp_triangles[1].plane;
		}
		else {
			tree_node->rightChild->bSolid = false;
			tree_node->rightChild->plane = bsp_triangles[0].plane;
		}
	}
}

bool BSPTree::SplittingTriangle(TriangleForBSP &triangle, Eigen::Vector4d &plane, short &nStatus,
	std::vector<TriangleForBSP> &new_triangles, std::vector<bool> &positions) {

	//	nStatus -	0 (on the plane)
	//				1 (below the plane)
	//				2 (above the plane)
	const double eps = 1.0e-5;
	bool bAboveFound, bBelowFound;
	short nVerStatus[3];
	new_triangles.clear();
	positions.clear();

	bAboveFound = bBelowFound = false;
	//--------------------------------------------------------------------------------------------------------
	//	Detect the position of vertices relative to the plane
	for (size_t i = 0; i < 3; i++) {

		double distance;
		distance = triangle.vertices[i](0) * plane(0) + triangle.vertices[i](1) * plane(1) +
			triangle.vertices[i](2) * plane(2) + plane(3);
		if (distance > eps) {
			bAboveFound = true;
			nVerStatus[i] = 2;
		}
		else if (distance < -eps) {
			bBelowFound = true;
			nVerStatus[i] = 1;
		}
		else { nVerStatus[i] = 0; }

	}

	//--------------------------------------------------------------------------------------------------------
	//	If the triangle has no intersection with the plane, detect its relative position to the plane
	if (!(bAboveFound && bBelowFound)) {
		if ((!bAboveFound) && (!bBelowFound)) nStatus = 0;
		else if (bBelowFound) nStatus = 1;
		else nStatus = 2;
		if (nStatus == 0) {
			double sign = triangle.plane(0) * plane(0) + triangle.plane(1) * plane(1) + triangle.plane(2) * plane(2);
			if (sign < 0.0) nStatus = 3;
		}
		return false;
	}



	//--------------------------------------------------------------------------------------------------------
	//	Create new triangles
	double d1, d2, alpha;
	std::vector<Eigen::Vector3d> intersect_points(2);
	int intersect_edge_index[2], intersect_points_num = 0;
	for (size_t i = 0; i < 3; i++) {
		if (nVerStatus[i] == 0 || nVerStatus[(i + 1) % 3] == 0) continue;
		if (nVerStatus[i] == nVerStatus[(i + 1) % 3]) continue;
		intersect_edge_index[intersect_points_num] = i;
		d1 = triangle.vertices[i](0) * plane(0) + triangle.vertices[i](1) * plane(1) +
			triangle.vertices[i](2) * plane(2) + plane(3);
		d2 = triangle.vertices[(i + 1) % 3](0) * plane(0) + triangle.vertices[(i + 1) % 3](1) * plane(1) +
			triangle.vertices[(i + 1) % 3](2) * plane(2) + plane(3);

		alpha = fabs(d1) + fabs(d2);    //if (alpha<1.0e-8) alpha=1.0;
		alpha = fabs(d1) / alpha;

		intersect_points[intersect_points_num] =
			(1.0 - alpha) * triangle.vertices[i] + alpha * triangle.vertices[(i + 1) % 3];

		intersect_points_num++;
		if (intersect_points_num == 2) break;
	}
	if (intersect_points_num == 1) {    // Two triangles are created
		int current_index = intersect_edge_index[0];
		TriangleForBSP triangle1;
		triangle1.vertices.resize(3);
		triangle1.vertices[0] = intersect_points[0];
		triangle1.vertices[1] = triangle.vertices[(current_index + 1) % 3];
		triangle1.vertices[2] = triangle.vertices[(current_index + 2) % 3];
		triangle1.plane = triangle.plane;
		new_triangles.push_back(triangle1);
		if (nVerStatus[(current_index + 1) % 3] == 2)
			positions.push_back(true);
		else
			positions.push_back(false);

		TriangleForBSP triangle2;
		triangle2.vertices.resize(3);
		triangle2.vertices[0] = intersect_points[0];
		triangle2.vertices[1] = triangle.vertices[(current_index + 2) % 3];
		triangle2.vertices[2] = triangle.vertices[(current_index + current_index) % 3];
		triangle2.plane = triangle.plane;
		new_triangles.push_back(triangle2);
		if (nVerStatus[current_index] == 2)
			positions.push_back(true);
		else
			positions.push_back(false);

	}
	else if (intersect_points_num == 2) {    // Three triangles are created
		int j;
		int current_index;
		if ((intersect_edge_index[0] + 1) % 3 == intersect_edge_index[1]) {
			current_index = intersect_edge_index[0];
			j = 0;
		}
		else {
			current_index = intersect_edge_index[1];
			j = 1;
		}

		TriangleForBSP triangle1;
		triangle1.vertices.resize(3);
		triangle1.vertices[0] = intersect_points[j];
		triangle1.vertices[1] = triangle.vertices[(current_index + 1) % 3];
		triangle1.vertices[2] = intersect_points[(j + 1) % 2];
		triangle1.plane = triangle.plane;
		new_triangles.push_back(triangle1);
		if (nVerStatus[(current_index + 1) % 3] == 2)
			positions.push_back(true);
		else
			positions.push_back(false);

		d1 = (intersect_points[j] - triangle.vertices[(current_index + 2) % 3]).norm();
		d2 = (intersect_points[(j + 1) % 2] - triangle.vertices[current_index]).norm();

		if (d1 < d2) {
			TriangleForBSP triangle2;
			triangle2.vertices.resize(3);
			triangle2.vertices[0] = intersect_points[j];
			triangle2.vertices[1] = intersect_points[(j + 1) % 2];
			triangle2.vertices[2] = triangle.vertices[(current_index + 2) % 3];
			triangle2.plane = triangle.plane;
			new_triangles.push_back(triangle2);
			if (nVerStatus[(current_index + 2) % 3] == 2)
				positions.push_back(true);
			else
				positions.push_back(false);

			TriangleForBSP triangle3;
			triangle3.vertices.resize(3);
			triangle3.vertices[0] = intersect_points[j];
			triangle3.vertices[1] = triangle.vertices[(current_index + 2) % 3];
			triangle3.vertices[2] = triangle.vertices[current_index];
			triangle3.plane = triangle.plane;
			new_triangles.push_back(triangle3);
			if (nVerStatus[(current_index + 2) % 3] == 2)
				positions.push_back(true);
			else
				positions.push_back(false);

		}
		else {
			TriangleForBSP triangle2;
			triangle2.vertices.resize(3);
			triangle2.vertices[0] = intersect_points[j];
			triangle2.vertices[1] = intersect_points[(j + 1) % 2];
			triangle2.vertices[2] = triangle.vertices[current_index];
			triangle2.plane = triangle.plane;
			new_triangles.push_back(triangle2);
			if (nVerStatus[current_index] == 2)
				positions.push_back(true);
			else
				positions.push_back(false);

			TriangleForBSP triangle3;
			triangle3.vertices.resize(3);
			triangle3.vertices[0] = triangle.vertices[current_index];
			triangle3.vertices[1] = intersect_points[(j + 1) % 2];
			triangle3.vertices[2] = triangle.vertices[(current_index + 2) % 3];
			triangle3.plane = triangle.plane;
			new_triangles.push_back(triangle3);
			if (nVerStatus[current_index] == 2)
				positions.push_back(true);
			else
				positions.push_back(false);
		}
	}
	else {
		std::cout << "Warning: no intersection point is found (trgl Splitting)!" << std::endl;
	}
	return true;
}

bool BSPTree::IsInside(const Eigen::Vector3d &point, BSPTreeNode *node) {

	double distance;
	Eigen::Vector4d plane;
	plane = node->plane;

	distance = point(0) * plane(0) + point(1) * plane(1) +
		point(2) * plane(2) + plane(3);

	if (distance < 0) {
		if (!node->rightChild->IsLeaveNode())
			return IsInside(point, node->rightChild);
		else
			return true;
	}
	else {
		if (!node->leftChild->IsLeaveNode())
			return IsInside(point, node->leftChild);
		else
			return false;
	}

}

bool BSPTree::IsInside(const Eigen::Vector3d &point) {

	return IsInside(point, tree_root);
}


void BSPTree::test() {
	std::cout << "--------------test-------------" << std::endl;
	this->Traverse(tree_root);
	std::cout << "-------------------------------" << std::endl;



}
void BSPTree::Traverse(BSPTreeNode *node)
{
	if (node->leftChild) {
		std::cout << node->leftChild->plane(0) << " " << node->leftChild->plane(1) << " "
			<< node->leftChild->plane(2) << " " << node->leftChild->plane(3) << std::endl;
		std::cout << node->leftChild->bSolid << std::endl;

		Traverse(node->leftChild);
	}
}

void BSPTree::ClippingPolyByBSPTree(std::vector<TriangleForBSP> mesh, BSPTreeNode *tree_node,
	std::vector<TriangleForBSP> &remesh) {

	std::vector<TriangleForBSP> upper_triangles;
	std::vector<TriangleForBSP> lower_triangles;
	this->ClippingPolyByPlane(mesh, tree_node->plane, upper_triangles, lower_triangles);
	if (tree_node->rightChild->bSolid) {
		for (size_t i = 0; i < lower_triangles.size(); ++i)
			remesh.push_back(lower_triangles[i]);
	}
	if ((!tree_node->leftChild->IsLeaveNode()) && upper_triangles.size())
		ClippingPolyByBSPTree(upper_triangles, tree_node->leftChild, remesh);
	if ((!tree_node->rightChild->IsLeaveNode()) && lower_triangles.size())
		ClippingPolyByBSPTree(lower_triangles, tree_node->rightChild, remesh);
}

void BSPTree::ClippingPolyByPlane(std::vector<TriangleForBSP> &mesh, Eigen::Vector4d &plane,
	std::vector<TriangleForBSP> &upper_triangles,
	std::vector<TriangleForBSP> &lower_triangles) {

	upper_triangles.clear();
	lower_triangles.clear();
	for (size_t i = 0; i < mesh.size(); ++i) {
		TriangleForBSP triangle;
		triangle = mesh[i];
		short nState;
		std::vector<bool> positions;
		std::vector<TriangleForBSP> new_triangles;
		if (this->SplittingTriangle(triangle, plane, nState, new_triangles, positions)) {
			if (new_triangles.size()) {
				for (size_t i = 0; i < positions.size(); ++i) {
					if (positions[i] == true)
						upper_triangles.push_back(new_triangles[i]);
					else
						lower_triangles.push_back(new_triangles[i]);
				}
			}
		}
		else {
			if (nState == 1) {
				lower_triangles.push_back(triangle);
			}
			else if (nState == 2) {
				upper_triangles.push_back(triangle);
			}
		}
	}
}

void BSPTree::MeshTrimming(QMeshPatch *mesh) {
	//std::vector<TriangleForBSP> mesh_soup;
	//mesh_soup.resize(mesh->GetFaceNumber());
	//for (size_t i = 0; i<mesh_soup.size(); ++i) {
	//	mesh_soup[i].vertices.resize(3);
	//	mesh_soup[i].vertices[0] = mesh->vectices.row(mesh->faces(i, 0));
	//	mesh_soup[i].vertices[1] = mesh->vectices.row(mesh->faces(i, 1));
	//	mesh_soup[i].vertices[2] = mesh->vectices.row(mesh->faces(i, 2));
	//}
	//std::vector<TriangleForBSP> remesh;
	//ClippingPolyByBSPTree(mesh_soup, tree_root, remesh);
	//Eigen::MatrixXd new_vertices;
	//Eigen::MatrixXi new_faces;
	//new_vertices.resize(3 * remesh.size(), 3);
	//new_faces.resize(remesh.size(), 3);
	//for (size_t i = 0; i<remesh.size(); ++i) {
	//	new_vertices.row(3 * i + 0) = remesh[i].vertices[0];
	//	new_vertices.row(3 * i + 1) = remesh[i].vertices[1];
	//	new_vertices.row(3 * i + 2) = remesh[i].vertices[2];
	//	new_faces.row(i) = Eigen::RowVector3i(3 * i + 0, 3 * i + 1, 3 * i + 2);
	//}
	//mesh->vectices = new_vertices;
	//mesh->faces = new_faces;
}
