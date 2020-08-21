//////
////// Created by daichengkai on 1-5-17.
//////
////Notice that this code is not used in the project.
#include <queue>
//#include "bst.h"

#include "QMeshVoxel.h"

Eigen::MatrixXd append_vertices(const Eigen::MatrixXd &v1, const Eigen::MatrixXd &v2) {

	Eigen::MatrixXd new_vertices;
	new_vertices.resize(v1.rows() + v2.rows(), 3);

	for (auto i = 0; i < v1.rows(); ++i)
		new_vertices.row(i) << v1.row(i);

	for (auto i = 0; i < v2.rows(); ++i)
		new_vertices.row(i + v1.rows()) << v2.row(i);

	return new_vertices;
}


////https://stackoverflow.com/questions/12991758/creating-all-possible-k-combinations-of-n-items-in-c
void comb(int n, int k, std::vector<std::vector<int> > &combinations) {
	combinations.clear();
	using namespace std;

	vector<int> selected;
	vector<int> selector(n);
	fill(selector.begin(), selector.begin() + k, 1);
	do {
		for (int i = 0; i < n; i++) {
			if (selector[i]) {
				selected.push_back(i);
			}
		}
		combinations.push_back(selected);
		selected.clear();
	} while (prev_permutation(selector.begin(), selector.end()));
}

double point2plane(Eigen::Vector3d &point, Eigen::Vector4d &plane) {
	return fabs(point(0) * plane(0) + point(1) * plane(1) + point(2) * plane(2) + plane(3)) /
		sqrt(plane(0) * plane(0) + plane(1) * plane(1) + plane(2) * plane(2));
}

VoxelGrid::VoxelGrid(const Eigen::MatrixXd &V, double voxel_size, int offset) {

	this->mesh_vertices = V;
	this->size = voxel_size;
	this->bounding_box.min = V.colwise().minCoeff();
	this->bounding_box.max = V.colwise().maxCoeff();

	int i = 0;
	for (int x = -1; x < 2; x++) {
		for (int y = -1; y < 2; y++) {
			for (int z = -1; z < 2; z++) {
				xoff[i] = x;
				yoff[i] = y;
				zoff[i] = z;
				i++;
			}
		}
	}

	this->init(offset);

	for (i = 0; i < voxels_.size(); ++i) {
		voxels_[i]->peeling_order = 0;
		voxels_[i]->processed = false;
		voxels_[i]->cluster = 0;
		voxels_[i]->preserved = false;
		voxels_[i]->layer = 0;
		voxels_[i]->visited = false;

		voxels_[i]->convex_vertices = false;
	}

	max_peeling_order = 0;
	min_peeling_order = 9999;
	max_growing_order = 0;
	min_growing_order = 9999;
}

//VoxelGrid::VoxelGrid(std::string filename) {
//	this->size = 0.8;
//
//	std::ifstream file(filename);
//
//	std::string line;
//	std::string first_line;
//
//	int count = 0;
//	std::getline(file, first_line);
//	std::vector<std::string> first_line_elements;
//
//	boost::algorithm::split(first_line_elements, first_line, boost::algorithm::is_any_of(" "));
//
//	int res_x = stoi(first_line_elements[0]);
//	int res_y = stoi(first_line_elements[1]);
//	int res_z = stoi(first_line_elements[2]);
//	res = Eigen::Vector3i(res_x, res_y, res_z);
//	for (int zi = 0; zi < res(2); zi++) {
//		for (int yi = 0; yi < res(1); yi++) {
//			for (int xi = 0; xi < res(0); xi++) {
//				voxel *vox = new voxel;
//				vox->x = xi;
//				vox->y = yi;
//				vox->z = zi;
//				vox->filled = false;
//				vox->enlarged = false;
//				vox->layer = 0;
//				vox->peeling_order = 0;
//				voxels_.push_back(vox);
//			}
//		}
//	}
//
//	while (getline(file, line)) {
//
//		std::vector<std::string> line_elements;
//
//		boost::algorithm::split(line_elements, line, boost::algorithm::is_any_of(" "));
//
//		int index_x = stoi(line_elements[0]);
//		int index_y = stoi(line_elements[1]);
//		int index_z = stoi(line_elements[2]);
//		double x = stod(line_elements[3]);
//		double y = stod(line_elements[4]);
//		double z = stod(line_elements[5]);
//
//		voxel *vox = this->GetVoxel(index_x, index_y, index_z);
//		vox->position = Eigen::Vector3d(x, y, z);
//		vox->filled = true;
//		vox->peeling_order = stoul(line_elements[7]);
//		vox->layer = stoul(line_elements[6]);
//
//	}
//
//	max_peeling_order = 0;
//	min_peeling_order = 9999;
//	max_growing_order = 0;
//	min_growing_order = 9999;
//
//}

VoxelGrid::~VoxelGrid() {

	for (auto it = voxels_.begin(); it != voxels_.end(); ++it) {
		delete (*it);
	}
	voxels_.clear();
}

//void VoxelGrid::Load(std::string filename) {
//
//	std::ifstream file(filename);
//
//	std::string line;
//	std::string first_line;
//
//	int count = 0;
//	std::getline(file, first_line);
//	std::vector<std::string> first_line_elements;
//
//	boost::algorithm::split(first_line_elements, first_line, boost::algorithm::is_any_of(" "));
//
//
//	while (getline(file, line)) {
//
//		std::vector<std::string> line_elements;
//
//		boost::algorithm::split(line_elements, line, boost::algorithm::is_any_of(" "));
//
//		int index_x = stoi(line_elements[0]);
//		int index_y = stoi(line_elements[1]);
//		int index_z = stoi(line_elements[2]);
//		double x = stod(line_elements[3]);
//		double y = stod(line_elements[4]);
//		double z = stod(line_elements[5]);
//
//		voxel *vox = this->GetVoxel(index_x, index_y, index_z);
//		vox->filled = true;
//		vox->peeling_order = stoul(line_elements[7]);
//		vox->layer = stoul(line_elements[6]);
//
//	}
//
//}

void VoxelGrid::voxelVisualization(PolygenMesh* voxelMesh) {
	QMeshPatch *voxelModel = new QMeshPatch;
	voxelModel->SetIndexNo(voxelMesh->GetMeshList().GetCount()); //index begin from 0
	voxelMesh->meshList.AddTail(voxelModel);

	std::vector<voxel *> solid_voxels;
	std::vector<voxel *> boundary_voxels;
	this->getSolidVoxels(solid_voxels);
	this->findVoxelsOnBoundary(solid_voxels, boundary_voxels);

	for (int i = 0; i < boundary_voxels.size(); i++) {
		QMeshNode* Node = new QMeshNode;
		Node->SetCoord3D(boundary_voxels[i]->x, boundary_voxels[i]->y, boundary_voxels[i]->z);
		Node->SetMeshPatchPtr(voxelModel);
		Node->SetIndexNo(voxelModel->GetNodeList().GetCount() + 1);
		voxelModel->GetNodeList().AddTail(Node);

		//detect the boundary voxel in which direction
		voxel *thisVoxel = boundary_voxels[i];
		int index[3] = { thisVoxel->x, thisVoxel->y, thisVoxel->z };
		if (GetVoxel(index[0] + 1, index[1], index[2]) == nullptr ||
			GetVoxel(index[0] + 1, index[1], index[2])->filled == false) Node->voxelFlags[0] = true;
		if (GetVoxel(index[0] - 1, index[1], index[2]) == nullptr ||
			GetVoxel(index[0] - 1, index[1], index[2])->filled == false) Node->voxelFlags[1] = true;
		if (GetVoxel(index[0], index[1] + 1, index[2]) == nullptr ||
			GetVoxel(index[0], index[1] + 1, index[2])->filled == false) Node->voxelFlags[2] = true;
		if (GetVoxel(index[0], index[1] - 1, index[2]) == nullptr ||
			GetVoxel(index[0], index[1] - 1, index[2])->filled == false) Node->voxelFlags[3] = true;
		if (GetVoxel(index[0], index[1], index[2] + 1) == nullptr ||
			GetVoxel(index[0], index[1], index[2] + 1)->filled == false) Node->voxelFlags[4] = true;
		if (GetVoxel(index[0], index[1], index[2] - 1) == nullptr ||
			GetVoxel(index[0], index[1], index[2] - 1)->filled == false) Node->voxelFlags[5] = true;
	}
	voxelMesh->voxelSize = this->size;
}

void VoxelGrid::getMinMaxLayer(unsigned &max, unsigned &min) {
	for (int i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->layer) {
			if (voxels_[i]->layer < min_growing_order)
				min_growing_order = voxels_[i]->layer;
			if (voxels_[i]->layer > max_growing_order)
				max_growing_order = voxels_[i]->layer;
		}
	}
	max = max_growing_order;
	min = min_growing_order;
}

void VoxelGrid::getMinMaxLayer(std::vector<voxel *> &voxels, unsigned &max, unsigned &min) {
	int max_order = 0;
	int min_order = 9999;
	for (int i = 0; i < voxels.size(); ++i) {
		if (voxels[i]->layer) {
			if (voxels[i]->layer < min_order)
				min_order = voxels[i]->layer;
			if (voxels[i]->layer > max_order)
				max_order = voxels[i]->layer;
		}
	}
	max = max_order;
	min = min_order;
}


//void VoxelGrid::loadPlatform(const std::string &path) {
//	igl::readOBJ(path, platform_vertices, platform_faces);
//}

void VoxelGrid::getAllVoxels(std::vector<voxel *> &all_voxels) {
	all_voxels.clear();
	all_voxels = voxels_;
}

neighs *VoxelGrid::GetNeighbourhood(voxel *voxel) {

	neighs *neigh = new neighs();
	for (size_t i = 0; i < 27; i++) {
		int x, y, z;
		x = voxel->x;
		y = voxel->y;
		z = voxel->z;
		neigh->voxs[i] = this->GetVoxel(x + xoff[i], y + yoff[i], z + zoff[i]);
	}
	return neigh;
}


voxel *VoxelGrid::GetVoxel(int x, int y, int z) {

	if (x < 0 || x >= res(0) || y < 0 || y >= res(1) || z < 0 || z >= res(2))
		return nullptr;
	else
		return voxels_[x + res(0) * (y + res(1) * z)];

}

int VoxelGrid::getIndex(int x, int y, int z) {
	return x + res(0) * (y + res(1) * z);
}


void VoxelGrid::init(int offset) {

	Eigen::Vector3d vmin = bounding_box.min;
	Eigen::Vector3d vmax = bounding_box.max;

	res(0) = (int)ceil((vmax(0) - vmin(0)) / size);
	res(1) = (int)ceil((vmax(1) - vmin(1)) / size);
	res(2) = (int)ceil((vmax(2) - vmin(2)) / size);

	offset_ = offset;

	const Eigen::Vector3d old_cen = 0.5 * (vmin + vmax);
	vmax = vmin + size * res.cast<double>() +
		Eigen::Vector3d(double(offset) * size, double(offset) * size, double(offset) * size);
	const Eigen::Vector3d cen = 0.5 * (vmin + vmax);

	res += Eigen::Vector3i(offset, offset, offset);
	bounding_box.min = vmin + old_cen - cen;
	bounding_box.max = bounding_box.min + size * res.cast<double>();

	for (int zi = 0; zi < res(2); zi++) {
		const auto lerp = [&](const int di, const int d) -> double {
			return bounding_box.min(d) + 0.5 * size +
				(double)di / (double)(res(d)) * (bounding_box.max(d) - bounding_box.min(d));
		};
		const double z = lerp(zi, 2);
		for (int yi = 0; yi < res(1); yi++) {
			const double y = lerp(yi, 1);
			for (int xi = 0; xi < res(0); xi++) {
				const double x = lerp(xi, 0);
				voxel *vox = new voxel;
				vox->x = xi;
				vox->y = yi;
				vox->z = zi;
				vox->position = Eigen::Vector3d(x, y, z);
				vox->filled = false;
				vox->enlarged = false;
				voxels_.push_back(vox);
			}
		}
	}


	std::cout << "\nVoxel Grids Initialized ..." << std::endl;

}

void VoxelGrid::Voxelization(BSPTree *bsp_tree) {
	for (size_t i = 0; i < voxels_.size(); ++i) {
		if (bsp_tree->IsInside(voxels_[i]->position)) {
			voxels_[i]->filled = true;
		}
	}

	std::cout << "\nVoxelization Completed ..." << std::endl;

	std::vector<voxel *> solid_voxels;
	this->getSolidVoxels(solid_voxels);
	std::cout << "solid voxel size " << solid_voxels.size() << std::endl;

}

void VoxelGrid::GetBoundarySolidVoxels(Eigen::MatrixXd &GV) {
	std::vector<Eigen::Vector3d> points;
	std::vector<voxel *> solid_voxels;
	std::vector<voxel *> boundary_voxels;
	this->getSolidVoxels(solid_voxels);
	this->findVoxelsOnBoundary(solid_voxels, boundary_voxels);
	std::cout << "solid " << solid_voxels.size() << std::endl;
	std::cout << "boundary " << boundary_voxels.size() << std::endl;

	for (size_t i = 0; i < boundary_voxels.size(); ++i) {
		points.push_back(boundary_voxels[i]->position);
	}
	GV.resize(points.size(), 3);
	for (size_t i = 0; i < points.size(); ++i) {
		GV.row(i) = points[i];
	}
}

void VoxelGrid::Enlarge() {

	if (offset_ == 0) {
		std::cout << "No offset voxel grid, can not enlarge" << std::endl;
		exit(-1);
	}
	for (size_t i = 0; i < voxels_.size(); ++i) {

		if (voxels_[i]->filled) {
			neighs *neighbourhood = this->GetNeighbourhood(voxels_[i]);
			for (size_t j = 0; j < 27; j++) {
				if (!neighbourhood->voxs[j]->filled || !neighbourhood->voxs[j]->enlarged)
					neighbourhood->voxs[j]->enlarged = true;
			}
			neighbourhood = nullptr;
			delete neighbourhood;
		}
	}
	for (size_t i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->enlarged && voxels_[i]->position(1) > 0)
			voxels_[i]->filled = true;
	}

	std::vector<voxel *> solid_voxels;
	this->getSolidVoxels(solid_voxels);
	std::cout << "solid voxel size " << solid_voxels.size() << std::endl;
}

//void VoxelGrid::generateConvexHull(const Eigen::MatrixXd &vertices, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF) {
//	Eigen::MatrixXd convex_v;
//	Eigen::MatrixXi convex_f;
//	Eigen::MatrixXd convex_n;
//	Eigen::VectorXd convex_o;
//
//	orgQhull::Qhull qhull;
//
//	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_v(vertices);
//
//	orgQhull::PointCoordinates points(vertices.cols(), "");
//	points.append(row_v.size(), row_v.data());
//
//	std::string qh_command = "Qt Qx";
//	qhull.runQhull(points.comment().c_str(), points.dimension(), points.count(), &*points.coordinates(),
//		qh_command.c_str());
//
//	// check for errors
//	if (qhull.hasQhullMessage()) {
//		std::cerr << "\nQhull message:\n" << qhull.qhullMessage();
//		qhull.clearQhullMessage();
//		throw std::runtime_error("Qhull Error (see message)");
//	}
//
//	auto vertices_q = qhull.vertexList().toStdVector();
//	auto facets_q = qhull.facetList().toStdVector();
//
//	convex_v.setZero(vertices_q.size(), 3);
//	convex_f.setZero(facets_q.size(), 3);
//	convex_n.setZero(facets_q.size(), 3);
//	convex_o.setZero(facets_q.size());
//
//	std::unordered_map<int, int> map_q;
//
//	for (unsigned i = 0; i < vertices_q.size(); ++i) {
//		orgQhull::QhullPoint p = vertices_q[i].point();
//		convex_v.row(i) = Eigen::RowVector3d(p[0], p[1], p[2]);
//		map_q.insert(std::make_pair(vertices_q[i].id(), i));
//
//	}
//
//
//	for (unsigned i = 0; i < facets_q.size(); ++i) {
//		auto fv = facets_q[i].vertices().toStdVector();
//		for (int j = 0; j < fv.size(); ++j) {
//			auto search = map_q.find(fv[j].id());
//			if (search != map_q.end()) {
//				convex_f.row(i)[j] = search->second;
//			}
//			else {
//				std::cout << "Something goes wrong in qhull" << std::endl;
//				exit(-1);
//			}
//		}
//
//		Eigen::RowVector3d vec_1 = convex_v.row((convex_f.row(i)(1))) - convex_v.row((convex_f.row(i)(0)));
//		Eigen::RowVector3d vec_2 = convex_v.row((convex_f.row(i)(2))) - convex_v.row((convex_f.row(i)(0)));
//
//		Eigen::RowVector3d normal = vec_1.cross(vec_2);
//
//		Eigen::RowVector3d qhull_normal = Eigen::RowVector3d(facets_q[i].getFacetT()->normal[0],
//			facets_q[i].getFacetT()->normal[1],
//			facets_q[i].getFacetT()->normal[2]);
//
//		convex_n.row(i) = qhull_normal;
//		convex_o(i) = facets_q[i].hyperplane().offset();
//
//
//		if (normal.dot(qhull_normal) < 0) {
//			unsigned temp = convex_f.row(i)[0];
//			convex_f.row(i)[0] = convex_f.row(i)[2];
//			convex_f.row(i)[2] = temp;
//
//		}
//
//	}
//
//	CV = convex_v;
//	CF = convex_f;
//
//}

void VoxelGrid::getVoxelsPos(const std::vector<voxel *> &voxels, Eigen::MatrixXd &pos) {
	pos.setZero();
	pos.resize(voxels.size(), 3);
	for (int i = 0; i < voxels.size(); ++i)
		pos.row(i) = Eigen::RowVector3d(voxels[i]->position);
}

void VoxelGrid::getVoxelsPoswithPlatform(const std::vector<voxel *> &voxels, const Eigen::MatrixXd &platform,
	Eigen::MatrixXd &pos) {
	Eigen::MatrixXd voxel_pos;
	this->getVoxelsPos(voxels, voxel_pos);
	pos = append_vertices(voxel_pos, platform);
}


//bool VoxelGrid::getVoxelsInRadius(voxel *vox, double radius, std::vector<voxel *> &neighbor_voxels, kdtree *tree) {
//	double pos[3] = { vox->position(0), vox->position(1), vox->position(2) };
//	kdres *res = kd_nearest_range(tree, pos, radius);
//	if (kd_res_size(res)) {
//		while (!kd_res_end(res)) {
//			voxel *near_vox = (voxel *)kd_res_item(res, pos);
//			neighbor_voxels.push_back(near_vox);
//			kd_res_next(res);
//		}
//		kd_res_free(res);
//		return true;
//	}
//	else
//		return false;
//}

//bool VoxelGrid::getVoxelsInRadius(Eigen::RowVector3d point, double radius, std::vector<voxel *> &neighbor_voxels,
//	kdtree *tree) {
//	double pos[3] = { point(0), point(1), point(2) };
//	kdres *res = kd_nearest_range(tree, pos, radius);
//	if (kd_res_size(res)) {
//		while (!kd_res_end(res)) {
//			voxel *near_vox = (voxel *)kd_res_item(res, pos);
//			neighbor_voxels.push_back(near_vox);
//			kd_res_next(res);
//		}
//		kd_res_free(res);
//		return true;
//	}
//	else
//		return false;
//
//}


//void VoxelGrid::expandCluster(std::vector<voxel *> &neighbor_voxels, int cluster_index, double radius, kdtree *tree) {
//	std::vector<voxel *> neighbor_voxels_unprocessed;
//	for (size_t i = 0; i < neighbor_voxels.size(); i++) {
//		if (!neighbor_voxels[i]->processed) {
//			neighbor_voxels[i]->cluster = cluster_index;
//			neighbor_voxels[i]->processed = true;
//			neighbor_voxels_unprocessed.push_back(neighbor_voxels[i]);
//		}
//	}
//
//	for (size_t i = 0; i < neighbor_voxels_unprocessed.size(); i++) {
//		voxel *voxel_near = neighbor_voxels_unprocessed[i];
//		std::vector<voxel *> neighbor_neighbor_voxels;
//		if (this->getVoxelsInRadius(voxel_near, radius, neighbor_neighbor_voxels, tree)) {
//			for (size_t j = 0; j < neighbor_neighbor_voxels.size(); j++) {
//				voxel *pVoxel = neighbor_neighbor_voxels[j];
//				if (!pVoxel->processed)
//					pVoxel->processed = true;
//				if (pVoxel->cluster < 0) {
//					pVoxel->cluster = cluster_index;
//					neighbor_voxels_unprocessed.push_back(pVoxel);
//				}
//			}
//		}
//	}
//}

//bool VoxelGrid::checkIsolated(std::vector<voxel *> &voxels, double radius, std::vector<std::vector<voxel *> > &clusters) {
//
//	kdtree *kd_tree;
//
//	kd_tree = kd_create(3);
//
//	int nCluster = 0;
//
//
//	for (size_t i = 0; i < voxels.size(); ++i) {
//		double pos[3];
//		voxels[i]->processed = false;
//		pos[0] = voxels[i]->position(0);
//		pos[1] = voxels[i]->position(1);
//		pos[2] = voxels[i]->position(2);
//		kd_insert(kd_tree, pos, voxels[i]);
//		voxels[i]->cluster = -1;
//	}
//
//	for (size_t i = 0; i < voxels.size(); ++i) {
//		voxel *pVoxel = voxels[i];
//		if (!pVoxel->processed) {
//			pVoxel->processed = true;
//			std::vector<voxel *> neighbor_voxels;
//			if (this->getVoxelsInRadius(pVoxel, radius, neighbor_voxels, kd_tree)) {
//				pVoxel->cluster = nCluster;
//				pVoxel->processed = true;
//				this->expandCluster(neighbor_voxels, nCluster, radius, kd_tree);
//				nCluster++;
//			}
//		}
//	}
//
//	for (size_t i = 0; i < voxels.size(); ++i)
//		voxels[i]->processed = false;
//
//	clusters.clear();
//
//	clusters.resize(nCluster);
//	for (int i = 0; i < voxels.size(); ++i) {
//		for (int j = 0; j < nCluster; ++j) {
//			if (voxels[i]->cluster == j)
//				clusters[j].push_back(voxels[i]);
//		}
//	}
//	if (nCluster > 1) {
//		return true;
//
//	}
//	else
//		return false;
//}


double VoxelGrid::getCuttingPlane(const std::vector<voxel *> &cluster1, const std::vector<voxel *> &cluster2,
	Eigen::Vector4d &plane, Eigen::Vector3d mid_point) {

	Eigen::MatrixXd a1, b1;
	this->getVoxelsPos(cluster1, a1);
	this->getVoxelsPos(cluster2, b1);

	double min = 9999.0;
	std::pair<Eigen::RowVector3d, Eigen::RowVector3d> p;

	for (int i = 0; i < a1.rows(); ++i) {
		for (int j = 0; j < b1.rows(); ++j) {
			Eigen::RowVector3d vec = a1.row(i) - b1.row(j);
			if (vec.norm() <= min) {
				min = vec.norm();
				p.first = a1.row(i);
				p.second = b1.row(j);
			}

		}
	}

	mid_point = (p.first + p.second) / 2;
	Eigen::Vector3d normal = -(p.first - p.second).reverse();
	normal.normalize();

	double a = normal.x();
	double b = normal.y();
	double c = normal.z();
	double d = -a * mid_point.x() - b * mid_point.y() - c * mid_point.z();

	plane = Eigen::Vector4d(a, b, c, d);

	return min;

}

void VoxelGrid::getNearestPair(const std::vector<voxel *> &cluster1, const std::vector<voxel *> &cluster2,
	std::pair<voxel *, voxel *> &p) {

	Eigen::MatrixXd a1, b1;
	this->getVoxelsPos(cluster1, a1);
	this->getVoxelsPos(cluster2, b1);

	double min = 9999.0;

	for (int i = 0; i < a1.rows(); ++i) {
		for (int j = 0; j < b1.rows(); ++j) {
			Eigen::RowVector3d vec = a1.row(i) - b1.row(j);
			if (vec.norm() <= min) {
				min = vec.norm();
				p.first = cluster1[i];
				p.second = cluster2[j];
			}

		}
	}
}


int getNum(std::vector<voxel *> &voxels) {
	int count = 0;
	for (int i = 0; i < voxels.size(); ++i) {
		if (voxels[i]->peeling_order == 0)
			count++;
	}


	return count;

}

//unsigned VoxelGrid::generatePeelingOrder() {
//	Eigen::MatrixXd voxels_with_platform;
//	Eigen::MatrixXd convex_vertices;
//	Eigen::MatrixXi convex_facets;
//
//	std::vector<voxel *> boundary_voxels;
//	std::vector<voxel *> solid_voxels;
//
//	this->getSolidVoxels(solid_voxels);
//	this->findVoxelsOnBoundary(solid_voxels, boundary_voxels);
//	this->getVoxelsPoswithPlatform(boundary_voxels, platform_vertices, voxels_with_platform);
//	this->generateConvexHull(voxels_with_platform, convex_vertices, convex_facets);
//
//	std::vector<voxel *> current_front;
//	std::vector<voxel *> previous_front;
//
//	this->findVoxelsOnConvex(convex_vertices, convex_facets, boundary_voxels, current_front, size * 1.5);
//	unsigned peeling_order = 1;
//	std::vector<voxel *> rest_voxels;
//	int previous_rest_size = 0;
//
//	while (getNum(solid_voxels) > 0) {
//
//
//		rest_voxels.clear();
//		for (size_t i = 0; i < current_front.size(); ++i)
//			current_front[i]->peeling_order = peeling_order;
//		for (size_t i = 0; i < solid_voxels.size(); ++i) {
//			if (!solid_voxels[i]->peeling_order)
//				rest_voxels.push_back(solid_voxels[i]);
//		}
//
//
//		std::cout << "current peeling order is " << peeling_order;
//		std::cout << " rest voxels size is " << rest_voxels.size() << std::endl;
//
//		std::vector<std::vector<voxel *> > clusters;
//
//		if (!rest_voxels.size() || rest_voxels.size() == previous_rest_size)
//			break;
//
//		this->findVoxelsOnBoundary(solid_voxels, boundary_voxels);
//
//		this->getVoxelsPoswithPlatform(boundary_voxels, platform_vertices, voxels_with_platform);
//
//		this->generateConvexHull(voxels_with_platform, convex_vertices, convex_facets);
//
//		this->findVoxelsOnConvex(convex_vertices, convex_facets, boundary_voxels, current_front, size * 1.5);
//
//		peeling_order++;
//
//		previous_rest_size = rest_voxels.size();
//	}
//
//	max_peeling_order = 0;
//	min_peeling_order = 9999;
//
//	for (int i = 0; i < voxels_.size(); ++i) {
//		if (voxels_[i]->peeling_order > max_peeling_order)
//			max_peeling_order = voxels_[i]->peeling_order;
//	}
//
//	for (int i = 0; i < voxels_.size(); ++i) {
//		if (voxels_[i]->peeling_order < min_peeling_order)
//			min_peeling_order = voxels_[i]->peeling_order;
//	}
//
//
//	this->getSolidVoxels(solid_voxels);
//	for (int i = 0; i < solid_voxels.size(); ++i) {
//		if (!solid_voxels[i]->peeling_order) {
//
//			solid_voxels[i]->filled = false;
//		}
//		else
//			solid_voxels[i]->peeling_order = peeling_order + 1 - solid_voxels[i]->peeling_order;
//	}
//
//	return max_peeling_order;
//}


void VoxelGrid::findVoxelsOnBoundary(std::vector<voxel *> &solid_voxels, std::vector<voxel *> &boundary_voxels) {

	const int bndNeighborNum = 6;
	const int bndNeighborDelta[][3] = { { -1, 0,  0 },
	{ 1,  0,  0 },
	{ 0,  1,  0 },
	{ 0,  -1, 0 },
	{ 0,  0,  1 },
	{ 0,  0,  -1 } };
	boundary_voxels.clear();
	for (size_t i = 0; i < solid_voxels.size(); ++i) {
		voxel *current_voxel = solid_voxels[i];
		if (current_voxel->peeling_order != 0) continue;

		bool is_boundary = false;

		if (solid_voxels[i]->preserved) {
			is_boundary = true;
		}

		for (size_t j = 0; j < bndNeighborNum; ++j) {
			int new_x = current_voxel->x + bndNeighborDelta[j][0];
			int new_y = current_voxel->y + bndNeighborDelta[j][1];
			int new_z = current_voxel->z + bndNeighborDelta[j][2];

			voxel *neighbor_voxel = this->GetVoxel(new_x, new_y, new_z);

			if (new_x < 0 || new_x >= res(0) || new_y < 0 || new_y >= res(1) || new_z < 0 || new_z >= res(2) ||
				neighbor_voxel->peeling_order != 0 || !neighbor_voxel->filled) {

				is_boundary = true;
				break;
			}
			neighbor_voxel = nullptr;
			delete neighbor_voxel;
		}

		if (is_boundary)
			boundary_voxels.push_back(current_voxel);

		current_voxel = nullptr;
		delete current_voxel;
	}

}

void
VoxelGrid::findVoxelsOnBoundary(std::vector<voxel *> &solid_voxels, std::vector<voxel *> &boundary_voxels, int layer) {

	const int bndNeighborNum = 26;
	//    const int bndNeighborDelta[][3] = {{-1, 0,  0},
	//                                       {1,  0,  0},
	//                                       {0,  1,  0},
	//                                       {0,  -1, 0},
	//                                       {0,  0,  1},
	//                                       {0,  0,  -1}};
	const int bndNeighborDelta[][3] = { { -1, -1, -1 },
	{ -1, -1, 0 },
	{ -1, -1, 1 },
	{ 0,  -1, -1 },
	{ 0,  -1, 0 },
	{ 0,  -1, 1 },
	{ 1,  -1, -1 },
	{ 1,  -1, 0 },
	{ 1,  -1, 1 },

	{ -1, 0,  -1 },
	{ -1, 0,  0 },
	{ -1, 0,  1 },
	{ 0,  0,  -1 },
		//                                         {0, 0, 0},
	{ 0,  0,  1 },
	{ 1,  0,  -1 },
	{ 1,  0,  0 },
	{ 1,  0,  1 },

	{ -1, 1,  -1 },
	{ -1, 1,  0 },
	{ -1, 1,  1 },
	{ 0,  1,  -1 },
	{ 0,  1,  0 },
	{ 0,  1,  1 },
	{ 1,  1,  -1 },
	{ 1,  1,  0 },
	{ 1,  1,  1 }
	};
	boundary_voxels.clear();
	for (size_t i = 0; i < solid_voxels.size(); ++i) {
		voxel *current_voxel = solid_voxels[i];
		if (current_voxel->layer == 0) continue;

		bool is_boundary = false;

		if (solid_voxels[i]->preserved) {
			is_boundary = true;
		}

		for (size_t j = 0; j < bndNeighborNum; ++j) {
			int new_x = current_voxel->x + bndNeighborDelta[j][0];
			int new_y = current_voxel->y + bndNeighborDelta[j][1];
			int new_z = current_voxel->z + bndNeighborDelta[j][2];

			voxel *neighbor_voxel = this->GetVoxel(new_x, new_y, new_z);

			if (new_x < 0 || new_x >= res(0) || new_y < 0 || new_y >= res(1) || new_z < 0 || new_z >= res(2) ||
				neighbor_voxel->layer == 0 || !neighbor_voxel->filled || neighbor_voxel->layer > layer) {

				is_boundary = true;
				break;
			}
			neighbor_voxel = nullptr;
			delete neighbor_voxel;
		}

		if (is_boundary)
			boundary_voxels.push_back(current_voxel);

		current_voxel = nullptr;
		delete current_voxel;
	}

}

void VoxelGrid::getSolidVoxels(std::vector<voxel *> &solid_voxels) {
	solid_voxels.clear();
	for (size_t i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->filled == true)
			solid_voxels.push_back(voxels_[i]);
	}
}

//void VoxelGrid::findVoxelsOnConvex(const Eigen::MatrixXd &CV, const Eigen::MatrixXi &CF, std::vector<voxel *> &all_voxels,
//	std::vector<voxel *> &above_voxels, double eps) {
//	Eigen::VectorXi I;
//	Eigen::VectorXd S;
//	Eigen::MatrixXd C, N, GV;
//	above_voxels.clear();
//	this->getVoxelsPos(all_voxels, GV);
//	igl::signed_distance(GV, CV, CF, igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL, S, I, C, N);
//	for (size_t i = 0; i < S.rows(); ++i) {
//
//		//        if (S(i) >= -eps && S(i) <= eps)
//		if (S(i) >= -eps || (S(i) != S(i)))
//			//        if (S(i) >= 0 && S(i) <= eps)
//
//			above_voxels.push_back(all_voxels[i]);
//	}
//}

void VoxelGrid::removeCritialPoints(std::vector<voxel *> &voxels, std::vector<voxel *> &critical_voxels) {
	std::vector<voxel *> new_front;
	std::vector<bool> bFlag;
	bFlag.resize(voxels_.size());
	for (int i = 0; i < voxels_.size(); ++i)
		bFlag[i] = false;
	for (int i = 0; i < critical_voxels.size(); ++i) {
		int index = this->getIndex(critical_voxels[i]->x, critical_voxels[i]->y, critical_voxels[i]->z);
		bFlag[index] = true;
	}
	for (int i = 0; i < voxels.size(); ++i) {
		int index = this->getIndex(critical_voxels[i]->x, critical_voxels[i]->y, critical_voxels[i]->z);
		if (bFlag[index] == false)
			new_front.push_back(voxels[i]);
	}
	voxels = new_front;
	new_front.clear();
}

void VoxelGrid::test(std::vector<voxel *> &voxels, Eigen::MatrixXd &points, int order) {
	std::vector<voxel *> ttv;
	for (int i = 0; i < voxels.size(); ++i) {
		if (voxels[i]->peeling_order == order)
			ttv.push_back(voxels[i]);
	}
	this->getVoxelsPos(ttv, points);

}


void VoxelGrid::getNeighborVoxels(voxel *current_voxel, std::vector<voxel *> &neighbor_voxels) {
	neighbor_voxels.clear();
	for (size_t i = 0; i < 27; i++) {
		int x, y, z;
		x = current_voxel->x;
		y = current_voxel->y;
		z = current_voxel->z;
		int x_neighbor, y_neighbor, z_neighbor;
		x_neighbor = x + xoff[i];
		y_neighbor = y + yoff[i];
		z_neighbor = z + zoff[i];

		if (x_neighbor < 0 || x_neighbor >= res(0) || y_neighbor < 0 || y_neighbor >= res(1) || z_neighbor < 0 ||
			z_neighbor >= res(2))
			continue;
		else if (x_neighbor == x && y_neighbor == y && z_neighbor == z)
			continue;
		else
			neighbor_voxels.push_back(this->GetVoxel(x_neighbor, y_neighbor, z_neighbor));
	}
}


//void VoxelGrid::buildGraph(std::vector<voxel *> &voxels, UndirectedGraph &graph) {
//	for (size_t i = 0; i < voxels.size(); ++i)
//		voxels[i]->processed = true;
//
//	//    std::cout << "Building graph from Voxels Grid..." << std::endl;
//
//	Eigen::MatrixXi index_mat;
//
//	index_mat.resize(voxels.size(), 3);
//
//	for (size_t i = 0; i < voxels.size(); ++i)
//		index_mat.row(i) = Eigen::RowVector3i(voxels[i]->x, voxels[i]->y, voxels[i]->z);
//
//	Eigen::RowVector3i min_index = index_mat.colwise().minCoeff();
//	Eigen::RowVector3i max_index = index_mat.colwise().maxCoeff();
//
//	int x_min = min_index(0);
//	int y_min = min_index(1);
//	int z_min = min_index(2);
//	int x_max = max_index(0);
//	int y_max = max_index(1);
//	int z_max = max_index(2);
//
//
//	std::vector<voxel *> test_voxels;
//	for (size_t z = z_min; z <= z_max; ++z) {
//		for (size_t y = y_min; y <= y_max; ++y) {
//			for (size_t x = x_min; x <= x_max; ++x) {
//				voxel *current_voxel;
//				current_voxel = this->GetVoxel(x, y, z);
//				if (current_voxel->processed) {
//
//					test_voxels.push_back(current_voxel);
//
//					std::vector<voxel *> neighbour_voxels;
//					this->getNeighborVoxels(current_voxel, neighbour_voxels);
//
//					const uint vertex_ID = (uint)(current_voxel->x +
//						res(0) * (current_voxel->y + res(1) * current_voxel->z));
//
//
//					for (size_t i = 0; i < neighbour_voxels.size(); ++i) {
//						const int neighbour_x = neighbour_voxels[i]->x;
//						const int neighbour_y = neighbour_voxels[i]->y;
//						const int neighbour_z = neighbour_voxels[i]->z;
//
//						Eigen::Vector3i diff = Eigen::Vector3i(neighbour_x - current_voxel->x,
//							neighbour_y - current_voxel->y,
//							neighbour_z - current_voxel->z);
//						const Weight weight = sqrt(
//							(double)(diff(0) * diff(0) + diff(1) * diff(1) + diff(2) * diff(2)));
//
//						if ((neighbour_x >= x_min) && (neighbour_x <= x_max)
//							&& (neighbour_y >= y_min) && (neighbour_y <= y_max)
//							&& (neighbour_z >= z_min) && (neighbour_z <= z_max)) {
//
//							if (this->GetVoxel(neighbour_x, neighbour_y, neighbour_z)->processed) {
//								test_voxels.push_back(neighbour_voxels[i]);
//
//								const uint neighbour_vertex_ID = (uint)(neighbour_x +
//									res(0) * (neighbour_y + res(1) * neighbour_z));
//								boost::add_edge(boost::vertex(static_cast<int>(vertex_ID), graph),
//									boost::vertex(static_cast<int>(neighbour_vertex_ID), graph),
//									weight, graph);
//							}
//						}
//					}
//					current_voxel = nullptr;
//					delete current_voxel;
//					neighbour_voxels.clear();
//
//				}
//
//			}
//		}
//	}
//	test_voxels.clear();
//
//
//}

//double VoxelGrid::getShortestPathDijkstra(voxel *start_voxel, voxel *end_voxel, std::vector<voxel *> &path_voxels,
//	UndirectedGraph &graph) {
//	path_voxels.clear();
//
//	const uint start_vertex_ID = (uint) this->getIndex(start_voxel->x, start_voxel->y, start_voxel->z);
//	const uint goal_vertex_ID = (uint) this->getIndex(end_voxel->x, end_voxel->y, end_voxel->z);
//
//	std::vector<Vertex> predecessors(boost::num_vertices(graph));
//	std::vector<Weight> distances(boost::num_vertices(graph));
//	IndexMap index_map = boost::get(boost::vertex_index, graph);
//	PredecessorMap predecessor_map(&predecessors[0], index_map);
//	DistanceMap distance_map(&distances[0], index_map);
//	NameMap name_map = boost::get(boost::vertex_name, graph);
//	Vertex start_vertex = boost::vertex(static_cast<int>(start_vertex_ID), graph);
//	Vertex goal_vertex = boost::vertex(static_cast<int>(goal_vertex_ID), graph);
//
//	boost::dijkstra_shortest_paths(graph,
//		start_vertex,
//		boost::distance_map(distance_map).predecessor_map(predecessor_map));
//
//	typedef std::vector<UndirectedGraph::edge_descriptor> PathType;
//	PathType path;
//	Vertex v = goal_vertex;
//	for (Vertex u = predecessor_map[v]; u != v; v = u, u = predecessor_map[v]) {
//		std::pair<UndirectedGraph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph);
//		UndirectedGraph::edge_descriptor edge = edgePair.first;
//		path.push_back(edge);
//	}
//
//	for (PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator) {
//		const uint vertex_ID = static_cast<uint>(boost::source(*pathIterator, graph));
//		voxel *path_voxel = voxels_[vertex_ID];
//		path_voxels.push_back(path_voxel);
//		path_voxel = nullptr;
//		delete path_voxel;
//	}
//
//
//	return distance_map[goal_vertex];
//
//
//}

void VoxelGrid::exportPeelingOrder(const std::string &path) {
	std::ofstream file = std::ofstream(path);
	for (size_t i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->peeling_order) {
			file << voxels_[i]->x << " ";
			file << voxels_[i]->y << " ";
			file << voxels_[i]->z << " ";
			file << voxels_[i]->peeling_order << std::endl;
		}
	}
	file.close();
	std::cout << "export peeling order finished" << std::endl;
}

void VoxelGrid::exportGrowingOrder(const std::string &path) {
	std::ofstream file = std::ofstream(path);
	file << res(0) << " " << res(1) << " " << res(2) << std::endl;
	for (size_t i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->layer) {
			file << voxels_[i]->x << " ";
			file << voxels_[i]->y << " ";
			file << voxels_[i]->z << " ";
			file << voxels_[i]->position(0) << " ";
			file << voxels_[i]->position(1) << " ";
			file << voxels_[i]->position(2) << " ";
			file << voxels_[i]->layer << " ";
			file << voxels_[i]->peeling_order << std::endl;

		}
	}
	file.close();
	std::cout << "export growing order finished" << std::endl;

}

double VoxelGrid::getAverageHeight(std::vector<voxel *> &cluster) {
	double total_height = 0.0;
	for (size_t i = 0; i < cluster.size(); ++i) {
		double height = cluster[i]->position.y();
		total_height += height;
	}
	return total_height / double(cluster.size());
}

//unsigned VoxelGrid::generateGrowingOrder(std::vector<voxel *> &missing_voxels) {
//	for (int i = 0; i < voxels_.size(); ++i) {
//		voxels_[i]->processed = false;
//	}
//
//
//	std::vector<voxel *> solid_voxels;
//	std::vector<voxel *> current_front;
//	this->getSolidVoxels(solid_voxels);
//	int lowest = +9999;
//	for (int i = 0; i < solid_voxels.size(); ++i) {
//		if (solid_voxels[i]->y < lowest)
//			lowest = solid_voxels[i]->y;
//	}
//	for (int i = 0; i < solid_voxels.size(); ++i) {
//		if (solid_voxels[i]->y <= lowest) {
//			solid_voxels[i]->layer = 1;
//			solid_voxels[i]->normal = Eigen::Vector3d(0., 0., 1.0);
//
//			current_front.push_back(solid_voxels[i]);
//		}
//	}
//
//	Eigen::MatrixXd first_front_with_platform;
//	Eigen::MatrixXd current_convex_front_vertices;
//	Eigen::MatrixXi current_convex_front_faces;
//
//	this->getVoxelsPoswithPlatform(current_front, platform_vertices, first_front_with_platform);
//	this->generateConvexHull(first_front_with_platform, current_convex_front_vertices, current_convex_front_faces);
//
//	unsigned current_isovalue = 3;
//	unsigned layer_count = 1;
//	unsigned max_isovalue = this->max_peeling_order;
//	unsigned last_shadow_count = 0;
//	std::vector<voxel *> last_shadow_points;
//
//	while (current_isovalue <= max_isovalue) {
//		std::cout << "current_isovalue " << current_isovalue << std::endl;
//		while (current_front.size() > 0) {
//			std::vector<voxel *> next_front;
//			std::vector<voxel *> neighbor_voxels;
//			for (int i = 0; i < current_front.size(); ++i) {
//				for (int j = 0; j < neighbor_num; ++j) {
//					int ii = current_front[i]->x + neighbor_delta[j][0];
//					int jj = current_front[i]->y + neighbor_delta[j][1];
//					int kk = current_front[i]->z + neighbor_delta[j][2];
//					voxel *neighbor_voxel = this->GetVoxel(ii, jj, kk);
//					if (neighbor_voxel == nullptr || neighbor_voxel->layer != 0 || neighbor_voxel->processed) continue;
//					if (neighbor_voxel->peeling_order > current_isovalue) continue;
//					if (!neighbor_voxel->filled) continue;
//					neighbor_voxel->processed = true;
//					neighbor_voxels.push_back(neighbor_voxel);
//				}
//			}
//
//			Eigen::MatrixXd points;
//			points.resize(neighbor_voxels.size(), 3);
//			for (unsigned k = 0; k < points.rows(); ++k)
//				points.row(k) = Eigen::RowVector3d(neighbor_voxels[k]->position);
//			Eigen::VectorXd signed_distances;
//			Eigen::MatrixXd normals;
//			std::vector<voxel *> next_front_candidates;
//			this->signedDistanceToConvexHullwithNormal(points, current_convex_front_vertices,
//				current_convex_front_faces,
//				signed_distances, normals);
//			for (unsigned k = 0; k < signed_distances.rows(); ++k) {
//				double distance_tolerance;
//				distance_tolerance = size;
//				if (signed_distances(k) >= -2.0 * distance_tolerance &&
//					signed_distances(k) <= 2.0 * distance_tolerance) {
//					next_front_candidates.push_back(neighbor_voxels[k]);
//					neighbor_voxels[k]->normal = Eigen::Vector3d(normals.row(k));
//				}
//			}
//			for (int j = 0; j < neighbor_voxels.size(); ++j)
//				neighbor_voxels[j]->processed = false;
//			neighbor_voxels.clear();
//
//			if (!next_front_candidates.size())
//				break;
//
//			unsigned shadow_count;
//
//			std::vector<voxel *> shadow_points;
//
//			shadow_count = this->checkSetShadowed(current_front, next_front_candidates, solid_voxels, shadow_points);
//
//			if (shadow_count <= last_shadow_count) {
//				for (int j = 0; j < next_front_candidates.size(); ++j)
//					next_front.push_back(next_front_candidates[j]);
//
//			}
//			else {
//
//				std::cout << "shadowed! " << shadow_count << " last shadowed " << last_shadow_count << std::endl;
//
//				std::vector<voxel *> safe_candidates;
//
//				std::vector<voxel *> unprocessed;
//				for (int i = 0; i < next_front_candidates.size(); ++i)
//					unprocessed.push_back(next_front_candidates[i]);
//				for (int i = 0; i < shadow_points.size(); ++i)
//					unprocessed.push_back(shadow_points[i]);
//				this->getNonShadowedVoxels(current_front, next_front_candidates, unprocessed, safe_candidates);
//				//            this->getNonShadowedVoxelsIncremental(current_front, candidate_voxels, solid_voxels, safe_candidates);
//
//				next_front = safe_candidates;
//
//				last_shadow_count = this->checkSetShadowed(current_front, next_front, solid_voxels, last_shadow_points);
//
//			}
//			std::cout << "shadowed! " << shadow_count << " next_front_candidates size " << next_front_candidates.size()
//				<< " safe size " << next_front.size() << std::endl;
//
//			////////////////////////////////////////////////////////////////
//			//            next_front=next_front_candidates;
//			////////////////////////////////////////////////////////////////
//
//			if (!next_front.size()) {
//				next_front = next_front_candidates;
//			}
//
//			if (!next_front.size())
//				break;
//
//
//			for (int j = 0; j < next_front.size(); ++j)
//				next_front[j]->layer = layer_count + 1;
//
//			for (int j = 0; j < current_front.size(); ++j)
//
//				next_front.push_back(current_front[j]);
//
//
//			Eigen::MatrixXd next_front_vertices;
//
//			this->getVoxelsPoswithPlatform(next_front, platform_vertices, next_front_vertices);
//			this->generateConvexHull(next_front_vertices, current_convex_front_vertices, current_convex_front_faces);
//			current_front.clear();
//			current_front = next_front;
//
//			next_front.clear();
//			std::cout << "current layer " << layer_count << std::endl;
//
//			layer_count++;
//
//		}
//		current_isovalue += 3;
//		if (current_isovalue < max_isovalue + 3 && current_isovalue > max_isovalue)
//			current_isovalue = max_isovalue;
//
//	}
//
//	max_growing_order = 0;
//	min_growing_order = 9999;
//
//	missing_voxels.clear();
//	for (size_t i = 0; i < solid_voxels.size(); ++i) {
//		if (!solid_voxels[i]->layer)
//			missing_voxels.push_back(solid_voxels[i]);
//	}
//
//	for (int i = 0; i < voxels_.size(); ++i) {
//		if (voxels_[i]->layer > max_growing_order)
//			max_growing_order = voxels_[i]->layer;
//	}
//
//	for (int i = 0; i < voxels_.size(); ++i) {
//		if (voxels_[i]->layer < min_growing_order)
//			min_growing_order = voxels_[i]->layer;
//	}
//
//	solid_voxels.clear();
//	current_front.clear();
//	return max_growing_order;
//}

//void VoxelGrid::signedDistanceToConvexHull(Eigen::MatrixXd &points, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF,
//	Eigen::VectorXd &distances) {
//	Eigen::VectorXi I;
//	Eigen::MatrixXd C, N;
//	igl::signed_distance(points, CV, CF, igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL, distances, I, C, N);
//	for (int i = 0; i < distances.size(); ++i) {
//		if (distances(i) != distances(i))
//			distances(i) = 0;
//	}
//}
//
//void VoxelGrid::signedDistanceToConvexHullwithNormal(Eigen::MatrixXd &points, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF,
//	Eigen::VectorXd &distances, Eigen::MatrixXd &normals) {
//	Eigen::VectorXi I;
//	Eigen::MatrixXd C, N;
//	igl::signed_distance(points, CV, CF, igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL, distances, I, C, N);
//	for (int i = 0; i < distances.size(); ++i) {
//		if (distances(i) != distances(i))
//			distances(i) = 0;
//	}
//	normals = N;
//}

void VoxelGrid::voxelMesh(std::vector<voxel *> &voxels, Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces,
	Eigen::MatrixXd &colors, bool is_growed, bool is_heatmap) {
	int voffset = 0;
	int foffset = 0;
	float color_table[][3] = {
		{ 255, 255, 255 },
		{ 255, 0,   128 },
		{ 0,   255, 255 },
		{ 128, 255, 0 },
		{ 128, 128, 64 },
		{ 255, 0,   0 },
		{ 0,   255, 0 },
		{ 0,   0,   255 },
		{ 128, 128, 192 },
		{ 255, 255, 128 },
		{ 255, 128, 0 },
		{ 255, 128, 255 },
		{ 255, 214, 202 },
		{ 128, 128, 192 },
		{ 255, 165, 0 }, //orange
		{ 255, 128, 192 },
		{ 128, 128, 64 },
		{ 0,   255, 255 },
		{ 238, 130, 238 },//violet
		{ 220, 220, 220 },//gainsboro
		{ 188, 143, 143 }, // rosy brown
		{ 46,  139, 87 },//sea green
		{ 210, 105, 30 },//chocolate
		{ 100, 149, 237 }//cornflower blue
	};


	std::vector<Eigen::RowVector3d> pos_vec;

	std::vector<Eigen::RowVector3d> boundary_voxels_pos;
	std::vector<voxel *> boundary_voxels;
	unsigned min, max;
	this->getMinMaxLayer(voxels, max, min);
	this->findVoxelsOnBoundary(voxels, boundary_voxels, max);

	for (int i = 0; i < boundary_voxels.size(); ++i)
		boundary_voxels_pos.push_back(Eigen::RowVector3d(boundary_voxels[i]->position));

	double width = size;
	vertices.setZero(boundary_voxels_pos.size() * 8, 3);
	faces.setZero(boundary_voxels_pos.size() * 12, 3);
	colors.setZero(boundary_voxels_pos.size() * 12, 3);

	for (unsigned i = 0; i < boundary_voxels_pos.size(); ++i) {

		double nRed, nGreen, nBlue;

		if (!is_growed) {
			if (!is_heatmap) {
				nRed = color_table[(boundary_voxels[i]->peeling_order) % 22][0] / 255.0f;
				nGreen = color_table[(boundary_voxels[i]->peeling_order) % 22][1] / 255.0f;
				nBlue = color_table[(boundary_voxels[i]->peeling_order) % 22][2] / 255.0f;
			}
			else
				this->doubleRGB(boundary_voxels[i]->peeling_order, max_peeling_order, min_peeling_order, nRed, nGreen,
					nBlue);

		}
		else {
			if (!is_heatmap) {
				nRed = color_table[(boundary_voxels[i]->layer) % 22][0] / 255.0f;
				nGreen = color_table[(boundary_voxels[i]->layer) % 22][1] / 255.0f;
				nBlue = color_table[(boundary_voxels[i]->layer) % 22][2] / 255.0f;
			}
			else

				this->doubleRGB(boundary_voxels[i]->layer, max_growing_order, min_growing_order, nRed, nGreen, nBlue);

		}


		Eigen::RowVector3d color_vec(nRed, nGreen, nBlue);

		Eigen::RowVector3d corner_1 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(-1, -1, -1);
		Eigen::RowVector3d corner_2 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(-1, -1, 1);
		Eigen::RowVector3d corner_3 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(1, -1, -1);
		Eigen::RowVector3d corner_4 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(1, -1, 1);
		Eigen::RowVector3d corner_5 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(-1, 1, -1);
		Eigen::RowVector3d corner_6 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(-1, 1, 1);
		Eigen::RowVector3d corner_7 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(1, 1, -1);
		Eigen::RowVector3d corner_8 = boundary_voxels_pos[i] + width / 2 * Eigen::RowVector3d(1, 1, 1);


		vertices.row(voffset + 0) = corner_1;
		vertices.row(voffset + 1) = corner_2;
		vertices.row(voffset + 2) = corner_3;
		vertices.row(voffset + 3) = corner_4;
		vertices.row(voffset + 4) = corner_5;
		vertices.row(voffset + 5) = corner_6;
		vertices.row(voffset + 6) = corner_7;
		vertices.row(voffset + 7) = corner_8;

		colors.row(foffset + 0) = color_vec;
		colors.row(foffset + 1) = color_vec;
		colors.row(foffset + 2) = color_vec;
		colors.row(foffset + 3) = color_vec;
		colors.row(foffset + 4) = color_vec;
		colors.row(foffset + 5) = color_vec;
		colors.row(foffset + 6) = color_vec;
		colors.row(foffset + 7) = color_vec;
		colors.row(foffset + 7) = color_vec;
		colors.row(foffset + 8) = color_vec;
		colors.row(foffset + 9) = color_vec;
		colors.row(foffset + 10) = color_vec;
		colors.row(foffset + 11) = color_vec;


		faces.row(foffset + 0) = Eigen::RowVector3i(voffset + 1, voffset + 0, voffset + 2);
		faces.row(foffset + 1) = Eigen::RowVector3i(voffset + 3, voffset + 1, voffset + 2);
		faces.row(foffset + 2) = Eigen::RowVector3i(voffset + 1, voffset + 4, voffset + 0);
		faces.row(foffset + 3) = Eigen::RowVector3i(voffset + 5, voffset + 4, voffset + 1);
		faces.row(foffset + 4) = Eigen::RowVector3i(voffset + 4, voffset + 5, voffset + 7);
		faces.row(foffset + 5) = Eigen::RowVector3i(voffset + 4, voffset + 7, voffset + 6);
		faces.row(foffset + 6) = Eigen::RowVector3i(voffset + 7, voffset + 3, voffset + 2);
		faces.row(foffset + 7) = Eigen::RowVector3i(voffset + 7, voffset + 2, voffset + 6);
		faces.row(foffset + 8) = Eigen::RowVector3i(voffset + 5, voffset + 1, voffset + 3);
		faces.row(foffset + 9) = Eigen::RowVector3i(voffset + 5, voffset + 3, voffset + 7);
		faces.row(foffset + 10) = Eigen::RowVector3i(voffset + 4, voffset + 6, voffset + 0);
		faces.row(foffset + 11) = Eigen::RowVector3i(voffset + 0, voffset + 6, voffset + 2);

		voffset += 8;
		foffset += 12;
	}
}

void VoxelGrid::getPeeling(unsigned layer, std::vector<voxel *> &peeling_voxels) {
	peeling_voxels.clear();
	if (layer < min_peeling_order)
		return;
	for (int i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->peeling_order <= layer && voxels_[i]->peeling_order)
			peeling_voxels.push_back(voxels_[i]);
	}

}

void VoxelGrid::getGrowing(unsigned layer, std::vector<voxel *> &growing_voxels) {
	growing_voxels.clear();
	if (layer < min_growing_order)
		return;
	for (int i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->layer <= layer && voxels_[i]->layer)
			growing_voxels.push_back(voxels_[i]);
	}
}

void VoxelGrid::getGrowingB(unsigned layer, std::vector<voxel *> &growing_voxels) {
	growing_voxels.clear();
	for (int i = 0; i < voxels_.size(); ++i) {
		if (voxels_[i]->layer == layer)
			growing_voxels.push_back(voxels_[i]);
	}
}

void VoxelGrid::doubleRGB(int num, int max, int min, double &r, double &g, double &b) {
	double x;
	if (min == max)
		x = 0.5;
	else
		x = (double)(num - min) / (double)(max - min);
	b = std::min(std::max(4 * (0.75 - x), 0.), 1.0);
	r = std::min(std::max(4 * (x - 0.25), 0.), 1.0);
	g = std::min(std::max(4 * fabs(x - 0.5) - 1, 0.), 1.0);
}

//void VoxelGrid::searchNextwithCluster(std::vector<voxel *> &cluster, Eigen::MatrixXd &current_convex_front_vertices,
//	Eigen::MatrixXi &current_convex_front_faces, std::vector<voxel *> &growing_next,
//	int current_isovalue, bool is_greedy) {
//	std::vector<voxel *> next_front;
//	std::vector<voxel *> neighbor_voxels;
//
//
//	int current_layer = cluster[0]->layer;
//	for (int i = 0; i < cluster.size(); ++i) {
//		if (cluster[i]->layer != current_layer) {
//			std::cout << "clusters layer is not the same" << std::endl;
//			exit(-1);
//		}
//	}
//
//	for (int i = 0; i < cluster.size(); ++i) {
//		for (int j = 0; j < neighbor_num; ++j) {
//			int ii = cluster[i]->x + neighbor_delta[j][0];
//			int jj = cluster[i]->y + neighbor_delta[j][1];
//			int kk = cluster[i]->z + neighbor_delta[j][2];
//
//			voxel *neighbor_voxel = this->GetVoxel(ii, jj, kk);
//			if (neighbor_voxel == nullptr || neighbor_voxel->layer != 0 || neighbor_voxel->processed) continue;
//			if (is_greedy) {
//				if (neighbor_voxel->layer >= current_layer) continue;
//			}
//			else if (neighbor_voxel->peeling_order > current_isovalue) continue;
//			if (neighbor_voxel->layer >= current_layer) continue;
//			if (!neighbor_voxel->filled) continue;
//			neighbor_voxel->processed = true;
//			neighbor_voxels.push_back(neighbor_voxel);
//		}
//	}
//
//	Eigen::MatrixXd points;
//	points.resize(neighbor_voxels.size(), 3);
//	for (unsigned k = 0; k < points.rows(); ++k)
//		points.row(k) = Eigen::RowVector3d(neighbor_voxels[k]->position);
//	Eigen::VectorXd signed_distances;
//	this->signedDistanceToConvexHull(points, current_convex_front_vertices, current_convex_front_faces,
//		signed_distances);
//	for (unsigned k = 0; k < signed_distances.rows(); ++k) {
//		double distance_tolerance;
//		distance_tolerance = size;
//		if (signed_distances(k) >= -2.0 * distance_tolerance && signed_distances(k) <= 2.0 * distance_tolerance) {
//			next_front.push_back(neighbor_voxels[k]);
//			neighbor_voxels[k]->layer = current_layer + 1;
//		}
//	}
//	for (int j = 0; j < neighbor_voxels.size(); ++j)
//		neighbor_voxels[j]->processed = false;
//
//
//}


//unsigned
//VoxelGrid::generateGreedyGrowingOrder(std::vector<voxel *> &missing_voxels) {
//
//	for (int i = 0; i < voxels_.size(); ++i) {
//		voxels_[i]->processed = false;
//	}
//
//	std::vector<voxel *> solid_voxels;
//	std::vector<voxel *> current_front;
//	this->getSolidVoxels(solid_voxels);
//	int lowest = +9999;
//	for (int i = 0; i < solid_voxels.size(); ++i) {
//		if (solid_voxels[i]->y < lowest)
//			lowest = solid_voxels[i]->y;
//	}
//	std::cout << "lowest " << lowest << std::endl;
//	for (int i = 0; i < solid_voxels.size(); ++i) {
//		if (solid_voxels[i]->y <= lowest) {
//			solid_voxels[i]->layer = 1;
//			current_front.push_back(solid_voxels[i]);
//		}
//	}
//
//	Eigen::MatrixXd current_convex_front_vertices;
//	Eigen::MatrixXi current_convex_front_faces;
//
//	Eigen::MatrixXi convex_edge;
//	std::vector<std::pair<Eigen::RowVector3d, Eigen::RowVector3d>> convex_edges;
//	this->generateConvexHull(platform_vertices, current_convex_front_vertices, current_convex_front_faces);
//	unsigned current_isovalue = 3;
//	unsigned layer_count = 1;
//	unsigned max_isovalue = this->max_peeling_order;
//	unsigned last_shadow_count = 0;
//	std::vector<voxel *> last_shadow_points;
//
//
//	std::vector<voxel *> rest_voxels;
//	std::vector<voxel *> critical_voxels;
//
//
//	clock_t t;
//	t = clock();
//
//	while (current_front.size() > 0) {
//
//		rest_voxels.clear();
//
//		std::vector<voxel *> next_front;
//		std::vector<voxel *> candidate_voxels;
//		std::vector<voxel *> neighbor_voxels;
//		for (int i = 0; i < current_front.size(); ++i) {
//			for (int j = 0; j < neighbor_num; ++j) {
//				int ii = current_front[i]->x + neighbor_delta[j][0];
//				int jj = current_front[i]->y + neighbor_delta[j][1];
//				int kk = current_front[i]->z + neighbor_delta[j][2];
//
//				voxel *neighbor_voxel = this->GetVoxel(ii, jj, kk);
//				if (neighbor_voxel == nullptr || neighbor_voxel->layer != 0 || neighbor_voxel->processed) continue;
//				if (!neighbor_voxel->filled) continue;
//				neighbor_voxel->processed = true;
//				neighbor_voxels.push_back(neighbor_voxel);
//			}
//		}
//
//		Eigen::MatrixXd points;
//		points.resize(neighbor_voxels.size(), 3);
//		for (unsigned k = 0; k < points.rows(); ++k)
//			points.row(k) = Eigen::RowVector3d(neighbor_voxels[k]->position);
//		Eigen::VectorXd signed_distances;
//		this->signedDistanceToConvexHull(points, current_convex_front_vertices, current_convex_front_faces,
//			signed_distances);
//
//		for (unsigned k = 0; k < signed_distances.rows(); ++k) {
//			double distance_tolerance;
//			distance_tolerance = size;
//
//			if (signed_distances(k) >= -2.0 * distance_tolerance && signed_distances(k) <= 2.0 * distance_tolerance) {
//
//				candidate_voxels.push_back(neighbor_voxels[k]);
//			}
//		}
//		for (int j = 0; j < neighbor_voxels.size(); ++j)
//			neighbor_voxels[j]->processed = false;
//		neighbor_voxels.clear();
//		/////////////////////////first check if shadow points will appear///////////////////////
//		unsigned shadow_count;
//
//		std::vector<voxel *> shadow_points;
//
//		shadow_count = this->checkSetShadowed(current_front, candidate_voxels, solid_voxels, shadow_points);
//
//		if (shadow_count <= last_shadow_count) {
//			for (int j = 0; j < candidate_voxels.size(); ++j)
//				next_front.push_back(candidate_voxels[j]);
//
//		}
//		else {
//
//			std::cout << "shadowed! " << shadow_count << "last shadowed" << last_shadow_count << std::endl;
//
//			std::vector<voxel *> safe_candidates;
//
//			std::vector<voxel *> unprocessed;
//			for (int i = 0; i < candidate_voxels.size(); ++i)
//				unprocessed.push_back(candidate_voxels[i]);
//			for (int i = 0; i < shadow_points.size(); ++i)
//				unprocessed.push_back(shadow_points[i]);
//			this->getNonShadowedVoxels(current_front, candidate_voxels, unprocessed, safe_candidates);
//			//            this->getNonShadowedVoxelsIncremental(current_front, candidate_voxels, solid_voxels, safe_candidates);
//
//			next_front = safe_candidates;
//
//			last_shadow_count = this->checkSetShadowed(current_front, next_front, solid_voxels, last_shadow_points);
//
//		}
//		//////////////////////////////////////////////////////////////////////////////////////
//		if (!next_front.size()) {
//			next_front = candidate_voxels;
//		};
//		////////////////////////////////////////////////////////////////////////////////////
//		if (!next_front.size()) {
//			break;
//		};
//
//		last_shadow_count = this->checkSetShadowed(current_front, next_front, solid_voxels, last_shadow_points);
//
//		std::cout << "last shadow count " << last_shadow_count << std::endl;
//
//
//		for (int j = 0; j < next_front.size(); ++j) {
//
//			next_front[j]->layer = layer_count + 1;
//
//		}
//
//		for (int j = 0; j < current_front.size(); ++j) {
//
//			next_front.push_back(current_front[j]);
//
//		}
//
//
//		Eigen::MatrixXd next_front_vertices;
//
//		for (int j = 0; j < next_front.size(); ++j)
//			next_front[j]->convex_vertices = false;
//
//		this->generateConvexHull(next_front, current_convex_front_vertices, current_convex_front_faces, convex_edge,
//			convex_edges, true);
//
//
//		current_front.clear();
//		current_front = next_front;
//
//		next_front.clear();
//		std::cout << "current layer " << layer_count << std::endl;
//
//		layer_count++;
//	}
//
//	missing_voxels.clear();
//	for (size_t i = 0; i < solid_voxels.size(); ++i) {
//		if (!solid_voxels[i]->layer)
//			missing_voxels.push_back(solid_voxels[i]);
//	}
//
//	t = clock() - t;
//
//	printf("It took me  (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);
//
//	max_growing_order = 0;
//	min_growing_order = 9999;
//
//
//	for (int i = 0; i < voxels_.size(); ++i) {
//		if (voxels_[i]->layer > max_growing_order)
//			max_growing_order = voxels_[i]->layer;
//	}
//
//	for (int i = 0; i < voxels_.size(); ++i) {
//		if (voxels_[i]->layer < min_growing_order)
//			min_growing_order = voxels_[i]->layer;
//	}
//
//	solid_voxels.clear();
//	current_front.clear();
//	time_file.close();
//	return max_growing_order;
//
//}

//void VoxelGrid::generateConvexHull(std::vector<voxel *> &voxels, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF,
//	bool with_platform) {
//
//
//	Eigen::MatrixXd points_all;
//	if (with_platform)
//		this->getVoxelsPoswithPlatform(voxels, platform_vertices, points_all);
//	else {
//		points_all.resize(voxels.size(), 3);
//		for (unsigned k = 0; k < points_all.rows(); ++k)
//			points_all.row(k) = Eigen::RowVector3d(voxels[k]->position);
//	}
//
//	Eigen::MatrixXd convex_v;
//	Eigen::MatrixXi convex_f;
//	Eigen::MatrixXd convex_n;
//	Eigen::VectorXd convex_o;
//
//	orgQhull::Qhull qhull;
//
//	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_v(points_all);
//
//	orgQhull::PointCoordinates points(points_all.cols(), "");
//	points.append(row_v.size(), row_v.data());
//
//	std::string qh_command = "Qt Qx";
//	qhull.runQhull(points.comment().c_str(), points.dimension(), points.count(), &*points.coordinates(),
//		qh_command.c_str());
//
//	// check for errors
//	if (qhull.hasQhullMessage()) {
//		std::cerr << "\nQhull message:\n" << qhull.qhullMessage();
//		qhull.clearQhullMessage();
//		throw std::runtime_error("Qhull Error (see message)");
//	}
//
//	auto vertices_q = qhull.vertexList().toStdVector();
//	auto facets_q = qhull.facetList().toStdVector();
//
//	convex_v.setZero(vertices_q.size(), 3);
//	convex_f.setZero(facets_q.size(), 3);
//	convex_n.setZero(facets_q.size(), 3);
//	convex_o.setZero(facets_q.size());
//
//	std::unordered_map<int, int> map_q;
//
//	for (unsigned i = 0; i < vertices_q.size(); ++i) {
//		orgQhull::QhullPoint p = vertices_q[i].point();
//		convex_v.row(i) = Eigen::RowVector3d(p[0], p[1], p[2]);
//		map_q.insert(std::make_pair(vertices_q[i].id(), i));
//		int point_index = vertices_q[i].point().id();
//		if (point_index < voxels.size())
//			voxels[vertices_q[i].point().id()]->convex_vertices = true;
//
//	}
//
//
//	for (unsigned i = 0; i < facets_q.size(); ++i) {
//		auto fv = facets_q[i].vertices().toStdVector();
//		for (int j = 0; j < fv.size(); ++j) {
//			auto search = map_q.find(fv[j].id());
//			if (search != map_q.end()) {
//				convex_f.row(i)[j] = search->second;
//			}
//			else {
//				std::cout << "Something goes wrong in qhull" << std::endl;
//				exit(-1);
//			}
//		}
//
//		Eigen::RowVector3d vec_1 = convex_v.row((convex_f.row(i)(1))) - convex_v.row((convex_f.row(i)(0)));
//		Eigen::RowVector3d vec_2 = convex_v.row((convex_f.row(i)(2))) - convex_v.row((convex_f.row(i)(0)));
//
//		Eigen::RowVector3d normal = vec_1.cross(vec_2);
//
//		Eigen::RowVector3d qhull_normal = Eigen::RowVector3d(facets_q[i].getFacetT()->normal[0],
//			facets_q[i].getFacetT()->normal[1],
//			facets_q[i].getFacetT()->normal[2]);
//
//		convex_n.row(i) = qhull_normal;
//		convex_o(i) = facets_q[i].hyperplane().offset();
//
//
//		if (normal.dot(qhull_normal) < 0) {
//			unsigned temp = convex_f.row(i)[0];
//			convex_f.row(i)[0] = convex_f.row(i)[2];
//			convex_f.row(i)[2] = temp;
//
//		}
//
//	}
//
//	CV = convex_v;
//	CF = convex_f;
//}

//void VoxelGrid::generateConvexHull(std::vector<voxel *> &voxels, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF,
//	Eigen::MatrixXi &CE,
//	std::vector<std::pair<Eigen::RowVector3d, Eigen::RowVector3d>> &edge_pairs,
//	bool with_platform) {
//	edge_pairs.clear();
//	Eigen::MatrixXd points_all;
//	if (with_platform)
//		this->getVoxelsPoswithPlatform(voxels, platform_vertices, points_all);
//	else {
//		points_all.resize(voxels.size(), 3);
//		for (unsigned k = 0; k < points_all.rows(); ++k)
//			points_all.row(k) = Eigen::RowVector3d(voxels[k]->position);
//	}
//
//	Eigen::MatrixXd convex_v;
//	Eigen::MatrixXi convex_f;
//	Eigen::MatrixXd convex_n;
//	Eigen::VectorXd convex_o;
//
//	orgQhull::Qhull qhull;
//
//	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_v(points_all);
//
//	orgQhull::PointCoordinates points(points_all.cols(), "");
//	points.append(row_v.size(), row_v.data());
//
//	std::string qh_command = "Qt Qx";
//	qhull.runQhull(points.comment().c_str(), points.dimension(), points.count(), &*points.coordinates(),
//		qh_command.c_str());
//
//	// check for errors
//	if (qhull.hasQhullMessage()) {
//		std::cerr << "\nQhull message:\n" << qhull.qhullMessage();
//		qhull.clearQhullMessage();
//		throw std::runtime_error("Qhull Error (see message)");
//	}
//
//	auto vertices_q = qhull.vertexList().toStdVector();
//	auto facets_q = qhull.facetList().toStdVector();
//
//	convex_v.setZero(vertices_q.size(), 3);
//	convex_f.setZero(facets_q.size(), 3);
//	convex_n.setZero(facets_q.size(), 3);
//	convex_o.setZero(facets_q.size());
//
//	std::unordered_map<int, int> map_q;
//
//	for (unsigned i = 0; i < vertices_q.size(); ++i) {
//		orgQhull::QhullPoint p = vertices_q[i].point();
//		convex_v.row(i) = Eigen::RowVector3d(p[0], p[1], p[2]);
//		map_q.insert(std::make_pair(vertices_q[i].id(), i));
//		int point_index = vertices_q[i].point().id();
//		if (point_index < voxels.size())
//			voxels[vertices_q[i].point().id()]->convex_vertices = true;
//
//	}
//
//
//	std::vector<bool> polygon_flag;
//	polygon_flag.resize(facets_q.size(), true);
//	for (unsigned i = 0; i < facets_q.size(); ++i) {
//		auto fv = facets_q[i].vertices().toStdVector();
//		for (int j = 0; j < fv.size(); ++j) {
//			auto search = map_q.find(fv[j].id());
//			if (search != map_q.end()) {
//				convex_f.row(i)[j] = search->second;
//			}
//			else {
//				std::cout << "Something goes wrong in qhull" << std::endl;
//				exit(-1);
//			}
//		}
//
//		Eigen::RowVector3d vec_1 = convex_v.row((convex_f.row(i)(1))) - convex_v.row((convex_f.row(i)(0)));
//		Eigen::RowVector3d vec_2 = convex_v.row((convex_f.row(i)(2))) - convex_v.row((convex_f.row(i)(0)));
//
//		Eigen::RowVector3d normal = vec_1.cross(vec_2);
//
//		Eigen::RowVector3d qhull_normal = Eigen::RowVector3d(facets_q[i].getFacetT()->normal[0],
//			facets_q[i].getFacetT()->normal[1],
//			facets_q[i].getFacetT()->normal[2]);
//
//		convex_n.row(i) = qhull_normal;
//		convex_o(i) = facets_q[i].hyperplane().offset();
//
//
//		if (normal.dot(qhull_normal) < 0) {
//			unsigned temp = convex_f.row(i)[0];
//			convex_f.row(i)[0] = convex_f.row(i)[2];
//			convex_f.row(i)[2] = temp;
//
//		}
//
//		for (int j = 0; j < fv.size(); ++j) {
//			if (fv[j].point().id() >= voxels.size())
//				polygon_flag[i] = false;
//		}
//	}
//
//	CV = convex_v;
//	CF = convex_f;
//	std::vector<Eigen::RowVector3i> polygon_facets_vec;
//	for (int i = 0; i < convex_f.rows(); ++i) {
//		if (polygon_flag[i])
//			polygon_facets_vec.push_back(convex_f.row(i));
//	}
//	Eigen::MatrixXi polygon_f;
//	polygon_f.resize(polygon_facets_vec.size(), 3);
//	for (int i = 0; i < polygon_facets_vec.size(); ++i) {
//		polygon_f.row(i) = polygon_facets_vec[i];
//	}
//
//	Eigen::MatrixXd new_v;
//	Eigen::MatrixXi new_f;
//	Eigen::VectorXi NI;
//
//	igl::remove_unreferenced(convex_v, polygon_f, new_v, new_f, NI);
//
//	Eigen::VectorXi bnd;
//	igl::boundary_loop(new_f, bnd);
//	for (int i = 0; i < bnd.size(); ++i) {
//		std::pair<Eigen::RowVector3d, Eigen::RowVector3d> edge;
//		int index_a = i % bnd.size();
//		int index_b = (i + 1) % bnd.size();
//		Eigen::RowVector3d a = new_v.row(bnd(index_a));
//		Eigen::RowVector3d b = new_v.row(bnd(index_b));
//		edge_pairs.push_back(std::make_pair(a, b));
//
//	}
//
//	CE = polygon_f;
//
//}


//unsigned
//VoxelGrid::detectShadowPoints(std::vector<voxel *> &voxels, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF, double eps,
//	std::vector<voxel *> &shadow_points) {
//
//	shadow_points.clear();
//	Eigen::MatrixXd voxel_points;
//	voxel_points.resize(voxels.size(), 3);
//	for (unsigned k = 0; k < voxel_points.rows(); ++k)
//		voxel_points.row(k) = Eigen::RowVector3d(voxels[k]->position);
//	Eigen::VectorXd signed_distances;
//	this->signedDistanceToConvexHull(voxel_points, CV, CF,
//		signed_distances);
//
//
//	unsigned shadow_count = 0;
//	for (int i = 0; i < signed_distances.size(); ++i) {
//		if (signed_distances(i) < eps) {
//			shadow_count++;
//			shadow_points.push_back(voxels[i]);
//		}
//
//	}
//
//	return shadow_count;
//
//}

//unsigned VoxelGrid::checkSetShadowed(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
//	std::vector<voxel *> &solid_voxels, std::vector<voxel *> &shadow_points) {
//
//	std::vector<voxel *> voxels_test;
//	voxels_test = previous;
//	std::vector<voxel *> unprocessed_voxels;
//
//
//	for (int i = 0; i < check_set.size(); ++i) {
//		voxels_test.push_back(check_set[i]);
//		check_set[i]->layer = 9999;
//	}
//	for (size_t i = 0; i < solid_voxels.size(); ++i) {
//		if (!solid_voxels[i]->layer)
//			unprocessed_voxels.push_back(solid_voxels[i]);
//	}
//
//	Eigen::MatrixXd CV;
//	Eigen::MatrixXi CF;
//	unsigned shadow_count = 0;
//	this->generateConvexHull(voxels_test, CV, CF, true);
//
//	shadow_count = this->detectShadowPoints(unprocessed_voxels, CV, CF, -2.0 * size, shadow_points);
//
//	for (int i = 0; i < check_set.size(); ++i)
//		check_set[i]->layer = 0;
//	if (shadow_count > 3)
//		return shadow_count;
//
//	else {
//
//		return 0;
//	}
//
//}
//
//unsigned VoxelGrid::checkSetShadowedRest(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
//	std::vector<voxel *> &rest_set, std::vector<voxel *> &shadow_points) {
//	std::vector<voxel *> voxels_test;
//	voxels_test = previous;
//	std::vector<voxel *> unprocessed_voxels;
//	for (int i = 0; i < check_set.size(); ++i) {
//		voxels_test.push_back(check_set[i]);
//	}
//	Eigen::MatrixXd CV;
//	Eigen::MatrixXi CF;
//	unsigned shadow_count = 0;
//	this->generateConvexHull(voxels_test, CV, CF, true);
//
//	shadow_count = this->detectShadowPoints(rest_set, CV, CF, -2.0 * size, shadow_points);
//
//	if (shadow_count > 3)
//		return shadow_count;
//
//	else {
//
//		return 0;
//	}
//
//}


//void VoxelGrid::roughClustering(Eigen::MatrixXd &reference, std::vector<voxel *> &points,
//	std::vector<std::vector<voxel *> > clusters) {
//
//	clusters.resize(reference.rows());
//
//	kdtree *kd_tree;
//
//	kd_tree = kd_create(3);
//
//	for (unsigned int i = 0; i < reference.rows(); ++i) {
//		auto *intPtr = new unsigned int(i);
//		double pos[3];
//		pos[0] = reference(i, 0);
//		pos[1] = reference(i, 1);
//		pos[2] = reference(i, 2);
//		std::cout << *(&i) << std::endl;
//		kd_insert(kd_tree, pos, intPtr);
//	}
//	std::cout << "////////////////////////////////" << std::endl;
//
//	for (int i = 0; i < points.size(); ++i) {
//		double pos[3] = { points[i]->position(0), points[i]->position(1), points[i]->position(2) };
//		std::cout << "1111" << std::endl;
//		std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
//		kdres *res = kd_nearest(kd_tree, pos);
//		auto *index = (unsigned int *)kd_res_item(res, pos);
//		clusters[*index].push_back(points[i]);
//	}
//
//	std::vector<std::vector<voxel *> > new_clusters;
//	for (int i = 0; i < clusters.size(); ++i) {
//		if (clusters[i].size() != 0)
//			new_clusters.push_back(clusters[i]);
//	}
//	new_clusters = clusters;
//
//	int count = 0;
//	for (int i = 0; i < clusters.size(); ++i) {
//		for (int j = 0; j < clusters[i].size(); ++j)
//			count++;
//	}
//
//}

void VoxelGrid::randomSample(std::vector<voxel *> &all_voxels, double percent,
	std::vector<voxel *> &sample_voxels) {
	sample_voxels.clear();
	int theVoxelsSize = all_voxels.size();

	std::vector<int> indexs;
	indexs.resize(theVoxelsSize);

	for (int i = 0; i < theVoxelsSize; i++) {
		indexs[i] = i;
	}

	int newNumberOfVoxels = theVoxelsSize * percent / 100.0;
	int voxelsToRemove = theVoxelsSize - newNumberOfVoxels;
	int lastVoxelIndexs = theVoxelsSize - 1;

	std::random_device rd;
	std::mt19937 gen(rd());

	for (int i = 0; i < voxelsToRemove; i++) {
		std::uniform_int_distribution<int> dist(0, lastVoxelIndexs);
		int index = dist(gen);
		std::swap(indexs[index], indexs[lastVoxelIndexs]);
		--lastVoxelIndexs;
	}

	indexs.resize(newNumberOfVoxels);

	for (int i = 0; i < indexs.size(); i++)
		sample_voxels.push_back(all_voxels[indexs[i]]);

}

void VoxelGrid::seperateVoxelsHalf(std::vector<voxel *> &all_voxels, std::vector<int> &selected,
	std::vector<int> &left_index, std::vector<int> &right_index) {

	left_index.clear();
	right_index.clear();


	std::vector<voxel *> selected_voxels;

	for (int i = 0; i < selected.size(); ++i) {
		selected_voxels.push_back(all_voxels[selected[i]]);
	}

	Eigen::MatrixXd points;
	this->getVoxelsPos(selected_voxels, points);

	Eigen::Vector3d centroid;
	Eigen::Matrix3d covariance;
	Eigen::MatrixXd eigenvectors;
	Eigen::VectorXd eigenvalues;

	centroid.setZero();

	for (int i = 0; i < points.rows(); i++) {
		centroid(0) += points(i, 0);
		centroid(1) += points(i, 1);
		centroid(2) += points(i, 2);
	}

	centroid /= points.rows();


	covariance.setZero();

	for (unsigned int i = 0; i < points.rows(); ++i) {
		Eigen::Vector3d pt;
		pt[0] = points(i, 0) - centroid[0];
		pt[1] = points(i, 1) - centroid[1];
		pt[2] = points(i, 2) - centroid[2];

		covariance(1, 1) += pt.y() * pt.y(); //the non X parts
		covariance(1, 2) += pt.y() * pt.z();
		covariance(2, 2) += pt.z() * pt.z();

		pt *= pt.x();
		covariance(0, 0) += pt.x(); //the X related parts
		covariance(0, 1) += pt.y();
		covariance(0, 2) += pt.z();
	}

	covariance(1, 0) = covariance(0, 1);
	covariance(2, 0) = covariance(0, 2);
	covariance(2, 1) = covariance(1, 2);


	/* normalize  */
	covariance /= points.rows();
	//
	//    // Calculate eigenvectors
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3d eigenvectorsMatrix = eigenSolver.eigenvectors();
	eigenvectorsMatrix.col(2) = eigenvectorsMatrix.col(0).cross(eigenvectorsMatrix.col(1));


	/* Create the inverse of the transformation matrix and reproject the point cloud, saving it in a different array
	* if reprojectInput is false or in the same array otherwise */
	Eigen::Matrix4d transformationMatrixInv = Eigen::Matrix4d::Identity();
	transformationMatrixInv.block<3, 3>(0, 0) = eigenvectorsMatrix.transpose();
	transformationMatrixInv.block<3, 1>(0, 3) = -1.0F * (transformationMatrixInv.block<3, 3>(0, 0) * centroid);
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix.block<3, 3>(0, 0) = eigenvectorsMatrix;
	transformationMatrix.block<3, 1>(0, 3) = centroid;
	Eigen::MatrixXd reproject_points;
	reproject_points.resize(points.rows(), 3);
	for (int i = 0; i < points.rows(); i++) {
		double x = points(i, 0);
		double y = points(i, 1);
		double z = points(i, 2);


		double r_x = transformationMatrixInv(0, 0) * x + transformationMatrixInv(0, 1) * y +
			transformationMatrixInv(0, 2) * z + transformationMatrixInv(0, 3);
		double r_y = transformationMatrixInv(1, 0) * x + transformationMatrixInv(1, 1) * y +
			transformationMatrixInv(1, 2) * z + transformationMatrixInv(1, 3);
		double r_z = transformationMatrixInv(2, 0) * x + transformationMatrixInv(2, 1) * y +
			transformationMatrixInv(2, 2) * z + transformationMatrixInv(2, 3);

		reproject_points.row(i) = Eigen::RowVector3d(r_x, r_y, r_z);
	}
	//
	//    // Calculate minimum and maximum points of the bounding box and the center
	Eigen::Vector3d min, max;
	min[0] = DBL_MAX;
	min[1] = DBL_MAX;
	min[2] = DBL_MAX;

	max[0] = -DBL_MAX;
	max[1] = -DBL_MAX;
	max[2] = -DBL_MAX;

	for (int i = 0; i < reproject_points.rows(); i++) {

		double x = reproject_points(i, 0);
		double y = reproject_points(i, 1);
		double z = reproject_points(i, 2);
		// Min/max on x
		if (x < min[0])
			min[0] = x;
		if (x > max[0])
			max[0] = x;

		// Min/max on y
		if (y < min[1])
			min[1] = y;
		if (y > max[1])
			max[1] = y;

		// Min/max on z
		if (z < min[2])
			min[2] = z;
		if (z > max[2])
			max[2] = z;
	}

	Eigen::Vector3d center = (min + max) / 2;

	for (int i = 0; i < reproject_points.rows(); i++) {

		double z = reproject_points(i, 2);

		if (z <= center(2))
			left_index.push_back(selected[i]);
		else
			right_index.push_back(selected[i]);

	}

}


bool VoxelGrid::seperateVoxelsHalf(std::vector<voxel *> &all_voxels, std::vector<int> &selected,
	std::vector<int> &left_index, std::vector<int> &right_index, int level) {
	left_index.clear();
	right_index.clear();


	std::vector<voxel *> selected_voxels;

	for (int i = 0; i < selected.size(); ++i) {
		selected_voxels.push_back(all_voxels[selected[i]]);
	}

	Eigen::MatrixXd points;
	this->getVoxelsPos(selected_voxels, points);


	Eigen::RowVector3d min = points.colwise().minCoeff();
	Eigen::RowVector3d max = points.colwise().maxCoeff();

	if (level % 3 == 0) {
		double seperate = (min(0) + max(0)) / 2.0;
		for (int i = 0; i < points.rows(); i++) {
			if (points(i, 0) < seperate)
				left_index.push_back(selected[i]);
			else
				right_index.push_back(selected[i]);

		}
	}
	else if (level % 3 == 1) {

		double seperate = (min(1) + max(1)) / 2.0;
		for (int i = 0; i < points.rows(); i++) {
			if (points(i, 1) < seperate)
				left_index.push_back(selected[i]);
			else
				right_index.push_back(selected[i]);

		}
	}
	else {
		double seperate = (min(2) + max(2)) / 2.0;

		for (int i = 0; i < points.rows(); i++) {
			if (points(i, 2) < seperate)
				left_index.push_back(selected[i]);
			else
				right_index.push_back(selected[i]);

		}
	}
	if (!left_index.size() || !right_index.size())
		return false;
	else
		return true;

}

//void VoxelGrid::getNonShadowedVoxels(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
//	std::vector<voxel *> &solid_voxels, std::vector<voxel *> &safe_set) {
//
//	safe_set.clear();
//
//	BSTree *tree = new BSTree;
//
//	std::cout << "start build" << std::endl;
//
//	clock_t t;
//	t = clock();
//
//
//	tree->BSTreeFromVoxelGrid(this, check_set, previous, solid_voxels);
//
//	std::cout << "end build" << std::endl;
//
//	t = clock() - t;
//
//	printf("Tree manipulation takes  (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);
//
//	std::vector<int> safe_indices;
//
//	time_file << ((float)t) / CLOCKS_PER_SEC << std::endl;
//
//	if (!tree->getSafeNodes(safe_indices)) {
//		delete tree;
//		return;
//	}
//	else {
//		for (int i = 0; i < safe_indices.size(); ++i) {
//			safe_set.push_back(check_set[safe_indices[i]]);
//		}
//		std::cout << "safe indices size " << safe_indices.size() << std::endl;
//
//		delete tree;
//	}
//
//
//}


//void VoxelGrid::getNonShadowedVoxelsIncremental(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
//	std::vector<voxel *> &solid_voxels, std::vector<voxel *> &safe_set) {
//
//
//	clock_t t;
//	t = clock();
//	safe_set.clear();
//	auto leftmost = std::distance(check_set.begin(), std::min_element(check_set.begin(), check_set.end(),
//		[](voxel *const &a, voxel *const &b) -> bool {
//		return a->position(0) < b->position(0);
//	}));
//
//
//	std::vector<double> distances;
//
//	for (int i = 0; i < check_set.size(); ++i) {
//		double distance = (check_set[i]->position - check_set[leftmost]->position).norm();
//		distances.push_back(distance);
//	}
//
//	std::vector<int> indices(distances.size());
//	std::size_t n(0);
//	std::generate(std::begin(indices), std::end(indices), [&] { return n++; });
//
//	std::sort(std::begin(indices),
//		std::end(indices),
//		[&](double i1, double i2) { return distances[i1] < distances[i2]; });
//
//	std::vector<bool> visited_flag;
//	visited_flag = std::vector<bool>(solid_voxels.size(), false);
//
//	std::vector<int> safe_indices;
//	for (int i = 0; i < indices.size(); ++i) {
//		std::vector<voxel *> tmp_set;
//		for (int j = 0; j < safe_set.size(); ++j)
//			tmp_set.push_back(safe_set[j]);
//
//		tmp_set.push_back(check_set[indices[i]]);
//		std::vector<voxel *> shadows;
//		////////////////////////////////////////////////////
//
//		visited_flag[i] = true;
//
//		std::vector<voxel *> unprocessed;
//		for (int j = 0; j < visited_flag.size(); ++j) {
//			if (!visited_flag[j])
//				unprocessed.push_back(solid_voxels[j]);
//		}
//		///////////////////////////////////////////////
//
//		if (this->checkSetShadowed(previous, tmp_set, unprocessed, shadows))
//			continue;
//		else {
//			safe_set.push_back(check_set[indices[i]]);
//			safe_indices.push_back(indices[i]);
//		}
//
//	}
//
//	std::cout << "safe indices size " << safe_indices.size() << std::endl;
//
//	t = clock() - t;
//
//	printf("manipulation takes  (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);
//
//}
