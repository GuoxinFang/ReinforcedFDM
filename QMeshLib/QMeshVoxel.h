#pragma once
//
// Created by daichengkai on 1-5-17.
//

#ifndef VOXELFAB_VOXEL_H
#define VOXELFAB_VOXEL_H

#include <vector>
#include <algorithm>
#include <random>
#include <time.h>

#include "BSPTree.h"
#include "..\QMeshLib\PolygenMesh.h"
//#include <libqhullcpp/Qhull.h>
//#include <libqhullcpp/QhullFacetList.h>
//#include <libqhullcpp/PointCoordinates.h>
//#include <libqhullcpp/QhullVertexSet.h>
//#include <libqhullcpp/QhullFacetSet.h>
//#include <libqhullcpp/RboxPoints.h>
#include <unordered_map>
//#include <igl/signed_distance.h>
//#include <igl/edges.h>
//#include <igl/is_boundary_edge.h>
//#include <igl/remove_unreferenced.h>
//#include <igl/is_vertex_manifold.h>
//#include <igl/is_edge_manifold.h>
//#include <igl/boundary_loop.h>
//#include <igl/copyleft/cgal/points_inside_component.h>

//#include "kdtree.h"
//#include <igl/readOBJ.h>
//#include <igl/signed_distance.h>
//
//#include <igl/copyleft/cgal/signed_distance_isosurface.h>
//#include <boost/graph/dijkstra_shortest_paths.hpp>
//#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/iteration_macros.hpp>
//#include <boost/graph/properties.hpp>
//#include <boost/property_map/property_map.hpp>
//#include <boost/graph/graphviz.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/filesystem/operations.hpp>
//#include <boost/filesystem/fstream.hpp>
//#include <boost/algorithm/string.hpp>
#include <fstream>

typedef double Weight;
//typedef uint IndexType;
//
//typedef boost::property<boost::edge_weight_t, Weight> EdgeWeightProperty;
//typedef boost::property<boost::vertex_name_t, std::string> NameProperty;
//
//typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, NameProperty, EdgeWeightProperty> UndirectedGraph;
//
//typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
//typedef boost::graph_traits<UndirectedGraph>::vertex_iterator VertexIterator;
//typedef boost::property_map<UndirectedGraph, boost::vertex_index_t>::type IndexMap;
//typedef boost::property_map<UndirectedGraph, boost::vertex_name_t>::type NameMap;
//typedef boost::iterator_property_map<Vertex *, IndexMap, Vertex, Vertex &> PredecessorMap;
//typedef boost::iterator_property_map<Weight *, IndexMap, Weight, Weight &> DistanceMap;


struct voxel {
	bool filled;
	bool enlarged;
	int x;
	int y;
	int z;
	Eigen::Vector3d position;
	unsigned peeling_order;
	bool processed;
	bool visited;
	int cluster;
	bool preserved;
	unsigned layer;
	bool convex_vertices;
	Eigen::Vector3d normal;
};


struct neighs {
	voxel *voxs[27];
};


class VoxelGrid {
public:

	VoxelGrid(const Eigen::MatrixXd &V, double voxel_size, int offset);

	VoxelGrid(std::string filename);

	~VoxelGrid();

	void Voxelization(BSPTree *bsp_tree);

	void voxelVisualization(PolygenMesh* voxelMesh);

	void Load(std::string filename);

	void Enlarge();

	void GetBoundarySolidVoxels(Eigen::MatrixXd &GV);

	unsigned generatePeelingOrder();

	//    unsigned generateGreedyGrowingOrder(std::vector<voxel*> &test);
	unsigned generateGreedyGrowingOrder(std::vector<voxel *> &missing_voxels);

	void getPeeling(unsigned layer, std::vector<voxel *> &peeling_voxels);

	void getGrowing(unsigned layer, std::vector<voxel *> &growing_voxels);

	void getMinMaxLayer(std::vector<voxel *> &voxels, unsigned &max, unsigned &min);

	void getMinMaxLayer(unsigned &max, unsigned &min);

	bool checkIsolated(std::vector<voxel *> &voxels, double radius, std::vector<std::vector<voxel *> > &clusters);

	void exportPeelingOrder(const std::string &path);

	void exportGrowingOrder(const std::string &path);

	unsigned generateGrowingOrder(std::vector<voxel *> &missing_voxels);

	void loadPlatform(const std::string &path);

	void
		voxelMesh(std::vector<voxel *> &voxels, Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces, Eigen::MatrixXd &colors,
			bool is_growed, bool is_heatmap);

	void getAllVoxels(std::vector<voxel *> &all_voxels);

	void generateConvexHull(const Eigen::MatrixXd &vertices, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF);

	void getVoxelsPos(const std::vector<voxel *> &voxels, Eigen::MatrixXd &pos);

	void seperateVoxelsHalf(std::vector<voxel *> &all_voxels, std::vector<voxel *> &left_voxels,
		std::vector<voxel *> &right_voxels);

	void seperateVoxelsHalf(std::vector<voxel *> &all_voxels, std::vector<int> &selected, std::vector<int> &left_index,
		std::vector<int> &right_index);

	bool seperateVoxelsHalf(std::vector<voxel *> &all_voxels, std::vector<int> &selected, std::vector<int> &left_index,
		std::vector<int> &right_index, int level);


	void getGrowingB(unsigned layer, std::vector<voxel *> &growing_voxels);

	unsigned checkSetShadowed(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
		std::vector<voxel *> &solid_voxels, std::vector<voxel *> &shadow_points);

	unsigned checkSetShadowedRest(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
		std::vector<voxel *> &rest_set, std::vector<voxel *> &shadow_points);

private:
	void init(int offset);

	void
		getVoxelsPoswithPlatform(const std::vector<voxel *> &voxels, const Eigen::MatrixXd &platform, Eigen::MatrixXd &pos);


	//bool getVoxelsInRadius(voxel *vox, double radius, std::vector<voxel *> &neighbor_voxels, kdtree *tree);

	//bool getVoxelsInRadius(Eigen::RowVector3d point, double radius, std::vector<voxel *> &neighbor_voxels, kdtree *tree);

	void getNonShadowedVoxels(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
		std::vector<voxel *> &solid_voxels, std::vector<voxel *> &safe_set);

	void getNonShadowedVoxelsIncremental(std::vector<voxel *> &previous, std::vector<voxel *> &check_set,
		std::vector<voxel *> &solid_voxels, std::vector<voxel *> &safe_set);


	//    bool getNodesInRadius(voxel* vx, double radius, std::vector<voxel*> &neighbor_voxels, kdtree* kd_tree);

	//void expandCluster(std::vector<voxel *> &neighbor_voxels, int cluster_index, double radius, kdtree *tree);

	void findVoxelsOnConvex(const Eigen::MatrixXd &CV, const Eigen::MatrixXi &CF, std::vector<voxel *> &all_voxels,
		std::vector<voxel *> &above_voxels, double eps);

	void findVoxelsOnBoundary(std::vector<voxel *> &solid_voxels, std::vector<voxel *> &boundary_voxels);

	void findVoxelsOnBoundary(std::vector<voxel *> &solid_voxels, std::vector<voxel *> &boundary_voxels, int layer);

	void getSolidVoxels(std::vector<voxel *> &solid_voxels);

	void removeCritialPoints(std::vector<voxel *> &voxels, std::vector<voxel *> &critical_voxels);

	void test(std::vector<voxel *> &voxels, Eigen::MatrixXd &points, int order);

	//    void getNeighborVoxels(voxel* voxel, std::vector<voxel*> &neighbor_voxels);

	void getNeighborVoxels(voxel *current_voxel, std::vector<voxel *> &neighbor_voxels);

	//void buildGraph(std::vector<voxel *> &voxels, UndirectedGraph &graph);

	//double getShortestPathDijkstra(voxel *start_voxel, voxel *end_voxel, std::vector<voxel *> &path_voxels,
	//	UndirectedGraph &graph);


	void roughClustering(Eigen::MatrixXd &convex_hull, std::vector<voxel *> &points,
		std::vector<std::vector<voxel *> > clusters);

	void iterativeClustering(Eigen::MatrixXd &reference, std::vector<voxel *> &points,
		std::vector<std::vector<voxel *> > clusters);


	void getNearestPair(const std::vector<voxel *> &cluster1, const std::vector<voxel *> &cluster2,
		std::pair<voxel *, voxel *> &p);

	void doubleRGB(int num, int max, int min, double &r, double &g, double &b);

	void randomSample(std::vector<voxel *> &all_voxels, double percent, std::vector<voxel *> &sample_voxels);

	//    void getShortestPathDijkstra(const std::initializer_list<IndexType>& start_voxel, const std::initializer_list<IndexType>& end_voxel,std::vector<IndexType>& path_voxels,UndirectedGraph &graph);


	double
		getCuttingPlane(const std::vector<voxel *> &cluster1, const std::vector<voxel *> &cluster2, Eigen::Vector4d &plane,
			Eigen::Vector3d mid_point);

	voxel *GetVoxel(int x, int y, int z);

	neighs *GetNeighbourhood(voxel *voxel);


	double getAverageHeight(std::vector<voxel *> &cluster);

	int getIndex(int x, int y, int z);

	void signedDistanceToConvexHull(Eigen::MatrixXd &point, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF,
		Eigen::VectorXd &distances);

	void searchNextwithCluster(std::vector<voxel *> &cluster, Eigen::MatrixXd &current_convex_front_vertices,
		Eigen::MatrixXi &current_convex_front_faces, std::vector<voxel *> &growing_next,
		int current_isovaluebool, bool is_greedy);

	void generateConvexHull(std::vector<voxel *> &voxels, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF, bool with_platform);

	void generateConvexHull(std::vector<voxel *> &voxels, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF, Eigen::MatrixXi &CE,
		std::vector<std::pair<Eigen::RowVector3d, Eigen::RowVector3d>> &edge_pairs,
		bool with_platform = true);


	unsigned detectShadowPoints(std::vector<voxel *> &voxels, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF, double eps,
		std::vector<voxel *> &shadow_points);

	void signedDistanceToConvexHullwithNormal(Eigen::MatrixXd &points, Eigen::MatrixXd &CV, Eigen::MatrixXi &CF,
		Eigen::VectorXd &distances, Eigen::MatrixXd &normals);


public:
	Eigen::Vector3i res;
	double size;

private:
	int xoff[27];
	int yoff[27];
	int zoff[27];
	BBox bounding_box;
	Eigen::MatrixXd mesh_vertices;
	Eigen::MatrixXd platform_vertices;
	Eigen::MatrixXi platform_faces;
	int offset_;
	std::vector<voxel *> voxels_;
	unsigned max_peeling_order;
	unsigned min_peeling_order;
	unsigned max_growing_order;
	unsigned min_growing_order;
	std::ofstream time_file;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


const int neighbor_num = 18;
const int neighbor_delta[18][3] = { { -1, 0,  0 },
{ 1,  0,  0 },
{ 0,  1,  0 },
{ 0,  -1, 0 },
{ 0,  0,  -1 },
{ 0,  0,  1 },
{ -1, 1,  0 },
{ -1, -1, 0 },
{ 1,  -1, 0 },
{ 1,  1,  0 },
{ -1, 0,  1 },
{ -1, 0,  -1 },
{ 1,  0,  -1 },
{ 1,  0,  1 },
{ 0,  -1, 1 },
{ 0,  -1, -1 },
{ 0,  1,  -1 },
{ 0,  1,  1 } };
const int cube_neighbor_num = 26;

const int cube_neighbor_point[26][3] = { { -1, -1, -1 },
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
{ 1,  1,  1 } };
#endif //VOXELFAB_VOXEL_H
