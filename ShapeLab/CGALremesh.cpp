#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>

#include <boost/function_output_iterator.hpp>
#include <fstream>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;

typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor     edge_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

struct halfedge2edge
{
	halfedge2edge(const Mesh& m, std::vector<edge_descriptor>& edges)
		: m_mesh(m), m_edges(edges)
	{}
	void operator()(const halfedge_descriptor& h) const
	{
		m_edges.push_back(edge(h, m_mesh));
	}
	const Mesh& m_mesh;
	std::vector<edge_descriptor>& m_edges;
};

int main(int argc, char* argv[])
{
	const char* filename = (argc > 1) ? argv[1] : "../data/14layer.off";
	std::ifstream input(filename);

	Mesh mesh;
	if (!input || !(input >> mesh) || !CGAL::is_triangle_mesh(mesh)) {
		std::cerr << "Not a valid input file." << std::endl;
		return 1;
	}

	double target_edge_length = 0.4;
	unsigned int nb_iter = 3;

	std::cout << "Split border...";

	std::vector<edge_descriptor> border;
	PMP::border_halfedges(faces(mesh),
		mesh,
		boost::make_function_output_iterator(halfedge2edge(mesh, border)));
	PMP::split_long_edges(border, target_edge_length, mesh);

	std::cout << "done." << std::endl;

	std::cout << "Start remeshing of " << filename
		<< " (" << num_faces(mesh) << " faces)..." << std::endl;

	PMP::isotropic_remeshing(
		faces(mesh),
		target_edge_length,
		mesh,
		PMP::parameters::number_of_iterations(nb_iter)
		.protect_constraints(true)//i.e. protect border, here
	);

	std::cout << num_faces(mesh) << std::endl;
	std::ofstream out("../data/14remesh.off");
	out << mesh;

	std::cout << "Remeshing done." << std::endl;
	//system("pause");
	return 0;
}
