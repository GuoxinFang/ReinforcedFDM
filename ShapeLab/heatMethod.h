/*Heat method field value computing*/
/*Written by Guoxin Fang 08-15-2020 (last updated)*/

#ifndef HEATMETHOD_H
#define HEATMETHOD_H

#include "PolygenMesh.h"
#include "../GLKLib/GLKObList.h"

class heatMethod
{
public:
    heatMethod(QMeshPatch* inputMesh);
    ~heatMethod();

    void compBoundaryHeatKernel();

    void runHeatMethod();
    void meshRefinement();

private:
    QMeshPatch* surfaceMesh;

    Eigen::SparseMatrix<double> L;
    Eigen::SparseMatrix<double> A;
    double t;
    Eigen::SparseMatrix<double> F;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> poissonSolver;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> heatSolver;

    void _initBoundaryHeatKernel();

    int detectGenus();
    void initialMeshIndex();
    void heatMethodPreProcess();
    void heatMethodCompwithConstrain(Eigen::VectorXd& u);

    void buildLaplacian(Eigen::SparseMatrix<double>& L, bool constrain) const;
    void buildAreaMatrix(Eigen::SparseMatrix<double>& A) const;
    void computeVectorField(Eigen::MatrixXd& gradients, const Eigen::VectorXd& u) const;
    void computeIntegratedDivergence(Eigen::VectorXd& integratedDivs,
        const Eigen::MatrixXd& gradients) const;
    double subtractMinimumDistance(Eigen::VectorXd& phi) const;
    double computeEdgeAngle(QMeshNode* Node, QMeshEdge* connectEdge) const;
    double meanEdgeLength() const;
};

#endif // HEATMETHOD_H
