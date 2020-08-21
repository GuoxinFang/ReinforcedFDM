#include "heatmethodfield.h"
#include <iostream>

using namespace std;
using namespace Eigen;

heatMethodField::heatMethodField(QMeshPatch* inputMesh)
{
    surfaceMesh = inputMesh;
    //genus = detectGenus();
    genus = 0;
}

heatMethodField::~heatMethodField(){}

////////////////////////////////
/* Heat method basic function */

void heatMethodField::compBoundaryHeatKernel() {
    _initBoundaryHeatKernel();
    runHeatMethod();

    /*After compute, install the geo-Field to boundary field*/
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        Node->boundaryValue = Node->geoFieldValue;
    }

    printf("-- (heatMethodField) Finish compute boundary heat kernel !\n\n");
}

void heatMethodField::inputStressFieldValue() {

    FILE* fp = fopen("../Model/Surface_Mesh/1700field.txt", "r");
    if (!fp) {
        printf("Can not open the data file - OFF File Import!\n"); return;
    }

    Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(surfaceMesh->GetNodeNumber());
    char linebuf[256];
    int i = 0;
    while (true) {
        fgets(linebuf, 255, fp);
        if (feof(fp)) break;
        fieldValue(i) = atof(linebuf);
        i++;
    }
    fclose(fp);

    int index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        Node->zigzagValue = fieldValue(index); index++;
    }

    printf("-- (heatMethodField) Finish input the stress field value !\n\n");
}

void heatMethodField::_initBoundaryHeatKernel() {
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        Node->selected = false; Node->geoFieldValue = 0;
    }

    //set boundary point as heat source
    for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
        if (edge->IsBoundaryEdge() == true) {
            edge->GetStartPoint()->geoFieldValue = 1; edge->GetEndPoint()->geoFieldValue = 1;
            edge->GetStartPoint()->selected = true; edge->GetEndPoint()->selected = true;
        }
    }
}

void heatMethodField::runHeatMethod(){
    initialMeshIndex();
    VectorXd u(surfaceMesh->GetNodeNumber());
    u.setZero();

    int index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        u(index) = Node->geoFieldValue;
        index++;
    }

    heatMethodPreProcess();
    heatMethodCompwithConstrain(u);
}

void heatMethodField::initialMeshIndex()
{
    int index = 0; int hIndex = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(index); index++;

        if (Node->selected) Node->heatmethodIndex = -1;
        else {
            Node->heatmethodIndex = hIndex; hIndex++;
        }
    }
    index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
        Edge->SetIndexNo(index);
        index++;
    }
    index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace *Face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
        Face->SetIndexNo(index);
        index++;
    }
}

void heatMethodField::heatMethodPreProcess(){
    int v = surfaceMesh->GetNodeNumber();

    // build Laplacian Matrix
    Eigen::SparseMatrix<double> L(v, v);

    buildLaplacian(L,false);
    poissonSolver.compute(L);

    // build Area Matrix
    Eigen::SparseMatrix<double> A(v, v);
    buildAreaMatrix(A);

    double mean_edge_length = meanEdgeLength();

    t = mean_edge_length * mean_edge_length;

    F = A - t * L;

    heatSolver.compute(F);
}

void heatMethodField::heatMethodCompwithConstrain(Eigen::VectorXd& u) {

    u = heatSolver.solve(u);

    // compute unit vector field X and divergence ∇.X
    Eigen::MatrixXd gradients(surfaceMesh->GetFaceNumber(), 3);
    computeVectorField(gradients, u);
    //cout << gradients << endl;

    Eigen::VectorXd integratedDivs(surfaceMesh->GetNodeNumber());
    computeIntegratedDivergence(integratedDivs, gradients);
    //cout << integratedDivs << endl;

    /*build the equation with constrain*/
    int vc = surfaceMesh->GetNodeNumber(); //constrained node number
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        if (Node->selected) vc--;
    }
    Eigen::VectorXd integratedDivs_constrain(vc); int Cindex = 0; int index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        if (Node->selected) index++;
        else {
            integratedDivs_constrain(Cindex) = integratedDivs(index);
            Cindex++;
            index++;
        }
    }

    Eigen::SparseMatrix<double> L(vc, vc);
    buildLaplacian(L, true);
    index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        if (Node->selected) continue;
        double sumCoefficients = 0.0;

        for (GLKPOSITION Pos1 = Node->GetEdgeList().GetHeadPosition(); Pos1;) {
            QMeshEdge *connectEdge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos1);
            double coefficient = computeEdgeAngle(Node, connectEdge);
            sumCoefficients += coefficient;

            QMeshNode* Node1 = connectEdge->GetEndPoint();
            if (Node == Node1) Node1 = connectEdge->GetStartPoint();

            if (Node1->selected)
                integratedDivs_constrain(index) -= Node1->geoFieldValue * coefficient;
        }
        index++;
    }

    poissonSolver.compute(L);

    // compute max and min phis
    Eigen::VectorXd phi = poissonSolver.solve(integratedDivs_constrain);
    Eigen::VectorXd phiAll(surfaceMesh->GetNodeNumber());

    // set phi to each vertex
    index = 0; Cindex = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        if (Node->selected) phiAll(index) = Node->geoFieldValue;
        else { phiAll(index) = phi(Cindex); Cindex++; }
        index++;
    }

    subtractMinimumDistance(phiAll); index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        Node->geoFieldValue = phiAll(index);
        index++;
    }

}


//////////////////////////////
/* Mesh processing function */
int heatMethodField::detectGenus() {
	int meshgenus = 0;
	int bNodeinprocessed = 0;

	//get boundary node and install
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
		if (Edge->IsBoundaryEdge()) {
			Edge->GetStartPoint()->isBoundaryNode = true;
			Edge->GetEndPoint()->isBoundaryNode = true;
		}
	}
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		if (Node->isBoundaryNode)
		{
			bNodeinprocessed++; Node->processed = false;
		}
	}

	//cout<<"boundary node num = "<<bNodeinprocessed<<endl;

	if (bNodeinprocessed == 0) {
		std::cout << "This is a closed mesh! cannot used to generate tool path!" << std::endl;
		return 0;
	}

	while (bNodeinprocessed != 0) {
		QMeshNode* startNode; //find the start node, should always be found
		for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
			startNode = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
			if (startNode->isBoundaryNode && startNode->processed == false) break;
		}
		startNode->processed = true;
		bNodeinprocessed--;

		QMeshNode* nextNode; bool nNodeDetected;
		do {
			nNodeDetected = false;
			//Notice that, node->getnodelist won't return anything since we didn't install the information!
			for (GLKPOSITION Pos = startNode->GetEdgeList().GetHeadPosition(); Pos;) {
				QMeshEdge* connectEdge = (QMeshEdge*)startNode->GetEdgeList().GetNext(Pos);
				if (!connectEdge->IsBoundaryEdge()) continue;

				nextNode = connectEdge->GetEndPoint();
				if (nextNode == startNode) nextNode = connectEdge->GetStartPoint();

				if (nextNode->processed == false) {
					bNodeinprocessed--;
					nNodeDetected = true;
					nextNode->processed = true;
					startNode = nextNode;
					break;
				}
			}
		} while (nNodeDetected == true);
		meshgenus++;
	}

	//std::cout<< "The boundary ring of this mesh is "<< meshgenus << std::endl <<std::endl;
	return meshgenus;
}

//we can input a plane equation to cut the given mesh for this function
void heatMethodField::planeCutSelection(double norm[]) {

	Vector3d centroid; centroid << 0, 0, 0;

	////compute the center of mesh as the plane cut position
	//for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
	//	double pp[3]; Node->GetCoord3D(pp);
	//	for (int i = 0; i < 3; i++) centroid(i) += pp[i];
	//}
	//for (int i = 0; i < 3; i++) centroid(i) /= surfaceMesh->GetNodeNumber();

	//   std::cout << "Center point = " << std::endl << centroid << std::endl;
	//   //Vector3d PCAdir; PCAdir << 1.0, 3.0, 0.0;
	Vector3d PCAdir; PCAdir << 1.0, 0.0, 0.0;
	PCAdir = PCAdir.normalized();

	//compute tht PCA direction for the given mesh
	Eigen::MatrixXd points(surfaceMesh->GetNodeNumber(), 3);
	int index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(points(index, 0), points(index, 1), points(index, 2));
		index++;
	}

	Eigen::Vector3d means = points.colwise().mean();

	centroid = means;
	//std::cout << "Center point = " << std::endl << centroid << std::endl;

	// Subtract mean image from the data set to get mean centered data vector
	Eigen::MatrixXd U = points.rowwise() - means.transpose();

	// Covariance matrix from the mean centered data matrix
	Eigen::MatrixXd covariance = (U.transpose() * U) / (double)(points.rows());

	// Calculate eigenvectors
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3d eigenvectorsMatrix = eigenSolver.eigenvectors();
	Eigen::Vector3d eigenvalue = eigenSolver.eigenvalues();

	//std::cout << "eigenvalue = " << std::endl << eigenvalue << std::endl;
	//std::cout << "eigenvectorsMatrix = " << std::endl << eigenvectorsMatrix << std::endl;

	std::ptrdiff_t Maxindex;
	//eigenvalue.maxCoeff(&index); //Max principle stress
	(eigenvalue.cwiseAbs()).maxCoeff(&Maxindex); //Max principle stress (ABS)

	PCAdir = eigenvectorsMatrix.col(Maxindex);

	PCAdir << 1.0, 0.0, 0.0;
	if (norm != NULL) {
		for (int i = 0; i < 3; i++) PCAdir(i) = norm[i];
	}
	PCAdir.normalize();

	/*Plane equation: Ax+By+Cz+D = 0, detect the intersect triangle*/
	double D = 0.0;
	for (int i = 0; i < 3; i++) D -= centroid(i) * PCAdir(i);
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
		Face->isIntersetwithPlane = false;
		int side = 0;
		for (int i = 0; i < 3; i++) {
			double pp[3];
			Face->GetNodeRecordPtr(i)->GetCoord3D(pp[0], pp[1], pp[2]);
			double planeDetect = PCAdir(0) * pp[0] + PCAdir(1) * pp[1] + PCAdir(2) * pp[2] + D;
			if (abs(planeDetect) < 0.00000001) std::cout << "special case!!!" << std::endl;
			if (planeDetect > 0) side++;
		}
		if (side == 1 || side == 2) {
			Face->isIntersetwithPlane = true;
			//std::cout << "Plane selected !" << std::endl;
		}
	}

	/*set Initial Seed Point Value with their distance to the interset edge*/
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Node->geoFieldValue = 0.0;
		Node->selected = false;
	}

	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace *Face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
		if (Face->isIntersetwithPlane) {
			for (int i = 0; i < 3; i++) {
				double pp[3];
				Face->GetNodeRecordPtr(i)->GetCoord3D(pp[0], pp[1], pp[2]);
				double planeDetect = PCAdir(0) * pp[0] + PCAdir(1) * pp[1] + PCAdir(2) * pp[2] + D;
				Face->GetNodeRecordPtr(i)->geoFieldValue = planeDetect;
				Face->GetNodeRecordPtr(i)->selected = true;
			}
		}
	}

}


//////////////////////////////////////////////////////
/* Below are the function for heat method computing */
void heatMethodField::buildLaplacian
(Eigen::SparseMatrix<double> &L,bool constrain) const {
    std::vector<Eigen::Triplet<double>> LTriplet;

    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        if (constrain && Node->heatmethodIndex<0) continue;
        double sumCoefficients = 0.0;

        for (GLKPOSITION Pos1 = Node->GetEdgeList().GetHeadPosition(); Pos1;) {
            QMeshEdge *connectEdge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos1);
            double coefficient = computeEdgeAngle(Node, connectEdge);
            sumCoefficients += coefficient;
            QMeshNode* Node1 = connectEdge->GetEndPoint(); if (Node == Node1) Node1 = connectEdge->GetStartPoint();

            if(!constrain) LTriplet.push_back(Eigen::Triplet<double>(Node->GetIndexNo(), Node1->GetIndexNo(), coefficient));
            else {
                if (Node1->heatmethodIndex < 0) continue;
                else LTriplet.push_back(Eigen::Triplet<double>(Node->heatmethodIndex, Node1->heatmethodIndex, coefficient));
            }
        }
        if (!constrain)
            LTriplet.push_back(Eigen::Triplet<double>(Node->GetIndexNo(), Node->GetIndexNo(), -sumCoefficients));
        else
            LTriplet.push_back(Eigen::Triplet<double>(Node->heatmethodIndex, Node->heatmethodIndex, -sumCoefficients));

    }
    L.setFromTriplets(LTriplet.begin(), LTriplet.end());
}

void heatMethodField::buildAreaMatrix
(Eigen::SparseMatrix<double> &A) const {
    std::vector<Eigen::Triplet<double>> ATriplet;

    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        ATriplet.push_back(Eigen::Triplet<double>(Node->GetIndexNo(), Node->GetIndexNo(), Node->dualArea()));
    }
    A.setFromTriplets(ATriplet.begin(), ATriplet.end());
}

double heatMethodField::computeEdgeAngle
(QMeshNode* Node, QMeshEdge* connectEdge) const {
    Eigen::Vector3d p0, p1, p2, v1, v2;
    double cotA, cotB;

    QMeshNode* Node1 = connectEdge->GetEndPoint(); if (Node == Node1) Node1 = connectEdge->GetStartPoint();

    if (connectEdge->GetLeftFace() != nullptr) {
        QMeshFace* LeftFace = connectEdge->GetLeftFace();
        QMeshNode* LFNode = LeftFace->GetNodeRecordPtr(0);

        if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(1);
        if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(2);
        if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == LFNode->GetIndexNo() || Node1->GetIndexNo() == LFNode->GetIndexNo())
            cout << "Left face is in a mess!" << endl;
        Node->GetCoord3D(p0(0), p0(1), p0(2));
        Node1->GetCoord3D(p1(0), p1(1), p1(2));
        LFNode->GetCoord3D(p2(0), p2(1), p2(2));
        v1 = p2 - p1;
        v2 = p2 - p0;
        cotA = v1.dot(v2) / v1.cross(v2).norm();
    }
    else cotA = 0;

    if (connectEdge->GetRightFace() != nullptr) {
        QMeshFace* RightFace = connectEdge->GetRightFace();
        QMeshNode* RFNode = RightFace->GetNodeRecordPtr(0);

        if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(1);
        if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(2);
        if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == RFNode->GetIndexNo() || Node1->GetIndexNo() == RFNode->GetIndexNo())
            cout << "Right face is in a mess!" << endl;
        Node->GetCoord3D(p0(0), p0(1), p0(2));
        Node1->GetCoord3D(p1(0), p1(1), p1(2));
        RFNode->GetCoord3D(p2(0), p2(1), p2(2));
        v1 = p2 - p1;
        v2 = p2 - p0;
        cotB = v1.dot(v2) / v1.cross(v2).norm();
    }
    else cotB = 0;
    //cout << "cotA = " << cotA << ", cotB = " << cotB << endl;
    return(0.5*(cotA + cotB));
}

void heatMethodField::computeVectorField
(Eigen::MatrixXd &gradients, const Eigen::VectorXd &u) const {
    gradients.setZero();
    for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace *Face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
        if (Face->isBoundaryFace() == false) {
            Eigen::Vector3d gradient, normal;
            gradient.setZero();
            Face->CalPlaneEquation();
            Face->GetNormal(normal(0), normal(1), normal(2));
            normal.normalize();

            for (int i = 0; i < 3; i++) {
                QMeshNode* Node = Face->GetNodeRecordPtr(i % 3);
                QMeshNode* Node1 = Face->GetNodeRecordPtr((i + 1) % 3);
                QMeshNode* Node2 = Face->GetNodeRecordPtr((i + 2) % 3);

                double ui = u(Node->GetIndexNo());
                double pp1[3], pp2[3];
                Node1->GetCoord3D(pp1[0], pp1[1], pp1[2]);
                Node2->GetCoord3D(pp2[0], pp2[1], pp2[2]);

                Eigen::Vector3d ei = Eigen::Vector3d(pp2[0] - pp1[0], pp2[1] - pp1[1], pp2[2] - pp1[2]);
                gradient += ui * normal.cross(ei);
            }
            gradient /= (2.0 * Face->CalArea());
            gradient.normalize();
            //cout << gradient << endl;
            for (int i = 0; i < 3; i++) {
                gradients(Face->GetIndexNo(), i) = -gradient(i);
            }
        }
    }
}

void heatMethodField::computeIntegratedDivergence
(Eigen::VectorXd &integratedDivs,const Eigen::MatrixXd &gradients) const {

    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        double integratedDiv = 0.0;
        Eigen::Vector3d p; Node->GetCoord3D(p(0), p(1), p(2));

        for (GLKPOSITION Pos1 = Node->GetFaceList().GetHeadPosition(); Pos1;) {
            QMeshFace *connectFace = (QMeshFace*)Node->GetFaceList().GetNext(Pos1);

            Eigen::Vector3d gradient = gradients.row(connectFace->GetIndexNo());
            int index = connectFace->getNodePtrNumber(Node);
            if (index == -1) cout << "faild to detect the index for this face!";

            QMeshNode* Node1 = connectFace->GetNodeRecordPtr((index + 1) % 3);
            QMeshNode* Node2 = connectFace->GetNodeRecordPtr((index + 2) % 3);

            Vector3d  p1, p2;
            Node1->GetCoord3D(p1(0), p1(1), p1(2));
            Node2->GetCoord3D(p2(0), p2(1), p2(2));
            Eigen::Vector3d e1 = p1 - p;
            Eigen::Vector3d e2 = p2 - p;
            Eigen::Vector3d ei = p2 - p1;

            double theta1 = acos((-e2).dot(-ei) / (e2.norm() * ei.norm()));
            double cot1 = 1.0 / tan(theta1);

            double theta2 = acos((-e1).dot(ei) / (e1.norm() * ei.norm()));
            double cot2 = 1.0 / tan(theta2);

            integratedDiv += e1.dot(gradient) * cot1 + e2.dot(gradient) * cot2;
        }
        integratedDivs[Node->GetIndexNo()] = 0.5 * integratedDiv;
    }
}

double heatMethodField::subtractMinimumDistance
(Eigen::VectorXd &phi) const {

    // compute max and min phis
    double minPhi = INFINITY;
    double maxPhi = -INFINITY;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode *Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        int	thisIndex = Node->GetIndexNo();

        if (minPhi > phi(thisIndex)) minPhi = phi(thisIndex);
        if (maxPhi < phi(thisIndex)) maxPhi = phi(thisIndex);
    }

    double range = maxPhi - minPhi;

    for (int i = 0; i<phi.size(); ++i)
        phi(i) = 1 - (phi(i) - minPhi) / range;

    return range;
}

double heatMethodField::meanEdgeLength() const
{
    double avgLength = 0.0;
    for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge *Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
        avgLength += Edge->CalLength();
    }
    avgLength /= (double)surfaceMesh->GetEdgeNumber();
    return avgLength;
}

void heatMethodField::meshRefinement() {

    int nodeNum = surfaceMesh->GetNodeNumber() + surfaceMesh->GetEdgeNumber();
    int faceNum = surfaceMesh->GetFaceNumber() * 4;

    Eigen::VectorXd scalarField = Eigen::VectorXd::Zero(nodeNum);

    // build nodeNum and nodeTable - refinement
    float* nodeTable;
    nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    int index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        double pp[3]; Node->GetCoord3D(pp); Node->SetIndexNo(index); scalarField(index) = Node->zigzagValue;
        for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
        index++;
    }
    
    index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
        double p1[3], p2[3];
        Edge->GetStartPoint()->GetCoord3D(p1); Edge->GetEndPoint()->GetCoord3D(p2);
        scalarField(index + surfaceMesh->GetNodeNumber()) = (Edge->GetStartPoint()->zigzagValue + Edge->GetEndPoint()->zigzagValue) / 2;
        for (int i = 0; i < 3; i++) nodeTable[(index + surfaceMesh->GetNodeNumber()) * 3 + i] = (float)((p1[i] + p2[i]) / 2);
        Edge->refineNodeIndex = index; index++;
    }

    // build faceNum and faceTable - refinement

    unsigned int* faceTable;
    faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);
    index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
        int nodeIndex[6];
        for (int i = 0; i < 3; i++) {
            nodeIndex[i] = face->GetNodeRecordPtr(i)->GetIndexNo();
            nodeIndex[3 + i] = face->GetEdgeRecordPtr(i+1)->refineNodeIndex + surfaceMesh->GetNodeNumber();
        }
          
        int faceNodeIndex[12] = { 1,4,6,4,2,5,4,5,6,6,5,3 };
        for (int i = 0; i < 12; i++) 
            faceTable[index * 12 + i] = nodeIndex[faceNodeIndex[i] - 1];
        index++;
        
    }

    // reconstruct the mesh
    surfaceMesh->ClearAll();
    surfaceMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

    //protect the stress field value
    index = 0;
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        Node->zigzagValue = scalarField(index); index++;
    }
    std::cout << surfaceMesh->GetNodeNumber() << std::endl;
  
    free(nodeTable);
    free(faceTable);

    std::cout << "finish refine the mesh once!" << std::endl;
}