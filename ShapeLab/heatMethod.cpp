#include"QMeshNode.h"
#include"QMeshFace.h"
#include"QMeshTetra.h"
#include"QMeshEdge.h"
#include"QMeshPatch.h"

#include<iostream>


#include"heatMethod.h"


heatMethod::heatMethod(QMeshPatch* MatSp) {
	materialSpace = MatSp;
	_initialiseMesh();

}

void heatMethod::_initialiseMesh() {


	QMeshPatch* patch = materialSpace;
	bool is_TetMesh = true;

	//----initial the edge, node and face index, start from 0
	int index = 0;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		Edge->FieldIndexNumber = index;
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		Face->FieldIndexNumber = index;
		Face->CalPlaneEquation(); // pre-compute the normal of face
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->FieldIndexNumber = index;
		index++;
	}
	// only tet mesh needs reorder the index of Tetra
	if (is_TetMesh) {
		index = 0;
		for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
			Tetra->FieldIndexNumber = index;
			index++;
		}
	}
}


void heatMethod::generateHeatField(int BoundCon) {
	_createWeightMatrix();
	_solveHeatField(BoundCon);
}

void heatMethod::_createWeightMatrix() {


	std::cout << "Creating Weight Matrix\n";
	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;
	std::cout << num_nodes << std::endl;


	WeightMatrix.resize(num_nodes, num_nodes);
	VolumeMatrix.resize(num_nodes, num_nodes);
	VolumeMatrix.reserve(Eigen::VectorXi::Constant(num_nodes, 2));
	WeightMatrix.reserve(Eigen::VectorXi::Constant(num_nodes, 35));

	Eigen::MatrixXd VolumeMatrix_temp = Eigen::MatrixXd(num_nodes, num_nodes);

	int ccount = 0;

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {

		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);


		for (int n = 0; n < 6; n++) {

			QMeshEdge* edge = tet->GetEdgeRecordPtr(n + 1);
			int i = edge->GetStartPoint()->FieldIndexNumber;
			int j = edge->GetEndPoint()->FieldIndexNumber;

			if (i == -1 || j == -1) {
				std::cerr << "Node inside CH encountered while computing weight!!\n";
			}

			QMeshNode* node_i = edge->GetStartPoint();
			QMeshNode* node_j = edge->GetEndPoint();


			double _weight = _getEdgeWeight(edge, tet);
	
			if (!edge->visited) {
				WeightMatrix.insert(i, j) = _weight;
				WeightMatrix.insert(j, i) = WeightMatrix.coeff(i, j);
				edge->visited = true;
			}
			else {
				WeightMatrix.coeffRef(i, j) += _weight;
				WeightMatrix.coeffRef(j, i) = WeightMatrix.coeff(i, j);
			}

			if (!node_i->visited_forWeight) {
				WeightMatrix.insert(i, i) = -_weight;
				node_i->visited_forWeight = true;
			}
			else {
				WeightMatrix.coeffRef(i, i) -= _weight;
			}

			if (!node_j->visited_forWeight) {
				WeightMatrix.insert(j, j) = -_weight;
				node_j->visited_forWeight = true;
			}
			else {
				WeightMatrix.coeffRef(j, j) -= _weight;
			}

		}

		tet->CalVolume();
		double _vol = tet->GetVolume();

		for (int n = 0; n < 4; n++) {
			QMeshNode* node = tet->GetNodeRecordPtr(n + 1);

			int k = node->FieldIndexNumber;
			if (!node->visited_forVol) {
				VolumeMatrix.insert(k, k) = (_vol / 4.0);
				node->visited_forVol = true;
			}
			else {
				VolumeMatrix.coeffRef(k, k) += (_vol / 4.0);
			}
		}


	}

}


double heatMethod::_getEdgeWeight(QMeshEdge* edge, QMeshTetra* tet) {
	QMeshNode* node1 = edge->GetStartPoint();
	QMeshNode* node2 = edge->GetEndPoint();

	int index1_in_tet = tet->GetNodeIndex(node1);
	int index2_in_tet = tet->GetNodeIndex(node2);

	QMeshFace* t_face1, * t_face2; //see the defination of discrete Laplacian for tet mesh to understand these faces and the edge
	QMeshEdge* t_edge;


	QMeshNode* otherNode1, * otherNode2;
	bool foundOne = false;

	for (int k = 1; k <= 4; k++) {
		if (k != index1_in_tet && k != index2_in_tet) {
			if (!foundOne) {
				otherNode1 = tet->GetNodeRecordPtr(k);
				foundOne = true;
			}
			else {
				otherNode2 = tet->GetNodeRecordPtr(k);
			}
		}
	}

	if (otherNode1 == NULL || otherNode2 == NULL) {
		std::cout << "GOT NULL OTHERNODE!!!!!!\n\n";
	}


	double x0, y0, z0, x1, y1, z1, x2, y2, z2;

	otherNode1->GetCoord3D(x0, y0, z0);
	otherNode2->GetCoord3D(x1, y1, z1);
	node1->GetCoord3D(x2, y2, z2);

	Eigen::Vector3d vfixed(x1 - x0, y1 - y0, z1 - z0);
	Eigen::Vector3d v0(x2 - x0, y2 - y0, z2 - z0);


	Eigen::Vector3d v1 = vfixed.cross(v0);

	node2->GetCoord3D(x2, y2, z2);
	Eigen::Vector3d v00(x2 - x0, y2 - y0, z2 - z0);


	Eigen::Vector3d v2 = vfixed.cross(v00);

	double length = abs(vfixed.norm());




	long double _dot = v1.dot(v2) / (v1.norm() * v2.norm());
	if (_dot > 1) _dot = 1;
	else if (_dot < -1) _dot = -1;

	double angle = acos(_dot);


	double eps = 0.00001;
	if (angle < eps) angle += eps;

	double _cotan = cos(angle) / sin(angle);


	return (double)length * _cotan / 6.0;


}



void heatMethod::_solveHeatField(int BoundaryCondition) {


	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;


	this->constNodeNumber = _defineBouindaryNodes();




	Eigen::VectorXd u(num_nodes);
	Eigen::SparseMatrix<double> S;

	Eigen::SparseMatrix<double> I = Eigen::SparseMatrix<double>(num_nodes, num_nodes);
	I.setIdentity();

	std::cout << "Initialising u...\n";
	_initialiseSystemValues2(u, 1.0, 0.0);
	Eigen::VectorXd uD(num_nodes);
	Eigen::VectorXd uN(num_nodes);
	uD.setZero();
	uN.setZero();



	double t = _getAverageEdgeLength();
	t = t * t;
	std::cout << "t: " << t << std::endl;

	std::cout << "Inversing Volume Matrix...\n";
	_inverseVolumeMatrix();

	Eigen::SparseMatrix<double> invVol = VolumeMatrix;
	Eigen::SparseMatrix<double> test(num_nodes, num_nodes);
	test.setIdentity();

	double neumann = ((double)BoundaryCondition / 100.0);
	double dirichlet = 1.0 - neumann;

	if (BoundaryCondition > 0) {
		std::cout << "Forcing Boundary Conditions..\n";
		for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
			QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
	


			if (node->isSource) {
				invVol.coeffRef(node->FieldIndexNumber, node->FieldIndexNumber) = 0.0;
			}
			else {
				test.coeffRef(node->FieldIndexNumber, node->FieldIndexNumber) = 0.0;
			}

		}

		std::cout << "Solving Now..\n";
		S = I - 100 * t * invVol * WeightMatrix;

		std::cout << "Initialising Sparse Solver..\n";
		Eigen::PardisoLU <Eigen::SparseMatrix<double>> SolverN;// (PardisoLU/SparseLU)
		SolverN.analyzePattern(S);
		SolverN.factorize(S);
		if (SolverN.info() != Eigen::Success)
			std::cout << "error here: factorize fail!" << std::endl;
		SolverN.compute(S);
		uN = SolverN.solve(u);

		std::cout << "Neuman Soln. Complete..\n";
		invVol = VolumeMatrix;
	}




	//Dirichlet Solution
	if (BoundaryCondition < 100) {
		std::cout << "Forcing Boundary Conditions..\n";
		for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
			QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
			if (node->in_ConvexHull) continue;


			if (node->isBoundary) {
				invVol.coeffRef(node->FieldIndexNumber, node->FieldIndexNumber) = 0.0;
			}
			else {
				test.coeffRef(node->FieldIndexNumber, node->FieldIndexNumber) = 0.0;
			}

		}

		std::cout << "Solving Now..\n";
		S = I - 100 * t * invVol * WeightMatrix;

		std::cout << "Initialising Sparse Solver..\n";
		Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;// (PardisoLU/SparseLU)
		Solver.analyzePattern(S);
		Solver.factorize(S);
		if (Solver.info() != Eigen::Success)
			std::cout << "error here: factorize fail!" << std::endl;
		Solver.compute(S);
		std::cout << "Sparse Solver Initialised\n";
		uD = Solver.solve(u);
		std::cout << "Dirichlet Soln. Complete..\n";
	}




	std::cout << "System solved for initial field...\n";

	double min = 99999, max = -9999;


	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

		if (node->in_ConvexHull) continue;

		int i = node->FieldIndexNumber;
		u[i] = (neumann * uN[i]) + (dirichlet * uD[i]);
		node->HeatFieldValue = u[i];
		node->scalarField = u[i];
		if (node->HeatFieldValue < min) min = node->HeatFieldValue;
		if (node->HeatFieldValue > max) max = node->HeatFieldValue;


	}

	std::cout << "Max: " << max << "Min: " << min << std::endl;
	std::cout << "\n\n\n";

	double range = max - min;

}


double heatMethod::_getAverageEdgeLength() {
	std::cout << "Getting average edge length...\n";
	double length = 0;
	double n = materialSpace->GetEdgeNumber();
	double count_ = 0;
	for (GLKPOSITION pos = materialSpace->GetEdgeList().GetHeadPosition(); pos;) {
		QMeshEdge* edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(pos);

		length += edge->GetLength();
		count_++;
	}

	return length / count_;
}




void heatMethod::_initialiseSystemValues2(Eigen::VectorXd& u, double sourceVal, double otherVal) {

	double boundingBox[6];
	materialSpace->ComputeBoundingBox(boundingBox);
	double minY = boundingBox[2];
	double maxY = boundingBox[3];


	int num = materialSpace->GetNodeNumber();
	int cnum = this->constNodeNumber;

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);

		if (node->in_ConvexHull) continue;

		if (node->isBoundary) {
			if (node->isSource) {
				u[node->FieldIndexNumber] = sourceVal;
			}
			else u[node->FieldIndexNumber] = otherVal;

		}
		else {
			u[node->FieldIndexNumber] = otherVal;
		}

	}

}

int heatMethod::_defineBouindaryNodes() {

	double boundingBox[6];
	materialSpace->ComputeBoundingBox(boundingBox);

	double minX = boundingBox[0];
	double maxX = boundingBox[1];

	double minY = boundingBox[2];
	double maxY = boundingBox[3];

	double minZ = boundingBox[4];
	double maxZ = boundingBox[5];

	double Xav = (minX + maxX) * 0.50;
	double Yav = (minY + maxY) * 0.50;
	double Zav = (minZ + maxZ) * 0.50;

	double Xrange = (-minX + maxX) / 4;
	double Yrange = (-minY + maxY) / 4;
	double Zrange = (-minZ + maxZ) / 4;


	int num = materialSpace->GetNodeNumber();
	int count = 0;

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);

		if (node->in_ConvexHull) continue;

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		if (yy < (minY + 1)  /*node->isBoundary*/)
		{
			node->isBoundary = true;
			std::cout << "I am here first\n";
			constIndex.push_back(true);
			node->isSource = true;
		}
		//else if (/*yy < (minY + 0.0001)*//*(abs(xx - Xav) < Xrange) && (abs(yy - Yav) < Yrange) && (abs(zz - Zav) < Zrange)*/ node->ConvexHull_boundary) {
		//	node->isBoundary = true;
		//	std::cout << "\nI am here\n\n";
		//	constIndex.push_back(true);
		//	//node->isTopBoundary = true;
		//	//node->isSource = true;
		//	
		//}
		else {
			node->ReducedFieldIndexNumber = count;
			node->isBoundary = false;
			constIndex.push_back(false);
			count++;
		}



	}

	return num - count;
}

void heatMethod::_buildTetLaplacianMatrix()
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;


	Eigen::SparseMatrix<double> diagonalMultiplier;


	TetLaplacianMatrix.resize(num_tets + num_tets, num_tets);
	TetLaplacianMatrix.setZero();
	TetLaplacianMatrix.reserve(Eigen::VectorXd::Constant(num_tets + num_tets, 10));

	diagonalMultiplier.resize(num_tets + num_tets, num_tets + num_tets);
	//diagonalMultiplier.reserve(Eigen::VectorXd::Constant(num_tets + pivotNumber, 2));
	diagonalMultiplier.setIdentity();
	//TetLaplacianMatrix.setIdentity();


	std::cout << "I am here 1\n";
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* face = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);

		QMeshTetra* tet1 = face->GetRightTetra();
		QMeshTetra* tet2 = face->GetLeftTetra();

		if (tet1 == NULL || tet2 == NULL) continue;
		//if (tet1->discard || tet2->discard) continue;



		int i = tet1->FieldIndexNumber;
		int j = tet2->FieldIndexNumber;

		TetLaplacianMatrix.insert(i, j) = -1;
		TetLaplacianMatrix.insert(j, i) = -1;

		if (TetLaplacianMatrix.coeff(i, i) == 0) {
			TetLaplacianMatrix.insert(i, i) = 1;
		}
		else {
			TetLaplacianMatrix.coeffRef(i, i)++;
		}

		if (TetLaplacianMatrix.coeff(j, j) == 0) {
			TetLaplacianMatrix.insert(j, j) = 1;
		}
		else {
			TetLaplacianMatrix.coeffRef(j, j)++;
		}

	}

	std::cout << "I am here 2\n";

	for (int l = 0; l < num_tets; l++) {
		diagonalMultiplier.coeffRef(l, l) = 1 / TetLaplacianMatrix.coeff(l, l);
	}



	std::cout << "I am here 3\n";
	int pivotInsertCount = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		//if (!tetra->isPivot) continue;
		int i = tetra->FieldIndexNumber;

		int indexx = pivotInsertCount + num_tets;
		//TetLaplacianMatrix.insert(indexx, i) = 0.001;
		if (tetra->isPivot) {
			if (tetra->isPivotA)
				TetLaplacianMatrix.coeffRef(indexx, i) = 9e8;
			if (tetra->isPivotB)
				TetLaplacianMatrix.coeffRef(indexx, i) = 500e4;
			else
				TetLaplacianMatrix.coeffRef(indexx, i) = 500e4;
		}
		pivotInsertCount++;
	}

	TetLaplacianMatrix = diagonalMultiplier * TetLaplacianMatrix;

}

void heatMethod::_createSolutionVectorForVectorSmoothing(int column, Eigen::VectorXd& u)
{

	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;

	std::cout << "Creating vector " << column << std::endl;
	int pivotInsertCount = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		int index = tetra->FieldIndexNumber;
		if (column < 3) {
			Eigen::Vector3d thisVector(VectorFieldMatrix(index, 0), VectorFieldMatrix(index, 1), VectorFieldMatrix(index, 2));
			//double vectorSize = thisVector.stableNorm();
			double vectorSize = 1.0;
			thisVector.stableNormalize();
			//u(num_tets + index) = 0.001 * thisVector(column);
			if (true) {
				if (tetra->isPivotA)
					u(num_tets + pivotInsertCount) = 9e8 * thisVector(column);
				if (tetra->isPivotB)
					u(num_tets + pivotInsertCount) = 500e4 * thisVector(column);
				else
					u(num_tets + pivotInsertCount) = 500e4 * thisVector(column);
				pivotInsertCount++;
			}
		}
		else {
			Eigen::Vector3d thisVector(VectorFieldMatrix(index, 0), VectorFieldMatrix(index, 1), VectorFieldMatrix(index, 2));
			double vectorSize = thisVector.stableNorm();
			//u(num_tets + index) = 0.001 * vectorSize;
			if (tetra->isPivot) {
				if (tetra->isPivotA)
					u(num_tets + pivotInsertCount) = 800 * vectorSize;
				else
					u(num_tets + pivotInsertCount) = 200 * vectorSize;
				pivotInsertCount++;
			}
		}

	}
}

void heatMethod::_smoothVectorField()
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;


	_buildTetLaplacianMatrix();

	Eigen::VectorXd v0 = Eigen::VectorXd::Zero(2 * num_tets);
	Eigen::VectorXd v1 = Eigen::VectorXd::Zero(2 * num_tets);
	Eigen::VectorXd v2 = Eigen::VectorXd::Zero(2 * num_tets);
	Eigen::VectorXd v3 = Eigen::VectorXd::Zero(2 * num_tets);

	Eigen::VectorXd v0_sol(num_tets);
	Eigen::VectorXd v1_sol(num_tets);
	Eigen::VectorXd v2_sol(num_tets);
	Eigen::VectorXd v3_sol(num_tets);

	_createSolutionVectorForVectorSmoothing(0, v0);
	_createSolutionVectorForVectorSmoothing(1, v1);
	_createSolutionVectorForVectorSmoothing(2, v2);
	_createSolutionVectorForVectorSmoothing(3, v3);

	std::cout << "Initialising Sparse Solver for vector smoothing..\n";
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> SolverN;// (PardisoLU/SparseLU)
	Eigen::SparseMatrix<double> S = TetLaplacianMatrix.transpose() * TetLaplacianMatrix;
	SolverN.analyzePattern(S);
	SolverN.factorize(S);
	if (SolverN.info() != Eigen::Success)
		std::cout << "error here: factorize fail!" << std::endl;
	SolverN.compute(S);
	std::cout << "Smoothing 1\n";
	v0 = TetLaplacianMatrix.transpose() * v0;
	v0_sol = SolverN.solve(v0);
	std::cout << "Smoothing 2\n";
	v1 = TetLaplacianMatrix.transpose() * v1;
	v1_sol = SolverN.solve(v1);
	std::cout << "Smoothing 3\n";
	v2 = TetLaplacianMatrix.transpose() * v2;
	v2_sol = SolverN.solve(v2);
	//std::cout << "Smoothing 4\n";
	//v3 = TetLaplacianMatrix.transpose() * v3;
	//v3_sol = SolverN.solve(v3);

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		int index = tetra->FieldIndexNumber;
		Eigen::Vector3d newNormal(v0_sol(index), v1_sol(index), v2_sol(index));
		newNormal.stableNormalize();
		//newNormal = newNormal * v3_sol(index);

		VectorFieldMatrix(index, 0) = newNormal.x();
		VectorFieldMatrix(index, 1) = newNormal.y();
		VectorFieldMatrix(index, 2) = newNormal.z();
	}

}


void heatMethod::_inverseVolumeMatrix() {
	int num = materialSpace->GetNodeNumber() - Num_nodesInConvexHull;

	for (int i = 0; i < num; i++) {
		if (VolumeMatrix.coeff(i, i) > 0.0001) {
			VolumeMatrix.coeffRef(i, i) = 1.0 / VolumeMatrix.coeff(i, i);
		}
		else {
			std::cout << "\nVol: " << VolumeMatrix.coeff(i, i) << std::endl;
		}

	}

}



void heatMethod::_generateVectorField() {

	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;

	VectorFieldMatrix = Eigen::MatrixXd(num_tets, 3);

	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(3, 4);
	Eigen::MatrixXd VertCoord = Eigen::MatrixXd::Zero(3, 3);
	Eigen::Vector4d VertField = Eigen::Vector4d::Zero(4);

	T(0, 0) = 1;
	T(1, 1) = 1;
	T(2, 2) = 1;

	T(0, 3) = T(1, 3) = T(2, 3) = -1;

	std::cout << "T: " << T << std::endl;

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);


		for (int i = 0; i < 4; i++) {

			QMeshNode* node = tet->GetNodeRecordPtr(i + 1);

			double x, y, z;
			node->GetCoord3D(x, y, z);
			if (i != 3) {

				VertCoord(i, 0) = x;
				VertCoord(i, 1) = y;
				VertCoord(i, 2) = z;
			}
			else {
				for (int j = 0; j < 3; j++) {
					VertCoord(j, 0) -= x;
					VertCoord(j, 1) -= y;
					VertCoord(j, 2) -= z;
				}
			}

			VertField(i) = node->HeatFieldValue;
		}

		Eigen::Vector3d temp = VertCoord.inverse() * T * VertField;
		temp.stableNormalize();



		VectorFieldMatrix(tet->FieldIndexNumber, 0) = temp[0];
		VectorFieldMatrix(tet->FieldIndexNumber, 1) = temp[1];
		VectorFieldMatrix(tet->FieldIndexNumber, 2) = temp[2];


	}

}





void heatMethod::_generateScalarField() {

	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;
	Eigen::VectorXd divVector = Eigen::VectorXd::Zero(num_nodes);
	_generateDivergence(divVector);

	Eigen::SparseMatrix<double> invVol = VolumeMatrix;
	Eigen::SparseMatrix<double> boundaryCompensator(num_nodes, num_nodes);
	boundaryCompensator.setIdentity();

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);


		int ii = node->FieldIndexNumber;

		if (node->isBoundary && node->isSource) {
			invVol.coeffRef(ii, ii) = 0.0;
			divVector(ii) = 0.0;
		}
		else {
			invVol.coeffRef(ii, ii) = 1.0;
			boundaryCompensator.coeffRef(ii, ii) = 0;
		}
	}
	/*Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(WeightMatrix);
	Eigen::VectorXd u = dec.solve(divVector);*/

	Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;// (PardisoLU/SparseLU)

	Eigen::SparseMatrix<double> S = invVol * WeightMatrix + boundaryCompensator;
	S = 100 * S;

	Solver.analyzePattern(S);
	Solver.factorize(S);
	if (Solver.info() != Eigen::Success)
		std::cout << "error here: factorize fail!" << std::endl;
	Solver.compute(S);
	divVector = 100 * divVector;
	Eigen::VectorXd u = Solver.solve(divVector);


	std::cout << "System Solved for Scalar Field...\n";

	double max = -999999, min = 99999;
	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);


		int i = node->FieldIndexNumber;
		node->HeatFieldValue = u[i];
		if (u[i] > max) max = u[i];
		if (u[i] < min) min = u[i];
	}

	double max2 = -999999, min2 = 99999;
	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);


		int i = node->FieldIndexNumber;
		node->HeatFieldValue = (u[i] - min) / (max - min);
		if (node->HeatFieldValue > max2) max2 = node->HeatFieldValue;
		if (node->HeatFieldValue < min2) min2 = node->HeatFieldValue;

		node->scalarField = node->HeatFieldValue;
	}


	std::cout << "Final MAX: " << max2 << "Final MIN: " << min2 << std::endl;



}


void heatMethod::_generateDivergence(Eigen::VectorXd& divVector) {

	std::cout << "Generating Divergence Field\n";

	for (GLKPOSITION pos = materialSpace->GetTetraList().GetHeadPosition(); pos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(pos);

	

		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = (QMeshNode*)tet->GetNodeRecordPtr(i);
			int node_index = node->FieldIndexNumber;
			int tet_index = tet->FieldIndexNumber;
			//int _facenumber = 999;

			double Sk;
			Eigen::Vector3d nk;

			nk = _getOppositeNormalWithMag(node, tet);
			Sk = 0.5 * abs(nk.stableNorm());
			nk.stableNormalize();


			/*switch(i) {
			case(1): _facenumber = 2;
			case(2): _facenumber = 3;
			case(3): _facenumber = 4;
			case(4): _facenumber = 1;
			}*/

			Eigen::Vector3d grad_vec(VectorFieldMatrix(tet_index, 0), VectorFieldMatrix(tet_index, 1), VectorFieldMatrix(tet_index, 2));
			grad_vec.stableNormalize();

			//QMeshFace* face = tet->GetFaceRecordPtr(_facenumber);

			/*face->CalPlaneEquation();
			face->CalArea();*/
			//	std::cout << "area: " << face->GetArea() << std::endl;




			divVector[node_index] -= Sk * nk.dot(grad_vec) / 3;

		}
	}


}

Eigen::Vector3d heatMethod::_getOppositeNormalWithMag(QMeshNode* node, QMeshTetra* tet) {
	QMeshNode* otNode1 = NULL, * otNode2 = NULL, * otNode3 = NULL;
	int node_index_in_Tet = tet->GetNodeIndex(node);

	for (int i = 1; i <= 4; i++) {
		if (i != node_index_in_Tet) {
			if (otNode1 == NULL) {
				otNode1 = tet->GetNodeRecordPtr(i);
			}
			else if (otNode2 == NULL) {
				otNode2 = tet->GetNodeRecordPtr(i);
			}
			else if (otNode3 == NULL) {
				otNode3 = tet->GetNodeRecordPtr(i);
			}
		}
	}

	Eigen::Vector3d v1, v2, v3;
	double x1, x2, x3, y1, y2, y3, z1, z2, z3;

	otNode1->GetCoord3D(x1, y1, z1);
	otNode2->GetCoord3D(x2, y2, z2);
	otNode3->GetCoord3D(x3, y3, z3);

	v1 = Eigen::Vector3d(x2 - x1, y2 - y1, z2 - z1);
	v2 = Eigen::Vector3d(x3 - x1, y3 - y1, z3 - z1);

	node->GetCoord3D(x2, y2, z2);

	v3 = Eigen::Vector3d(x2 - x1, y2 - y1, z2 - z1);

	Eigen::Vector3d nx = v1.cross(v2);
	if (nx.dot(v3) < -0.0000001) {
		nx = -1 * nx;
	}

	return nx;

}