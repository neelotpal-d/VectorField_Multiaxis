#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include<Eigen/Eigen>
#include <Eigen/PardisoSupport>
//#include<Eigen/PardisoSupport>

class PolygenMesh;
class RoughMachining {

public:
	RoughMachining() {};
	~RoughMachining() {};

	void initial(PolygenMesh* polygenMesh_MaterialSpace, PolygenMesh* polygenMesh_ConvexHull, PolygenMesh* polygenMesh_ModelSurf, PolygenMesh* polygenMesh_FixtureSurf, bool isIn);
	void convexHull_Boundary_Detection();
	void vectorField_Generation_ConvexMachining();
	void scalarField_Generation_ConvexMachining();
	void curvedLayer_Generation_ConvexMachining();
	void createHeatField(PolygenMesh* MaterialMesh, int BoundaryCondition);  //Added by Neelotpal
	void initialIndex();			//Added by Neelotpal

	/*For Distance Field Method <Added by Neelotpal on Feb 9, 2022>*/
	void createDistanceField();
	
	/*FOr Fixture FIeld, Added by Neelotpal on March 6, 2022*/
	void createFixtureField();


	/*New Pipeline*/
	void createGradientVectorField();
	void optimiseVectorField(double toolLength, double margin);
	void createScalarField();
	void _displayVectorField(QMeshPatch* vectorFieldPatch);

	//test
	void _cal_heatValue(PolygenMesh* MaterialMesh);
	void _cal_heatValue1(PolygenMesh* MaterialMesh);

private:
	void _index_initial(QMeshPatch* patch, bool is_TetMesh);
	void _build_tetraSet_4SpeedUp(QMeshPatch* patch);
	bool _eleCenter_in_convexHull(double xx, double yy, double zz);
	bool _eleCenter_in_convexHull2(double xx, double yy, double zz);
	bool IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2);
	Eigen::Vector3d _get_normal_of_NearestFace(Eigen::Vector3d face_center);
	void vectorFiled_Smoothing(int smoothLoop);
	void compTetMeshVolumeMatrix();

	/*Added by Neelotpal*/
	void _createWeightMatrix();
	void _createLaplacianMatrix();
	double _getEdgeWeight(QMeshEdge* edge, QMeshTetra* tet);
	void _createVolumeMatrix();
	void _solveHeatField(int BoundaryCondition);
	double _getAverageEdgeLength();
	void _initialiseSystemValues1(Eigen::VectorXd& u, Eigen::MatrixXd& S);
	void _initialiseSystemValues2(Eigen::VectorXd& u, double SourceVal, double otherVal);
	void _inverseVolumeMatrix();
	void _generateVectorField();
	void _generateScalarField();
	void _generateDivergence(Eigen::VectorXd& divVector);
	Eigen::Vector3d _getOppositeNormalWithMag(QMeshNode* node, QMeshTetra* tet);
	int _defineBouindaryNodes();
	void _createDensityMatrix();
	void computeDistanceRoughMachining();
	void initialisePrimitiveDiscard();
	void initialiseSourceOnConvexHull();
	void findBoundary();
	void preHeatMethod2();
	void preHeatMethod3(); //for inside CH
	void PrepareForHeatMethod();
	void _buildTetLaplacianMatrix();
	void _buildTetLaplacianMatrixPreserved();
	void _identifyPivotTets(double toolLength, double margin);
	void _replaceVectors();
	void _createSolutionVectorForVectorSmoothing(int column, Eigen::VectorXd& u);
	void _createSolutionVectorForVectorSmoothingPreserved(int column, Eigen::VectorXd& u);
	void _smoothVectorField();



	/*Added for New V.Field Operation*/
private:
	int pivotNumber = 0;
private:
	void _classifyBoundary();
	void _initiateVectorOnBound(bool setInnerBound, bool setouterBound, bool convexHullGuess = false);
	void _initiateVectorOnInteriorBoundTets();
	void _initiateVectorOnExteriorBoundTets();
	void _initiateVectorOnConvexHull();
	void _initiateGuideVectors();
	void _propagateVectorField();
	void _propagateVectorFieldPreserved();
	void _computeScalarField(double *minS =0, double * maxS = 0);
	void _displayDivergence();
	void _correctDivergence();
	void _detectSingualrity();
	void _correctSingularity(bool buildNewTet = true);
	void _checkGradient1();
	void _checkGradient2();
	void _convert2GradientField();
	void _computeDifference();
	void _translateVectorNode2Tets(QMeshNode* node);
	void _detectPlanarSingularity();
	void _correctPlanarSingularity(double* dir, bool flip = false);
	void _correctAlongInnerBoundary();
	void _identifyConstraints();
	void _printBoundaryNormalComps();
	void _enforceGradientBC(Eigen::VectorXd& divVector, Eigen::SparseMatrix<double>& sysMatrix);
	void _liftVectors();
	void _initiateCustomField();
	void _smoothPath(QMeshPatch* toollPath);
	void _spreadSelectionToFaceSegment(QMeshNode* handleNode);
	void _spreadToSimilarFace(QMeshFace* MeshFace);
	double _areSimilarFace(QMeshFace* MeshFace, QMeshFace* adjacentFace);
	void _makeSelectedFaceConstraint();
	
	
	



public:
	void spreadSelectionToFaceSegment(QMeshNode* handleNode) {
		_spreadSelectionToFaceSegment(handleNode);
	};
	void spreadSelectionToFaceSegment(QMeshFace* selectedFace) {
		
		selectedFace->isSelected = true;
		_spreadToSimilarFace(selectedFace);
		_makeSelectedFaceConstraint();
		stackCounter = 0;
	};

	void _pathIntersection(QMeshPatch* toolPath, QMeshPatch* refSurface);
	void classifyBoundary() {
		_classifyBoundary();
	};
	void initiateVectorOnBound(bool setInnerBound, bool setouterBound, bool setCHGuess = false) { 
		_initiateVectorOnBound(setInnerBound, setouterBound, setCHGuess);
	};
	void propagateVectorField() {
		_propagateVectorField();
		//_propagateVectorFieldWithCurl();
		_detectPlanarSingularity();
	

	};

	void computeScalarField(bool classifySingularPlane= true) {
		double minSing, maxSing;
		//_identifyConstraints();
		if (classifySingularPlane) {
			_computeScalarField(&minSing, &maxSing);
		}
		else {
			_computeScalarField();
		}
		if (classifySingularPlane) {
			min_SingularPlane = minSing;
			max_SingularPlane = maxSing;
		}
	};

	void initiateGuideVectors() {
		//_initiateGuideVectors(); 
		  buildNewTetLaplacian = true;
		_liftVectors();
		_propagateVectorFieldPreserved();
		//_initiateCustomField();
	};

	void detectSingularity() {
		_detectSingualrity();
	};

	void correctSingularity() {
		_correctSingularity(); _propagateVectorField(); _computeScalarField();
	};
	
	void checkDifference() {
		_computeDifference();
	};

	void convert2GradientField() {
		_convert2GradientField();
		_propagateVectorFieldPreserved();
		_computeScalarField();
	};

	void translateVectorNode2Tets(QMeshNode* node) {
		_translateVectorNode2Tets(node);
	};

	void correctPlanarSingularity(double* dirn, bool flip = false) {
		double dir[3];
		_correctPlanarSingularity(dir, flip);
		dirn[0] = dir[0];
		dirn[1] = dir[1];
		dirn[2] = dir[2];
	}

	void correctAlongInnerBoundary() {
		_correctAlongInnerBoundary();
	}

	void createVectorPatch(QMeshPatch* vectorPatch, bool showAll = true, bool colorPivots = true);
	/****************************/

private:
	QMeshPatch* materialSpace = NULL;
	QMeshPatch* convexHull = NULL;
	QMeshPatch* modelSurface = NULL;
	QMeshPatch* fixtureSurface = NULL;

	std::vector<QMeshTetra*> tetraSet_materialSpace;
	PolygenMesh* curvedLayer_Polygen;


	/*Added by Neelotpal*/
	int constNodeNumber = 0;
	std::vector<bool> constIndex;
	Eigen::SparseMatrix<double> WeightMatrix;
	Eigen::SparseMatrix<double> LaplacianMatrix;
	Eigen::SparseMatrix<double> VolumeMatrix;
	Eigen::SparseMatrix<double> DensityMatrix;
	Eigen::MatrixXd VectorFieldMatrix;
	bool isIn = false; //to check if the layer is to be generated inside the convex hull

	

	//to record change in TetLaplacianMatrix
	bool buildNewTetLaplacian = true;


	Eigen::SparseMatrix<double> TetLaplacianMatrix;
	Eigen::SparseMatrix<double> TetOptMatrix;
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> SolverTetLaplcaian;// (PardisoLU/SparseLU)


	int Num_tetInConvexHull = 0;
	int Num_nodesInConvexHull = 0;
	int Num_nodesOnConvexHullBoundary = 0;
public:
	double max_distance = 0.00; //to note the max distance recorded by heat field
	double min_distance = 0.00; //to note the min distance recorded by heat field
	double min_SingularPlane = 0.00;
	double max_SingularPlane = 0.00;
	bool layerInSingularZone = false;
	double extractionIndex = 0.0;
	double propagateMag = 0.0;
	int stackCounter = 0;
	GLKObject* stackLastEntity;
};
