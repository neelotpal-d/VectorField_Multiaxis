// QMeshNode.h: interface for the QMeshNode class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _QMESHNODE
#define _QMESHNODE

#include "../GLKLib/GLKObList.h"
#include <vector>

class QMeshPatch;
class QMeshFace;
class QMeshEdge;
class QMeshTetra;

class QMeshNode : public GLKObject  
{
public:
	QMeshNode();
	virtual ~QMeshNode();

public:
	int GetIndexNo();		//from 1 to n
	void SetIndexNo( const int _index = 1 );

	bool GetAttribFlag( const int whichBit );
	void SetAttribFlag( const int whichBit, const bool toBe = true );

	void GetCoord2D( double &x, double &y );
	void SetCoord2D( double x, double y );

	void GetCoord3D( double &x, double &y, double &z );
	void SetCoord3D( double x, double y, double z );

    void GetCoord3D_last( double &x, double &y, double &z );
	void SetCoord3D_last( double x, double y, double z );

	void SetMeanCurvatureNormalVector(double kHx, double kHy, double kHz);
	void GetMeanCurvatureNormalVector(double &kHx, double &kHy, double &kHz);
	
	void SetGaussianCurvature(double kG);
	double GetGaussianCurvature();
	
	void SetPMaxCurvature(double k1);
	double GetPMaxCurvature();

	void SetPMinCurvature(double k2);
	double GetPMinCurvature();

    void CalNormal();
    void CalNormal(double normal[]);
    void GetNormal(double &nx, double &ny, double &nz) {nx=m_normal[0]; ny=m_normal[1]; nz=m_normal[2];};
    void SetNormal(double nx, double ny, double nz) {m_normal[0]=nx; m_normal[1]=ny; m_normal[2]=nz;};

    void SetBoundaryDis(double dist);
    double GetBoundaryDis();

    void SetDensityFuncValue(double density) {m_densityFuncValue=density;};
    double GetDensityFuncValue() {return m_densityFuncValue;};

	void SetMeshPatchPtr(QMeshPatch* _mesh);
	QMeshPatch* GetMeshPatchPtr();

	void AddTetra(QMeshTetra *trglTetra);
	int GetTetraNumber();
	QMeshTetra* GetTetraRecordPtr(int No);	//from 1 to n
	GLKObList& GetTetraList();

	void AddFace(QMeshFace *_face);
	int GetFaceNumber();
	QMeshFace* GetFaceRecordPtr(int No);	//from 1 to n
    GLKObList& GetFaceList();

	void AddEdge(QMeshEdge *_edge);
	int GetEdgeNumber();
	QMeshEdge* GetEdgeRecordPtr(int No);	//from 1 to n
    GLKObList& GetEdgeList();

	void AddNode(QMeshNode *_node);
	int GetNodeNumber();
	QMeshNode* GetNodeRecordPtr(int No);	//from 1 to n
    GLKObList& GetNodeList();
	bool IsNodeInNodeList(QMeshNode *_node);

    void SetMinCurvatureVector(double vx, double vy, double vz);
    void GetMinCurvatureVector(double &vx, double &vy, double &vz);

    void SetMaxCurvatureVector(double vx, double vy, double vz);
    void GetMaxCurvatureVector(double &vx, double &vy, double &vz);

    void SetWeight(double weight) {m_weight=weight;};
    double GetWeight() {return m_weight;};

    void SetColor(float r, float g, float b) {m_rgb[0]=r; m_rgb[1]=g; m_rgb[2]=b;};
    void GetColor(float &r, float &g, float &b) {r=m_rgb[0]; g=m_rgb[1]; b=m_rgb[2];};

	GLKObList attachedList;

    double m_trackingPos[3];	int m_collideConstraintIndex;
    QMeshFace* m_trackingFace;

    void *attachedPointer, *attachedPointer1;

    int m_nIdentifiedPatchIndex = -1;
	bool selected = false;
	bool selectedforEdgeSelection;
	bool selectedforFaceSelection;
    bool boundary = false;
    bool boundary1,boundary2;
    bool active;
    bool visited = false;
    int featurePt;
    bool m_sepFlag;
	bool isFixed = false;
	bool isHandle = false;

	bool isSeed = false;

    double value1;
    double value2;

	double Alpha = 0; //For TopOpt Filter Variable

    double U,V,W;

    void GetCoord3D_FLP( double &x, double &y, double &z );
    void SetCoord3D_FLP( double x, double y, double z );

    void GetCoord3D_Laplacian( double &x, double &y, double &z ) {x=coord3D_Laplacian[0]; y=coord3D_Laplacian[1]; z=coord3D_Laplacian[2];};
    void SetCoord3D_Laplacian( double x, double y, double z ) {coord3D_Laplacian[0]=x; coord3D_Laplacian[1]=y; coord3D_Laplacian[2]=z;};

    void SetMixedArea(double area) {m_mixedArea=area;};

    int TempIndex; // for remeshing
    int identifiedIndex;
	bool inner, i_inner;

	// variables for RoughMachining
	bool isIn_convexHull = false; //***No longer used in Neelotpal's version of code
	bool isIn_convexHullTest = false; //for distance field
	bool isIn_modelSurface = false;
	bool isIn_modelSurfaceTest = false; //for distance field
	int convexMachining_Ind = -1;
	double scalarField = 0.0; //used for both distance field and heat method
	double distanceFromModel = 0.0; //for distance field
	double distanceFromCH = 0.0; //for distance field, distance from Convex Hull
	double distanceFromFixture = -0.1;
	bool isHybridBoundary = false; //to assign BC in Poisson


	double scalarField_init = -1.0;
	bool isoSurfaceBoundary = false; //curved layer slicing
	QMeshEdge* relatedTetEdge = NULL;// for record the node of iso-layers on which tet edge;
	double geoFieldValue; //for tool-path generation, compute from heat method
	double boundaryValue; //for tool-path generation, boundary distance field
	int heatmethodIndex; //for tool-path generation, heat method computing
	double dualArea();
	QMeshEdge* relatedLayerEdge = NULL; //for tool-path generation, iso-Node on which layer edge
	bool connectTPathProcessed; //for tool-path connection, indicate has been linked
	double isoValue = -1.0;//for tool-path connection, indicate isoNode's isoValue
	bool resampleChecked = false; //for tool-path connection, indicate whether the Node will be selected after resampling
	double nodePlaneDis; // parameter used in plane cutting
	bool planeCutNewNode = false;

	//temprorary
	/**added for vector editing**/
	double vectorDir[3] = {0.0,0.0,0.0};
	/***************************/

private:
	int indexno;
	bool flags[8];
				// bit 0 -- True for boundary points
				// bit 1 -- True for points on coarse mesh
				// bit 2 -- True for points on interpolation curve 
				// bit 3 -- True for points on hand (temp use) (or points adjacent to boundary face)
				// bit 4 -- True for points on arm (temp use) (or branch vertices)
				// bit 5 -- True for sharp-feature vertex (or vertex cannot be moved)
				// bit 6 -- True for sharp-feature-region vertex
				// bit 7 -- True for points moved (temp use or newly created)
	double coord2D[2];
				// 2D space coordinate
	double coord3D[3];
				// 3D space coordinate
	double coord3D_last[3];  // just for reset sewing operation for one step
                // 3D space coordinate in the last sewing step
    double coord3D_Laplacian[3];

    double m_meanCurvatureNormalVector[3], m_gaussianCurvature, m_pMaxCurvature, m_pMinCurvature;
    double m_minCurvatureVector[3], m_maxCurvatureVector[3];
    double m_boundaryDist, m_densityFuncValue;
    double m_mixedArea;

	QMeshPatch *meshSurface;		// QMesh contains this node

	GLKObList faceList;	// a list of faces contained this node (QMeshFace)
	GLKObList edgeList;	// a list of edges contained this node (QMeshEdge)
	GLKObList nodeList;	// a list of nodes coincident with this node (QMeshNode)
	GLKObList tetraList;	// a list of nodes coincident with this node (QMeshNode)

    double m_normal[3];
    double m_weight;
    double coord3D_FLP[3]; //For Keep the FLProcessData

    float m_rgb[3];

public:
	void GetNormal_last(double& nx, double& ny, double& nz);
	void SetNormal_last(double nx, double ny, double nz);

private:
	double m_normal_last[3];


public:
	//bool isBoundaryNode();

	bool isBoundary=false;
	bool isSupressedBoundary = false;
	bool tempFlag = false;
	bool isInteriorBoundary = false;
	bool isBaseBoundary = false;
	bool isTopBoundary = false; //redundant
	bool isSource = false;
	bool ConvexHull_boundary = false;
	bool in_ConvexHull = false;  // this convex hull flag is used in Neelotpal's version of code
	bool counted = false;
	bool isConstraint = false;



	/*Added by Neelotpal*/
	int FieldIndexNumber; //So that the original indices are not disturbed 
	int ReducedFieldIndexNumber = -1; //for the system without the constrained nodes
	double HeatFieldValue = 2.0;
	bool visited_forWeight = false;
	bool visited_forVol = false;
	bool displayNode = true;


	/*For new Tet mesh*/
	bool isChamberNode = false; 
	bool isBoundaryNode = false; //redundant
	int chamberIndex = -1;
	int boundaryIndex = -1;
	int InsertIndex = -1; //added by neelotpal
	QMeshNode* connectTETNode;
	bool findConnectTETNode = false;

	//test
	int ind_hardContraint_heat = 0;

};

#endif
