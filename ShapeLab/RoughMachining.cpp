#include "RoughMachining.h"
#include <Eigen/Eigen>
#include<fstream>
#include<time.h>

//#include<igl/cotmatrix.h>
#include<PQPLib/PQP.h>

#include<iostream>
#include<GLKGeometry.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

void RoughMachining::initial(PolygenMesh* polygenMesh_MaterialSpace, PolygenMesh* polygenMesh_ConvexHull, PolygenMesh* polygenMesh_modelSurface, PolygenMesh* polygenMesh_fixtureSurface, bool isInside) {

	materialSpace = (QMeshPatch*)polygenMesh_MaterialSpace->GetMeshList().GetHead();
	convexHull = (QMeshPatch*)polygenMesh_ConvexHull->GetMeshList().GetHead();
	if (polygenMesh_modelSurface != NULL) {
		modelSurface = (QMeshPatch*)polygenMesh_modelSurface->GetMeshList().GetHead();
		//fixtureSurface = (QMeshPatch*)polygenMesh_fixtureSurface->GetMeshList().GetHead();
	}
	isIn = isInside;
	
	this->_index_initial(convexHull, false);
	this->_build_tetraSet_4SpeedUp(materialSpace);
	

}


void RoughMachining::initialIndex() {

	this->_index_initial(materialSpace, true);
	
}




/**************************************************************************/
/* Function for creating distance field in the materialSpace */
/**************************************************************************/
void RoughMachining::createDistanceField()
{
	computeDistanceRoughMachining();
	//initialIndex();
	//this->_createWeightMatrix();
	//std::cout << "Solving Heat Field\n";
	//this->_solveHeatField(50);

	//std::cout << "Generating vector field from Heat Field...\n";
	//this->_generateVectorField();
	//////
	////std::cout << "Generating Final Scalar Field...\n";
	//this->_generateScalarField();


}

void RoughMachining::computeDistanceRoughMachining() {
	/* PQP compute distance */
	PQP_Model* pqpModel_CH = new PQP_Model();
	pqpModel_CH->BeginModel();  int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];

	for (GLKPOSITION Pos = convexHull->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(Pos);
		/*Face->CalPlaneEquation();
		double nn[3];
		Face->GetNormal(nn[0], nn[1], nn[2]);
		Eigen::Vector3d faceNorm(nn[0], nn[1], nn[2]);
		faceNorm.stableNormalize();

		Eigen::Vector3d downDir(0, 0, -1);

		if (faceNorm.dot(downDir) > 0.95) continue;*/

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel_CH->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel_CH->EndModel();

	PQP_Model* pqpModel_MS = new PQP_Model();
	pqpModel_MS->BeginModel();  index = 0;

	for (GLKPOSITION Pos = modelSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)modelSurface->GetFaceList().GetNext(Pos);

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel_MS->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel_MS->EndModel();


	/* compute the distance from convexHull*/

	double distance = 999999.99;
	double max = -9.99e10;
	double min = 1e-10;

	double max2 = -9.99e10;
	double min2 = 1e-10;

	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);

		PQP_DistanceResult dres;	dres.last_tri = pqpModel_CH->last_tri;
		PQP_REAL p[3];

		Node->GetCoord3D(p[0], p[1], p[2]);
		

		PQP_Distance(&dres, pqpModel_CH, p, 0.0, 0.0);

		float closestPt[3];	// closest point
		closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];

		int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero


		QMeshFace* cloestFace;
		int faceIndex = 0;
		for (GLKPOSITION _Pos = convexHull->GetFaceList().GetHeadPosition(); _Pos;) {
			QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(_Pos);
			if (faceIndex == minTriId) {
				cloestFace = Face;
				break;
			}
			faceIndex++;
		}

		Eigen::Vector3d faceNormal; double d;
		cloestFace->CalPlaneEquation();
		cloestFace->GetPlaneEquation(faceNormal[0], faceNormal[1], faceNormal[2], d);

		Eigen::Vector3d nodePos, cloestPos;
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
		for (int i = 0; i < 3; i++) cloestPos[i] = closestPt[i];

		Node->distanceFromCH = dres.Distance();
		
		Eigen::Vector3d pointDir = 100*(Eigen::Vector3d(p[0], p[1], p[2]) - Eigen::Vector3d(closestPt[0], closestPt[1], closestPt[2]));
		pointDir.stableNormalize();
		//if (abs(pointDir.norm()) < 1e-2) pointDir = 100 * pointDir;
		

		double _orient = pointDir.dot(faceNormal);

		double eps = 1.0e-5;

		/*if (abs(Node->scalarField) < eps) {
			Node->ConvexHull_boundary = true;
			if (max < Node->scalarField) max = Node->scalarField;
			else if (min > Node->scalarField) min = Node->scalarField;
			continue;
		}*/

		if (_orient < 0) {
			//std::cout << _orient << std::endl;
			bool doBreak = false;
			
			if (Node->distanceFromCH > 4) { 
				//std::cout << "----------------------------\n";
				//std::cout << pointDir[0] << " " << pointDir[1] << " " << pointDir[2] << " " << std::endl;
			}
				for (int ii = 1; ii <= 3; ii++) {
					QMeshEdge* faceEdge = cloestFace->GetEdgeRecordPtr(ii);
					QMeshFace* face1 = faceEdge->GetRightFace();
					if (face1 == NULL) std::cout << "No face\n";
					if (face1 == cloestFace) {
						face1 = faceEdge->GetLeftFace();
					}
					
					face1->CalPlaneEquation();
					Eigen::Vector3d faceNormal2;
					Eigen::Vector3d pointNormal;
					QMeshNode* pointNode = face1->GetNodeRecordPtr(1);
					double pointX, pointY, pointZ;
					pointNode->GetCoord3D(pointX, pointY, pointZ);
					pointNormal = Eigen::Vector3d(p[0], p[1], p[2]) - Eigen::Vector3d(pointX, pointY, pointZ);
					pointNormal.stableNormalize();

					double d2;
					face1->CalPlaneEquation();
					face1->GetPlaneEquation(faceNormal2[0], faceNormal2[1], faceNormal2[2], d2);
					//pointDir.normalize();
					faceNormal2 = 100 * faceNormal2;
					faceNormal2.stableNormalize();
					double _checkOrient = pointNormal.dot(faceNormal2);
					if (Node->distanceFromCH > 4) {
						
						//std::cout << faceNormal2[0] << " " << faceNormal2[1] << " " << faceNormal2[2] << " " << std::endl;
					}
					
					if (_checkOrient > -0.2) {
						
						if (max < Node->scalarField) max = Node->scalarField;
						else if (min > Node->scalarField) min = Node->scalarField;
						doBreak = true;
						break;
					}

					

					
				}
			//	
		

			if (doBreak) {
				continue;
			}
			if (!doBreak) {
				Node->isIn_convexHullTest = true;
				if (Node->distanceFromCH > 3) std::cout << "Abnormal\n\n\n";
				Node->distanceFromCH = -abs(Node->distanceFromCH);
				
			
			}
			//Node->scalarField = 0.00;
			PQP_DistanceResult dres2;	dres2.last_tri = pqpModel_MS->last_tri;
			PQP_Distance(&dres2, pqpModel_MS, p, 0.0, 0.0);

			float closestPt2[3];	// closest point
			closestPt2[0] = dres2.p1[0];	closestPt2[1] = dres2.p1[1]; closestPt2[2] = dres2.p1[2];

			int minTriId2 = dres2.last_tri->id;

			QMeshFace* cloestFace2;
			int faceIndex2 = 0;
			for (GLKPOSITION _Pos = modelSurface->GetFaceList().GetHeadPosition(); _Pos;) {
				QMeshFace* Face = (QMeshFace*)modelSurface->GetFaceList().GetNext(_Pos);
				if (faceIndex2 == minTriId2) {
					cloestFace2 = Face;
					break;
				}
				faceIndex2++;
			}

			Eigen::Vector3d faceNormal2; double d2;
			cloestFace2->CalPlaneEquation();
			cloestFace2->GetPlaneEquation(faceNormal2[0], faceNormal2[1], faceNormal2[2], d2);

			Eigen::Vector3d nodePos2, cloestPos2;
			Node->GetCoord3D(nodePos2[0], nodePos2[1], nodePos2[2]);
			for (int i = 0; i < 3; i++) cloestPos2[i] = closestPt2[i];

			Eigen::Vector3d pointDir2 = Eigen::Vector3d(p[0], p[1], p[2]) - Eigen::Vector3d(closestPt2[0], closestPt2[1], closestPt2[2]);
			double _orient2 = pointDir2.dot(faceNormal2);
			Node->distanceFromModel = dres2.Distance();

			/*if (abs(Node->distanceFromModel) < eps) {
				Node->ConvexHull_boundary = true;
				continue;
			}*/


			if (_orient2 < 0) {
				Node->isIn_modelSurface = true;
				Node->isIn_modelSurfaceTest = true;
				Node->scalarField = Node->scalarField;
				Node->distanceFromModel = -abs(Node->distanceFromModel);
			}
			else {
				Node->scalarField = Node->scalarField;
				if (Node->distanceFromModel > max2) max2 = Node->distanceFromModel;
				if (Node->distanceFromModel < min2) min2 = Node->distanceFromModel;
			}
			

		}
		else {
			if (max < Node->scalarField) max = Node->scalarField;
			else if (min > Node->scalarField) min = Node->scalarField;
		}
	}


	//for (GLKPOSITION edgePos = materialSpace->GetEdgeList().GetHeadPosition(); edgePos;) {
	//	QMeshEdge* _edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(edgePos);
	//	QMeshNode* StartNode = _edge->GetStartPoint();
	//	QMeshNode* EndNode = _edge->GetEndPoint();
	//	
	//	if ((StartNode->isIn_modelSurfaceTest && EndNode->isIn_modelSurfaceTest) || (!StartNode->isIn_modelSurfaceTest && !EndNode->isIn_modelSurfaceTest)) {
	//		continue;
	//	}
	//	else {
	//		//StartNode->isIn_modelSurface = false;
	//		//EndNode->isIn_modelSurface = false;
	//	}
	//}

	//initialisePrimitiveDiscard();
	//initialiseSourceOnConvexHull();


	max = -9999999;
	min = 0;

	double maxx = -9.99e10;
	double minn = 9.99e10;


	/***********Comment Block Start********/

	///*27/03/2022: Making changes here*/
	//for (GLKPOSITION edgePos = materialSpace->GetEdgeList().GetHeadPosition(); edgePos;) {
	//	QMeshEdge* edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(edgePos);
	//	QMeshNode* node1 = (QMeshNode*)edge->GetStartPoint();
	//	QMeshNode* node2 = (QMeshNode*)edge->GetEndPoint();
	//	if (node1->distanceFromCH < 0 && node2->distanceFromCH < 0) {
	//		node1->scalarField = -9999;
	//		node2->scalarField = -9999;
	//	}
	//	else {
	//		node1->scalarField = node1->distanceFromCH;
	//		node2->scalarField = node2->distanceFromCH;
	//	}
	//}
	//for (GLKPOSITION edgePos = materialSpace->GetEdgeList().GetHeadPosition(); edgePos;) {
	//	QMeshEdge* edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(edgePos);
	//	QMeshNode* node1 = (QMeshNode*)edge->GetStartPoint();
	//	QMeshNode* node2 = (QMeshNode*)edge->GetEndPoint();
	//	if ((node1->distanceFromCH >= 0 && node2->distanceFromCH <= 0)|| (node2->distanceFromCH >= 0 && node1->distanceFromCH <= 0)) {
	//		node1->scalarField = node1->distanceFromCH;
	//		node2->scalarField = node2->distanceFromCH;
	//	}
	//}


	//for(GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
	//	QMeshNode* _node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
	//	//if (_node->isIn_convexHullTest) {
	//	//	if (_node->isIn_modelSurface) {
	//	//		//_node->scalarField = 0;
	//	//		if (max < _node->scalarField) max = _node->scalarField;
	//	//		if (min > _node->scalarField) min = _node->scalarField;
	//	//	
	//	//	}
	//	//	else {

	//	//		if (max < _node->scalarField) max = _node->scalarField;
	//	//		if (min > _node->scalarField) min = _node->scalarField;
	//	//	}
	//	//}
	//	//else {
	//	//	//_node->scalarField = 0;
	//	//}
	//	if (_node->isSource) {
	//		_node->HeatFieldValue = 1 + _node->distanceFromCH;
	//		if (max < _node->HeatFieldValue) max = _node->HeatFieldValue;
	//		
	//	}
	//	//else if (_node->isBoundary) _node->scalarField = 0.5;
	//	else _node->HeatFieldValue = 0;


	//	
	//	/*27/03/2022: Making changes here*/


	//	if (_node->scalarField > -9999) {
	//		_node->scalarField = _node->distanceFromCH;
	//		if (maxx < _node->scalarField) maxx = _node->scalarField;
	//		else if (minn > _node->scalarField) minn = _node->scalarField;
	//	}
	//	else {
	//		_node->scalarField = -9999;
	//	}


	//}

	//for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
	//	QMeshNode* _node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
	//	
	//	if (_node->isSource) {
	//		_node->HeatFieldValue = _node->HeatFieldValue / max;

	//	}

	//	if (_node->scalarField > -9999) {
	//		_node->scalarField = (_node->scalarField) / (maxx);
	//	}
	//	else {
	//		_node->scalarField = -9999;
	//	}

	//}
	////max_distance = max-min;


	/***********Comment Block End********/
	//max_distance = maxx - minn;

	double maxxx = -1e10;
	double minnn = 1e10;
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* _node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		_node->scalarField = _node->distanceFromCH;
		if (maxxx < _node->scalarField) maxxx = _node->scalarField;
		if (minnn > _node->scalarField) minnn = _node->scalarField;

	}

	max_distance = maxxx;
	min_distance = minnn;

	std::cout << "MAXIMUM " << max << std::endl;
	std::cout << "MINIMUM " << min << std::endl;


	std::vector<QMeshNode*> nodelist(materialSpace->GetNodeNumber());
	index = 0;
	//for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* Node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);
	//	nodelist[index] = Node;
	//	if (!Node->isIn_modelSurface) {
	//		Node->scalarField = (Node->scalarField - min) / (max - min);
	//		//std::cout << Node->scalarField << std::endl;
	//	}
	//	else {
	//	//	Node->scalarField = -1;
	//		Node->scalarField = (Node->scalarField - min) / (max - min);
	//	}
	//	index++;
	//}


//#pragma omp parallel 
//		for (int i = 0; i < materialModel->GetNodeNumber(); i++) {
//			QMeshNode* Node = nodelist[i];
//			if (Node->scalarField < 0.001) {
//				Node->isTargetModelNode = true;
//				Node->scalarField = abs(Node->scalarField);
//			}
//			else {
//
//				Eigen::Vector3d pos;
//				Node->GetCoord3D(pos(0), pos(1), pos(2));
//				bool insideMesh = this->calculatePointInsideMesh(targetModel, pos);
//
//				if (insideMesh) {
//					//std::cout << "inside target surface !" << std::endl;
//					Node->scalarField = -abs(Node->scalarField);
//					Node->isMillingSpaceNode = false;
//				}
//
//				else {
//					Node->scalarField = Node->scalarField;
//
//				}
//
//			}
//
//
//		}
//	}

	//double min3 = 9.99e10;
	//double max3 = 1.0e-10;

	//for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
	//	QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);
	//	if (node->isIn_convexHullTest && !node->isIn_modelSurface) {
	//		double w = (node->distanceFromModel - min2) / (max2 - min2);
	//		//w = 0.5;
	//		node->scalarField = (1.0 - w) * node->distanceFromModel + w * node->scalarField;
	//		if (node->scalarField > max3) max3 = node->scalarField;
	//		if (node->scalarField < min3) min3 = node->scalarField;
	//	}
	//	else {
	//		node->scalarField = 0.0;
	//	}

	//	
	//}

	//for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
	//	QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);
	//	if (node->isIn_convexHullTest && !node->isIn_modelSurface) {
	//		node->scalarField = (node->scalarField - min3) / (max3 - min3);
	//		
	//	}

	//}

	//max_distance = max3 - min3;
}




void RoughMachining::initialisePrimitiveDiscard() {
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		int _in_count = 0;
		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = (QMeshNode*)tetra->GetNodeRecordPtr(i);
			if (node->isIn_modelSurface) {
				_in_count++;
				if (!node->in_ConvexHull) {
					node->in_ConvexHull = true;
					Num_nodesInConvexHull++;
				}
			}
		}
		
		

		if (_in_count == 4) {
			tetra->isIn_convexHull = true;
			Num_tetInConvexHull++;
			
			for (int j = 1; j <= 6; j++) {
				QMeshEdge* edge = tetra->GetEdgeRecordPtr(j);
				//QMeshNode* n1 = edge->GetStartPoint();
				//QMeshNode* n2 = edge->GetEndPoint();
				
				//if (n1->isIn_modelSurface || n2->isIn_modelSurface) {
					edge->in_convexHull = true;
				//}
			}
			
			for (int k = 1; k <= 4; k++) {
				QMeshFace* face = tetra->GetFaceRecordPtr(k);
				int face_node_count = 0;
				for (int l = 0; l < 3; l++) {
					QMeshNode* faceNode = face->GetNodeRecordPtr(l);
					if (faceNode->isIn_modelSurface) face_node_count++;
				}
				if (face_node_count > 0) face->in_convexHull = true;
			}
			continue;
		}

		_in_count = 0;

		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = (QMeshNode*)tetra->GetNodeRecordPtr(i);
			if (!node->isIn_convexHullTest) {
				_in_count++;

			}
		}

		_in_count = 0; //This has been added to preserve the model outside the CH

		if (_in_count == 4) {
			tetra->isIn_convexHull = true;
			Num_tetInConvexHull++;


			for (int i = 1; i <= 4; i++) {
				QMeshNode* node = (QMeshNode*)tetra->GetNodeRecordPtr(i);

				if (!node->in_ConvexHull) {
					node->in_ConvexHull = true;
					Num_nodesInConvexHull++;
				}

			}

			for (int j = 1; j <= 6; j++) {
				QMeshEdge* edge = tetra->GetEdgeRecordPtr(j);
				edge->in_convexHull = true;

			}

			for (int k = 1; k <= 4; k++) {
				QMeshFace* face = tetra->GetFaceRecordPtr(k);
				int face_node_count = 0;
				for (int l = 0; l < 3; l++) {
					QMeshNode* faceNode = face->GetNodeRecordPtr(l);
					if (faceNode->in_ConvexHull) face_node_count++;
				}
				if (face_node_count >= 3) face->in_convexHull = true;
			}
			continue;
		}
		

	}


	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (!tetra->isIn_convexHull) {
			for (int i = 1; i <= 4; i++) {
				QMeshNode* node = (QMeshNode*)tetra->GetNodeRecordPtr(i);
				if (node->in_ConvexHull) {
					node->in_ConvexHull = false;
					Num_nodesInConvexHull--;
				}
			}

			for (int i = 1; i <= 4; i++) {
				QMeshFace* face = (QMeshFace*)tetra->GetFaceRecordPtr(i);
				face->in_convexHull = false;
			}

			for (int i = 1; i <= 6; i++) {
				QMeshEdge* edge = (QMeshEdge*)tetra->GetEdgeRecordPtr(i);
				edge->in_convexHull = false;
			}
		}
	}


	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* face = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);
		if (!face->in_convexHull) {
			

			for (int i = 1; i <= 3; i++) {
				QMeshEdge* edge = (QMeshEdge*)face->GetEdgeRecordPtr(i);
				if (edge->in_convexHull) std::cout << "Error!!\n";
			}
		}
	}

}


void RoughMachining::initialiseSourceOnConvexHull() {
	for (GLKPOSITION edgePos = materialSpace->GetEdgeList().GetHeadPosition(); edgePos;) {
		QMeshEdge* edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(edgePos);
		if (edge->in_convexHull) {
			QMeshNode* n1 = edge->GetStartPoint();
			QMeshNode* n2 = edge->GetEndPoint();

			if ((n1->in_ConvexHull && !n2->in_ConvexHull) || (!n1->in_ConvexHull && n2->in_ConvexHull)) {
				if (!n1->in_ConvexHull) n1->isBoundary = true;
				else n2->isBoundary = true;

			}
		}
		QMeshNode* n1 = edge->GetStartPoint();
		QMeshNode* n2 = edge->GetEndPoint();

		if ((n1->isIn_convexHullTest && !n2->isIn_convexHullTest) || (!n1->isIn_convexHullTest && n2->isIn_convexHullTest)) {
			n1->isSource = true;
			n2->isSource = true;
		}
	}

}

/**************************************************************************/
/* Function for detecting the boundary of convexHull in the materialSpace */
/**************************************************************************/
void RoughMachining::convexHull_Boundary_Detection() {

	//detect the center of tet element of materialSpace in/out the convexHull surface
#pragma omp parallel   
	{
#pragma omp for 
		for (int i = 0; i < tetraSet_materialSpace.size(); i++) {
			QMeshTetra* each_Tetra = tetraSet_materialSpace[i];

			double xx, yy, zz;
			each_Tetra->CalCenterPos(xx, yy, zz);
			each_Tetra->isIn_convexHull = _eleCenter_in_convexHull2(xx, yy, zz);

			if (each_Tetra->isIn_convexHull) Num_tetInConvexHull++;

			//std::cout << each_Tetra->isIn_convexHull << std::endl;
		}
	}

	for (GLKPOSITION Pos = materialSpace->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)materialSpace->GetFaceList().GetNext(Pos);

		if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

		if ((Face->GetLeftTetra()->isIn_convexHull && !Face->GetRightTetra()->isIn_convexHull)
			|| (!Face->GetLeftTetra()->isIn_convexHull && Face->GetRightTetra()->isIn_convexHull)) {

			Face->convexHull_boundary = true;
			//transform the convexHull_boundary flag to edges
			for (int edgeInd = 0; edgeInd < 3; edgeInd++) {
				Face->GetEdgeRecordPtr(edgeInd + 1)->convexHull_boundary = true;
				
				Face->GetEdgeRecordPtr(edgeInd + 1)->GetStartPoint()->ConvexHull_boundary = true;
				if (!Face->GetEdgeRecordPtr(edgeInd + 1)->GetStartPoint()->counted) {
					Face->GetEdgeRecordPtr(edgeInd + 1)->GetStartPoint()->counted = true;
					Num_nodesOnConvexHullBoundary++;
				}

				Face->GetEdgeRecordPtr(edgeInd + 1)->GetEndPoint()->ConvexHull_boundary = true;
				if (!Face->GetEdgeRecordPtr(edgeInd + 1)->GetEndPoint()->counted) {
					Face->GetEdgeRecordPtr(edgeInd + 1)->GetEndPoint()->counted = true;
					Num_nodesOnConvexHullBoundary++;
				}
			}
		}
		//std::cout << Face->convexHull_boundary << std::endl;		
	}


	for (GLKPOSITION Pos = materialSpace->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)materialSpace->GetFaceList().GetNext(Pos);

		if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

		if ((Face->GetLeftTetra()->isIn_convexHull && Face->GetRightTetra()->isIn_convexHull)) {

			Face->in_convexHull = true;
			//transform the convexHull_boundary flag to edges
			for (int edgeInd = 0; edgeInd < 3; edgeInd++) {


				if (Face->GetEdgeRecordPtr(edgeInd + 1)->convexHull_boundary) continue;
				Face->GetEdgeRecordPtr(edgeInd + 1)->in_convexHull = true;
				

				QMeshNode* node_ = Face->GetEdgeRecordPtr(edgeInd + 1)->GetStartPoint();
				if (!node_->ConvexHull_boundary && !node_->counted) {
					node_->in_ConvexHull = true;
					node_->counted = true;
					Num_nodesInConvexHull++;
				}
				node_ = Face->GetEdgeRecordPtr(edgeInd + 1)->GetEndPoint();
				if (!node_->ConvexHull_boundary && !node_->counted) {
					node_->in_ConvexHull = true;
					node_->counted = true;
					Num_nodesInConvexHull++;
				}
			}
		}
		//std::cout << Face->convexHull_boundary << std::endl;		
	}
}


bool RoughMachining::_eleCenter_in_convexHull(double xx, double yy, double zz) {

	bool flag = true;
	for (GLKPOSITION Pos = convexHull->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(Pos);

		double A, B, C, D;
		Face->GetPlaneEquation(A, B, C, D);
		double temp = A * xx + B * yy + C * zz + D;
		// the normal of convexHull points out
		if (temp > 0.0) {
			flag = false;
			break;
		}
	}
	return flag;
}

bool RoughMachining::_eleCenter_in_convexHull2(double qx, double qy, double qz) {

	Eigen::Vector3d dir = { 1.0,0.0,0.0 };
	Eigen::Vector3d orig = { qx,qy,qz };
	int intersection_Time = 0;


	for (GLKPOSITION Pos = convexHull->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* each_face = (QMeshFace*)convexHull->GetFaceList().GetNext(Pos);

		double xx, yy, zz;
		each_face->GetNodeRecordPtr(0)->GetCoord3D(xx, yy, zz);
		Eigen::Vector3d v0 = { xx,yy,zz };

		each_face->GetNodeRecordPtr(1)->GetCoord3D(xx, yy, zz);
		Eigen::Vector3d v1 = { xx,yy,zz };

		each_face->GetNodeRecordPtr(2)->GetCoord3D(xx, yy, zz);
		Eigen::Vector3d v2 = { xx,yy,zz };

		if (this->IntersectTriangle(orig, dir, v0, v1, v2))
			intersection_Time++;
	}
	//std::cout << "intersection Num " << intersection_Time << std::endl;
	if (intersection_Time % 2 != 0) {
		//std::cout << "in the mesh" << std::endl;
		return true;
	}
	else return false;
	//std::cout << "be out of mesh" << std::endl;
}


void RoughMachining::_index_initial(QMeshPatch* patch, bool is_TetMesh) {

	//----initial the edge, node and face index, start from 0
	int index = 0;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		if (Edge->in_convexHull) continue;
		Edge->FieldIndexNumber = index;
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->in_convexHull) continue;
		Face->FieldIndexNumber = index;
		Face->CalPlaneEquation(); // pre-compute the normal of face
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (Node->in_ConvexHull) continue;
		Node->FieldIndexNumber = index;
		index++;
	}
	// only tet mesh needs reorder the index of Tetra
	if (is_TetMesh) {
		index = 0;
		for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
			if (Tetra->isIn_convexHull) continue;
			Tetra->FieldIndexNumber = index;
			/*

			for (int i = 0; i < 4; i++) {
				QMeshFace* fac = Tetra->GetFaceRecordPtr(i + 1);
				fac->FieldIndexNumber = i + 1;
				std::cout << "yay\n";
			}*/

			index++;
		}
	}
}

void RoughMachining::_build_tetraSet_4SpeedUp(QMeshPatch* patch) {

	tetraSet_materialSpace.resize(patch->GetTetraNumber());
	int index = 0;
	for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
		tetraSet_materialSpace[index] = Tetra; index++;
	}

	// protect operation
	if (patch->GetTetraNumber() != index) std::cout << "Error: please check the num of tet mesh(materialSpace)! " << std::endl;
}

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool RoughMachining::IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
	// E1
	Eigen::Vector3d E1 = v1 - v0;

	// E2
	Eigen::Vector3d E2 = v2 - v0;

	// P
	Eigen::Vector3d P = dir.cross(E2);

	// determinant
	float det = E1.dot(P);

	// keep det > 0, modify T accordingly
	Eigen::Vector3d T;
	if (det > 0)
	{
		T = orig - v0;
	}
	else
	{
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001f)
		return false;

	// Calculate u and make sure u <= 1
	double t, u, v;
	u = T.dot(P);
	if (u < 0.0f || u > det)
		return false;

	// Q
	Eigen::Vector3d Q = T.cross(E1);

	// Calculate v and make sure u + v <= 1
	v = dir.dot(Q);
	if (v < 0.0f || u + v > det)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = E2.dot(Q);
	if (t < 0) return false;

	float fInvDet = 1.0f / det;
	t *= fInvDet;
	u *= fInvDet;
	v *= fInvDet;

	return true;
}




void RoughMachining::compTetMeshVolumeMatrix() {

	//-- This function calculate the volume matrix 
	//   for each tetrahedral elements and installed in formate
	/* [   b1 c1 d1
		   b2 c2 d2
		   b3 c3 d3
		   b4 c4 d4   ] */

	for (GLKPOSITION Pos = materialSpace->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(Pos);

		Eigen::MatrixXd VolumeMatrix(4, 3);

		Eigen::Vector3d aa, bb, cc, dd, pp;
		Tet->CalCenterPos(pp(0), pp(1), pp(2));

		Tet->GetNodeRecordPtr(1)->GetCoord3D(aa(0), aa(1), aa(2));
		Tet->GetNodeRecordPtr(2)->GetCoord3D(bb(0), bb(1), bb(2));
		Tet->GetNodeRecordPtr(3)->GetCoord3D(cc(0), cc(1), cc(2));
		Tet->GetNodeRecordPtr(4)->GetCoord3D(dd(0), dd(1), dd(2));

		Eigen::Vector3d vap = pp - aa;
		Eigen::Vector3d vbp = pp - bb;

		Eigen::Vector3d vab = bb - aa;
		Eigen::Vector3d vac = cc - aa;
		Eigen::Vector3d vad = dd - aa;

		Eigen::Vector3d vbc = cc - bb;
		Eigen::Vector3d vbd = dd - bb;

		Eigen::Vector3d bd_bc = vbd.cross(vbc);
		Eigen::Vector3d ac_ad = vac.cross(vad);
		Eigen::Vector3d ad_ab = vad.cross(vab);
		Eigen::Vector3d ab_ac = vab.cross(vac);
		double volumeTet = Tet->CalVolume() * 6;

		VolumeMatrix.row(0) = bd_bc / volumeTet;
		VolumeMatrix.row(1) = ac_ad / volumeTet;
		VolumeMatrix.row(2) = ad_ab / volumeTet;
		VolumeMatrix.row(3) = ab_ac / volumeTet;

		Tet->VolumeMatrix = VolumeMatrix;
	}
}


/**************************************************************************/
/*					Function for generating the Heat Field					*/
/**************************************************************************/

void RoughMachining::createHeatField(PolygenMesh* MaterialMesh, int BoundaryCondition) {

	//materialSpace = (QMeshPatch*)MaterialMesh->GetMeshList().GetHead();
	int N_num = materialSpace->GetNodeNumber();
	int T_num = materialSpace->GetTetraNumber();

	//this->_index_initial(materialSpace, true);
	
	//std::cout << "NUUUUM: " << materialSpace->GetNodeNumber() << "\n\n\n";
	this->_createWeightMatrix();


	//std::cout << "Weight: \n" << WeightMatrix << std::endl;
	//std::cout <<"Volume: \n" << VolumeMatrix << std::endl;

	
	std::cout << "Creating Laplacian Matrix\n";
	this->_createLaplacianMatrix();

	//std::cout << "Creating Volume Matrix\n";
	//this->_createVolumeMatrix();
	
	std::cout << "Solving Heat Field\n";
	this->_solveHeatField(BoundaryCondition);

	std::cout << "Generating vector field from Heat Field...\n";
	this->_generateVectorField();
	//
	std::cout << "Generating Final Scalar Field...\n";
	this->_generateScalarField();
}

void RoughMachining::_createWeightMatrix() {


	std::cout << "Creating Weight Matrix\n";
	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;
	//std::cout << num_nodes << std::endl;


	WeightMatrix.resize(num_nodes, num_nodes);
	VolumeMatrix.resize(num_nodes, num_nodes);
	VolumeMatrix.reserve(Eigen::VectorXi::Constant(num_nodes, 2));
	WeightMatrix.reserve(Eigen::VectorXi::Constant(num_nodes, 35));
	
	//Eigen::MatrixXd VolumeMatrix_temp = Eigen::MatrixXd(num_nodes, num_nodes);

	int ccount = 0;

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
	
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		if (tet->isIn_convexHull) continue;

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
			/*std::cout << _weight << " Weight\n" << std::endl;*/

			/*if (!edge->visited) {
				edge->visited = true;
				WeightMatrix(i, i) += 1;
				WeightMatrix(j, j) += 1;
				WeightMatrix(i, j) = 1;
				WeightMatrix(j, i) = 1;
			}*/
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
				WeightMatrix.insert(i, i) = - _weight;
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

	/*for (int i = 0; i < num; i++) {
		double _num = WeightMatrix(i, i);
		

		for (int j = 0; j < num; j++) {
			if (WeightMatrix(i, j) != 0) {
				WeightMatrix(i, j) = -1 / _num;
			}
		}
		WeightMatrix(i, i) = 1;
	}*/


}

double RoughMachining::_getEdgeWeight(QMeshEdge* edge, QMeshTetra* tet) {
	QMeshNode* node1 = edge->GetStartPoint();
	QMeshNode* node2 = edge->GetEndPoint();

	int index1_in_tet = tet->GetNodeIndex(node1);
	int index2_in_tet = tet->GetNodeIndex(node2);

	QMeshFace* t_face1, * t_face2; //see the defination of discrete Laplacian for tet mesh to understand these faces and the edge
	QMeshEdge* t_edge;


	QMeshNode* otherNode1, * otherNode2;
	bool foundOne = false;

	for(int k = 1; k <= 4; k++){
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

	/*for (int k = 1; k <= 6; k++) {
		QMeshEdge* tempEdge = tet->GetEdgeRecordPtr(k);
		if (tempEdge->GetStartPoint() == otherNode1 || tempEdge->GetStartPoint() == otherNode1) {
			if (tempEdge->GetEndPoint() == otherNode1 || tempEdge->GetEndPoint() == otherNode1) {
				t_edge = tempEdge;
				break;
			}
		}
	}*/

	double x0, y0, z0, x1, y1, z1, x2, y2, z2;

	otherNode1->GetCoord3D(x0, y0, z0);
	otherNode2->GetCoord3D(x1, y1, z1);
	node1->GetCoord3D(x2, y2, z2);

	Eigen::Vector3d vfixed(x1 - x0, y1 - y0, z1 - z0);
	Eigen::Vector3d v0(x2 - x0, y2 - y0, z2 - z0);


	Eigen::Vector3d v1= vfixed.cross(v0);
	
	node2->GetCoord3D(x2, y2, z2);
	Eigen::Vector3d v00(x2 - x0, y2 - y0, z2 - z0);


	Eigen::Vector3d v2 = vfixed.cross(v00);

	double length = abs(vfixed.norm());


	
	////Now recall the node/edge/face arrangement for a tetra to understand the following
	//if (((index1_in_tet == 1) || (index1_in_tet == 2)) && ((index2_in_tet == 1) || (index2_in_tet == 2))) {
	//	//std::cout << index1_in_tet << " " << index2_in_tet << " :1 2\n";
	//	t_face1 = tet->GetFaceRecordPtr(3);
	//	t_face2 = tet->GetFaceRecordPtr(2);
	//	t_edge = tet->GetEdgeRecordPtr(6);
	//}
	//
	//else if (((index1_in_tet == 1) || (index1_in_tet == 3)) && ((index2_in_tet == 1) || (index2_in_tet == 3))) {
	//	//std::cout << index1_in_tet << " " << index2_in_tet << " :1 3\n";
	//	t_face1 = tet->GetFaceRecordPtr(4);
	//	t_face2 = tet->GetFaceRecordPtr(2);
	//	t_edge = tet->GetEdgeRecordPtr(5);
	//}

	//else if (((index1_in_tet == 1) || (index1_in_tet == 4)) && ((index2_in_tet == 1) || (index2_in_tet == 4))) {
	//	//std::cout << index1_in_tet << " " << index2_in_tet << " :1 4\n";
	//	t_face1 = tet->GetFaceRecordPtr(1);
	//	t_face2 = tet->GetFaceRecordPtr(2);
	//	t_edge = tet->GetEdgeRecordPtr(2);
	//}

	//else if (((index1_in_tet == 2) || (index1_in_tet == 3)) && ((index2_in_tet == 2) || (index2_in_tet == 3))) {
	//	//std::cout << index1_in_tet << " " << index2_in_tet << " :2 3\n";
	//	t_face1 = tet->GetFaceRecordPtr(4);
	//	t_face2 = tet->GetFaceRecordPtr(3);
	//	t_edge = tet->GetEdgeRecordPtr(4);
	//}

	//else if (((index1_in_tet == 2) || (index1_in_tet == 4)) && ((index2_in_tet == 2) || (index2_in_tet == 4))) {
	//	//std::cout << index1_in_tet << " " << index2_in_tet << " :2 4\n";
	//	t_face1 = tet->GetFaceRecordPtr(1);
	//	t_face2 = tet->GetFaceRecordPtr(3);
	//	t_edge = tet->GetEdgeRecordPtr(3);
	//}

	//else if (((index1_in_tet == 3) || (index1_in_tet == 4)) && ((index2_in_tet == 3) || (index2_in_tet == 4))) {
	//	//std::cout << index1_in_tet << " " << index2_in_tet << " :3 4\n";
	//	t_face1 = tet->GetFaceRecordPtr(1);
	//	t_face2 = tet->GetFaceRecordPtr(4);
	//	t_edge = tet->GetEdgeRecordPtr(1);
	//}

	//double x1, y1, z1, x2, y2, z2;

	//if (t_edge == NULL) std::cerr << "t_edge is empty\n";
	//if (t_face1 == NULL) std::cerr << "t_edge is empty\n";
	//if (t_face2 == NULL) std::cerr << "t_edge is empty\n";

	
	//t_edge->CalLength();
	//double length = t_edge->GetLength();

	//t_face1->GetNormal(x1, y1, z1);
	//t_face2->GetNormal(x2, y2, z2);

	//Eigen::Vector3d v1(x1, y1, z1);
	//Eigen::Vector3d v2(-x2, -y2, -z2);

	
	long double _dot = v1.dot(v2) / (v1.norm() * v2.norm());
	if (_dot > 1) _dot = 1;
	else if (_dot < -1) _dot = -1;
	
	double angle = acos(_dot);
	

	double eps = 0.00001;
	if (angle < eps) angle += eps;

	double _cotan = cos(angle) / sin(angle);


	//std::cout << length << " length\n";
	//std::cout << _cotan << " cotan\n";
	//std::cout <<angle << " angle\n";

	return (double) length * _cotan / 6.0;

	
}

void RoughMachining::_createLaplacianMatrix() {
	int num = materialSpace->GetNodeNumber();
	//LaplacianMatrix = Eigen::MatrixXd::Zero(num, num);

	return;

	for (int i = 0; i < num; i++) {
		for (int j = 0; j < num; j++) {
		//	LaplacianMatrix(i, j) = WeightMatrix(i, j);
		}
	}
	/*
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		for (int n = 0; n < 4; n++) {
			QMeshEdge* edge = tet->GetEdgeRecordPtr(n + 1);
			int i = edge->GetStartPoint()->FieldIndexNumber;
			int j = edge->GetEndPoint()->FieldIndexNumber;
			WeightMatrix(i, j) += _getEdgeWeight(edge, tet);
			WeightMatrix(j, i) = WeightMatrix(i, j);
		}
	}*/
}

void RoughMachining::_createDensityMatrix() {
	
	
	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;

	//this->constNodeNumber = _defineBouindaryNodes();
	Eigen::VectorXd rho(num_nodes);
	Eigen::SparseMatrix<double> boundaryCompensation_add(num_nodes, num_nodes);
	Eigen::SparseMatrix<double> boundaryCompensation_mult(num_nodes, num_nodes);
	DensityMatrix = Eigen::SparseMatrix<double>(num_nodes,num_nodes);
	DensityMatrix.setIdentity();
	boundaryCompensation_mult.setIdentity();

	_initialiseSystemValues2(rho, 1.0, 4.0);

	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

		if (node->in_ConvexHull) continue;

		if (node->isBoundary || node->isSource) {
			boundaryCompensation_add.insert(node->FieldIndexNumber, node->FieldIndexNumber) = 1.0;
			boundaryCompensation_mult.coeffRef(node->FieldIndexNumber, node->FieldIndexNumber) = 0.0;
		}
		else {
			rho(node->FieldIndexNumber) = 0.0;
		}
	}
	
	

	Eigen::SparseMatrix<double> S = boundaryCompensation_add + boundaryCompensation_mult * VolumeMatrix * WeightMatrix;
	
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> SolverN;// (PardisoLU/SparseLU)
	SolverN.analyzePattern(S);
	SolverN.factorize(S);
	if (SolverN.info() != Eigen::Success)
		std::cout << "error here: factorize fail!" << std::endl;
	SolverN.compute(S);

	///*********This part is to test; should be removed later**********/

	//for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
	//	QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

	//	if (node->in_ConvexHull) continue;
	//	rho(node->FieldIndexNumber) = node->distanceFromFixture;
	//	
	//}

	///****************************************************************/


	for (int i = 0; i < num_nodes; i++) {
		if (rho(i) < 1e-6) continue;
		DensityMatrix.coeffRef(i, i) =  1/rho(i);
	}

}

void RoughMachining::_solveHeatField(int BoundaryCondition) {

	
	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;


	//this->constNodeNumber = _defineBouindaryNodes();

	//std::cout << "constNodeNumber!!!! " << this->constNodeNumber << std::endl;



	/*Eigen::VectorXd u(num - (this->constNodeNumber));
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(num - (this->constNodeNumber), num - (this->constNodeNumber));
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num - (this->constNodeNumber), num - (this->constNodeNumber));*/


	Eigen::VectorXd u(num_nodes);
	Eigen::SparseMatrix<double> S;

	//S.setZero();
	Eigen::SparseMatrix<double> I = Eigen::SparseMatrix<double>(num_nodes, num_nodes);
	I.setIdentity();

	std::cout << "Initialising u...\n";
	//_initialiseSystemValues1(u,S);
	_initialiseSystemValues2(u,1.0,0.0);
	//return; //test
	Eigen::VectorXd uD(num_nodes);
	Eigen::VectorXd uN(num_nodes);
	uD.setZero();
	uN.setZero();

	/*std::cout << "Weight: " << WeightMatrix << std::endl;
	std::cout << "\nS " << S << std::endl;
	std::cout << "U: \n" << u << std::endl;*/
	
	double t = _getAverageEdgeLength();
	t = t * t;
	std::cout << "t: " << t << std::endl;

	std::cout << "Inversing Volume Matrix...\n";
	_inverseVolumeMatrix();
	_createDensityMatrix(); //comment out later
	DensityMatrix.setIdentity();


	Eigen::SparseMatrix<double> invVol = VolumeMatrix;

	Eigen::SparseMatrix<double> test(num_nodes, num_nodes);
	test.setIdentity();

	double neumann = ((double)BoundaryCondition / 100.0);
	double dirichlet = 1.0 - neumann;
	
	if (BoundaryCondition > 0) {
		std::cout << "Forcing Boundary Conditions..\n";
		for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
			QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
			if (node->in_ConvexHull) continue;


			if (node->isSource) {
				invVol.coeffRef(node->FieldIndexNumber, node->FieldIndexNumber) = 0.0;
			}
			else {
				test.coeffRef(node->FieldIndexNumber, node->FieldIndexNumber) = 0.0;
			}

		}

		std::cout << "Solving Now..\n";
		S = I - 100 * t * invVol * DensityMatrix * WeightMatrix;
		
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
		S = I - 100 * t * invVol * DensityMatrix * WeightMatrix;

		std::cout << "Initialising Sparse Solver..\n";
		Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;// (PardisoLU/SparseLU)
		Solver.analyzePattern(S);
		Solver.factorize(S);
		if (Solver.info() != Eigen::Success)
			std::cout << "error here: factorize fail!" << std::endl;
		Solver.compute(S);
		std::cout << "Sparse Solver Initialised\n";
		uD = Solver.solve(u);
		//uD = u;
		std::cout << "Dirichlet Soln. Complete..\n";
	}



	//u = dec.solve(u);
	//std::cout << "after u:\n" << u << std::endl;
	std::cout << "System solved for initial field...\n";
	//std::cout << u << std::endl;
	double min = 99999, max=-9999;


	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);
		
		if (node->in_ConvexHull) continue;
		
		int i = node->FieldIndexNumber;
		u[i] = (neumann * uN[i]) + (dirichlet * uD[i]);//restore this line; should not be commented out
		//node->HeatFieldValue = 1.0/DensityMatrix.coeff(node->FieldIndexNumber,node->FieldIndexNumber);
		node->HeatFieldValue = u[i];
		node->scalarField = u[i];

		if (node->HeatFieldValue < min) min = node->HeatFieldValue;
		if (node->HeatFieldValue > max) max = node->HeatFieldValue;
		
		
	}




	//for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
	//	QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

	//	if (node->in_ConvexHull) continue;

	//	int i = node->FieldIndexNumber;
	//	u[i] = (u[i] - min) / (max - min);
	//	//node->HeatFieldValue = 1.0/DensityMatrix.coeff(node->FieldIndexNumber,node->FieldIndexNumber);
	//	node->HeatFieldValue = u[i];
	//	node->scalarField = u[i];
	//


	//}

	std::cout << "Max: " << max << "Min: " << min << std::endl;
	std::cout<<"\n\n\n";

	double range = max - min;

	//double min2 = 99999, max2 = -9999;
	//for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
	//	QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);


	//	int i = node->FieldIndexNumber;
	//	node->HeatFieldValue = (u[i] - min) / range;
	//	std::cout << u[i] << std::endl;
	//	if (node->HeatFieldValue < min2) min2 = node->HeatFieldValue;
	//	if (node->HeatFieldValue > max2) max2 = node->HeatFieldValue;


	//}


	//std::cout << "Max: " << max2 << "Min: " << min2 << std::endl;
}


double RoughMachining::_getAverageEdgeLength() {
	std::cout << "Getting average edge length...\n";
	double length = 0;
	double n = materialSpace->GetEdgeNumber();
	double count_ = 0;
	for (GLKPOSITION pos = materialSpace->GetEdgeList().GetHeadPosition(); pos;) {
		QMeshEdge* edge = (QMeshEdge*) materialSpace->GetEdgeList().GetNext(pos);
		if (edge->in_convexHull) continue;
		length += edge->GetLength();
		count_++;
	}

	return length / count_;
}

void RoughMachining::_initialiseSystemValues1(Eigen::VectorXd& u, Eigen::MatrixXd& S) {

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
			//u[node->ReducedFieldIndexNumber] = 1.00;
			continue;
			//node->isBoundary = true;
			
		}
		else {

			u[node->ReducedFieldIndexNumber] = 0.00;
			int j = 0;
			for (int i = 0; i < num; i++) {

				if (!constIndex[i]) {

					S(node->ReducedFieldIndexNumber, j++) = WeightMatrix.coeff(node->FieldIndexNumber, i);
					std::cout << "check value: " << S(node->ReducedFieldIndexNumber, j - 1) << std::endl;
				}
				else {
					u[node->ReducedFieldIndexNumber] -= WeightMatrix.coeff(node->FieldIndexNumber, i);
					std::cout << "check value2: " <<WeightMatrix.coeff(node->ReducedFieldIndexNumber, i) << std::endl;
				}
			}
			//node->isBoundary = false;
		}

		//double xx, yy, zz;

		//node->GetCoord3D(xx, yy, zz);
		//if (yy < (minY + 0.001))
		//{
		//	u[node->FieldIndexNumber] = 1.00;
		//	node->isBoundary = true;
		//}
		///*else if (yy > (maxY - 0.001))
		//{
		//	u[node->FieldIndexNumber] = 0.00;
		//	node->isBoundary = true;
		//}*/
		//else {
		//	u[node->FieldIndexNumber] = 0.00;
		//	node->isBoundary = false;
		//}

		/*if (node->isBoundary) {
			u[node->FieldIndexNumber] = 1.00;
		}
		else {
			u[node->FieldIndexNumber] = 0.00;
		}*/

	}


}


void RoughMachining::_initialiseSystemValues2(Eigen::VectorXd& u, double sourceVal, double otherVal) {

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
				//u[node->FieldIndexNumber] = sourceVal;
				u[node->FieldIndexNumber] = node->HeatFieldValue;
				u[node->FieldIndexNumber] = sourceVal;
			}
			else u[node->FieldIndexNumber] = otherVal;

		}
		else {
			u[node->FieldIndexNumber] = otherVal;
		}
		
	}

}

int RoughMachining::_defineBouindaryNodes() {

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

	double Xrange = (-minX + maxX)/4;
	double Yrange = (-minY + maxY)/4;
	double Zrange = (-minZ + maxZ)/4;


	int num = materialSpace->GetNodeNumber();
	int count = 0;

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);

		if(node->in_ConvexHull) continue;
				
		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		if (/*yy < (minY + 1)  */node->isBoundary)
		{
			node->isBoundary = true;

			constIndex.push_back(true);
			if(isIn) node->isSource = true;
		}
		else if (/*yy < (minY + 0.0001)*//*(abs(xx - Xav) < Xrange) && (abs(yy - Yav) < Yrange) && (abs(zz - Zav) < Zrange)*/ node->ConvexHull_boundary) {
			node->isBoundary = true;

			constIndex.push_back(true);
			//node->isTopBoundary = true;
			if(!isIn) node->isSource = true;
			
		}
		else {
			node->ReducedFieldIndexNumber = count;
			node->isBoundary = false;
			constIndex.push_back(false);
			count++;
		}


		
	}

	return num-count;
}


void RoughMachining::_inverseVolumeMatrix() {
	int num = materialSpace->GetNodeNumber()-Num_nodesInConvexHull;

	for (int i = 0; i < num; i++) {
		if (VolumeMatrix.coeff(i, i) > 1e-10) {
			VolumeMatrix.coeffRef(i, i) = 1.0 / VolumeMatrix.coeff(i, i);
		}
		else {
			std::cout <<"\nVol: " << VolumeMatrix.coeff(i, i) << std::endl;
		}
		
	}

}


void RoughMachining::_generateVectorField() {

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

		if (tet->isIn_convexHull) continue;

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
			if (tet->isOnHull) VertField(i) = node->distanceFromCH;
		}

		Eigen::Vector3d temp = VertCoord.inverse() * T * VertField;
		temp.stableNormalize();



		VectorFieldMatrix(tet->FieldIndexNumber, 0) = -temp[0];
		VectorFieldMatrix(tet->FieldIndexNumber, 1) = -temp[1];
		VectorFieldMatrix(tet->FieldIndexNumber, 2) = -temp[2];

		
		
	}

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		if (tet->isInsideHull) {
			VectorFieldMatrix(tet->FieldIndexNumber, 0) = -VectorFieldMatrix(tet->FieldIndexNumber, 0);
			VectorFieldMatrix(tet->FieldIndexNumber, 1) = -VectorFieldMatrix(tet->FieldIndexNumber, 1);
			VectorFieldMatrix(tet->FieldIndexNumber, 2) = -VectorFieldMatrix(tet->FieldIndexNumber, 2);
		}


	}

	std::cout << "Smoothing Vector Field....\n";
	_smoothVectorField();
}
	
void RoughMachining::_generateScalarField() {

	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;
	Eigen::VectorXd divVector = Eigen::VectorXd::Zero(num_nodes);
	_generateDivergence(divVector);
	//_generateDivergence(divVector);

	Eigen::SparseMatrix<double> invVol = VolumeMatrix;
	Eigen::SparseMatrix<double> boundaryCompensator(num_nodes, num_nodes);
	boundaryCompensator.setIdentity();
	std::cout << "Solving Scalar System...\n";
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);

		if (node->in_ConvexHull) continue;

		int ii = node->FieldIndexNumber;

		if (false/*node->isBoundary &&*/ /*node->isSource*/ /*&& node->distanceFromCH >= 0*/) {
			invVol.coeffRef(ii, ii) = 0.0;
			divVector(ii) = (node->distanceFromCH);
		}
		else {
			invVol.coeffRef(ii, ii) = 1.0;
			boundaryCompensator.coeffRef(ii, ii) = 0;
		}
	}
	/*Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(WeightMatrix);
	Eigen::VectorXd u = dec.solve(divVector);*/

	Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;// (PardisoLU/SparseLU)


	//Eigen::SparseMatrix<double> S = WeightMatrix;
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

	double max = -999999, min = 999999;

	//for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
	//	QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
	//	if (!tet->isOnHull) continue;
	//	for (int l = 1; l <= 4; l++) {
	//		QMeshNode* node = tet->GetNodeRecordPtr(l);
	//		int i = node->FieldIndexNumber;
	//		node->HeatFieldValue = node->distanceFromCH;
	//	}
	//}


	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

		if (node->in_ConvexHull) continue;

		int i = node->FieldIndexNumber;
		node->HeatFieldValue = u[i];
		if (u[i] > max) max = u[i];
		if (u[i] < min) min = u[i];
		node->scalarField = node->HeatFieldValue;
	}


	double inMax = -9999999;
	QMeshNode* MaxNode = NULL;

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		if (!tetra->isInsideHull) continue;



		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = tetra->GetNodeRecordPtr(i);
			if (node->HeatFieldValue > inMax) {
				inMax = node->HeatFieldValue;
				MaxNode = node;
			}
		}
	}

	double hullValue = inMax + abs(MaxNode->distanceFromCH);



	/*for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		if (tetra->isInsideHull) continue;



		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = tetra->GetNodeRecordPtr(i);

			node->HeatFieldValue = node->distanceFromCH ;
			node->scalarField = node->HeatFieldValue;
		}
	}*/



	max_distance = max;
	min_distance = min;

	std::cout << "Max Distance: " << max_distance << std::endl;



	double max2 = -999999, min2 = 99999;
	/*for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

		if (node->in_ConvexHull) continue;

		int i = node->FieldIndexNumber;
		if (isIn) node->HeatFieldValue = -(u[i] - max) / (max - min);
		else node->HeatFieldValue = (u[i] - min) / (max - min);

		if (node->HeatFieldValue > max2) max2 = node->HeatFieldValue;
		if (node->HeatFieldValue < min2) min2 = node->HeatFieldValue;

		node->scalarField = node->HeatFieldValue;
	}*/


	/*std::ofstream fieldfile;
	fieldfile.open("FieldFile.txt");

	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

		double xp, yp, zp;
		double xp, yp, zp;
		node->GetCoord3D(xp, yp, zp);
		fieldfile << xp << " " << yp << " " << zp << " " << node->scalarField << "\n";
	}

	fieldfile.close();*/


	//std::cout << " MAX: " << max << " MIN: " << min << std::endl;
	//std::cout << "Final MAX: " << max2 << "Final MIN: " << min2 << std::endl;



}

void RoughMachining::_generateDivergence(Eigen::VectorXd& divVector) {

	std::cout << "Generating Divergence Field\n";

	for (GLKPOSITION pos = materialSpace->GetTetraList().GetHeadPosition(); pos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(pos);

		if (tet->isIn_convexHull) continue;

		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = (QMeshNode*)tet->GetNodeRecordPtr(i);
			int node_index = node->FieldIndexNumber;
			int tet_index = tet->FieldIndexNumber;
			//int _facenumber = 999;

			double Sk;
			Eigen::Vector3d nk;

			nk = _getOppositeNormalWithMag(node, tet);
			Sk = 0.5*abs(nk.stableNorm());
			nk.stableNormalize();


			/*switch(i) {
			case(1): _facenumber = 2;
			case(2): _facenumber = 3;
			case(3): _facenumber = 4;
			case(4): _facenumber = 1;
			}*/

			Eigen::Vector3d grad_vec(VectorFieldMatrix(tet_index, 0), VectorFieldMatrix(tet_index, 1), VectorFieldMatrix(tet_index, 2));
			//grad_vec.stableNormalize(); //Uncomment this

			//QMeshFace* face = tet->GetFaceRecordPtr(_facenumber);
			
			/*face->CalPlaneEquation();
			face->CalArea();*/
		//	std::cout << "area: " << face->GetArea() << std::endl;


			

			divVector[node_index] -= Sk * nk.dot(grad_vec) / 3;

		}
	}

	
}

Eigen::Vector3d RoughMachining::_getOppositeNormalWithMag(QMeshNode* node, QMeshTetra* tet) {
	QMeshNode* otNode1=NULL, * otNode2=NULL, * otNode3=NULL;
	int node_index_in_Tet = tet->GetNodeIndex(node);

	for (int i = 1; i <= 4; i++) {
		if (i != node_index_in_Tet) {
			if (otNode1 == NULL) {
				otNode1 = tet->GetNodeRecordPtr(i);
			}
			else if (otNode2 == NULL) {
				otNode2 = tet->GetNodeRecordPtr(i);
			}
			else if(otNode3 == NULL) {
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

void RoughMachining::_cal_heatValue(PolygenMesh* MaterialMesh) {

	materialSpace = (QMeshPatch*)MaterialMesh->GetMeshList().GetHead();

	this->_index_initial(materialSpace, true);

	double boundingBox[6];
	materialSpace->ComputeBoundingBox(boundingBox);
	double minY = boundingBox[2];
	double maxY = boundingBox[3];
	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);
		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		if (yy < (minY + 0.001)){
			node->isBoundary = true;
		}
		else if (yy > (maxY - 0.001)) {
			node->isTopBoundary = true;
			node->isBoundary = true;
		}
		else {
			node->isBoundary = false;
		}
	}

	int tempInd = 0;
	//get the number of node which is not on the boundary 
	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);

		//std::cout << "FieldIndexNumber " << node->FieldIndexNumber << std::endl;

		if (node->isBoundary)  continue;
				
		node->ind_hardContraint_heat = tempInd;
		//std::cout << "ind_hardContraint_heat " <<  node->ind_hardContraint_heat << std::endl;
		tempInd++;
	}
	int nodeNum = tempInd;



	Eigen::SparseMatrix<double> Parameter(nodeNum, nodeNum); //A
	std::cout << "A matrix size is " << nodeNum << "," << nodeNum << std::endl;

	Eigen::VectorXd guideField(nodeNum); //x

	Eigen::VectorXd b(nodeNum); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);

		if (node->isBoundary) continue;

		//get the neighbor node of each Node
		std::vector<QMeshNode*> connectNode_set;
		for (GLKPOSITION Pos = node->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* ConnectEdge = (QMeshEdge*)node->GetEdgeList().GetNext(Pos);

			QMeshNode* ConnectNode = ConnectEdge->GetStartPoint();
			if (ConnectNode == node) ConnectNode = ConnectEdge->GetEndPoint();

			connectNode_set.push_back(ConnectNode);
		}
	
		ParaTriplet.push_back(Eigen::Triplet<double>(node->ind_hardContraint_heat, node->ind_hardContraint_heat, -1.0)); // infill A
		for (int i = 0; i < connectNode_set.size();i++) {
			QMeshNode* ConnectNode = connectNode_set[i];
			if (!ConnectNode->isBoundary) {
				ParaTriplet.push_back(Eigen::Triplet<double>(node->ind_hardContraint_heat, ConnectNode->ind_hardContraint_heat, 1.0 / connectNode_set.size()));
			}
			else {
				double c = node->isTopBoundary ? 0.00 : 1.00;
				b(node->ind_hardContraint_heat) -= (1.0*c) / connectNode_set.size();
			}
		}
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());


	//std::cout << "Parameter:\n" << Parameter << std::endl;
	//std::cout << "b:\n" << b << std::endl;

	std::printf("------------------------------------------------\n");
	long time = clock();

	Eigen::SparseMatrix<double> ATA(nodeNum, nodeNum);
	ATA = Parameter.transpose() * Parameter;
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;// (PardisoLU/SparseLU)

	Solver.analyzePattern(ATA);
	Solver.factorize(ATA);
	if (Solver.info() != Eigen::Success)
		std::cout << "error here: factorize fail!" << std::endl;
	Solver.compute(ATA);


	Eigen::VectorXd ATb(nodeNum);
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);
	std::cout << "Soln: \n" << guideField << std::endl;

	std::printf("------------------------------------------------\n");
	std::printf("--> Solve takes %ld ms.\n", clock() - time);
	std::printf("------------------------------------------------\n");

	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < nodeNum; i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;
	//std::cout << " " << range << " " << minPhi << " " << maxPhi << std::endl;

	Eigen::VectorXd guideFieldNormalize(nodeNum);
	for (int i = 0; i < nodeNum; i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);

		if (!Node->isBoundary) {

			Node->scalarField = guideFieldNormalize(Node->ind_hardContraint_heat);
			Node->HeatFieldValue = Node->scalarField;
			//std::cout << Node->HeatFieldValue << std::endl;
			Node->scalarField_init = guideField(Node->ind_hardContraint_heat);
		}
		else {
			if (Node->isTopBoundary) {
				Node->scalarField = 1.00;
				Node->HeatFieldValue = Node->scalarField;
				//std::cout << Node->HeatFieldValue << std::endl;

				Node->scalarField_init = guideField(Node->ind_hardContraint_heat);
			}
			else {
				Node->scalarField = 0.00;
				Node->HeatFieldValue = Node->scalarField;
				//std::cout << Node->HeatFieldValue << std::endl;
				Node->scalarField_init = guideField(Node->ind_hardContraint_heat);
			}
		}
	}
}

void RoughMachining::_cal_heatValue1(PolygenMesh* MaterialMesh) {

	materialSpace = (QMeshPatch*)MaterialMesh->GetMeshList().GetHead();

	this->_index_initial(materialSpace, true);

	double boundingBox[6];
	materialSpace->ComputeBoundingBox(boundingBox);
	double minY = boundingBox[2];
	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);
		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		int count_ = 0;
		if (node->isBoundary) {
			node->isBoundary = true;
		}
		else {
			count_++;
			node->isBoundary = false;
		}
	}
	int nodeNum = materialSpace->GetNodeNumber();

	Eigen::SparseMatrix<double> Parameter(nodeNum, nodeNum); //A
	std::cout << "The matrix size is " << nodeNum << "," << nodeNum << std::endl;

	Eigen::VectorXd guideField(nodeNum); //x

	Eigen::VectorXd b(nodeNum); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);

		//get the neighbor node of each Node
		std::vector<QMeshNode*> connectNode_set;
		for (GLKPOSITION Pos = node->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* ConnectEdge = (QMeshEdge*)node->GetEdgeList().GetNext(Pos);

			QMeshNode* ConnectNode = ConnectEdge->GetStartPoint();
			if (ConnectNode == node) ConnectNode = ConnectEdge->GetEndPoint();

			connectNode_set.push_back(ConnectNode);
		}

		ParaTriplet.push_back(Eigen::Triplet<double>(node->FieldIndexNumber, node->FieldIndexNumber, -1.0)); // infill A
		for (int i = 0; i < connectNode_set.size(); i++) {
			QMeshNode* ConnectNode = connectNode_set[i];
			
			ParaTriplet.push_back(Eigen::Triplet<double>(node->FieldIndexNumber, ConnectNode->FieldIndexNumber, 1.0 / connectNode_set.size()));
		}
		if(node->isBoundary) b(node->FieldIndexNumber) = 1.0;
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> Vi(materialSpace->GetNodeNumber(), materialSpace->GetNodeNumber());
	Eigen::MatrixXd V = Eigen::MatrixXd::Zero(materialSpace->GetNodeNumber(), materialSpace->GetNodeNumber());
	

	Eigen::SparseMatrix<double> I(materialSpace->GetNodeNumber(), materialSpace->GetNodeNumber());

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		tet->CalVolume();
		double vol = tet->GetVolume();
		for (int i = 1; i <= 4; i++) {
			QMeshNode* _node = tet->GetNodeRecordPtr(i);
			V(_node->FieldIndexNumber, _node->FieldIndexNumber) += vol/4;
			
		}
	}
	for (int i = 0; i < materialSpace->GetNodeNumber(); i++) {
		if (V(i, i) > 0.0000000001) {
			V(i, i) = 1.0 / V(i, i);
		}
		else {
			V(i, i) = 10000000;
		}
	}

	for (int i = 0; i < materialSpace->GetNodeNumber(); i++) {
		Vi.insert(i, i) = V(i, i);
		I.insert(i, i) = 1.0;
	}


	double len = 0;
	for (GLKPOSITION ePos = materialSpace->GetEdgeList().GetHeadPosition(); ePos;) {
		QMeshEdge* _edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(ePos);
		_edge->CalLength();
		len += _edge->GetLength();
	}

	len = len / materialSpace->GetEdgeNumber();
	

	Parameter = Vi - pow(len, 2) * Parameter;

	//std::cout << "Parameter:\n" << Parameter << std::endl;
	//std::cout << "b:\n" << b << std::endl;

	std::printf("------------------------------------------------\n");
	long time = clock();

	Eigen::SparseMatrix<double> ATA(nodeNum, nodeNum);
	ATA = Parameter.transpose() * Parameter;
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;// (PardisoLU/SparseLU)

	Solver.analyzePattern(ATA);
	Solver.factorize(ATA);
	if (Solver.info() != Eigen::Success)
		std::cout << "error here: factorize fail!" << std::endl;
	Solver.compute(ATA);


	Eigen::VectorXd ATb(nodeNum);
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);
	std::cout << "Soln: \n" << guideField << std::endl;

	std::printf("------------------------------------------------\n");
	std::printf("--> Solve takes %ld ms.\n", clock() - time);
	std::printf("------------------------------------------------\n");

	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < nodeNum; i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;
	//std::cout << " " << range << " " << minPhi << " " << maxPhi << std::endl;

	Eigen::VectorXd guideFieldNormalize(nodeNum);
	for (int i = 0; i < nodeNum; i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);

		if (!Node->isBoundary) {

			Node->scalarField = guideFieldNormalize(Node->ind_hardContraint_heat);
			Node->HeatFieldValue = Node->scalarField;
			//std::cout << Node->HeatFieldValue << std::endl;
			Node->scalarField_init = guideField(Node->ind_hardContraint_heat);
		}
	}
}



void RoughMachining::createFixtureField() {

	
	this->initialIndex();
	this->_createWeightMatrix();
	this->PrepareForHeatMethod();
	this->_solveHeatField(50);
	/*this->_generateVectorField();
	this->_generateScalarField();*/
	return;

	/* PQP compute distance */
	PQP_Model* pqpModel_CH = new PQP_Model();
	pqpModel_CH->BeginModel();  int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];





	findBoundary();
	preHeatMethod2();

	for (GLKPOSITION Pos = fixtureSurface->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)fixtureSurface->GetFaceList().GetNext(Pos);

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel_CH->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel_CH->EndModel();


	/* compute the distance from Fixture*/

	double distance = 999999.99;
	double max = -9.99e10;
	double min = 9.99e10;

	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);


		//if (Node->distanceFromCH <= 0) continue;
		if (Node->in_ConvexHull) continue;

		PQP_DistanceResult dres;	dres.last_tri = pqpModel_CH->last_tri;
		PQP_REAL p[3];

		Node->GetCoord3D(p[0], p[1], p[2]);


		PQP_Distance(&dres, pqpModel_CH, p, 0.0, 0.0);

		float closestPt[3];	// closest point
		closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];

		int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero


		QMeshFace* closestFace;
		int faceIndex = 0;
		for (GLKPOSITION _Pos = fixtureSurface->GetFaceList().GetHeadPosition(); _Pos;) {
			QMeshFace* Face = (QMeshFace*)fixtureSurface->GetFaceList().GetNext(_Pos);
			if (faceIndex == minTriId) {
				closestFace = Face;
				break;
			}
			faceIndex++;
		}
		
		

		Eigen::Vector3d faceNormal; double d;
		closestFace->CalPlaneEquation();
		closestFace->GetPlaneEquation(faceNormal[0], faceNormal[1], faceNormal[2], d);

		Eigen::Vector3d nodePos, closestPos;
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
		for (int i = 0; i < 3; i++) closestPos[i] = closestPt[i];

		Node->distanceFromFixture = dres.Distance();
		double cProd = faceNormal.dot(nodePos - closestPos);
		if (cProd < 0) { 
			
			Node->distanceFromFixture = -abs(Node->distanceFromFixture); 
		}
		else {
			if (max < Node->distanceFromFixture) max = Node->distanceFromFixture;
			else if (min > Node->distanceFromFixture) min = Node->distanceFromFixture;
		}
	
	}

	for (GLKPOSITION NodePos = materialSpace->GetNodeList().GetHeadPosition(); NodePos; ) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(NodePos);

		if (node->distanceFromFixture > 0) {
			node->distanceFromFixture = (node->distanceFromFixture-min) / (max-min);
			//std::cout << node->distanceFromFixture << std::endl;
		}
		else {
			node->distanceFromFixture = 1.0;
		}
	}

	


	
	double max2 = -1e10;
	double min2 = 1e10;
	for (GLKPOSITION NodePos = materialSpace->GetNodeList().GetHeadPosition(); NodePos; ) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(NodePos);

		/*if (node->scalarField > -9999 && node->distanceFromFixture > 0) {
			if ((node->scalarField > node->distanceFromFixture) && node->distanceFromFixture<0.4) node->scalarField = node->distanceFromFixture;
		}*/

		/*if (node->scalarField > -9999 && node->distanceFromFixture > 0) {
			double a = abs(node->distanceFromFixture);
			a = (1 - exp(-1 * a)) / (1 - exp(-1));
			double b = abs(node->scalarField);
			double alpha = 1 / (a + 1e-5);

			node->scalarField = ((a) * node->scalarField + (1-a) * node->distanceFromFixture);
			node->HeatFieldValue = node->scalarField;
		}
		else {
			node->scalarField = -9999;
		}*/


		/*if (!node->in_ConvexHull) {
			double a = abs(node->distanceFromFixture);
			a = (1 - exp(-1 * a)) / (1 - exp(-1));
			double b = abs(node->scalarField);
			double alpha = 1 / (a + 1e-5);

			node->scalarField = ((a)*node->scalarField + (1 - a) * node->distanceFromFixture);
			node->HeatFieldValue = node->scalarField;
			if (node->HeatFieldValue > max2) max2 = node->HeatFieldValue;
			if (node->HeatFieldValue < min2) min2 = node->HeatFieldValue;
		}
		else {
			node->HeatFieldValue = -9999;
			node->scalarField = -9999;
		}*/

		if (!node->in_ConvexHull) {
			double a = abs(node->distanceFromFixture);
			//a = (1 - exp(-1 * a)) / (1 - exp(-1));
			double b = abs(node->scalarField);
			double alpha = 1 / (a + 1e-5);

			node->scalarField = (a*b)/(a+b);
			node->HeatFieldValue = node->scalarField;
			if (node->HeatFieldValue > max2) max2 = node->HeatFieldValue;
			if (node->HeatFieldValue < min2) min2 = node->HeatFieldValue;
		}
		else {
			node->HeatFieldValue = -9999;
			node->scalarField = -9999;
		}

		/*if (!node->in_ConvexHull) {
			node->scalarField = 0.5;
		}
		else {
			node->scalarField = -9999;
		}*/
		
	}


	/*for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		
		if (node->scalarField <= -9999) continue;

		node->scalarField = (node->scalarField - min2) / (max2 - min2);
	}*/


	//std::cout << materialSpace->GetNodeNumber() << std::endl;
	//std::cout << Num_nodesInConvexHull << std::endl;
	
	this->initialIndex();
	this->_createWeightMatrix();
	this->_generateVectorField();
	this->_generateScalarField(); 


}



/*This function is to find nodes to assign boundary points*/
void RoughMachining::findBoundary() {
	for (GLKPOSITION edgePos = materialSpace->GetEdgeList().GetHeadPosition(); edgePos;) {
		QMeshEdge* edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(edgePos);
		QMeshNode* node1 = edge->GetStartPoint();
		QMeshNode* node2 = edge->GetEndPoint();

		if (node1->scalarField == -9999 && node2->scalarField > -9999) {
			node1->isSource = true;
			node2->isSource = false;
			
		}
		else if(node2->scalarField == -9999 && node1->scalarField > -9999){
			node1->isSource = true;
			node2->isSource = false;

		}
		else {
			node1->isSource = false;
			node2->isSource = false;
		}
	}
}


/*This function is to reassign all the tets to the domain for Heat Method/Eikonal Constraint*/
void RoughMachining::preHeatMethod2() {
	
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		bool isThisIn = false;
		int inCount = 0;
		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = tet->GetNodeRecordPtr(i);
			if (node->distanceFromCH < 0) {
				//isThisIn=true;
				//break;
				inCount++;
			}


		}

		if (inCount == 4) isThisIn = true;


		if (isThisIn) {
			tet->isIn_convexHull = true;
			Num_tetInConvexHull++;
			for (int i = 1; i <= 6; i++) {
				QMeshEdge* edge = tet->GetEdgeRecordPtr(i);
				edge->in_convexHull = true;

			}
			for (int i = 1; i <= 4; i++) {
				QMeshFace* face = tet->GetFaceRecordPtr(i);
				face->in_convexHull = true;

			}
			for (int i = 1; i <= 4; i++) {
				QMeshNode* node = tet->GetNodeRecordPtr(i);
				if (!node->in_ConvexHull) {
					Num_nodesInConvexHull++;
					node->in_ConvexHull = true;
				}

			}
			
		}

		else {
			for (int i = 1; i <= 4; i++) {
				QMeshNode* node = tet->GetNodeRecordPtr(i);
				if (node->scalarField==-9999) {
					node->scalarField = node->distanceFromCH;
				}

			}

		}
	}


	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (tet->isIn_convexHull) continue;

		/*int inCount = 0;
		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = tet->GetNodeRecordPtr(i);
			if (node->in_ConvexHull) {
				inCount++;
			}


		}*/

		if (true) {
			for (int i = 1; i <= 6; i++) {
				QMeshEdge* edge = tet->GetEdgeRecordPtr(i);
				edge->in_convexHull = false;

			}
			for (int i = 1; i <= 4; i++) {
				QMeshFace* face = tet->GetFaceRecordPtr(i);
				face->in_convexHull = false;

			}
			for (int i = 1; i <= 4; i++) {
				QMeshNode* node = tet->GetNodeRecordPtr(i);
				if (node->in_ConvexHull) {
					Num_nodesInConvexHull--;
					node->in_ConvexHull = false;
				}
				node->scalarField = node->distanceFromCH;

			}

		}
	}


}


/*This function is to reassign all the tets to the domain for Heat Method/Eikonal Constraint*/
void RoughMachining::preHeatMethod3() {

	return;

}



void RoughMachining::PrepareForHeatMethod() {
	std::ofstream myFile2;
	myFile2.open("wow_index.txt");

	for (GLKPOSITION edgePos = materialSpace->GetEdgeList().GetHeadPosition(); edgePos;) {
		QMeshEdge* edge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(edgePos);
		
		QMeshNode* node1 = (QMeshNode*)edge->GetStartPoint();
		QMeshNode* node2 = (QMeshNode*)edge->GetEndPoint();

		if ((node1->distanceFromCH >= 0 && node2->distanceFromCH <= 0) || (node1->distanceFromCH <= 0 && node2->distanceFromCH >= 0)) {
			if (abs(node1->distanceFromCH) > abs(node2->distanceFromCH)) {
				node2->isSource = true;
				node2->isBoundary = true;
				node2->HeatFieldValue = 1;// node2->distanceFromCH;
				
				myFile2 << node2->FieldIndexNumber << std::endl;
			}
			else {
				node1->isSource = true;
				node1->isBoundary = true;
				node1->HeatFieldValue = 1; //node1->distanceFromCH;
				
				myFile2 << node1->FieldIndexNumber << std::endl;
			}
		}
		if(abs(node1->distanceFromCH)<1e-3){
			node1->isSource = true;
			node1->isBoundary = true;
			node1->HeatFieldValue = 1; //node1->distanceFromCH;

			myFile2 << node1->FieldIndexNumber << std::endl;
		}
		if (abs(node2->distanceFromCH) < 1e-3) {
			node2->isSource = true;
			node2->isBoundary = true;
			node2->HeatFieldValue = 1;// node2->distanceFromCH;

			myFile2 << node2->FieldIndexNumber << std::endl;
		}
	}
	

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		int inCount = 0;
		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = tet->GetNodeRecordPtr(i);
			if (node->distanceFromCH < 0) inCount++;
		}

		
		if (inCount == 4) tet->isInsideHull = true;
		else if (inCount > 0) tet->isOnHull = true;
	}


	double mins[6];
	materialSpace->ComputeBoundingBox(mins);
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		double x, y, z;
		if (node->distanceFromCH > 0) continue;
		
		node->GetCoord3D(x, y, z);
		
		if (node->isBoundary) {
			if (x<mins[0] + 1e-4 || x>mins[1] - 1e-4) {
				node->isSource = true;
				node->isBoundary = true;
				node->HeatFieldValue = 1;// node2->distanceFromCH;

				myFile2 << node->FieldIndexNumber << std::endl;
			}
			else if (y<mins[2] + 1e-4 || y>mins[3] - 1e-4) {
				node->isSource = true;
				node->isBoundary = true;
				node->HeatFieldValue = 1;// node2->distanceFromCH;

				myFile2 << node->FieldIndexNumber << std::endl;
			}
		}
	}


	myFile2.close();
}


void RoughMachining::_buildTetLaplacianMatrix() {
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;


	Eigen::SparseMatrix<double> diagonalMultiplier;


	TetLaplacianMatrix.resize(num_tets + pivotNumber, num_tets);
	TetLaplacianMatrix.setZero();
	TetLaplacianMatrix.reserve(Eigen::VectorXd::Constant(num_tets + pivotNumber, 10));

	diagonalMultiplier.resize(num_tets + pivotNumber, num_tets + pivotNumber);
	//diagonalMultiplier.reserve(Eigen::VectorXd::Constant(num_tets + pivotNumber, 2));
	diagonalMultiplier.setIdentity();
	//TetLaplacianMatrix.setIdentity();




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




	for (int l = 0; l < num_tets; l++) {
		diagonalMultiplier.coeffRef(l, l) = 1 / TetLaplacianMatrix.coeff(l, l);
	}





	int pivotInsertCount = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (!tetra->isPivot) continue;
		int i = tetra->FieldIndexNumber;

		int indexx = pivotInsertCount + num_tets;
		//TetLaplacianMatrix.insert(indexx, i) = 0.001;
		if (tetra->isPivot) { 
			if (tetra->isPivotA) {
				TetLaplacianMatrix.coeffRef(indexx, i) = pow(10, propagateMag);
			}
		else if (tetra->isPivotB)
			TetLaplacianMatrix.coeffRef(indexx, i) = 500e4;
		else
			TetLaplacianMatrix.coeffRef(indexx, i) = 500e4;
		}
		pivotInsertCount++;
	}

	TetLaplacianMatrix = diagonalMultiplier * TetLaplacianMatrix;

}

void RoughMachining::_buildTetLaplacianMatrixPreserved()
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;


	Eigen::SparseMatrix<double> diagonalMultiplier;

	
	TetLaplacianMatrix.resize(num_tets*2, num_tets);
	TetLaplacianMatrix.setZero();
	TetLaplacianMatrix.reserve(Eigen::VectorXd::Constant(num_tets*2, 10));

	diagonalMultiplier.resize(num_tets*2, num_tets*2);
	//diagonalMultiplier.reserve(Eigen::VectorXd::Constant(num_tets + pivotNumber, 2));
	diagonalMultiplier.setIdentity();
	//TetLaplacianMatrix.setIdentity();




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




	for (int l = 0; l < num_tets; l++) {
		diagonalMultiplier.coeffRef(l, l) = 1 / TetLaplacianMatrix.coeff(l, l);
	}





	int pivotInsertCount = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		//if (!tetra->isPivot) continue;
		int i = tetra->FieldIndexNumber;

		int indexx = i + num_tets;
		//TetLaplacianMatrix.insert(indexx, i) = 0.001;
		if (tetra->isPivot) {
			if (tetra->isPivotA)
				TetLaplacianMatrix.coeffRef(indexx, i) = 1;
			if (tetra->isPivotB)
				TetLaplacianMatrix.coeffRef(indexx, i) = 0.01;
			else
				TetLaplacianMatrix.coeffRef(indexx, i) = 0.01;
		}
		else {
			TetLaplacianMatrix.coeffRef(indexx, i) = 0.01;
		}
		pivotInsertCount++;
	}

	TetLaplacianMatrix = diagonalMultiplier * TetLaplacianMatrix;
}



void RoughMachining::_smoothVectorField() {
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
	std::cout << "Smoothing 4\n";
	v3 = TetLaplacianMatrix.transpose() * v3;
	v3_sol = SolverN.solve(v3);

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		int index = tetra->FieldIndexNumber;
		Eigen::Vector3d newNormal(v0_sol(index), v1_sol(index), v2_sol(index));
		newNormal.stableNormalize();
		newNormal = newNormal * v3_sol(index);

		VectorFieldMatrix(index, 0) = newNormal.x();
		VectorFieldMatrix(index, 1) = newNormal.y();
		VectorFieldMatrix(index, 2) = newNormal.z();
	}

}

void RoughMachining::_classifyBoundary()
{
	std::cout << "Classifying boundary...\n";
	//need to be accelerated
	std::vector<QMeshNode*> nodeVector;
	int nodeNums = materialSpace->GetNodeNumber();
	nodeVector.reserve(nodeNums);

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos; ) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		if (!thisNode->isBoundary)  continue;
		nodeVector.push_back(thisNode);
	}

	int total_thread = omp_get_max_threads();
	
#pragma omp parallel for num_threads(total_thread-2)
	for (int i = 0; i<nodeVector.size(); i++) {
		QMeshNode* thisNode = nodeVector[i];
		if (!thisNode->isBoundary)  continue;
		double xx, yy, zz;
		thisNode->GetCoord3D(xx, yy, zz);
		bool isIn = _eleCenter_in_convexHull2(xx, yy, zz);

		thisNode->isInteriorBoundary = isIn;
	}

	/*for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos; ) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		if (!thisNode->isBoundary)  continue;
		double xx, yy, zz;
		thisNode->GetCoord3D(xx, yy, zz);
		bool isIn = _eleCenter_in_convexHull2(xx, yy, zz);
		
		thisNode->isInteriorBoundary = isIn;
	}*/

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos; ) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		if (!thisNode->isBoundary)  continue;
		double xx, yy, zz;
		thisNode->GetCoord3D(xx, yy, zz);
		if (yy < 2e-1) { //Need to update this to lower bounding box plane
			thisNode->isBoundary = false;
			thisNode->isInteriorBoundary = false;
			thisNode->isSupressedBoundary = true;
		}
	}

	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* thisFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);
		if (!thisFace->isBoundary) continue;
		int boundCount = 0;
		int inBoundCount = 0;
		for (int i = 0; i < 3; i++) {
			QMeshNode* faceNode = thisFace->GetNodeRecordPtr(i);
			if (faceNode->isBoundary) {
				boundCount++;
				if (faceNode->isInteriorBoundary) {
				
					inBoundCount++;
				}
			}
		}

		if (boundCount == 3) {
			if (/*true*/inBoundCount == 3) {
				thisFace->isInteriorBoundary = true;
				thisFace->CalPlaneEquation();
				Eigen::Vector3d norm1, norm2;
				thisFace->GetNormal(norm1(0), norm1(1), norm1(2));
				/*if (norm1(1) < -0.99) */
				/*norm2 << 0.0, -1.0, 0.0;
				norm1.stableNormalize();
				if (norm1.dot(norm2) > 0.3) {
					thisFace->isInteriorBoundary = true;
				}*/

				/*if (norm1.y() > 0.99) continue;
				if (abs(norm1.x()) > 0.99) continue;
				if (abs(norm1.z()) > 0.99) continue;*/
				//thisFace->isInteriorBoundary = true;
			}
			/*else if(inBoundCount ==0){
				thisFace->CalPlaneEquation();
				Eigen::Vector3d norm1, norm2;
				thisFace->GetNormal(norm1(0), norm1(1), norm1(2));
				if (norm1.y() > 0.99) {
					thisFace->isBoundary = true;
					
				}
				else {
					thisFace->isBoundary = false;
				}

			}
			else {
				thisFace->isBoundary = false;
			}*/
		}
		else {
			thisFace->isBoundary = false;
		}


		}
	buildNewTetLaplacian = true;
}

void RoughMachining::_initiateVectorOnBound(bool setInnerBound, bool setouterBound, bool convexHullGuess)
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;

	VectorFieldMatrix = Eigen::MatrixXd(num_tets, 3);

	if (setInnerBound) {
		std::cout << "Initiating field on inner boundary!\n";
		_initiateVectorOnInteriorBoundTets();
	}

	if (setouterBound) {
		std::cout << "Initiating field on outer boundary!\n";
		_initiateVectorOnExteriorBoundTets();

	}

	if (convexHullGuess) {
		std::cout << "Initiating field on convex hull boundary!\n";
		_initiateVectorOnConvexHull();
	}
}

void RoughMachining::_initiateVectorOnInteriorBoundTets()
{
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* thisFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);
		if (!thisFace->isInteriorBoundary) continue;
		QMeshTetra* thisTet = thisFace->GetLeftTetra();
		if (!thisTet) thisTet = thisFace->GetRightTetra();
		
		double nx, ny, nz;
		thisFace->CalPlaneEquation();
		thisFace->GetNormal(nx, ny, nz);
		Eigen::Vector3d normalDir(nx, ny, nz);
		normalDir.stableNormalize();
		

		VectorFieldMatrix(thisTet->FieldIndexNumber, 0) = -normalDir.x();
		VectorFieldMatrix(thisTet->FieldIndexNumber, 1) = -normalDir.y();
		VectorFieldMatrix(thisTet->FieldIndexNumber, 2) = -normalDir.z();

		if(!thisTet->isPivot) pivotNumber++;
		thisTet->isPivot = true;
		
	}
}

void RoughMachining::_initiateVectorOnExteriorBoundTets()
{
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* thisFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);
		if (!thisFace->isBoundary) continue;
		if (thisFace->isInteriorBoundary) continue;

		QMeshTetra* thisTet = thisFace->GetLeftTetra();
		if (!thisTet) thisTet = thisFace->GetRightTetra();

		double nx, ny, nz;
		thisFace->CalPlaneEquation();
		thisFace->GetNormal(nx, ny, nz);
		Eigen::Vector3d normalDir(nx, ny, nz);
		normalDir.stableNormalize();
		if (abs((normalDir.x()) < 0.99)) continue;

		VectorFieldMatrix(thisTet->FieldIndexNumber, 0) = normalDir.x();
		VectorFieldMatrix(thisTet->FieldIndexNumber, 1) = normalDir.y();
		VectorFieldMatrix(thisTet->FieldIndexNumber, 2) = normalDir.z();

		if (!thisTet->isPivot) {
			pivotNumber++;
		}

		thisTet->isPivot = true;
		thisTet->isPivotB = true;
		
	}
}

void RoughMachining::_initiateVectorOnConvexHull()
{
	PQP_Model* pqpModel_CH = new PQP_Model();
	pqpModel_CH->BeginModel();  int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];

	for (GLKPOSITION Pos = convexHull->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(Pos);

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel_CH->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel_CH->EndModel();

	std::vector<QMeshNode*> nodeList;
	int nodeNum = materialSpace->GetNodeList().GetCount();
	nodeList.reserve(nodeNum);

	for (GLKPOSITION Pos = materialSpace->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)materialSpace->GetNodeList().GetNext(Pos);
		nodeList.push_back(Node);
	}

#pragma omp parallel for
	for (int ii = 0; ii < nodeNum; ii++) {
		QMeshNode* Node = (QMeshNode*)nodeList[ii];

		PQP_DistanceResult dres;	dres.last_tri = pqpModel_CH->last_tri;
		PQP_REAL p[3];

		Node->GetCoord3D(p[0], p[1], p[2]);


		PQP_Distance(&dres, pqpModel_CH, p, 0.0, 0.0);

		float closestPt[3];	// closest point
		closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];

		int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero


		QMeshFace* cloestFace;
		int faceIndex = 0;
		for (GLKPOSITION _Pos = convexHull->GetFaceList().GetHeadPosition(); _Pos;) {
			QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(_Pos);
			if (faceIndex == minTriId) {
				cloestFace = Face;
				break;
			}
			faceIndex++;
		}

		Eigen::Vector3d faceNormal; double d;
		cloestFace->CalPlaneEquation();
		cloestFace->GetPlaneEquation(faceNormal[0], faceNormal[1], faceNormal[2], d);

		Eigen::Vector3d nodePos, cloestPos;
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
		for (int i = 0; i < 3; i++) cloestPos[i] = closestPt[i];
		Eigen::Vector3d pointDir = 100 * (Eigen::Vector3d(p[0], p[1], p[2]) - Eigen::Vector3d(closestPt[0], closestPt[1], closestPt[2]));
		pointDir.stableNormalize();
		double _orient = pointDir.dot(faceNormal);
		if (_orient < 0) {
			Node->isIn_convexHullTest = true;
		}
	}

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		int inCount = 0;
		for (int i = 1; i <= 4; i++) {
			QMeshNode* tetNode = tetra->GetNodeRecordPtr(i);
			if (tetNode->isIn_convexHullTest) inCount++;
		}

		if ((inCount == 0) || (inCount == 4)) continue;

		PQP_REAL centPos[3];
		tetra->CalCenterPos(centPos[0], centPos[1], centPos[2]);

		PQP_DistanceResult dres;	dres.last_tri = pqpModel_CH->last_tri;

		PQP_Distance(&dres, pqpModel_CH, centPos, 0.0, 0.0);

		int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero


		QMeshFace* closestFace;
		int faceIndex = 0;
		for (GLKPOSITION _Pos = convexHull->GetFaceList().GetHeadPosition(); _Pos;) {
			QMeshFace* Face = (QMeshFace*)convexHull->GetFaceList().GetNext(_Pos);
			if (faceIndex == minTriId) {
				closestFace = Face;
				break;
			}
			faceIndex++;
		}

		double nx, ny, nz;
		closestFace->CalPlaneEquation();
		closestFace->GetNormal(nx, ny, nz);
		int indexx = tetra->FieldIndexNumber;
		VectorFieldMatrix(indexx, 0) = nx;
		VectorFieldMatrix(indexx, 1) = ny;
		VectorFieldMatrix(indexx, 2) = nz;
		if (!tetra->isPivot) pivotNumber++;
		tetra->isPivot = true;
		tetra->isPivotB = true;
		tetra->isOnHull = true;
	}


}

void RoughMachining::_initiateGuideVectors()
{
	int count = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (thisTet->isPivot) continue;
		double xc, yc, zc;
		thisTet->CalCenterPos(xc, yc, zc);
		if ((abs(xc) < 1) && (abs(zc -12) < 1)  && (abs(yc - 40) < 3)) {
			//std::cout << "Found one\n";
			int index = thisTet->FieldIndexNumber;
			VectorFieldMatrix(index, 0) = 0.0;
			VectorFieldMatrix(index, 1) = 1.0;
			VectorFieldMatrix(index, 2) = 0.0;
			thisTet->isPivot = true;
			pivotNumber++;
			thisTet->isPivotA = true;
			if(count >1)
			break;
			count++;
		}
	}
	
	count = 0; 
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (thisTet->isPivot) continue;
		double xc, yc, zc;
		thisTet->CalCenterPos(xc, yc, zc);
		if ((abs(xc) < 1) && (abs(zc - 18) < 1) && (abs(yc - 10) < 3) && (zc>0)) {
			//std::cout << "Found two\n";
			int index = thisTet->FieldIndexNumber;
			VectorFieldMatrix(index, 0) = 0.0;
			VectorFieldMatrix(index, 1) = 0.0;
			VectorFieldMatrix(index, 2) = 1.0;
			thisTet->isPivot = true;
			pivotNumber++;
			thisTet->isPivotA = true;
			if (count > 1)
				break;
			count++;
		}
	}

	count = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (thisTet->isPivot) continue;
		double xc, yc, zc;
		thisTet->CalCenterPos(xc, yc, zc);
		if ((abs(xc) < 1) && (abs(zc + 18) < 1) && (abs(yc - 10) < 3) && (zc<0)) {
			//std::cout << "Found three\n";
			int index = thisTet->FieldIndexNumber;
			VectorFieldMatrix(index, 0) = 0.0;
			VectorFieldMatrix(index, 1) = 0.0;
			VectorFieldMatrix(index, 2) = -1.0;
			thisTet->isPivot = true;
			pivotNumber++;
			thisTet->isPivotA = true;
			if (count > 1)
				break;
			count++;
		}
	}

	count = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (thisTet->isPivot) continue;
		double xc, yc, zc;
		thisTet->CalCenterPos(xc, yc, zc);
		if ((abs(xc + 18) < 1) && (abs(zc) < 1) && (abs(yc - 10) < 3) && (xc < 0)) {
			//std::cout << "Found four\n";
			int index = thisTet->FieldIndexNumber;
			VectorFieldMatrix(index, 0) = -1.0;
			VectorFieldMatrix(index, 1) = 0.0;
			VectorFieldMatrix(index, 2) = 0.0;
			thisTet->isPivot = true;
			pivotNumber++;
			thisTet->isPivotA = true;
			if (count > 1)
				break;
			count++;
		}
	}

	count = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		if (thisTet->isPivot) continue;
		double xc, yc, zc;
		thisTet->CalCenterPos(xc, yc, zc);
		if ((abs(xc - 18) < 1) && (abs(zc) < 1) && (abs(yc - 10) < 3) && (xc > 0)) {
			//std::cout << "Found five\n";
			int index = thisTet->FieldIndexNumber;
			VectorFieldMatrix(index, 0) = 1.0;
			VectorFieldMatrix(index, 1) = 0.0;
			VectorFieldMatrix(index, 2) = 0.0;
			thisTet->isPivot = true;
			pivotNumber++;
			thisTet->isPivotA = true;
			if (count > 1)
				break;
			count++;
		}
	}
}

void RoughMachining::_propagateVectorField()
{
	if (pivotNumber == 0) {
		std::cout << "WARNING!!! Need to specify at least one seed direction or a method to guess the field.\n";
		return;
	}

	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;
	
	std::cout << "Pivot nums " << pivotNumber << std::endl;
	_buildTetLaplacianMatrix();
//	_buildTetOptMatrixWithCurl();



	Eigen::VectorXd v0 = Eigen::VectorXd::Zero(2 * num_tets);
	Eigen::VectorXd v1 = Eigen::VectorXd::Zero(2 * num_tets);
	Eigen::VectorXd v2 = Eigen::VectorXd::Zero(2 * num_tets);
	//Eigen::VectorXd v3 = Eigen::VectorXd::Zero(2 * num_tets);

	Eigen::VectorXd v0_sol(num_tets);
	Eigen::VectorXd v1_sol(num_tets);
	Eigen::VectorXd v2_sol(num_tets);
	//Eigen::VectorXd v3_sol(num_tets);

	_createSolutionVectorForVectorSmoothing(0, v0);
	_createSolutionVectorForVectorSmoothing(1, v1);
	_createSolutionVectorForVectorSmoothing(2, v2);
//	_createSolutionVectorForVectorSmoothing(3, v3);

	std::cout << " Size of vector " << v0.rows() << std::endl;
	std::cout << "Size of Laplacian " << TetLaplacianMatrix.rows() << std::endl;

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
	std::cout << " Size of vector after Mult " << v0.rows() << std::endl;
	v0_sol = SolverN.solve(v0);
	std::cout << "Smoothing 2\n";
	v1 = TetLaplacianMatrix.transpose() * v1;
	v1_sol = SolverN.solve(v1);
	std::cout << "Smoothing 3\n";
	v2 = TetLaplacianMatrix.transpose() * v2;
	v2_sol = SolverN.solve(v2);

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		int index = tetra->FieldIndexNumber;
		Eigen::Vector3d newNormal(v0_sol(index), v1_sol(index), v2_sol(index));
		newNormal.stableNormalize();
		//newNormal = newNormal * v3_sol(index);

		double cx, cy, cz;
		tetra->CalCenterPos(cx, cy, cz);
		//double mag = (cx+25.1)/5.0;
		double mag = 1;// (0.1 / 1250)* cx* cx + (0.9 / 50) * cx + 0.5;

		VectorFieldMatrix(index, 0) = mag * newNormal.x();
		VectorFieldMatrix(index, 1) = mag * newNormal.y();
		VectorFieldMatrix(index, 2) = mag * newNormal.z();
	}



	//std::cout << "Testing.............\n";
	//Eigen::Vector3d y = Eigen::Vector3d::Zero(5);
	//Eigen::Matrix2d B = Eigen::Matrix2d::Identity(3, 3);

	//Eigen::SparseMatrix<double> D = Eigen::SparseMatrix<double>(3, 3);
	//D.setIdentity();
	//y(0) = 1;
	//y(1) = 2;
	//y(2) = 3;


	//Eigen::Vector3d z = D * y;
	//std::cout << "Size of z " << z.rows() << std::endl;
	//std::cout << "\n\nPrinting result.....\n";
	///*
	//for (int i = 0; i < 3; i++) {
	//	std::cout << z(i) << std::endl;
	//}
	//*/
}

void RoughMachining::_propagateVectorFieldPreserved()
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;

	if (buildNewTetLaplacian) {
		_buildTetLaplacianMatrixPreserved();
	}

	Eigen::VectorXd v0 = Eigen::VectorXd::Zero(2 * num_tets);
	Eigen::VectorXd v1 = Eigen::VectorXd::Zero(2 * num_tets);
	Eigen::VectorXd v2 = Eigen::VectorXd::Zero(2 * num_tets);
	//Eigen::VectorXd v3 = Eigen::VectorXd::Zero(2 * num_tets);

	Eigen::VectorXd v0_sol(num_tets);
	Eigen::VectorXd v1_sol(num_tets);
	Eigen::VectorXd v2_sol(num_tets);
	//Eigen::VectorXd v3_sol(num_tets);

	_createSolutionVectorForVectorSmoothingPreserved(0, v0);
	_createSolutionVectorForVectorSmoothingPreserved(1, v1);
	_createSolutionVectorForVectorSmoothingPreserved(2, v2);
	//	_createSolutionVectorForVectorSmoothing(3, v3);
	if (buildNewTetLaplacian) {
		std::cout << "Initialising Sparse Solver for vector smoothing..\n";

		Eigen::SparseMatrix<double> S = TetLaplacianMatrix.transpose() * TetLaplacianMatrix;
		SolverTetLaplcaian.analyzePattern(S);
		SolverTetLaplcaian.factorize(S);
		if (SolverTetLaplcaian.info() != Eigen::Success)
			std::cout << "error here: factorize fail!" << std::endl;
		SolverTetLaplcaian.compute(S);
		buildNewTetLaplacian = false;
	}
	std::cout << "Smoothing 1\n";
	v0 = TetLaplacianMatrix.transpose() * v0;
	v0_sol = SolverTetLaplcaian.solve(v0);
	std::cout << "Smoothing 2\n";
	v1 = TetLaplacianMatrix.transpose() * v1;
	v1_sol = SolverTetLaplcaian.solve(v1);
	std::cout << "Smoothing 3\n";
	v2 = TetLaplacianMatrix.transpose() * v2;
	v2_sol = SolverTetLaplcaian.solve(v2);

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




void RoughMachining::_computeScalarField(double* minS, double* maxS)
{
	clock_t tim_;
	tim_ = clock();
	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;
	Eigen::VectorXd divVector = Eigen::VectorXd::Zero(num_nodes);
	_generateDivergence(divVector);
	//_generateDivergence(divVector);

	_createWeightMatrix();
	Eigen::SparseMatrix<double> invVol = VolumeMatrix;
	Eigen::SparseMatrix<double> boundaryCompensator(num_nodes, num_nodes);
	boundaryCompensator.setIdentity();
	std::cout << "Solving Scalar System...\n";
	int bcount = 0;
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);

		if (node->in_ConvexHull) continue;

		int ii = node->FieldIndexNumber;

		if (node->isConstraint/*node->isBoundary &&*/ /*node->isSource*/ /*&& node->distanceFromCH >= 0*/) {
			invVol.coeffRef(ii, ii) = 0.0;
			divVector(ii) = (node->scalarField);
			bcount++;
		}
		else {
			invVol.coeffRef(ii, ii) = 1.0;
			boundaryCompensator.coeffRef(ii, ii) = 0;
		}
	}
	/*Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(WeightMatrix);
	Eigen::VectorXd u = dec.solve(divVector);*/

	Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;// (PardisoLU/SparseLU)


	//Eigen::SparseMatrix<double> S = WeightMatrix;
	Eigen::SparseMatrix<double> S = invVol * WeightMatrix + boundaryCompensator;


	 
	//_enforceGradientBC(divVector, S);

	S = 100 * S;

	Solver.analyzePattern(S);
	Solver.factorize(S);
	if (Solver.info() != Eigen::Success)
		std::cout << "error here: factorize fail!" << std::endl;
	Solver.compute(S);
	divVector = 100 * divVector;
	Eigen::VectorXd u = Solver.solve(divVector);

	tim_ = clock() - tim_;
	std::cout << "Poisson time: " << ((double)tim_) / CLOCKS_PER_SEC << std::endl;

	std::cout << "System Solved for Scalar Field...\n";

	double max = -9.99999e15, min = 999999e15;
	double singularMax = -99e10, singularMin = 99e10;
	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

		if (node->in_ConvexHull) continue;

		int i = node->FieldIndexNumber;
		node->HeatFieldValue = u[i];
		if (u[i] > max) max = u[i];
		if (u[i] < min) min = u[i];
		node->scalarField = node->HeatFieldValue;
		if (node->isSource) {
			if (singularMax < node->scalarField) singularMax = node->scalarField;
			if (singularMin > node->scalarField) singularMin = node->scalarField;
		}
	}

	for (GLKPOSITION pos = materialSpace->GetNodeList().GetHeadPosition(); pos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(pos);

		if (node->in_ConvexHull) continue;

		int i = node->FieldIndexNumber;
		node->HeatFieldValue = (u[i]-min)/(max-min);
		//node->scalarField = node->HeatFieldValue;
		//std::cout << node->scalarField << std::endl;
	}

	max_distance = max;
	min_distance = min;

	std::cout << " MAX: " << max << " MIN: " << min << std::endl;
	if (minS) {
		*minS = singularMin; 
	}

	if (maxS) *maxS = singularMax;
}

void RoughMachining::_displayDivergence()
{
	int num = materialSpace->GetNodeNumber();
	int num_nodes = num - Num_nodesInConvexHull;
	Eigen::VectorXd divVector = Eigen::VectorXd::Zero(num_nodes);
	_generateDivergence(divVector);
	double max = -9e10;
	double min = 9e10;
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		thisNode->scalarField = abs(divVector[thisNode->FieldIndexNumber]);
		if (thisNode->scalarField > max) max = thisNode->scalarField;
		if (thisNode->scalarField < min) min = thisNode->scalarField;
	}

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		thisNode->scalarField = (thisNode->scalarField - min) / (max - min);
	}

	std::cout << "Done generating divergence field\n";
}



void RoughMachining::_correctDivergence()
{
	QMeshNode* maxNode;
	double max = -9e10;
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		if (thisNode->scalarField > max) {
			max = thisNode->scalarField;
			maxNode = thisNode;
		}
	}

	int nnum = maxNode->GetTetraNumber();
	std::cout << "Tetra Num. " << nnum << std::endl;
	for (int i = 0; i < nnum; i++) {
		QMeshTetra* nodeTetra = maxNode->GetTetraRecordPtr(i+1);
		VectorFieldMatrix(nodeTetra->FieldIndexNumber, 0) = 0.0;
		VectorFieldMatrix(nodeTetra->FieldIndexNumber, 1) = 1.0;
		VectorFieldMatrix(nodeTetra->FieldIndexNumber, 2) = 0.0;
		nodeTetra->isPivot = true;
		nodeTetra->isPivotA = true;
		pivotNumber++;
	}
	
}

void RoughMachining::_detectSingualrity()
{
	std::cout << "Detecting Singularity\n";
	std::vector<QMeshNode*>* nodeVector = new std::vector<QMeshNode*>;
	int vectorLength = materialSpace->GetNodeList().GetCount();
	nodeVector->reserve(vectorLength);
	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		nodeVector->push_back(thisNode);
	}
	
	for (int i = 0; i < vectorLength; i++) {
		QMeshNode* tempNode = (*nodeVector)[i];
		if (tempNode->isBoundary) continue;
		if (tempNode->isSupressedBoundary) continue;
		double thisVal = tempNode->scalarField;
		int neighbourNum = tempNode->GetEdgeNumber();
		int lesserCount = 0;
		int greaterCount = 0;
		for (int j = 0; j < neighbourNum; j++) {
			QMeshEdge* tempEdge = tempNode->GetEdgeRecordPtr(j+1);
			QMeshNode* otherTempNode = tempEdge->GetEndPoint();
			if (otherTempNode == tempNode) otherTempNode = tempEdge->GetStartPoint();
			if (otherTempNode->scalarField >= thisVal) lesserCount++;
			if (otherTempNode->scalarField <= thisVal) greaterCount++;
 		}
		if ((lesserCount == neighbourNum) || (greaterCount == neighbourNum)) {
			std::cout << "Found Singularity Point\n";
			tempNode->tempFlag = true;
		}
	}
	delete nodeVector;
}

void RoughMachining::_correctSingularity(bool buildNewLap)
{

	for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* thisNode = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		if (!thisNode->tempFlag) continue;
		buildNewTetLaplacian = buildNewLap;
		int nnum = thisNode->GetTetraNumber();
		//std::cout << "Tetra Num. " << nnum << std::endl;
		for (int i = 0; i < nnum; i++) {
			QMeshTetra* nodeTetra = thisNode->GetTetraRecordPtr(i + 1);
			VectorFieldMatrix(nodeTetra->FieldIndexNumber, 0) = 0.0;
			VectorFieldMatrix(nodeTetra->FieldIndexNumber, 1) = 1.0;
			VectorFieldMatrix(nodeTetra->FieldIndexNumber, 2) = 0.0;

			if (!nodeTetra->isPivot) pivotNumber++;
			nodeTetra->isPivot = true;
			nodeTetra->isPivotA = true;
			
		}
		
	}

	
}

void RoughMachining::_checkGradient1()
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;
	for (int i = 0; i < num_tets; i++) {
		Eigen::Vector3d normDir(VectorFieldMatrix(i, 0), VectorFieldMatrix(i, 1), VectorFieldMatrix(i, 2));
		double x = normDir.norm();
		if (x < 0.9990) { 
			std::cout << x << std::endl;
			std::cout << "High Curl\n"; 
		}
	}
}

void RoughMachining::_checkGradient2()
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;


	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(3, 4);
	Eigen::MatrixXd VertCoord = Eigen::MatrixXd::Zero(3, 3);
	Eigen::Vector4d VertField = Eigen::Vector4d::Zero(4);

	T(0, 0) = 1;
	T(1, 1) = 1;
	T(2, 2) = 1;

	T(0, 3) = T(1, 3) = T(2, 3) = -1;


	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		if (tet->isIn_convexHull) continue;

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



			VertField(i) = node->scalarField;
			
		}

		Eigen::Vector3d temp = VertCoord.inverse() * T * VertField;
		if (temp.norm() < 0.99) {
			std::cout << "High Curl\n";
			std::wcout << temp.norm() << std::endl;
			break;
		}



	}
}

void RoughMachining::_convert2GradientField()
{
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;


	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(3, 4);
	Eigen::MatrixXd VertCoord = Eigen::MatrixXd::Zero(3, 3);
	Eigen::Vector4d VertField = Eigen::Vector4d::Zero(4);

	T(0, 0) = 1;
	T(1, 1) = 1;
	T(2, 2) = 1;

	T(0, 3) = T(1, 3) = T(2, 3) = -1;


	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		if (tet->isIn_convexHull) continue;

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



			VertField(i) = node->scalarField;

		}

		Eigen::Vector3d temp = VertCoord.inverse() * T * VertField;
		temp.stableNormalize();
		VectorFieldMatrix(tet->FieldIndexNumber, 0) = temp.x();
		VectorFieldMatrix(tet->FieldIndexNumber, 1) = temp.y();
		VectorFieldMatrix(tet->FieldIndexNumber, 2) = temp.z();
	}

//	_initiateVectorOnExt+eriorBoundTets();
//	_initiateVectorOnInteriorBoundTets();
//	_correctSingularity();
	
}

void RoughMachining::_computeDifference()
{
	Eigen::MatrixXd vectorFieldPrev = VectorFieldMatrix;
	_convert2GradientField();
	int num = materialSpace->GetTetraNumber();
	int num_tets = num - Num_tetInConvexHull;
	double ASum=0;
	double* Vol;
	Vol = (double*)malloc(num_tets * sizeof(double));
	double totalVol = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		int indxx = tetra->FieldIndexNumber;
		Vol[indxx] = tetra->CalVolume();
		totalVol += Vol[indxx];
	}

	for (int i = 0; i < num_tets; i++) {
		double tempSum = 0;
		tempSum += (vectorFieldPrev(i, 0) - VectorFieldMatrix(i, 0)) * (vectorFieldPrev(i, 0) - VectorFieldMatrix(i, 0));
		tempSum += (vectorFieldPrev(i, 1) - VectorFieldMatrix(i, 1)) * (vectorFieldPrev(i, 1) - VectorFieldMatrix(i, 1));
		tempSum += (vectorFieldPrev(i, 2) - VectorFieldMatrix(i, 2)) * (vectorFieldPrev(i, 2) - VectorFieldMatrix(i, 2));
		ASum += (tempSum) * Vol[i];
	}

	double BSum = 0;
	int ccount = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		
		if (!thisTet->isPivot) continue;
		if (thisTet->isPivotA) continue;
		//if (!thisTet->isSingular) continue;
		int i = thisTet->FieldIndexNumber;
		double tempSum = 0;
		tempSum += (vectorFieldPrev(i, 0) - VectorFieldMatrix(i, 0)) * (vectorFieldPrev(i, 0) - VectorFieldMatrix(i, 0));
		tempSum += (vectorFieldPrev(i, 1) - VectorFieldMatrix(i, 1)) * (vectorFieldPrev(i, 1) - VectorFieldMatrix(i, 1));
		tempSum += (vectorFieldPrev(i, 2) - VectorFieldMatrix(i, 2)) * (vectorFieldPrev(i, 2) - VectorFieldMatrix(i, 2));
		BSum += sqrt(tempSum);
		ccount++;

	}

	/*int thirdCount = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		QMeshFace* boundFace;
		bool isBound = false;
		for (int kk = 0; kk < 3; kk++) {
			boundFace = thisTet->GetFaceRecordPtr(kk + 1);
			if (boundFace->isInteriorBoundary) {
				isBound = true;
				break;
			}
		}


		if (!isBound) continue;

		int i = thisTet->FieldIndexNumber;
		Eigen::Vector3d faceNorm;
		boundFace->CalPlaneEquation();
		boundFace->GetNormal(faceNorm(0), faceNorm(1), faceNorm(2));
		Eigen::Vector3d origNorm, newNorm;
		
		origNorm.x() = vectorFieldPrev(i, 0);
		origNorm.y() = vectorFieldPrev(i, 1);
		origNorm.z() = vectorFieldPrev(i, 2);

		newNorm.x() = VectorFieldMatrix(i, 0);
		newNorm.y() = VectorFieldMatrix(i, 1);
		newNorm.z() = VectorFieldMatrix(i, 2);

		double aa = faceNorm.dot(newNorm);
		double bb = faceNorm.dot(origNorm);
		thirdCount++;
		std::cout << "......." << aa << "|||" << bb << std::endl << std::endl;
		if(aa>-0.5) boundFace->isSingular = true;
		if (thirdCount > 100) break;
	}*/


	for (int i = 0; i < num_tets; i++) {
		VectorFieldMatrix(i, 0) = vectorFieldPrev(i, 0);
		VectorFieldMatrix(i, 1) = vectorFieldPrev(i, 1);
		VectorFieldMatrix(i, 2) = vectorFieldPrev(i, 2);
	}

	std::cout << "Average Difference: " << ASum/totalVol << std::endl;
	//std::cout << "Average Pivot Difference: " << BSum / ccount << std::endl;
}

void RoughMachining::_translateVectorNode2Tets(QMeshNode* node)
{
	int numm = node->GetTetraNumber();
	for (int i = 0; i < numm; i++) {
		QMeshTetra* incidentTetra = node->GetTetraRecordPtr(i + 1);
		int tetraIndex = incidentTetra->FieldIndexNumber;
		VectorFieldMatrix(tetraIndex, 0) = node->vectorDir[0];
		VectorFieldMatrix(tetraIndex, 1) = node->vectorDir[1];
		VectorFieldMatrix(tetraIndex, 2) = node->vectorDir[2];

		if (!incidentTetra->isPivot) pivotNumber++;

		incidentTetra->isPivot = true;
		incidentTetra->isPivotA = true;
		incidentTetra->isPivotB = false;
	}

}



void RoughMachining::_detectPlanarSingularity()
{
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos; ) {
		QMeshFace* thisFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);
		QMeshTetra* lTet = thisFace->GetLeftTetra();
		if (!lTet) continue;
		QMeshTetra* rTet = thisFace->GetRightTetra();
		if (!rTet) continue;

		int lIndex = lTet->FieldIndexNumber;
		Eigen::Vector3d lVec = Eigen::Vector3d(VectorFieldMatrix(lIndex, 0), VectorFieldMatrix(lIndex, 1), VectorFieldMatrix(lIndex, 2));

		int rIndex = rTet->FieldIndexNumber;
		Eigen::Vector3d rVec = Eigen::Vector3d(VectorFieldMatrix(rIndex, 0), VectorFieldMatrix(rIndex, 1), VectorFieldMatrix(rIndex, 2));

		double dProd = lVec.dot(rVec);

		if (dProd < 0) { thisFace->isSingular = true; }
		else continue;

		for (int i = 0; i < 3; i++) {
			QMeshNode* faceNode = thisFace->GetNodeRecordPtr(i);
			faceNode->isSource = true;
		}

		lTet->isSingular = true;
		rTet->isSingular = true;
		
		//std::cout << lVec.x() << " " << lVec.y() << " " << lVec.z() << "\n";
		//std::cout << rVec.x() << " " << rVec.y() << " " << rVec.z() << "\n";
	}
}

void RoughMachining::_correctPlanarSingularity(double* dir, bool flip)
{

	Eigen::Vector3d dir1, dir2;
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		dir1.setZero();
		dir2.setZero();
		
		QMeshFace* face = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);
		if (!face->isSingular) continue;

		QMeshTetra* lTet = face->GetLeftTetra();
		if (!lTet) continue;

		QMeshTetra* rTet = face->GetRightTetra();
		if (!rTet) continue;

		int indexx = lTet->FieldIndexNumber;
		dir1 << VectorFieldMatrix(indexx, 0), VectorFieldMatrix(indexx, 1), VectorFieldMatrix(indexx, 2);
		//std::cout << dir1[0] << " " << dir1[1] << " " << dir1[2] << std::endl;

		indexx = rTet->FieldIndexNumber;
		dir2 << VectorFieldMatrix(indexx, 0), VectorFieldMatrix(indexx, 1), VectorFieldMatrix(indexx, 2);
		//std::cout << dir2[0] << " " << dir2[1] << " " << dir2[2] << std::endl;


		if (dir1.dot(dir2) < -0.707)
		{
			break;
		}
	}

	if (flip) {
		std::cout << "Flipping..\n";
		Eigen::Vector3d temp = dir1;
		dir1.setZero();
		dir1 = dir2;
		dir2.setZero();
		dir2 = temp;

		//std::cout << dir1[0] << " " << dir1[1] << " " << dir1[2] << std::endl;
		//std::cout << dir2[0] << " " << dir2[1] << " " << dir2[2] << std::endl;
	}


	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos; ) {
		QMeshTetra* Tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		bool proceedCheck = false;
		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = Tetra->GetNodeRecordPtr(i);
			if (node->isSource) proceedCheck = true;
		}

		if (!proceedCheck) continue; 

		int indexx = Tetra->FieldIndexNumber;
		Eigen::Vector3d tetNorm = Eigen::Vector3d(VectorFieldMatrix(indexx, 0), VectorFieldMatrix(indexx, 1), VectorFieldMatrix(indexx, 2));
		
		if (tetNorm.dot(dir2) < 0) {
			VectorFieldMatrix(indexx, 0) = dir2.x();
			VectorFieldMatrix(indexx, 1) = dir2.y();
			VectorFieldMatrix(indexx, 2) = dir2.z();

			/*for (int k = 0; k < 4; k++) {
				QMeshNode* otherNode = Tetra->GetNodeRecordPtr(k + 1);
				int otherTetrNum = otherNode->GetTetraNumber();
				for (int l = 0; l < otherTetrNum; l++) {
					QMeshTetra* otherTetra = otherNode->GetTetraRecordPtr(l+1);

					if (otherTetra->isSingular) continue;

					int indexx2 = otherTetra->FieldIndexNumber;

					Eigen::Vector3d tetNorm2 = Eigen::Vector3d(VectorFieldMatrix(indexx2, 0), VectorFieldMatrix(indexx2, 1), VectorFieldMatrix(indexx2, 2));
					if (tetNorm2.dot(dir2) < 0) {
						VectorFieldMatrix(indexx2, 0) = dir2.x();
						VectorFieldMatrix(indexx2, 1) = dir2.y();
						VectorFieldMatrix(indexx2, 2) = dir2.z();
					}

				}
			}*/

			 for (int k = 0; k < 4; k++) {
				 QMeshFace* tetraFace = (QMeshFace*)Tetra->GetFaceRecordPtr(k+1);
				 QMeshTetra* otherTetra = tetraFace->GetLeftTetra();
				 if (!otherTetra) continue;
				 if (otherTetra == Tetra) otherTetra = tetraFace->GetRightTetra();
				 if (!otherTetra) continue;

				 if (otherTetra->isSingular) continue;

				 int indexx2 = otherTetra->FieldIndexNumber;
				 
				 Eigen::Vector3d tetNorm2 = Eigen::Vector3d(VectorFieldMatrix(indexx2, 0), VectorFieldMatrix(indexx2, 1), VectorFieldMatrix(indexx2, 2));
				 if (tetNorm2.dot(dir2) < 0) {
					 VectorFieldMatrix(indexx2, 0) = dir2.x();
					 VectorFieldMatrix(indexx2, 1) = dir2.y();
					 VectorFieldMatrix(indexx2, 2) = dir2.z();
				 }
			 }
		}
		else {

			VectorFieldMatrix(indexx, 0) = dir2.x();
			VectorFieldMatrix(indexx, 1) = dir2.y();
			VectorFieldMatrix(indexx, 2) = dir2.z();

			/*for (int i = 1; i <= 4; i++) {
				QMeshNode* node = Tetra->GetNodeRecordPtr(i);
				if (!node->isSource) {
					node->isConstraint = true;
				}
			}*/

			/*for (int i = 1; i <= 4; i++) {
				QMeshNode* node = Tetra->GetNodeRecordPtr(i);
				if (!node->isSource) {
					int edNum = node->GetEdgeNumber();
					for (int k = 0; k < edNum; k++) {
						QMeshEdge* nodeEdge = node->GetEdgeRecordPtr(k+1);
						QMeshNode* otherNode = nodeEdge->GetStartPoint();
						if (otherNode == node) otherNode = nodeEdge->GetEndPoint();
						if (otherNode->isSource) continue;
						int edNum2 = otherNode->GetEdgeNumber();
						bool setConstr = true;
						for (int l = 0; l < edNum2; l++) {
							QMeshEdge* OthernodeEdge = otherNode->GetEdgeRecordPtr(l + 1);
							QMeshNode* otherOtherNode = OthernodeEdge->GetStartPoint();
							if (otherOtherNode == otherNode) otherOtherNode = OthernodeEdge->GetEndPoint();
							if (otherOtherNode->isSource) setConstr = false;
						}
						if (setConstr) otherNode->isConstraint = true;
					}
				}
			}*/

			for (int i = 1; i <= 4; i++) {
				QMeshFace* face = Tetra->GetFaceRecordPtr(i);
				
				QMeshTetra* otherTetra = face->GetLeftTetra();
				if (!otherTetra) continue;
				if (otherTetra == Tetra) otherTetra = face->GetRightTetra();
				if (!otherTetra) continue;

				int indexx2 = otherTetra->FieldIndexNumber;

				Eigen::Vector3d tetNorm2 = Eigen::Vector3d(VectorFieldMatrix(indexx2, 0), VectorFieldMatrix(indexx2, 1), VectorFieldMatrix(indexx2, 2));
				if (tetNorm2.dot(dir2) < 0) {
					continue;
				}

				if (otherTetra->isSingular) {
				//	std::cout << "Singular otherTet\n";
					continue;
				}
				for (int i = 1; i <= 4; i++) {
					QMeshNode* node = otherTetra->GetNodeRecordPtr(i);
					int edgeNum = node->GetEdgeNumber();
					bool setConst = true;
					for (int l = 1; l <= edgeNum; l++) {
						QMeshEdge* nodeEdge = node->GetEdgeRecordPtr(l);
						QMeshNode* otherNode = nodeEdge->GetStartPoint();
						if (otherNode == node) otherNode = nodeEdge->GetEndPoint();
						if (otherNode->isSource) setConst = false;
					}
					if (setConst) node->isConstraint = true;
				}

			}
		}
	}

	dir[0] = dir2.x();
	dir[1] = dir2.y();
	dir[2] = dir2.z();

	/*for (GLKPOSITION nodePos = materialSpace->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)materialSpace->GetNodeList().GetNext(nodePos);
		if (!node->isSource) continue;
		int numm = node->GetEdgeNumber();
		for (int i = 1; i <= numm; i++) {
			QMeshEdge* nodeEdge = 
		}
	}*/

	/*std::cout << "One\n";
	for (GLKPOSITION edgePos = materialSpace->GetEdgeList().GetHeadPosition(); edgePos;) {
		QMeshEdge* thisEdge = (QMeshEdge*)materialSpace->GetEdgeList().GetNext(edgePos);

		QMeshNode* sNode = thisEdge->GetStartPoint();
		QMeshNode* eNode = thisEdge->GetEndPoint();

		if (sNode->isSource) {
			thisEdge->in_convexHull = true;
			sNode->isIn_convexHull = true;
		}

		if (eNode->isSource) {
			thisEdge->in_convexHull = true;
			eNode->isIn_convexHull = true;
		}
	}


	std::cout << "Two\n";
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = thisTet->GetNodeRecordPtr(i);
			if (node->isIn_convexHull) thisTet->isIn_convexHull = true;
		}
	}

	std::cout << "Three\n";
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* thisFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);

		for (int i = 0; i < 3; i++) {
			QMeshNode* node = thisFace->GetNodeRecordPtr(i);
			if (node->isIn_convexHull) thisFace->in_convexHull = true;
		}
	}*/

}

void RoughMachining::_correctAlongInnerBoundary()
{
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* face = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);

		if (!face->isInteriorBoundary)  continue;

		QMeshTetra* faceTet = face->GetLeftTetra();
		if (!faceTet) faceTet = face->GetRightTetra();
		
		Eigen::Vector3d faceNorm;
		face->CalPlaneEquation();
		face->GetNormal(faceNorm.x(), faceNorm.y(), faceNorm.z());
		faceNorm.stableNormalize();
		faceNorm = -1 * faceNorm;

		Eigen::Vector3d tetDir;
		int indexx = faceTet->FieldIndexNumber;
		tetDir << VectorFieldMatrix(indexx, 0), VectorFieldMatrix(indexx, 1), VectorFieldMatrix(indexx, 2);

		tetDir = 0.6 * (faceNorm) + 0.4 * (tetDir);

		tetDir.normalize();

		VectorFieldMatrix(indexx, 0) = tetDir.x();
		VectorFieldMatrix(indexx, 1) = tetDir.y();
		VectorFieldMatrix(indexx, 2) = tetDir.z();

		if (!faceTet->isPivot) pivotNumber++;
		faceTet->isPivot = true;
		faceTet->isPivotB = true;
	}
}

void RoughMachining::_identifyConstraints()
{
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* face = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);

		if (face->isBoundary && !face->isInteriorBoundary) {
			face->CalPlaneEquation();
			double nx, ny, nz;
			face->GetNormal(nx, ny, nz);
			if (ny < 0.99) continue;
			for (int i = 0; i < 3; i++) {
				QMeshNode* faceNode = face->GetNodeRecordPtr(i);
				faceNode->isConstraint = true;
			}
		}
	
	}
}

void RoughMachining::_enforceGradientBC(Eigen::VectorXd& divVector, Eigen::SparseMatrix<double>& sysMatrix)
{
	QMeshFace* boundFace;
	QMeshTetra* boundTetra;
	bool twoBound = false;
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		
		boundFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);
		if (boundFace->isInteriorBoundary) {
			boundTetra = boundFace->GetLeftTetra();
			if (!boundTetra) boundTetra = boundFace->GetRightTetra();
			//break;
			/*for (int k = 0; k < 3; k++) {
				QMeshFace* otherFace = boundTetra->GetFaceRecordPtr(k + 1);
				if (otherFace->isBoundary) {
					if (otherFace != boundFace) {
						twoBound = true;
					}
				}
			}
			if (twoBound) break;*/
		}
		else { continue; }

		//if (!twoBound) return;
		//std::cout << "Enforcing BC\n";

		//std::cout << ".\n" << std::endl;
		int indexx = boundTetra->FieldIndexNumber;
		Eigen::Vector3d gradDir;
		gradDir << VectorFieldMatrix(indexx, 0), VectorFieldMatrix(indexx, 1), VectorFieldMatrix(indexx, 2);
		QMeshEdge* faceEdge = boundTetra->GetEdgeRecordPtr(1);
		QMeshNode* nodeA = faceEdge->GetStartPoint();
		QMeshNode* nodeB = faceEdge->GetEndPoint();

		int itr = 2;
		while (1) {
			if (itr > 6) break;
			if ((nodeA->isBoundary || nodeA->isSupressedBoundary) && ((nodeB->isBoundary || nodeB->isSupressedBoundary))) {
				faceEdge = boundTetra->GetEdgeRecordPtr(itr++);
				nodeA = faceEdge->GetStartPoint();
				nodeB = faceEdge->GetEndPoint();
			}
			else break;
			
		}
		//if(itr>6) continue;

		if (nodeB->isBoundary) {
			QMeshNode* temp;
			temp = nodeA;
			nodeA = nodeB;
			nodeB = temp;
		}

		Eigen::Vector3d edgeVec, nodeAPos, nodeBPos;
		nodeA->GetCoord3D(nodeAPos(0), nodeAPos(1), nodeAPos(2));
		nodeB->GetCoord3D(nodeBPos(0), nodeBPos(1), nodeBPos(2));
		edgeVec = nodeAPos - nodeBPos;
		//std::cout << "..\n";
		double gradVal = gradDir.dot(edgeVec);

		int AIndex = nodeA->FieldIndexNumber;
		int BIndex = nodeB->FieldIndexNumber;


		int num = materialSpace->GetNodeNumber();
		int num_nodes = num - Num_nodesInConvexHull;
		//std::cout << "...\n";
//#pragma omp parallel for
		//for (int l = 0; l < num_nodes; l++) {
			//if (sysMatrix.coeffRef(AIndex, l) != 0) {
				//sysMatrix.coeffRef(AIndex, l) = 0;
				sysMatrix.row(AIndex) *= 0;
		//	}
		//}
		//std::cout << "....\n";
		sysMatrix.coeffRef(AIndex, AIndex) = 1;
		sysMatrix.coeffRef(AIndex, BIndex) = -1;
		divVector(AIndex) = gradVal;
		//std::cout << gradVal << std::endl;
		//std::cout << gradDir << std::endl;
	}
}

void RoughMachining::_liftVectors()
{

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		thisTet->isVectorFiled_Sorce = false;
		if (!thisTet->isPivot) { 
			
			continue; }
		if (thisTet->isOnHull) {
			//std::cout << "found\n";
			thisTet->isVectorFiled_Sorce = true;
		}
		pivotNumber--;
		thisTet->isPivot = false;
		thisTet->isPivotA = false;
		thisTet->isPivotB = false;
	}


	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* thisFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);

		QMeshNode* faceNode = thisFace->GetNodeRecordPtr(1);
		if (!faceNode->isSupressedBoundary) continue;

		int tetNum = faceNode->GetTetraNumber();

		//for (int k = 0; k < tetNum; k++) {
		//	double xx, yy, zz;
		//	

		//	QMeshTetra* boundTet = faceNode->GetTetraRecordPtr(k+1);
		//	boundTet->CalCenterPos(xx, yy, zz);

		//	double distance_ = xx * xx + zz * zz;

		//	if (distance_ < 40 * 40) continue;

		//	/*QMeshTetra* boundTet = thisFace->GetLeftTetra();
		//	if (!boundTet) boundTet = thisFace->GetRightTetra();*/

		//	Eigen::Vector3d posVec;
		//	posVec << xx, 0.0, zz;
		//	posVec.stableNormalize();

		//	Eigen::Vector3d upVec;
		//	upVec << 0.0, 1.0, 0.0;

		//	posVec = posVec + upVec;
		//	posVec.stableNormalize();

		//	int indexx = boundTet->FieldIndexNumber;

		//	VectorFieldMatrix(indexx, 0) = upVec.x();
		//	VectorFieldMatrix(indexx, 1) = upVec.y();
		//	VectorFieldMatrix(indexx, 2) = upVec.z();


		//	if (!boundTet->isPivot) pivotNumber++;
		//	boundTet->isPivot = true;
		//	boundTet->isPivotA = true;
		//}
	}

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* thisTet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		double xx, yy, zz;
		thisTet->CalCenterPos(xx, yy, zz);

		double distance_ = xx * xx + zz * zz;
		if (distance_ < 20 * 20) continue;

		if (yy > 40) continue;

		thisTet->isVectorFiled_Sorce = false;
		Eigen::Vector3d posVec;
		posVec << xx, 0.0, zz;
		posVec.stableNormalize();

		Eigen::Vector3d upVec;
		upVec << 0.0, 1.0, 0.0;
	
		posVec = posVec + upVec;
		posVec.stableNormalize();

		int indexx = thisTet->FieldIndexNumber;

		VectorFieldMatrix(indexx, 0) = posVec.x();
		VectorFieldMatrix(indexx, 1) = posVec.y();
		VectorFieldMatrix(indexx, 2) = posVec.z();


		if (!thisTet->isPivot) pivotNumber++;
		thisTet->isPivot = true;
		thisTet->isPivotA = true;

	}
}

void RoughMachining::_initiateCustomField()
{
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		
		double cx, cy, cz;
		tetra->CalCenterPos(cx, cy, cz);
		double distance_ = cx * cx + (cy-40) * (cy-40);
		//std::cout << distance_ << std::endl;
		if (distance_ < 20.0 * 20.0) {
			int indexx = tetra->FieldIndexNumber;
			VectorFieldMatrix(indexx, 0) = 0;
			VectorFieldMatrix(indexx, 1) = 0;
			VectorFieldMatrix(indexx, 2) = 1.0;
		


			if (!tetra->isPivot) pivotNumber++;
			tetra->isPivot = true;
			tetra->isPivotA = true;
		}
		else {
			if (cx < 0) {
				int indexx = tetra->FieldIndexNumber;
				VectorFieldMatrix(indexx, 0) = -1.0;
				VectorFieldMatrix(indexx, 1) = 0;
				VectorFieldMatrix(indexx, 2) = 0.0;

				if (!tetra->isPivot) pivotNumber++;
				tetra->isPivot = true;
				tetra->isPivotA = true;
			}
			else {
				int indexx = tetra->FieldIndexNumber;
				VectorFieldMatrix(indexx, 0) = 1.0;
				VectorFieldMatrix(indexx, 1) = 0;
				VectorFieldMatrix(indexx, 2) = 0.0;

				if (!tetra->isPivot) pivotNumber++;
				tetra->isPivot = true;
				tetra->isPivotA = true;
			}
		}
	}
}

void RoughMachining::_smoothPath(QMeshPatch* toolPath)
{

	for (GLKPOSITION edgePos = toolPath->GetEdgeList().GetHeadPosition(); edgePos;) {
		QMeshEdge* edge = (QMeshEdge*)toolPath->GetEdgeList().GetNext(edgePos);

		QMeshNode* startNode = edge->GetStartPoint();
		QMeshNode* endNode = edge->GetEndPoint();

		double x1, x2, y1, y2, z1, z2;

		startNode->GetCoord3D(x1, y1, z1);
		endNode->GetCoord3D(x2, y2, z2);

		double dist_ = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2);
		if (dist_ > 2.5 * 2.5) {
			edge->isMiddleEdge = true; //to indicate a long edge
			startNode->boundary1 = true; //indicates jump edge start
			endNode->boundary2 = true;
		}
	}

	int count = 0;
	int totalCount = toolPath->GetNodeList().GetCount();
	for (int i = 0; i < 10; i++) {
		for (GLKPOSITION nodePos = toolPath->GetNodeList().GetHeadPosition(); nodePos;) {

			if (count == 0) continue;

			QMeshNode* prevNode = (QMeshNode*)toolPath->GetNodeList().GetAt(nodePos->prev);
			QMeshNode* node = (QMeshNode*)toolPath->GetNodeList().GetNext(nodePos);
			QMeshNode* nextNode = (QMeshNode*)toolPath->GetNodeList().GetAt(nodePos);

			if (node->boundary1) continue;
			if (node->boundary2) continue;

			Eigen::Vector3d curr_norm, prev_norm, next_norm;

			prevNode->GetNormal(prev_norm[0], prev_norm[1], prev_norm[2]);
			node->GetNormal(curr_norm[0], curr_norm[1], curr_norm[2]);
			nextNode->GetNormal(next_norm[0], next_norm[1], next_norm[2]);

			//curr_norm = 0.5 * (0.5 * (prev_norm + next_norm) + curr_norm);
			curr_norm =  (0.5 * (prev_norm + next_norm));
			node->SetNormal(curr_norm[0], curr_norm[1], curr_norm[2]);

		}
	}


}


void RoughMachining::_spreadSelectionToFaceSegment(QMeshNode* handleNode)
{
	int faceNum = handleNode->GetFaceNumber();
	//std::cout << "Adjacent Faces: " << faceNum << std::endl;
	std::cout << "Selection Running ";
	for (int i = 0; i < faceNum; i++) {
		QMeshFace* nodeFace = handleNode->GetFaceRecordPtr(i+1);
		if (!nodeFace->isBoundary) continue;
		nodeFace->isSelected = true;
		_spreadToSimilarFace(nodeFace);
		if (stackCounter > 9400) {
			stackCounter = 0;
			_spreadToSimilarFace((QMeshFace*)stackLastEntity);
		}
		stackCounter = 0;
	}
	std::cout << "Selection Complete\n";
	_makeSelectedFaceConstraint();
}

 



void RoughMachining::_spreadToSimilarFace(QMeshFace* MeshFace)
{
	stackCounter++;
	//std::cout << "1\n";
	if (stackCounter % 1000 == 0) std::cout << "#";
	
	for (int i = 0; i < 3; i++) {

		QMeshNode* faceNode = MeshFace->GetNodeRecordPtr(i);

		//std::cout << "2\n";
		int nodeFaceNum = faceNode->GetFaceNumber();
		for (int k = 0; k < nodeFaceNum; k++) {
			
			QMeshFace* adjacentFace = faceNode->GetFaceRecordPtr(k + 1);
			
			if (adjacentFace == MeshFace) continue;
			if (!adjacentFace->isBoundary) continue;
			if (adjacentFace->isSelected) continue;
			if (stackCounter % 1000 == 0) std::cout << "#";
			//std::cout << "3\n";
			double* checkVal = new double;
			*checkVal = _areSimilarFace(MeshFace, adjacentFace);
	
			if (abs(*checkVal) > 0.95) {
				adjacentFace->isSelected = true;
				delete checkVal;
				if (stackCounter < 9400) {
					_spreadToSimilarFace(adjacentFace);
				}
				else {
					stackLastEntity = adjacentFace;
				}
			}
			else {
				delete checkVal;
			}
		
			
		}
	}
}

double RoughMachining::_areSimilarFace(QMeshFace* MeshFace, QMeshFace* adjacentFace)
{
	Eigen::Vector3d thisNorm;
	MeshFace->GetNormal(thisNorm[0], thisNorm[1], thisNorm[2]);
	thisNorm.stableNormalize();

	adjacentFace->CalPlaneEquation();
	Eigen::Vector3d adjNorm;
	adjacentFace->GetNormal(adjNorm[0], adjNorm[1], adjNorm[2]);
	adjNorm.stableNormalize();

	double checkVal = adjNorm.dot(thisNorm);
	return checkVal;
}

void RoughMachining::_makeSelectedFaceConstraint()
{
	for (GLKPOSITION facePos = materialSpace->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* thisFace = (QMeshFace*)materialSpace->GetFaceList().GetNext(facePos);

		if (!thisFace->isSelected) continue;

		QMeshTetra* faceTetra = thisFace->GetLeftTetra();
		if (!faceTetra) faceTetra = thisFace->GetRightTetra();

		if (!faceTetra) std::cerr << "No tetra found attached to the selected face!\n";

		int index_ = faceTetra->FieldIndexNumber;
		
		Eigen::Vector3d nor;
		thisFace->GetNormal(nor[0], nor[1], nor[2]);
		if (thisFace->isInteriorBoundary) {
			nor = -nor;
		}
		nor.stableNormalize();
		for (int i = 0; i < 3; i++) {
			VectorFieldMatrix(index_, i) = nor[i];
		}

		if (!faceTetra->isPivot) pivotNumber++;
		faceTetra->isPivot = true;
		faceTetra->isPivotA = true;

	}

}









void RoughMachining::_pathIntersection(QMeshPatch* toolPath, QMeshPatch* refSurface)
{
	int numm = toolPath->GetNodeList().GetCount();

	std::vector<QMeshNode*> TP_Vector;
	TP_Vector.reserve(numm);

	int nodeCount = 0;
	for(GLKPOSITION nodePos = toolPath->GetNodeList().GetHeadPosition(); nodePos;){
		QMeshNode* node = (QMeshNode*)toolPath->GetNodeList().GetNext(nodePos);
		if (!node->resampleChecked) continue;
		TP_Vector.push_back(node);
		nodeCount++;
	}

	numm = nodeCount;
	for (GLKPOSITION facePos = refSurface->GetFaceList().GetHeadPosition(); facePos;) {
		QMeshFace* face = (QMeshFace*)refSurface->GetFaceList().GetNext(facePos);

		face->CalPlaneEquation();
	}

//	_smoothPath(toolPath);

#pragma omp parallel for
	for (int i = 0; i < numm; i++) {
		QMeshNode* tNode = TP_Vector[i];
		Eigen::Vector3d tNormal;
		double tPoint[3], tNorm[3];
	
		tNode->GetCoord3D(tPoint[0], tPoint[1], tPoint[2]);
		tNode->GetNormal(tNormal[0], tNormal[1], tNormal[2]);
		tNode->GetNormal(tNorm[0], tNorm[1], tNorm[2]);

		bool isIntersect;
		GLKGeometry tempGeomClass;
		tNode->scalarField = 999999.99;
		for (GLKPOSITION facePos = refSurface->GetFaceList().GetHeadPosition(); facePos;) {
			QMeshFace* face = (QMeshFace*)refSurface->GetFaceList().GetNext(facePos);
			Eigen::Vector3d fNormal;
			face->GetNormal(fNormal[0],fNormal[1], fNormal[2]);

			if (fNormal.dot(tNormal) < 0.00) continue;

			double x[3], y[3], z[3];
			for (int k = 0; k < 3; k++) {
				QMeshNode* faceNode = face->GetNodeRecordPtr(k);
				faceNode->GetCoord3D(x[k], y[k], z[k]);
			}

			double A, B, C, D;
			face->GetPlaneEquation(A, B, C, D);

			double mu;
			isIntersect = tempGeomClass.CalLineFacetIntersection(tPoint, tNorm, mu, x, y, z, A, B, C, D);
			if (isIntersect && (mu>=0.0)) {
				if (mu < tNode->scalarField) {
					tNode->scalarField = mu;
					if (mu == 0) {
						std::cout << "zero mu" << std::endl;
					}
					
				}
				isIntersect = false;
			}
		}

		
	}


	

	std::ofstream distFile;
	distFile.open("C:/Users/neelo/Documents/distFile.txt", std::ios::out);
	for (int i = 0; i < numm; i++) {
		QMeshNode* tNode = TP_Vector[i];
		if (abs(tNode->scalarField) > 50.0) {
		//	tNode->isConstraint = true;
			tNode->scalarField = 0;
		}
		

		distFile << tNode->scalarField << std::endl;
	}
	distFile.close();


}

void RoughMachining::createVectorPatch(QMeshPatch* vectorPatch, bool showAll, bool colorPivots)
{
	int count = 0;
	int sourceCount = 0;
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);
		
		if (!showAll) {
			if (!tetra->isPivot) continue;
		}
		
		if (tetra->isPivot) {
			
			count++;
			sourceCount++;
			 if (!(sourceCount % 100 == 0)) continue;
			
		}
		else {
			count++;
			if (!(count % 100 == 0)) continue;
		}
		QMeshNode* newNode = new QMeshNode;
		double cx, cy, cz;
		tetra->CalCenterPos(cx, cy, cz);
		newNode->SetCoord3D(cx, cy, cz);

		int indexx = tetra->FieldIndexNumber;
		
		cx = VectorFieldMatrix(indexx, 0);
		cy = VectorFieldMatrix(indexx, 1);
		cz = VectorFieldMatrix(indexx, 2);

		newNode->SetNormal(cx, cy, cz);
		if (colorPivots) {
			if (tetra->isVectorFiled_Sorce) {
				newNode->SetColor(1.0, 0.6, 0.8);
			
				newNode->isConstraint = true;
			}
			else if (tetra->isPivot) {
				//newNode->isConstraint = true;
				
				
				newNode->SetColor(0.6, 0.6, 1.0);
				newNode->SetColor(1.0, 0.6, 0.8);
				newNode->isConstraint = true;
			}
			else {
				newNode->SetColor(0.8, 0.8, 1.0);
			}
		
			
		}

		vectorPatch->GetNodeList().AddTail(newNode);
	}

	std::ofstream outfile;
	outfile.open("C:/Users/neelo/Documents/vectorField.txt", std::ios::out);
	for (GLKPOSITION nodePos = vectorPatch->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* thisNode = (QMeshNode*)vectorPatch->GetNodeList().GetNext(nodePos);
		double px, py, pz, nx, ny, nz;
		thisNode->GetCoord3D(px, py, pz);
		thisNode->GetNormal(nx, ny, nz);
		float color[3];
		thisNode->GetColor(color[0], color[1], color[2]);
		outfile << px << " " << py << " " << pz << " " << nx << " " << ny << " " << nz << " " << color[0] << " " << color[1] << " " << color[2] << "\n";

	}
	outfile.close();

}





void RoughMachining::_createSolutionVectorForVectorSmoothing(int column, Eigen::VectorXd& u) {

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
			double vectorSize =		1.0;
			thisVector.stableNormalize();
			//u(num_tets + index) = 0.001 * thisVector(column);
			if (tetra->isPivot) { 
				if(tetra->isPivotA)
					u(num_tets + pivotInsertCount) = pow(10, propagateMag) * thisVector(column);
				else if (tetra->isPivotB)
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

void RoughMachining::_createSolutionVectorForVectorSmoothingPreserved(int column, Eigen::VectorXd& u)
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
			if (tetra->isPivot) {
				if (tetra->isPivotA)
					u(num_tets + tetra->FieldIndexNumber) = 1 * thisVector(column);
				if (tetra->isPivotB)
					u(num_tets + tetra->FieldIndexNumber) = 0.01* thisVector(column);
				else
					u(num_tets + tetra->FieldIndexNumber) = 0.01 * thisVector(column);
				
			}
			else {
				u(num_tets + tetra->FieldIndexNumber) = 0.01 * thisVector(column);
			}
			pivotInsertCount++;
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


void RoughMachining::_replaceVectors() {
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

	}
}

void RoughMachining::createGradientVectorField() {
	this->_generateVectorField();
}

void RoughMachining::optimiseVectorField(double toolLength, double margin) {
	std::cout << "Idenify pivot Tets\n";
	//_identifyPivotTets( toolLength, margin);

	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tet = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		VectorFieldMatrix(tet->FieldIndexNumber, 0) = 0;
		VectorFieldMatrix(tet->FieldIndexNumber, 1) = 1;
		VectorFieldMatrix(tet->FieldIndexNumber, 2) = 0;
		
		/*double x=0, y=0, z=0;
		double sum_ = 0;
		for (int i = 1; i < 5; i++) {
			QMeshNode* node_ = tet->GetNodeRecordPtr(i);
			node_->GetCoord3D(x, y, z);
			sum_ += x;
		}
		sum_ = sum_ / 80;
		VectorFieldMatrix(tet->FieldIndexNumber, 1) = 1*(abs(sum_)+1);*/
		continue;


		if (tet->isPivot) {
			//std::cout << "changing vector...\n";

			if (tet->isPivotA) {
				/************Test*********/
				int faceOnBoundCount = 0;
				QMeshFace* BoundFace;
				for (int l = 1; l < 5; l++) {
					QMeshFace* face = tet->GetFaceRecordPtr(l);
					if (face->isBoundary) {
						faceOnBoundCount++;
						BoundFace = face;
					}
				}
				if (faceOnBoundCount != 1) continue;
				double nx, ny, nz;
				BoundFace->CalPlaneEquation();
				BoundFace->GetNormal(nx, ny, nz);

				VectorFieldMatrix(tet->FieldIndexNumber, 0) = -nx;
				VectorFieldMatrix(tet->FieldIndexNumber, 1) = -ny;
				VectorFieldMatrix(tet->FieldIndexNumber, 2) = -nz;
			}
			else if(tet->isPivotB) {
				VectorFieldMatrix(tet->FieldIndexNumber, 0) = 0;
				VectorFieldMatrix(tet->FieldIndexNumber, 1) = 1;
				VectorFieldMatrix(tet->FieldIndexNumber, 2) = 0;
			}
			continue;
			/***************************/

			double nx, ny, nz;
			nx = VectorFieldMatrix(tet->FieldIndexNumber, 0);
			ny = VectorFieldMatrix(tet->FieldIndexNumber, 1);
			nz = VectorFieldMatrix(tet->FieldIndexNumber, 2); 

			Eigen::Vector3d dir1 ( 0,1,0 );
			Eigen::Vector3d norm(nx, ny, nz);

			Eigen::Vector3d dir2 = norm.cross(dir1);
			if (dir2.norm() < 1e-7) {
				continue;
			}
			dir2 = norm.cross(dir2);
			dir2.stableNormalize();
		
			if (dir2.y() < 0) dir2 = -1 * dir2;

			dir2 = 0.5 * (norm + dir2);
			dir2.normalize();

			VectorFieldMatrix(tet->FieldIndexNumber, 0) = dir2.x();
			VectorFieldMatrix(tet->FieldIndexNumber, 1) = dir2.y();
			VectorFieldMatrix(tet->FieldIndexNumber, 2) = dir2.z();
		}


	}

	std::cout << "smooth vectors\n";
	_smoothVectorField();
}

void RoughMachining::createScalarField() {
//	_generateScalarField();

}



void RoughMachining::_displayVectorField(QMeshPatch* vectorFieldPatch) {
	std::cout << "display vector field...\n";
	
	for (GLKPOSITION tetPos = materialSpace->GetTetraList().GetHeadPosition(); tetPos;) {
		QMeshTetra* tetra = (QMeshTetra*)materialSpace->GetTetraList().GetNext(tetPos);

		int index_ = tetra->FieldIndexNumber;
		//if ((index_ % 5) > 1e-2) continue;

		double x_mid=0, y_mid=0, z_mid=0;

		for (int i = 1; i <= 4; i++) {
			QMeshNode* node = tetra->GetNodeRecordPtr(i);
			double x, y, z;
			node->GetCoord3D(x, y, z);
			x_mid = x_mid + x;
			y_mid = y_mid + y;
			z_mid = z_mid + z;
		}
		x_mid = x_mid / 4.0;
		y_mid = y_mid / 4.0;
		z_mid = z_mid / 4.0; 

		QMeshNode* newNode = new QMeshNode;
		newNode->SetCoord3D(x_mid, y_mid, z_mid);
		vectorFieldPatch->GetNodeList().AddTail(newNode);
		int _index = tetra->FieldIndexNumber;
		newNode->SetNormal(VectorFieldMatrix(_index, 0), VectorFieldMatrix(_index, 1), VectorFieldMatrix(_index, 2));
		for (int i = 0; i < 3; i++) {
			tetra->vectorField(i) = VectorFieldMatrix(_index, i);
		}

		if (tetra->FieldIndexNumber % 30 == 0) std::cout <<"vector: " << tetra->vectorField.transpose() << std::endl;
	}
}


