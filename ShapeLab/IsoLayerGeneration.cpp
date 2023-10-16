#include "IsoLayerGeneration.h"
#include <fstream>

void IsoLayerGeneration::generateIsoSurface(PolygenMesh* isoSurface, int _layerNum, bool fixedIndex, double indexVal) {

	double isoCurveValue = 0.0;

	double layer_space = _layerNum;
	//if (max_distance > 1) max_distance = 1;

	int layerNum = (int)((max_distance - min_distance) / layer_space);
	std::cout << min_SingularDistance << "*________________* " << max_SingularDistance << std::endl;

	isInSingularZone = false;

	/**************Remove these****************/
//	layerNum = 30; //Remember to remove this
	//layer_space = 2;

	/*****************************************/
	


	std::cout << "max_distance: " << max_distance << std::endl;
	std::cout << "min_distance: " << min_distance << std::endl;
	//std::cout << "layer Num: " << layerNum << std::endl;



	double first_distance = 5.00;

	isoCurveValue = min_distance + 0.2 - layer_space;

	double maxBoundScalar = -9e10;
	for (GLKPOSITION nodePos = tetMesh->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* meshNode = (QMeshNode*)tetMesh->GetNodeList().GetNext(nodePos);
		if (meshNode->isInteriorBoundary) {
			if (maxBoundScalar < meshNode->scalarField) {
				maxBoundScalar = meshNode->scalarField;
			}
		}
	}


	int i = 0;
	if (fixedIndex) {
		QMeshPatch* layer;
		std::cout << "IndexVal\t" << indexVal << std::endl;
		layer = generatesingleIsoSurface(indexVal, isoSurface);

		if (layer->GetNodeNumber() == 0) {
			std::cout << "this layer have no node!" << std::endl;
		}
		else {
			layer->SetIndexNo(isoSurface->GetMeshList().GetCount());
			isoSurface->meshList.AddTail(layer);
			std::cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue << ", nodeNum = " << layer->GetNodeNumber() << std::endl;
		}
	}
	else {
		while (isoCurveValue <= max_distance) {
			//std::cout << "i: " << i << std::endl;
			/*if(i>20)
				isoCurveValue = isoCurveValue + 2 * layer_space;*/
			if (true)
				isoCurveValue = isoCurveValue + layer_space;
			else
				isoCurveValue = isoCurveValue + 1;
			/*if (i>2)
				isoCurveValue = isoCurveValue + 1.5 * layer_space;
			else
				isoCurveValue = isoCurveValue + 1;*/

			if (isoCurveValue > max_distance) break;

			i++;

			QMeshPatch* layer;

			if ((isoCurveValue >= min_SingularDistance) && (isoCurveValue <= max_SingularDistance)) {
				isInSingularZone = true;
				singularZoneIndex = isoCurveValue;
			}
			/*if (isoCurveValue > -0.6 && isoCurveValue < 0.6) {
				layer = hullMesh->CopyMesh();
				planeCutSurfaceMesh_delete(layer, 0, 1);
				planeCutSurfaceMesh_delete(layer, 30, 2);


			}
			else*/ layer = generatesingleIsoSurface(isoCurveValue, isoSurface);
			/*bool isValid = planeCutSurfaceMesh_delete(layer, 23, 3);
			if (!isValid) continue;
			else {
				isValid = planeCutSurfaceMesh_delete(layer, 56, 0);
				if (!isValid) continue;
				else {
					isValid = planeCutSurfaceMesh_delete(layer, 64, 1);
					if (!isValid) continue;
					else {
						isValid = planeCutSurfaceMesh_delete(layer, 40.5, 2);
						if (!isValid) continue;
						else {
							isValid = planeCutSurfaceMesh_delete(layer, 150, 4);
							if (!isValid) continue;
						}
					}
				}
			}*/



			if (layer->GetNodeNumber() == 0) {
				std::cout << "this layer have no node!" << std::endl; continue;
			}


			//index begin from 0
			layer->SetIndexNo(isoSurface->GetMeshList().GetCount());
			isoSurface->meshList.AddTail(layer);
			std::cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue << ", nodeNum = " << layer->GetNodeNumber() << std::endl;


		}

		reoOrderLayers(isoSurface);
	}

	

	if (isInSingularZone) {
		std::cout << " One layer in planar singularity prone area; Index = " << singularZoneIndex << std::endl;
	}
	//std::cout << "layer Num: " << layerNum << std::endl;

}


/*Main function for iso-surface substraction from tetrahedral model*/
QMeshPatch* IsoLayerGeneration::generatesingleIsoSurface(double isoValue, PolygenMesh* isoSurface)
{

	QMeshPatch* layer = new QMeshPatch;
	layer->isoSurfaceValue = isoValue;

	//when the node iso-value is equal to surface value, add this eps.
	double eps = 1.0e-5;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		//if (Node->in_ConvexHull) continue;

		if (abs(Node->scalarField - isoValue) < eps) {
			if (Node->scalarField > isoValue) Node->scalarField = isoValue + eps;
			else Node->scalarField = isoValue - eps;
		}
	}

	// build node list for isoSurface, this comes from the edge list
	for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);
		

		Edge->installedIsoNode = nullptr;
		Edge->isLocateIsoNode = false;
		
		if (Edge->in_convexHull) continue;



		double a = Edge->GetStartPoint()->scalarField;
		double b = Edge->GetEndPoint()->scalarField;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++)
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedTetEdge = Edge;
			isoNode->SetMeshPatchPtr(layer);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(layer->GetNodeList().GetCount() + 1);
			layer->GetNodeList().AddTail(isoNode);

			Edge->installedIsoNode = isoNode;
			Edge->isLocateIsoNode = true;

		}
	}


	// build edge list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);

		

		Face->installedIsoEdge = nullptr;
		Face->isLocatedIsoEdge = false;
		if(Face->in_convexHull) continue;


		int positiveNum = 0;
		for (int i = 0; i < 3; i++) {
			if (Face->GetNodeRecordPtr(i)->scalarField > isoValue) positiveNum++;
		}
		
		if (positiveNum == 0 || positiveNum == 3) continue;
		
		else if (positiveNum == 1) {
			QMeshEdge* isoEdge = new QMeshEdge;

			//detect which node is positive
			QMeshNode* PostiveNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				PostiveNode = Face->GetNodeRecordPtr(index);
				if (PostiveNode->scalarField > isoValue) break;
			}
	

			QMeshEdge* Edge = Face->GetEdgeRecordPtr(index + 1);
			if (Edge->in_convexHull) { std::cout << "Got the error!!!\n";   }
			
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			if (Edge->in_convexHull) { std::cout << "Got the error!!!\n"; }
		
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
		else if (positiveNum == 2) {
			QMeshEdge* isoEdge = new QMeshEdge;
			//detect which node is negative
			QMeshNode* NegativeNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				NegativeNode = Face->GetNodeRecordPtr(index);
				if (NegativeNode->scalarField < isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
	}



	
	// build face list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		if (Tetra->isIn_convexHull) continue; 

		int isoEdgeNum = 0;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) isoEdgeNum++;
		}
		if (isoEdgeNum == 0) continue;
		else if (isoEdgeNum == 2 || isoEdgeNum == 1) std::cout << "Error! isoEdgeNum cannot equal to 1 or 2!" << std::endl << std::endl;
		else if (isoEdgeNum == 3) {
			QMeshFace* isoFace = new QMeshFace;
			
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(3);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) {
				if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) {
					FaceList[faceIndex] = Tetra->GetFaceRecordPtr(i + 1);
					faceIndex++;
				}
			}

			//sorting
			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
			if (firstDir == true) {
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}
			else {
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}

			//using the first face and add its edge into the isoFace.
			for (int i = 0; i < 3; i++) {
				isoFace->SetEdgeRecordPtr(i, FaceList[i]->installedIsoEdge);
				isoFace->SetDirectionFlag(i, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])));
				if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
					FaceList[i]->installedIsoEdge->SetLeftFace(isoFace);
				else FaceList[i]->installedIsoEdge->SetRightFace(isoFace);
			}

			//push this isoFace back to layer and compute norm
			isoFace->SetEdgeNum(3);
			isoFace->CalPlaneEquation();
			isoFace->SetMeshPatchPtr(layer);
			isoFace->SetIndexNo(layer->GetFaceList().GetCount() + 1);
			layer->GetFaceList().AddTail(isoFace);
		}

		else if (isoEdgeNum == 4) {
			std::vector<QMeshFace*> isoFace; isoFace.resize(2);
			for (int i = 0; i < 2; i++) {
				isoFace[i] = new QMeshFace;
			}
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(4);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) FaceList[i] = Tetra->GetFaceRecordPtr(i + 1);

			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
																						//sorting edge
			if (firstDir == true) {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);
			}
			else {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);

			}
			////test the sorting
			//cout << Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])) << endl;
			//for (int i = 0; i < 4; i++) {
			//	cout << FaceList[i]->installedIsoEdge->GetStartPoint()->GetIndexNo() << " , " <<
			//		FaceList[i]->installedIsoEdge->GetEndPoint()->GetIndexNo() << endl;	
			//}

			QMeshEdge* midEdge1 = new QMeshEdge;
			midEdge1->isMiddleEdge1 = true;
			if (firstDir == true) {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge1->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge1->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/


			QMeshEdge* midEdge2 = new QMeshEdge;
			midEdge2->isMiddleEdge = true;
			if (firstDir == true) {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());

				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());

				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge2->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge2->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/

			//midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
			/*
			bool dir = false;
			if (FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetStartPoint());
			dir = true;
			}
			else if(FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetEndPoint());
			else cout << "Error!" << endl;*/

			QMeshEdge* midEdge;

			if (midEdge1->CalLength() <= midEdge2->CalLength()) {

				midEdge = midEdge1;
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, FaceList[1]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[0]->SetEdgeRecordPtr(2, midEdge);
				isoFace[0]->SetDirectionFlag(2, false);

				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 1) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetRightFace(isoFace[0]);

				isoFace[1]->SetEdgeRecordPtr(0, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[3]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, true);
				for (int i = 0; i < 4; i++) {
					if (i == 2 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[1]);
			}

			else {
				midEdge = midEdge2;
				//first triangle
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, midEdge);
				isoFace[0]->SetDirectionFlag(1, true);
				isoFace[0]->SetEdgeRecordPtr(2, FaceList[3]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(2, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[0]);

				//first triangle
				isoFace[1]->SetEdgeRecordPtr(0, FaceList[1]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, false);
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 1 || i == 2) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[1]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[1]);
					}
				}
				midEdge->SetRightFace(isoFace[1]);
			}

			//push back midEdge
			midEdge->SetMeshPatchPtr(layer);
			midEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge);


			/*midEdge1->SetMeshPatchPtr(layer);
			midEdge1->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge1);*/

			//midEdge2->SetMeshPatchPtr(layer);
			//midEdge2->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			//layer->GetEdgeList().AddTail(midEdge2);



			//push this isoFace back to layer and compute norm
			for (int i = 0; i < 2; i++) {
				isoFace[i]->SetEdgeNum(3);
				isoFace[i]->CalPlaneEquation();
				isoFace[i]->SetMeshPatchPtr(layer);
				isoFace[i]->SetIndexNo(layer->GetFaceList().GetCount() + 1);
				layer->GetFaceList().AddTail(isoFace[i]);
			}
		}
	}


	//give each node face list
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);

		//if (Face->in_convexHull) continue;

		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetFaceList().AddTail(Face);
		}
	}

	return layer;
}

void IsoLayerGeneration::smoothingIsoSurface(PolygenMesh* isoSurface) {

	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* thisEdge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);
			if (thisEdge->IsBoundaryEdge()) {
				thisEdge->GetStartPoint()->isoSurfaceBoundary = true;
				thisEdge->GetEndPoint()->isoSurfaceBoundary = true;
			}
		}

		//laplacian smoothness
		for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

			if (thisNode->isoSurfaceBoundary) continue;
			else {
				double pp[3] = { 0 }; int neighNum = 0;
				for (GLKPOSITION Pos = thisNode->GetEdgeList().GetHeadPosition(); Pos;) {
					QMeshEdge* neighEdge = (QMeshEdge*)thisNode->GetEdgeList().GetNext(Pos);

					QMeshNode* neighNode = neighEdge->GetStartPoint();
					if (neighNode == thisNode) neighNode = neighEdge->GetEndPoint();

					double p1[3];
					neighNode->GetCoord3D(p1[0], p1[1], p1[2]);

					for (int i = 0; i < 3; i++) pp[i] += p1[i];
					neighNum++;
				}
				for (int i = 0; i < 3; i++) pp[i] /= neighNum;
				thisNode->SetCoord3D(pp[0], pp[1], pp[2]);
			}
		}

	}
}

void IsoLayerGeneration::outputSurfaceMesh(PolygenMesh* isoSurface, bool isRun, std::string outFolderName, int _fileOffset) {

	if (!isRun) return;

	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);


		std::string LAYER_dir = "../DataSet/CURVED_LAYER/IN/" + std::to_string(each_layer->GetIndexNo() + _fileOffset);

		if (_fileOffset == 0)
			LAYER_dir = "../DataSet/CURVED_LAYER/OUT/"  + std::to_string(each_layer->GetIndexNo()+_fileOffset);

			
		output_OneSurfaceMesh(each_layer, LAYER_dir);

		
	}
	std::cout << "Finish output layers into : " << ".. / DataSet / CURVED_LAYER /" << std::endl;
}

void IsoLayerGeneration::output_OneSurfaceMesh(QMeshPatch* isoSurface, std::string path) {

	double pp[3];
	path += ".obj";
	std::ofstream nodeSelection(path);

	int index = 0;
	for (GLKPOSITION posNode = isoSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)isoSurface->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posFace = isoSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(posFace);
		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	nodeSelection.close();
}

void IsoLayerGeneration::reoOrderLayers(PolygenMesh* isoSurface)
{
	std::vector<QMeshPatch*> newMeshListVector;
	int TotalCount = isoSurface->GetMeshList().GetCount();
	newMeshListVector.reserve(TotalCount);
	for (GLKPOSITION meshPos = isoSurface->GetMeshList().GetHeadPosition(); meshPos;) {
		QMeshPatch* meshPatch = (QMeshPatch*)isoSurface->GetMeshList().GetNext(meshPos);
		newMeshListVector.push_back(meshPatch);

	}


	isoSurface->GetMeshList().RemoveAll();


	for (auto iterator_ = newMeshListVector.rbegin(); iterator_ != newMeshListVector.rend(); iterator_++) {
		QMeshPatch* insertPatch = *iterator_;
		insertPatch->SetIndexNo(isoSurface->GetMeshList().GetCount());
		isoSurface->GetMeshList().AddTail(insertPatch);
	}
}

bool IsoLayerGeneration::planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh, double last_dist, int dirIndex) {

	//printf("--- begin cut the mesh by convex hull plane \n\n");

	int cutFaceCase[3][9] = {
		{ 1,4,5,1,2,4,4,3,5 },
		{ 1,4,5,4,2,5,2,3,5 },
		{ 1,4,5,4,2,5,1,5,3 } };

	// Plane equation: Ax+By+Cz+D = 0
	Eigen::Vector3d PlaneDir; double D = last_dist;

	switch (dirIndex) {
	case 0: 
		PlaneDir << 1.0, 0.0, 0.0; 
		break;
	case 1: 
		PlaneDir << -1.0, 0.0, 0.0;
		break;
	case 2: 
		PlaneDir << 0.0, 1.0, 0.0;
		break;
	case 3: 
		PlaneDir << 0.0, -1.0, 0.0;
		break;
	case 4: 
		PlaneDir << 0.0, 0.0, 1.0;
		break;
	case 5: 
		PlaneDir << 0.0, 0.0, -1.0;
		break;
	}
	
	//std::cout << PlaneDir[0] << " " << PlaneDir[1] << " " << PlaneDir[2] << std::endl;
	//--------------------------------------------------------------
	/* pre-process to compute node-plane distance*/

	int positiveNodeNum = 0; int negativeNodeNum = 0;

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Eigen::Vector3d pp; Node->GetCoord3D(pp(0), pp(1), pp(2));
		Node->nodePlaneDis = pp.dot(PlaneDir) - D;

		if (Node->nodePlaneDis > 0) {
			positiveNodeNum++; Node->SetIndexNo(-1);
			
		}
		else {
			Node->SetIndexNo(negativeNodeNum); negativeNodeNum++; // node index start from 0
		}
	}
	

	if (positiveNodeNum == 0) return true; //do not do anything
	else if (negativeNodeNum == 0) {
		//surfaceMesh->ClearAll();
		return false; //delete this layer
	}
	//--------------------------------------------------------------
	/* detect the intersect edge, build intersect node*/
	std::cout << "Cutting...\n";

	int cutNodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* thisEdge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

		double a = thisEdge->GetStartPoint()->nodePlaneDis;
		double b = thisEdge->GetEndPoint()->nodePlaneDis;
		if (a * b > 0) continue;

		double alpha = fabs(a) / fabs(b - a);
		double p1[3], p2[3], pp[3];
		thisEdge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
		thisEdge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

		for (int j = 0; j < 3; j++)
			pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];

		QMeshNode* isoNode = new QMeshNode;
		isoNode->SetMeshPatchPtr(surfaceMesh);
		isoNode->SetCoord3D(pp[0], pp[1], pp[2]);

		//std::cout << pp[0] << "," << pp[1] << "," << pp[2] << std::endl;

		surfaceMesh->GetNodeList().AddTail(isoNode);
		isoNode->planeCutNewNode = true;

		isoNode->SetIndexNo(negativeNodeNum + cutNodeIndex); // index start from 0
		cutNodeIndex++;

		thisEdge->intersectNodeIndex = isoNode->GetIndexNo(); // if exist, the node index will large than 0
	}
	//std::cout << "finish generate the cutting node" << std::endl;

	///* Build topology */

	// -- node table
	float* nodeTable;
	int nodeNum = surfaceMesh->GetNodeNumber() - positiveNodeNum;
	nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo() < 0) continue; //delete those face have node on the left plane
		double pp[3]; Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) nodeTable[Node->GetIndexNo() * 3 + i] = (float)pp[i];

		//std::cout << pp[0] << "," << pp[1] << "," << pp[2] << std::endl;

	}

	// -- face table
	int faceNum = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		int face_posNodeNum = 0;
		for (int i = 0; i < 3; i++) {
			if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) face_posNodeNum++;
		}
		if (face_posNodeNum == 0 || face_posNodeNum == 2) faceNum += 1;
		else if (face_posNodeNum == 1) faceNum += 2;
	}

	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

	int faceNodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		int face_posNodeNum = 0;
		for (int i = 0; i < 3; i++) {
			if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) face_posNodeNum++;
		}

		if (face_posNodeNum == 3) continue;
		else if (face_posNodeNum == 0) {
			for (int i = 0; i < 3; i++)
				faceTable[faceNodeIndex + i] = face->GetNodeRecordPtr(i)->GetIndexNo();
			faceNodeIndex += 3; continue;
		}

		int nodeIndexArray[5] = { 0 }; int noCutEdgeIndex = -1; int cutEdgeIndex = 0;
		for (int i = 0; i < 3; i++)
			nodeIndexArray[i] = face->GetNodeRecordPtr(i)->GetIndexNo();
		for (int i = 0; i < 3; i++) {
			if (face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex < 0) {
				noCutEdgeIndex = i; continue;
			}
			else {
				nodeIndexArray[3 + cutEdgeIndex] = face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex;
				cutEdgeIndex++;
			}
		}

		if (face_posNodeNum == 1) {

			int positiveNodeIndex = -1;
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) positiveNodeIndex = i;
			}

			for (int i = 0; i < 3; i++) { //case
				if (noCutEdgeIndex == i) {

					int addFaceIndex = 0;
					for (int j = 0; j < 3; j++) { //face

						bool keepThisFace = true;
						for (int k = 0; k < 3; k++) {
							int currentNodeIndex = cutFaceCase[i][3 * j + k] - 1;
							if (currentNodeIndex == positiveNodeIndex) keepThisFace = false;
						}
						if (keepThisFace == false) continue;

						for (int k = 0; k < 3; k++)  //node
							faceTable[faceNodeIndex + 3 * addFaceIndex + k] = nodeIndexArray[cutFaceCase[i][3 * j + k] - 1];
						addFaceIndex++;
					}
				}
			}

			faceNodeIndex += 6;
		}

		else if (face_posNodeNum == 2) {

			int negativeNodeIndex = -1;
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->nodePlaneDis < 0) negativeNodeIndex = i;
			}

			for (int i = 0; i < 3; i++) { //case
				if (noCutEdgeIndex == i) {
					for (int j = 0; j < 3; j++) { //face

						bool keepThisFace = false;
						for (int k = 0; k < 3; k++) {
							int currentNodeIndex = cutFaceCase[i][3 * j + k] - 1;
							if (currentNodeIndex == negativeNodeIndex) keepThisFace = true;
						}
						if (keepThisFace == false) continue;
						for (int k = 0; k < 3; k++)  //node
							faceTable[faceNodeIndex + k] = nodeIndexArray[cutFaceCase[i][3 * j + k] - 1];
					}
				}
			}
			faceNodeIndex += 3;
		}
	}

	surfaceMesh->ClearAll();
	surfaceMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	free(nodeTable);
	free(faceTable);

	return true;
	//printf("--- finish cut mesh by plane ---------");
}