#include "meshOperation.h"
#include "tetgen.h"

#include <iostream>
#include <fstream>      // std::ifstream
#include <cstring>

void meshOperation::tetMeshGeneration_singleSurfaceMesh(
    QMeshPatch* inputMesh, QMeshPatch* outputMesh, std::string tetgenCommand) {

    tetgenio in, out;
    tetgenio::facet* f;
    tetgenio::polygon* p;

    // All indices start from 0.
    //in.firstnumber = 1;

    /* fill node list - tetgen */
    in.numberofpoints = inputMesh->GetNodeNumber();
    in.pointlist = new REAL[in.numberofpoints * 3];

    int nodeIndex = 0;
    for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);
        Node->GetCoord3D(in.pointlist[3 * nodeIndex], in.pointlist[3 * nodeIndex + 1], in.pointlist[3 * nodeIndex + 2]);
        nodeIndex++;
    }

    /* fill face list - tetgen */
    in.numberoffacets = inputMesh->GetFaceNumber();
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];

    int faceIndex = 0;
    for (GLKPOSITION Pos = inputMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)inputMesh->GetFaceList().GetNext(Pos);

        f = &in.facetlist[faceIndex];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = NULL;

        p = &f->polygonlist[0];
        p->numberofvertices = 3;
        p->vertexlist = new int[p->numberofvertices];
        p->vertexlist[0] = thisFace->GetNodeRecordPtr(0)->GetIndexNo();
        p->vertexlist[1] = thisFace->GetNodeRecordPtr(1)->GetIndexNo();
        p->vertexlist[2] = thisFace->GetNodeRecordPtr(2)->GetIndexNo();

        faceIndex++;
    }

    /* Output the PLC to files 'barin.node' and 'barin.poly'. */ 
    //in.save_nodes((char*)"../barin");
    //in.save_poly((char*)"../barin");

    // Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
    //   do quality mesh generation (q) with a specified quality bound
    // 
    //   (1.414), and apply a maximum volume constraint (a0.1).

    //tetrahedralize((char*)"pq1.414a0.1", &in, &out);
    const char* inputCommand = tetgenCommand.c_str();

    tetrahedralize((char*)inputCommand, &in, &out);


    /* rebuild mesh from both node and element list */
    //patch->ClearAll();

    int nodeNum = out.numberofpoints;
    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    for (int i = 0; i < nodeNum * 3; i++) nodeTable[i] = out.pointlist[i];

    int tetraNum = out.numberoftetrahedra;
    unsigned int* tetTable = (unsigned int*)malloc(sizeof(unsigned int) * tetraNum * 4);
    for (int i = 0; i < tetraNum * 4; i++) tetTable[i] = out.tetrahedronlist[i];

    std::string path = "../model/newTet.tet";
    std::ofstream tetOutput(path);
    tetOutput << nodeNum << " vertices" << std::endl;
    tetOutput << tetraNum << " tets" << std::endl;
    for (int i = 0; i < nodeNum; i++) {
        tetOutput << nodeTable[i * 3] << " " << nodeTable[i * 3 + 1] << " " << nodeTable[i * 3 + 2] << std::endl;
    }
    for (int i = 0; i < tetraNum; i++) {
        tetOutput << "4 " << tetTable[i * 4] << " " << tetTable[i * 4 + 1] << " " << tetTable[i * 4 + 2] << " " << tetTable[i * 4 + 3] << std::endl;
    }
    tetOutput.close();

    delete nodeTable, tetTable;

    const char* myCharArr = path.c_str();
    outputMesh->inputTETFile((char*)myCharArr, true);

}

void meshOperation::tetMeshGeneration_outerSkin_Chamber(
    QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* outputMesh) {

    /* combine skin and chamber to an entire mesh */
    QMeshPatch* combinedMesh = new QMeshPatch;

    //this->_combineTwoSurfaceMesh(skin, chamber, outputMesh);
    this->_combineTwoSurfaceMesh(skin, chamber, combinedMesh);
    this->tetMeshGeneration_singleSurfaceMesh(combinedMesh, outputMesh, "Ya2.0");

}

void meshOperation::chamberSelection(QMeshPatch* tetMesh, QMeshPatch* chamber) {

    /* build tet list by tetMesh */
    std::vector<QMeshTetra*> tetraSet_materialSpace(tetMesh->GetTetraNumber());
    int tetIndex = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* thisTetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        tetraSet_materialSpace[tetIndex] = thisTetra; tetIndex++;
    }

    std::cout << "obj model contains face num = " << chamber->GetFaceNumber() << std::endl;
    //detect the center of tet element of materialSpace in/out the convexHull surface/ model surface

#pragma omp parallel   
    {
#pragma omp for 
        for (int i = 0; i < tetraSet_materialSpace.size(); i++) {
            QMeshTetra* each_Tetra = tetraSet_materialSpace[i];

            Eigen::Vector3d centerPos;
            each_Tetra->CalCenterPos(centerPos(0), centerPos(1), centerPos(2));
            each_Tetra->isChamber[0] = this->_calculatePointInsideMesh(chamber, centerPos);
            //std::cout << i << "-th element " << each_Tetra->chamberElement << std::endl;
        }
    }

    /* also find out the chamber face - for simulation usage */
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        int chamberTetNum = 0;
        if (thisFace->GetLeftTetra() != NULL && thisFace->GetRightTetra() != NULL) {
            if (thisFace->GetLeftTetra()->isChamber[0])     chamberTetNum++;
            if (thisFace->GetRightTetra()->isChamber[0])    chamberTetNum++;
        }
      
        if (chamberTetNum == 1) thisFace->isChamberBoundary = true;
    }
}

void meshOperation::_combineTwoSurfaceMesh(
    QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* combinedMesh) {

    int nodeNum = skin->GetNodeNumber() + chamber->GetNodeNumber();
    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

    int faceNum = skin->GetFaceNumber() + chamber->GetFaceNumber();
    unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

    /* build node list - skin and chamber mesh should not have intersection !!! */
    int nodeIndex = 0;  double pp[3];
    for (GLKPOSITION Pos = skin->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)skin->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);    
        Node->GetCoord3D(pp[0], pp[1], pp[2]);
        for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
        nodeIndex++;
    }

    for (GLKPOSITION Pos = chamber->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)chamber->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);
        Node->GetCoord3D(pp[0], pp[1], pp[2]);
        for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
        nodeIndex++;
    }

    /* build face list */
    int faceIndex = 0;
    for (GLKPOSITION Pos = skin->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)skin->GetFaceList().GetNext(Pos);
        for (int i = 0; i < 3; i++)
            faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->GetIndexNo();  
        faceIndex++;
    }

    for (GLKPOSITION Pos = chamber->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)chamber->GetFaceList().GetNext(Pos);
        for (int i = 0; i < 3; i++)
            faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->GetIndexNo();
        faceIndex++;
    }

    combinedMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

}

bool meshOperation::_calculatePointInsideMesh(QMeshPatch* target_mesh, Eigen::Vector3d& orig) {
    // calculate distance between Pnt with faces of model
    Eigen::Vector3d dir = { 1.0,0.0,0.0 };


    int intersection_Time = 0;




    for (GLKPOSITION Pos = target_mesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* each_face = (QMeshFace*)target_mesh->GetFaceList().GetNext(Pos);

        double xx, yy, zz;
        each_face->GetNodeRecordPtr(0)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v0 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(1)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v1 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(2)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v2 = { xx,yy,zz };


        if (this->_IntersectTriangle(orig, dir, v0, v1, v2))
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

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool meshOperation::_IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
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
