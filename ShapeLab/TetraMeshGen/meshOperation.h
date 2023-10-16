#pragma once

#include "PolygenMesh.h"

class meshOperation
{

public:
	meshOperation() {};
	~meshOperation() {};

	void tetMeshGeneration_singleSurfaceMesh(
		QMeshPatch* inputMesh, QMeshPatch* outputMesh, std::string tetgenCommand);
	
	void tetMeshGeneration_outerSkin_Chamber(
		QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* outputMesh);
	
	void chamberSelection(
		QMeshPatch* tetMesh, QMeshPatch* chamber);

	void updateOBJMesh_chamber_skin(QMeshPatch* tetMesh, QMeshPatch* skin, QMeshPatch* chamber);

private:
	void _combineTwoSurfaceMesh(
		QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* combinedMesh);

	bool _IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2);

	bool _calculatePointInsideMesh(QMeshPatch* target_mesh, Eigen::Vector3d& orig);

};

