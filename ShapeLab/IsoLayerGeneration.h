#pragma once
class PolygenMesh;
class QMeshPatch;
#include "../QMeshLib/PolygenMesh.h"

class IsoLayerGeneration {
public:
    IsoLayerGeneration(QMeshPatch* Patch) { tetMesh = Patch; }
    ~IsoLayerGeneration() {};

    void generateIsoSurface(PolygenMesh* isoSurface, int layerNum, bool fixedIndex = false, double indexVal = 0.0);
    void smoothingIsoSurface(PolygenMesh* isoSurface);
    bool planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh, double last_dist, int dirIndex);
    void outputSurfaceMesh(PolygenMesh* isoSurface, bool isRun, std::string outFolderName, int _fileOffset);
    double max_distance = 0.00; //added by Neelotpal
    double min_distance = 0.00; //added by Neelotpal
    double min_SingularDistance = 99e15; //added by Neelotpal
    double max_SingularDistance = 99e15;  //added by Neelotpal
    bool isInSingularZone = false;
    double singularZoneIndex = -99e10;

private:
    QMeshPatch* generatesingleIsoSurface(double isoValue, PolygenMesh* isoSurface);
    void output_OneSurfaceMesh(QMeshPatch* isoSurface, std::string path);
    void reoOrderLayers(PolygenMesh* isoSurface);

    QMeshPatch* tetMesh = NULL;
    
};