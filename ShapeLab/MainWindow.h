#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"
#include <omp.h>
#include <QTimer>
#include <QLabel>
#include "RoughMachining.h"
#include "IsoLayerGeneration.h"
#include "toolpathGeneration.h"

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

using namespace std;

class DeformTet;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
	// Qtimer - defined function
    //void doTimerGcodeMoving();

private:
    Ui::MainWindow *ui;
    GLKLib *pGLK;

    /* add for Gcode generation */
    //QTimer Gcode_timer; //Gcode Simulation timer
    //int gocodetimerItertime;
    //int simuLayerInd;
    //Eigen::MatrixXf Gcode_Table;
    //unsigned int operationTime = 0;
    /* ------------------------ */
	GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();
	//void showTetraDeformationRatio();
	//void MoveHandleRegion();
	//void QTgetscreenshoot();

    PolygenMesh *getSelectedPolygenMesh();

    /*Added by Neelotpal*/
    void setBoundaryNodes(PolygenMesh* polymesh);


    QSignalMapper *signalMapper;
    QStandardItemModel *treeModel;

	DeformTet *Deformation;
    RoughMachining* RoughMach;
    PolygenMesh* isoLayerSet;
    double SectionMin_X;
    double SectionMax_X;
    double SectionMin_Z;
    double SectionMax_Z;
private:
    PolygenMesh* _buildPolygenMesh(mesh_type type, std::string name);
    PolygenMesh* _detectPolygenMesh(mesh_type type);

    /*New additions for vector field*/
    void _intr_updateVectorDirDisplay();
    QMeshNode* _getHandleNode();
    void _intr_setVectorDir(double nx, double ny, double nz);
    void _intr_translateVectorNode2Tet();
    void _extr_layer(double indexx, double* NormalDir);
    QMeshPatch* _extract_faces_with_normal(double* NormDir, QMeshPatch* surfacePatch);
    void _mark_Diff();
    void _deSelectHandle();
    QMeshFace* _getSelectedFace();
    
    


protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void open();
    void save();
	void saveSelection();
	void readSelection();

    void signalNavigation(int flag);
    void shiftToOrigin();
    void updateTree();
	void mouseMoveEvent(QMouseEvent *event);
    void on_pushButton_clearAll_clicked();
    void on_treeView_clicked(const QModelIndex &index);

	/*This is rough machining*/
    void input_machiningData();
    void initialise_data();
    void initiate_vector_on_bound();
    void set_guide_field();
    void propagate_vector_field();
    void detect_singularity();
    void correct_singularity();
    void compute_scalar_field();
    void compute_field_difference();
    void convert_2_GradField();
    void correct_Planar_Singularity();
    void curvedLayer_Generation();
    void toolPath_Generation();
    void correct_Along_Inner_Bound();



    /*This is for Display*/
    void change_isoLayer_Display();
    void all_isoLayer_Display();
    void update_Section_view();
    void update_vector_viz();

    void extr_layer();

    /*Interactive Vector Editing*/
    void int_displayVector();
    void intr_editVectorDir();
    void intr_setVectorDir();
    void intr_updateVectorDirDisplay();
    void intr_translateVectorNode2Tet();
    void intr_deSelectHandle();
    void intr_selectFacePatch();
    


    /*This for Workholding and Fixture Aware Machining*/
   



    /*For Heat Field Method <Added by Neelotpal>*/
    void create_heatField();


    /*NEW functions for vector field pipeline; to replace or as addition to old ones*/



    

};

#endif // MAINWINDOW_H
