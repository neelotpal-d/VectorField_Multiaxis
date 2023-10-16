#include "stdafx.h"

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <QtDebug>
#include <QDesktopWidget>
#include <QCoreApplication>
#include <QMimeData>
#include <QTreeView>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QScreen>
#include <QStyleFactory>
#include <fstream>
#include<time.h>

#include "../GLKLib/GLKCameraTool.h"
#include "../GLKLib/InteractiveTool.h"
#include "../GLKLib/GLKMatrixLib.h"
#include "../GLKLib/GLKGeometry.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshTetra.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

#include "alphanum.hpp"
#include <dirent.h>

#include "./TetraMeshGen/meshOperation.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	QApplication::setStyle(QStyleFactory::create("Fusion"));
    signalMapper = new QSignalMapper(this);
    addToolBar(ui->toolBar);
    addToolBar(ui->navigationToolBar);
    addToolBar(ui->selectionToolBar);

    createTreeView();
    createActions();

    pGLK = new GLKLib();
    ui->horizontalLayout->addWidget(pGLK);
    ui->horizontalLayout->setMargin(0);
    pGLK->setFocus();
    
    pGLK->clear_tools();
    pGLK->set_tool(new GLKCameraTool(pGLK,ORBITPAN));
	
	//connect timer with timer function
	//connect(&Gcode_timer, SIGNAL(timeout()), this, SLOT(doTimerGcodeMoving()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::createActions()
{
    // file IO
    connect(ui->actionOpen, SIGNAL(triggered(bool)), this, SLOT(open()));
    connect(ui->actionSave, SIGNAL(triggered(bool)), this, SLOT(save()));
	connect(ui->actionSaveSelection, SIGNAL(triggered(bool)), this, SLOT(saveSelection()));
	connect(ui->actionReadSelection, SIGNAL(triggered(bool)), this, SLOT(readSelection()));

    // navigation
    connect(ui->actionFront, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBack, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionTop, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBottom, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionLeft, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionRight, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionIsometric, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_In, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Out, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_All, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Window, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionFront, 0);
    signalMapper->setMapping (ui->actionBack, 1);
    signalMapper->setMapping (ui->actionTop, 2);
    signalMapper->setMapping (ui->actionBottom, 3);
    signalMapper->setMapping (ui->actionLeft, 4);
    signalMapper->setMapping (ui->actionRight, 5);
    signalMapper->setMapping (ui->actionIsometric, 6);
    signalMapper->setMapping (ui->actionZoom_In, 7);
    signalMapper->setMapping (ui->actionZoom_Out, 8);
    signalMapper->setMapping (ui->actionZoom_All, 9);
    signalMapper->setMapping (ui->actionZoom_Window, 10);

    // view
    connect(ui->actionShade, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionMesh, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionProfile, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionFaceNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNodeNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionShade, 20);
    signalMapper->setMapping (ui->actionMesh, 21);
    signalMapper->setMapping (ui->actionNode, 22);
    signalMapper->setMapping (ui->actionProfile, 23);
    signalMapper->setMapping (ui->actionFaceNormal, 24);
    signalMapper->setMapping (ui->actionNodeNormal, 25);
    ui->actionShade->setChecked(true);

    connect(ui->actionShifttoOrigin, SIGNAL(triggered(bool)), this, SLOT(shiftToOrigin()));

    // select
    connect(ui->actionSelectNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectEdge, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectFace, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectFix, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectHandle, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));

	signalMapper->setMapping (ui->actionSelectNode, 30);
    signalMapper->setMapping (ui->actionSelectEdge, 31);
    signalMapper->setMapping (ui->actionSelectFace, 32);
	signalMapper->setMapping(ui->actionSelectFix, 33);
	signalMapper->setMapping(ui->actionSelectHandle, 34);


    connect (signalMapper, SIGNAL(mapped(int)), this, SLOT(signalNavigation(int)));

	//Button
    connect(ui->pushButton_readData, SIGNAL(released()), this, SLOT(input_machiningData()));
    connect(ui->pushButton_initialise, SIGNAL(released()), this, SLOT(initialise_data()));
    connect(ui->pushButton_setBoundaryVectors, SIGNAL(released()), this, SLOT(initiate_vector_on_bound()));
    connect(ui->pushButton_propagateField, SIGNAL(released()), this, SLOT(propagate_vector_field()));
    connect(ui->pushButton_generateScalarField, SIGNAL(released()), this, SLOT(compute_scalar_field()));
    connect(ui->pushButton_setGuideFields, SIGNAL(released()), this, SLOT(set_guide_field()));
    connect(ui->pushButton_correctSingularity, SIGNAL(released()), this, SLOT(correct_singularity()));
    connect(ui->pushButton_convert2GradField, SIGNAL(released()), this, SLOT(convert_2_GradField()));
    connect(ui->pushButton_detectSingularity, SIGNAL(released()), this, SLOT(detect_singularity()));
    connect(ui->pushButton_checkFieldDifference, SIGNAL(released()), this, SLOT(compute_field_difference()));

    connect(ui->pushButton_isoLayerGeneration, SIGNAL(released()), this, SLOT(curvedLayer_Generation()));
    connect(ui->pushButton_toolPathGeneration, SIGNAL(released()), this, SLOT(toolPath_Generation()));
    
    connect(ui->pushButton_ShowAllLayers, SIGNAL(released()), this, SLOT(all_isoLayer_Display()));
    connect(ui->spinBox_ShowLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(change_isoLayer_Display()));
    //connect(ui->radioButton_SectionX, SIGNAL(toggled()), this, SLOT(update_Section_view()));
    connect(ui->pushButton_updateSection, SIGNAL(released()), this, SLOT(update_Section_view()));
    connect(ui->pushButton_clearAll, SIGNAL(released()), this, SLOT(on_pushButton_clearAll_clicked()));

    connect(ui->pushButton_intr_editVectorDirection, SIGNAL(released()), this, SLOT(intr_editVectorDir()));
    connect(ui->pushButton_intr_getVectorDirection, SIGNAL(released()), this, SLOT(intr_updateVectorDirDisplay()));
    connect(ui->pushButton_intr_setVectorDirection, SIGNAL(released()), this, SLOT(intr_setVectorDir()));
    connect(ui->pushButton_intr_Vector_Node2Tets, SIGNAL(released()), this, SLOT(intr_translateVectorNode2Tet()));
    connect(ui->pushButton_correctPlanarSingularity, SIGNAL(released()), this, SLOT(correct_Planar_Singularity()));
    connect(ui->pushButton_extractLayer, SIGNAL(released()), this, SLOT(extr_layer()));
    connect(ui->pushButton_correctInnerBound, SIGNAL(released()), this, SLOT(correct_Along_Inner_Bound()));
    connect(ui->pushButton_updateVecViz, SIGNAL(released()), this, SLOT(update_vector_viz()));
    connect(ui->pushButton_deSelectNode, SIGNAL(released()), this, SLOT(intr_deSelectHandle()));
    connect(ui->pushButton_facePatchSelect, SIGNAL(released()), this, SLOT(intr_selectFacePatch()));
}

void MainWindow::open()
{
    QString filenameStr = QFileDialog::getOpenFileName(this, tr("Open File,"), "..", tr(""));
    QFileInfo fileInfo(filenameStr);
    QString fileSuffix = fileInfo.suffix();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    
    if (QString::compare(fileSuffix,"obj") == 0){
        PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);
        polygenMesh->ImportOBJFile(filename,modelName);
        polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
        pGLK->AddDisplayObj(polygenMesh,true);
        polygenMeshList.AddTail(polygenMesh);
    }

	else if (QString::compare(fileSuffix, "tet") == 0) {
		PolygenMesh *polygenMesh = new PolygenMesh(TET_MODEL);
		std::cout << filename << std::endl;
		std::cout << modelName << std::endl;
		polygenMesh->ImportTETFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
	}

    updateTree();

    shiftToOrigin();
    pGLK->refresh(true);
}

void MainWindow::save()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (!polygenMesh)
		return;
	QString filenameStr = QFileDialog::getSaveFileName(this, tr("OBJ File Export,"), "..", tr("OBJ(*.obj)"));
	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();

	if (QString::compare(fileSuffix, "obj") == 0) {
		QFile exportFile(filenameStr);
		if (exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
			QTextStream out(&exportFile);
			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
					QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
					double xx, yy, zz;
					node->GetCoord3D(xx, yy, zz);
					float r, g, b;
					node->GetColor(r, g, b);
					out << "v " << xx << " " << yy << " " << zz << " " << node->value1 << endl;
				}
				for (GLKPOSITION posFace = patch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
					QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(posFace);
					out << "f " << face->GetNodeRecordPtr(0)->GetIndexNo() << " " << face->GetNodeRecordPtr(1)->GetIndexNo() << " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
				}
			}
		}
		exportFile.close();
	}
}

void MainWindow::saveSelection()
{
	//printf("%s exported\n", Model->ModelName);

	/*PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)*/
    PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
    if (polygenMesh == NULL || polygenMesh->meshType != TET_MODEL) {
        std::printf(" -- system contains no tet model, return! \n");
        return;
    }
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();
	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char output_filename[256];
	strcpy(output_filename, "..\\selection_file\\");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(output_filename, filetype);

	ofstream nodeSelection(output_filename);
	if (!nodeSelection)
		cerr << "Sorry!We were unable to build the file NodeSelect!\n";
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		nodeSelection << CheckNode->GetIndexNo() << ":";
		//for the selection of fixing part
		if (CheckNode->isFixed == true) nodeSelection << "1:";
		else nodeSelection << "0:";
		//for the selection of hard part
		if (CheckNode->isHandle == true) nodeSelection << "1:" << endl;
		else nodeSelection << "0:" << endl;
	}

	nodeSelection.close();
	printf("Finish output selection \n");
}

void MainWindow::readSelection()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();

	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
	strcpy(input_filename, "..\\selection_file\\");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(input_filename, filetype);

	ifstream nodeSelect(input_filename);
	if (!nodeSelect)
		cerr << "Sorry!We were unable to open the file!\n";
	vector<int> NodeIndex(patch->GetNodeNumber()), checkNodeFixed(patch->GetNodeNumber()), checkNodeHandle(patch->GetNodeNumber());
	//string line;
	int LineIndex1 = 0;
	string sss;
	while (getline(nodeSelect, sss)){
		const char * c = sss.c_str();
		sscanf(c, "%d:%d:%d", &NodeIndex[LineIndex1], &checkNodeFixed[LineIndex1], &checkNodeHandle[LineIndex1]);
		LineIndex1++;
	}

	nodeSelect.close();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (checkNodeFixed[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isFixed = true;
		if (checkNodeHandle[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isHandle = true;
	}

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)
			Face->isHandleDraw = true;
		else Face->isHandleDraw = false;

		if (Face->GetNodeRecordPtr(0)->isFixed == true &&
			Face->GetNodeRecordPtr(1)->isFixed == true &&
			Face->GetNodeRecordPtr(2)->isFixed == true)
			Face->isFixedDraw = true;
		else Face->isFixedDraw = false;
	}
	printf("Finish input selection \n");
	pGLK->refresh(true);

}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
	//QMouseEvent *e = (QMouseEvent*)event;
	//QPoint pos = e->pos();
	//cout << "Mouse position updated" << endl;
	//double wx, wy, wz;
	//pGLK->screen_to_wcl(100.0, 100.0, wx, wy, wz);
	//ui->CorrdinateMouse->setText(QString("X = %1").arg(wx));

	//QString text;
	//text = QString("%1 X %2").arg(event->pos().x()).arg(event->pos().y());
	///** Update the info text */
	//ui->statusBar->showMessage(text);
}

void MainWindow::signalNavigation(int flag)
{
    if (flag <= 10)
        pGLK->setNavigation(flag);
    if (flag >=20 && flag <=25){
        pGLK->setViewModel(flag-20);
        switch (flag) {
        case 20:
            ui->actionShade->setChecked(pGLK->getViewModel(0));
            break;
        case 21:
            ui->actionMesh->setChecked(pGLK->getViewModel(1));
            break;
        case 22:
            ui->actionNode->setChecked(pGLK->getViewModel(2));
            break;
        case 23:
            ui->actionProfile->setChecked(pGLK->getViewModel(3));
            break;
        case 24:
            ui->actionFaceNormal->setChecked(pGLK->getViewModel(4));
            break;
        case 25:
            ui->actionNodeNormal->setChecked(pGLK->getViewModel(5));
            break;
        }
    }
    if (flag==30 || flag==31 || flag==32 || flag == 33 || flag == 34){
        InteractiveTool *tool;
        switch (flag) {
        case 30:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NODE, ui->boxDeselect->isChecked());
            break;
        case 31:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), EDGE, ui->boxDeselect->isChecked());
            break;
        case 32:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FACE, ui->boxDeselect->isChecked());
            break;
		case 33:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FIX, ui->boxDeselect->isChecked());
			break;
		case 34:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NHANDLE, ui->boxDeselect->isChecked());
			break;
        }
        pGLK->set_tool(tool);
    }
}

void MainWindow::shiftToOrigin()
{
    
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls())
        event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent *event)
{
    QString filenameStr;
    foreach (const QUrl &url, event->mimeData()->urls())
        filenameStr = url.toLocalFile();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    int i = 0;
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
        std::string name = (polygen->getModelName()).substr(0,(polygen->getModelName()).find(' '));
        if (name == modelName)
            i++;
    }
    if (i > 0)
        modelName += " "+std::to_string(i);

	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();
	if (QString::compare(fileSuffix, "obj") == 0) {
		polygenMesh->ImportOBJFile(filename, modelName);
	}
	else if (QString::compare(fileSuffix, "tet") == 0) {
		polygenMesh->ImportTETFile(filename, modelName);
        polygenMesh->meshType = TET_MODEL;
	}
	polygenMesh->m_bVertexNormalShading = false;	
    polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(polygenMesh,true);
    polygenMeshList.AddTail(polygenMesh);
    
    updateTree();
}

void MainWindow::createTreeView()
{
    treeModel = new QStandardItemModel();
    ui->treeView->setModel(treeModel);
    ui->treeView->setHeaderHidden(true);
    ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->treeView->expandAll();
}

void MainWindow::updateTree()
{
    treeModel->clear();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        QStandardItem *modelListItem = new QStandardItem(modelName);
        modelListItem->setCheckable(true);
        modelListItem->setCheckState(Qt::Checked);
        treeModel->appendRow(modelListItem);
    }
	pGLK->refresh(true);
}

PolygenMesh *MainWindow::getSelectedPolygenMesh()
{
    if (!treeModel->hasChildren())
        return nullptr;
    QModelIndex index = ui->treeView->currentIndex();
    QString selectedModelName = index.data(Qt::DisplayRole).toString();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        if (QString::compare(selectedModelName,modelName) == 0)
            return polygenMesh;
    }
    return nullptr;
}

void MainWindow::on_pushButton_clearAll_clicked()
{
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        pGLK->DelDisplayObj(polygenMesh);
    }
    polygenMeshList.RemoveAll();
    RoughMach = NULL;
    isoLayerSet = NULL;
    pGLK->ClearDisplayObjList();
    pGLK->refresh();
    updateTree();
}

void MainWindow::on_treeView_clicked(const QModelIndex &index)
{
    ui->treeView->currentIndex();
    QStandardItem *modelListItem = treeModel->itemFromIndex(index);
    ui->treeView->setCurrentIndex(index);
    PolygenMesh *polygenMesh = getSelectedPolygenMesh();
    if (modelListItem->checkState() == Qt::Checked)
        polygenMesh->bShow = true;
    else
        polygenMesh->bShow = false;
    pGLK->refresh(true);
}

PolygenMesh* MainWindow::_buildPolygenMesh(mesh_type type, std::string name) {

    PolygenMesh* newMesh = new PolygenMesh(type);
    newMesh->setModelName(name);
    newMesh->BuildGLList(newMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(newMesh, true);
    polygenMeshList.AddTail(newMesh);
    updateTree();
    return newMesh;

}

PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type) {

    PolygenMesh* detectedMesh = NULL;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (thispolygenMesh->meshType == type) {
            detectedMesh = thispolygenMesh; break;
        }
    }
    return detectedMesh;

}

void MainWindow::_intr_updateVectorDirDisplay()
{
    QMeshNode* handleNode = _getHandleNode();
    if (!handleNode) return;
    ui->doubleSpinBox_intr_VecEdit_nx->setValue(handleNode->vectorDir[0]);
    ui->doubleSpinBox_intr_VecEdit_ny->setValue(handleNode->vectorDir[1]);
    ui->doubleSpinBox_intr_VecEdit_nz->setValue(handleNode->vectorDir[2]);
    pGLK->refresh(true);

}

QMeshNode* MainWindow::_getHandleNode()
{
    PolygenMesh* TetMesh = _detectPolygenMesh(TET_MODEL);
    QMeshPatch* TetPatch = (QMeshPatch*)TetMesh->GetMeshList().GetHead();
    for (GLKPOSITION nodePos = TetPatch->GetNodeList().GetHeadPosition(); nodePos;) {
        QMeshNode* node = (QMeshNode*)TetPatch->GetNodeList().GetNext(nodePos);
        if (node->isHandle) return node;
    }
    return nullptr;
}

void MainWindow::_intr_setVectorDir(double nx, double ny, double nz)
{
    QMeshNode* handleNode = _getHandleNode();
    if (!handleNode) return;
    double mag = nx * nx + ny * ny + nz * nz;
    mag = sqrt(mag);
    if (mag < 1e-5) mag = 1e-5;

    handleNode->vectorDir[0] = nx / mag;
    handleNode->vectorDir[1] = ny / mag;
    handleNode->vectorDir[2] = nz / mag;

}

void MainWindow::_intr_translateVectorNode2Tet()
{
    QMeshNode* handleNode = _getHandleNode();
    if (!handleNode) return;
    RoughMach->translateVectorNode2Tets(handleNode); 
    handleNode->isHandle = false;
    handleNode->isSeed = true;

}

void MainWindow::_extr_layer(double indexx, double* NormalDir= 0)
{
    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* isoLayerSet2;

    //isoLayerSet2 = this->_buildPolygenMesh(CURVED_LAYER, "isoLayers2");
    isoLayerSet2 = this->_detectPolygenMesh(CURVED_LAYER);

    IsoLayerGeneration* convexMachiningSlicer = new IsoLayerGeneration(model);
    convexMachiningSlicer->generateIsoSurface(isoLayerSet2, ui->doubleSpinBox_isoLayerNumber->value(), true, indexx);
    convexMachiningSlicer->smoothingIsoSurface(isoLayerSet2);

    if (NormalDir) {
        QMeshPatch* originalLayer = (QMeshPatch*)isoLayerSet2->GetMeshList().GetTail();
        QMeshPatch* newLayer = _extract_faces_with_normal(NormalDir, originalLayer);
        isoLayerSet2->GetMeshList().RemoveTail();
        newLayer->SetIndexNo(isoLayerSet2->GetMeshList().GetCount());
        isoLayerSet2->GetMeshList().AddTail(newLayer);
    }


    convexMachiningSlicer->outputSurfaceMesh(isoLayerSet2, ui->checkBox_outputLayer->isChecked(), ui->lineEdit_SorceDataDir->text().toStdString(), 0);
}

QMeshPatch* MainWindow::_extract_faces_with_normal(double* NormDir, QMeshPatch* surfacePatch)
{
    float* nodeList;
    unsigned int* faceList;
    int nodeNum = 0;
    int faceNum = 0;
    Eigen::Vector3d normDir = Eigen::Vector3d(NormDir[0], NormDir[1], NormDir[2]);

    for (GLKPOSITION facePos = surfacePatch->GetFaceList().GetHeadPosition(); facePos;) {
        QMeshFace* face = (QMeshFace*)surfacePatch->GetFaceList().GetNext(facePos);
        face->CalPlaneEquation();
        Eigen::Vector3d faceNorm;

        face->GetNormal(faceNorm(0), faceNorm(1), faceNorm(2));

        if (faceNorm.dot(normDir) <= 0) continue;
        face->InsertIndexNumber = faceNum++;

        for (int i = 0; i < 3; i++) {
            QMeshNode* faceNode = face->GetNodeRecordPtr(i);
            if (faceNode->InsertIndex == -1) faceNode->InsertIndex = nodeNum++;
        }
    }


    nodeList = (float*)malloc(3 * sizeof(float) * nodeNum);
    faceList = (unsigned int*)malloc(3 * sizeof(unsigned int) * faceNum);

  
    for (GLKPOSITION facePos = surfacePatch->GetFaceList().GetHeadPosition(); facePos;) {
        QMeshFace* face = (QMeshFace*)surfacePatch->GetFaceList().GetNext(facePos);
        if (face->InsertIndexNumber == -1) continue;
        for (int i = 0; i < 3; i++) {
            QMeshNode* thisNode = face->GetNodeRecordPtr(i);
            faceList[3 * (face->InsertIndexNumber) + i] = (unsigned int)thisNode->InsertIndex;
        }
        face->InsertIndexNumber = -1;
    }

 
    for (GLKPOSITION nodePos = surfacePatch->GetNodeList().GetHeadPosition(); nodePos;) {
        QMeshNode* node = (QMeshNode*)surfacePatch->GetNodeList().GetNext(nodePos); 

        if (node->InsertIndex == -1) continue;

        double x, y, z;
        node->GetCoord3D(x, y, z);
        nodeList[3 * (node->InsertIndex) + 0] = x;
        nodeList[3 * (node->InsertIndex) + 1] = y;
        nodeList[3 * (node->InsertIndex) + 2] = z;
        node->InsertIndex = -1;
    }

    QMeshPatch* newPatch = new QMeshPatch;
    newPatch->constructionFromVerFaceTable(nodeNum, nodeList, faceNum, faceList);
    return newPatch;
}

void MainWindow::_mark_Diff()
{
    PolygenMesh* tMesh = _detectPolygenMesh(TOOL_PATH);
    QMeshPatch* tPatch = (QMeshPatch*) tMesh->GetMeshList().GetHead();


    int numm = tPatch->GetNodeList().GetCount();

    std::vector<QMeshNode*> TP_Vector;
    TP_Vector.reserve(numm);

    int nodeCount = 0;
    for (GLKPOSITION nodePos = tPatch->GetNodeList().GetHeadPosition(); nodePos;) {
        QMeshNode* node = (QMeshNode*)tPatch->GetNodeList().GetNext(nodePos);
        if (!node->resampleChecked) continue;
        TP_Vector.push_back(node);
        nodeCount++;
    }

    int jumpCount = 0;
    for (int i = 0; i < (nodeCount - 1);i++) {
        QMeshNode* node = TP_Vector[i];
   
        QMeshNode* nextNode = TP_Vector[i+1];

        double depth = abs(node->scalarField - nextNode->scalarField);
        if (depth > 1.5) { 
            node->isSource = true; 
            std::cout << jumpCount++<< std::endl;
        }
    }
}

void MainWindow::_deSelectHandle()
{
    QMeshNode* handleNode = _getHandleNode();
    QMeshFace* selectedFace = _getSelectedFace();

    if (selectedFace) {
        selectedFace->isSelectedTemp = false;
    }

    if (!handleNode) return;
    handleNode->isHandle = false;
    handleNode->isSeed = false;
}

QMeshFace* MainWindow::_getSelectedFace()
{
    PolygenMesh* TetMesh = _detectPolygenMesh(TET_MODEL);
    QMeshPatch* TetPatch = (QMeshPatch*)TetMesh->GetMeshList().GetHead();
    for (GLKPOSITION facePos = TetPatch->GetFaceList().GetHeadPosition(); facePos;) {
        QMeshFace* face = (QMeshFace*)TetPatch->GetFaceList().GetNext(facePos);
        if (face->isSelectedTemp) return face;
    }
    return nullptr;
}

void MainWindow::change_isoLayer_Display() {

    bool single = ui->checkBox_EachLayerSwitch->isChecked();

    int currentLayerIndex = ui->spinBox_ShowLayerIndex->value();

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (polygenMesh->meshType != CURVED_LAYER
            && polygenMesh->meshType != TOOL_PATH) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;

            if (single == true) {
                if (Patch->GetIndexNo() == currentLayerIndex)
                    Patch->drawThisPatch = true;
            }
            else {
                if (Patch->GetIndexNo() <= currentLayerIndex)
                    Patch->drawThisPatch = true;
            }
        }
    }
    pGLK->refresh(true);
}


void MainWindow::update_Section_view() {

    double v = (double)ui->horizontalSlider_Section->value();
    v = v / 100.00;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

        if ((polygenMesh->meshType != TET_MODEL)&& (polygenMesh->meshType != VECTOR_FIELD_MESH)) continue;
         
        for (GLKPOSITION Pos = polygenMesh->GetMeshList().GetHeadPosition(); Pos;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(Pos);

            for (GLKPOSITION nodePos = Patch->GetNodeList().GetHeadPosition(); nodePos;) {
                QMeshNode* node = (QMeshNode*)Patch->GetNodeList().GetNext(nodePos);
                double x, y, z;
                node->displayNode = true;
                node->isBoundary = false;
                node->GetCoord3D(x, y, z);
                if (ui->radioButton_SectionX->isChecked()) {
                    if (x < (1-v)*SectionMin_X+v*SectionMax_X) {
                        node->displayNode = false;
                     
                    }
                }
                if (ui->radioButton_SectionZ->isChecked()) {
                    if (z < (1 - v) * SectionMin_Z + v * SectionMax_Z) {
                        node->displayNode = false;
                    
                    }
                }

            }


            /*for (GLKPOSITION edgePos = Patch->GetEdgeList().GetHeadPosition(); edgePos;) {
                QMeshEdge* edge = (QMeshEdge*)Patch->GetEdgeList().GetNext(edgePos);

                QMeshNode* sNode = edge->GetStartPoint();
                QMeshNode* eNode = edge->GetEndPoint();
                if (sNode->displayNode && !eNode->displayNode) {
                    sNode->boundary = true;
                    
                    
                    
                    
                    
                    
                    << "Found2\n";
                }
                else if(!sNode->displayNode && eNode->displayNode) {
                    eNode->boundary = true;
                    std::cout << "Found2\n";
                }
            }*/

           /* for (GLKPOSITION facePos = Patch->GetFaceList().GetHeadPosition(); facePos;) {
                QMeshFace* face = (QMeshFace*)Patch->GetFaceList().GetNext(facePos);

                if (!face->inner) {
                    for (int i = 0; i < 3; i++) {
                        QMeshNode* faceNode = face->GetNodeRecordPtr(i);

                        faceNode->boundary = true;
                    }
                }

                int boundaryCount = 0;
                for (int i = 0; i < 3; i++) {
                    QMeshNode* faceNode = face->GetNodeRecordPtr(i);
                    if (faceNode->boundary && faceNode->displayNode) boundaryCount++;
                }

                if (boundaryCount == 3) {
                    face->dispFace = true;
                }
                else {
                    face->dispFace = false;
                }
            }*/
            

            for (GLKPOSITION facePos = Patch->GetFaceList().GetHeadPosition(); facePos;) {
                QMeshFace* face = (QMeshFace*)Patch->GetFaceList().GetNext(facePos);

                int dispCount = 0;
                for (int i = 0; i < 3; i++) {
                    QMeshNode* faceNode = face->GetNodeRecordPtr(i);
                    if (faceNode->displayNode) dispCount++;
                }

                if (dispCount != 3) {
                    face->dispFace = false;
                    continue;
                }

                if (!face->inner) {
                    face->dispFace = true;
                    continue;
                }

                QMeshTetra* lTet = face->GetLeftTetra();
                QMeshTetra* rTet = face->GetRightTetra();

                dispCount = 0;

                for (int i = 0; i < 4; i++) {
                    QMeshNode* tetNode = lTet->GetNodeRecordPtr(i + 1);
                    if (tetNode->displayNode) dispCount++;
                }
                for (int i = 0; i < 4; i++) {
                    QMeshNode* tetNode = rTet->GetNodeRecordPtr(i + 1);
                    if (tetNode->displayNode) dispCount++;
                }

                if (dispCount == 7) {
                    face->dispFace = true;
                }
                else {
                    face->dispFace = false;
                }

            }


        }
    }

    pGLK->refresh(true);
}

void MainWindow::update_vector_viz()
{
    /**********/
    PolygenMesh* tPathMesh = _detectPolygenMesh(TOOL_PATH);
    QMeshPatch* tPatch = (QMeshPatch *) tPathMesh->GetMeshList().GetHead();

  //  char fileLoc[] = "C:/Users/neelo/5AXISMAKER/remesh_bat/cup_2d.obj";
    char fileLoc[] = "C:/Users/neelo/5AXISMAKER/remesh_bat/cupFinal .obj";
   // char fileLoc[] = "C:/Users/neelo/5AXISMAKER/remesh_bat/mannequin_3R.obj";
    //char fileLoc[] = "C:/Users/neelo/5AXISMAKER/remesh_bat/Mannequin_machining_remeshed_curved.obj"; 
    QMeshPatch* newLayerPatch = new QMeshPatch;
    newLayerPatch->SetIndexNo(isoLayerSet->GetMeshList().GetCount());
    bool isSuccess = newLayerPatch->inputOBJFile(fileLoc);
    if (!isSuccess) std::cout << "Error!!\n";

    RoughMach->_pathIntersection(tPatch, newLayerPatch);
    _mark_Diff();
    pGLK->refresh(true);
    return;
    /***********/
    PolygenMesh* vectorMesh = _detectPolygenMesh(VECTOR_FIELD_MESH);
    if (vectorMesh) {

        vectorMesh->DeleteGLList();
        pGLK->DelDisplayObj(vectorMesh);
        polygenMeshList.Remove(vectorMesh);

        updateTree();
    }

    vectorMesh = _buildPolygenMesh(VECTOR_FIELD_MESH, "vector_field_mesh");
    QMeshPatch* vectorPatch = new QMeshPatch;
    vectorPatch->SetIndexNo(vectorMesh->GetMeshList().GetCount());
    vectorMesh->GetMeshList().AddTail(vectorPatch);
    
    RoughMach->createVectorPatch(vectorPatch, true, ui->checkBox_colorPivots->isChecked());

    for (GLKPOSITION nodePos = vectorPatch->GetNodeList().GetHeadPosition(); nodePos;) {
        QMeshNode* node = (QMeshNode*)vectorPatch->GetNodeList().GetNext(nodePos);
        if (node->isConstraint) {
            //node->isConstraint = false;
            continue;
         }
        else {
            continue;
            node->SetColor(ui->doubleSpinBox_colorR->value(), ui->doubleSpinBox_colorG->value(), ui->doubleSpinBox_colorB->value());
        }
    }
    /**********/

    pGLK->refresh(true);
}

void MainWindow::extr_layer()
{
    _extr_layer(ui->doubleSpinBox_ExtractLayerIndex->value());    
    pGLK->refresh(true);

}

void MainWindow::int_displayVector()
{
    std::cout << "Displaying vector!!\n";
    
    pGLK->refresh(true);
}

void MainWindow::intr_editVectorDir()
{
    std::cout << "start vector editing\n";
    InteractiveTool* tool;
    tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), VECTOR, ui->boxDeselect->isChecked());
    pGLK->set_tool(tool);
    pGLK->refresh(true);
}

void MainWindow::intr_setVectorDir()
{
    std::cout << "Setting vector direction to selected node\n";
    _intr_setVectorDir(ui->doubleSpinBox_intr_VecEdit_nx->value(), ui->doubleSpinBox_intr_VecEdit_ny->value(), ui->doubleSpinBox_intr_VecEdit_nz->value());
    pGLK->refresh(true);
   
}

void MainWindow::intr_updateVectorDirDisplay()
{
    _intr_updateVectorDirDisplay();
}

void MainWindow::intr_translateVectorNode2Tet()
{
    std::cout << "Translating node vector info to incident tets.\n";
    _intr_translateVectorNode2Tet();
    pGLK->update();
    pGLK->refresh(true);
}

void MainWindow::intr_deSelectHandle()
{
    _deSelectHandle();
    pGLK->refresh(true);
}

void MainWindow::intr_selectFacePatch()
{
    QMeshFace* selectedFace = _getSelectedFace();
    if (selectedFace) {
        RoughMach->spreadSelectionToFaceSegment(selectedFace);
        selectedFace->isSelectedTemp = false;
    }
    else {
        QMeshNode* handleNode = _getHandleNode();
        if (!handleNode) return;
        RoughMach->spreadSelectionToFaceSegment(handleNode);
        handleNode->isHandle = false;
        handleNode->isSeed = false;
    }
    pGLK->refresh(true);
    
}


void MainWindow::all_isoLayer_Display() {
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

        if (polygenMesh->meshType != CURVED_LAYER
            && polygenMesh->meshType != TOOL_PATH) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
            Patch->drawThisPatch = true;
        }
    }
    pGLK->refresh(true);
}

void MainWindow::input_machiningData() {

    
    std::string model_name = (ui->lineEdit_SorceDataDir->text()).toStdString();


    PolygenMesh* materialSpace = this->_buildPolygenMesh(TET_MODEL, model_name + "_materialSpace");
    PolygenMesh* targetSurface = this->_buildPolygenMesh(SURFACE_MESH, model_name + "_targetSurface");
    PolygenMesh* modelSurface = this->_buildPolygenMesh(MODEL_MESH, model_name + "_modelSurface");
   // PolygenMesh* fixtureSurface = this->_buildPolygenMesh(FIXTURE_MESH, model_name + "_fixtureSurface");

    char filename_targetSurface[1024];    char filename_materialSpace[1024];    char filename_modelSurface[1024];
    char filename_fixtureSurface[1024];

    if (false) {


        if (false) {
            sprintf(filename_targetSurface, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_modelSurf.obj");
            sprintf(filename_materialSpace, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_convexHull.tet");
        }
        else {
            sprintf(filename_targetSurface, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_convexHull.obj");
            sprintf(filename_materialSpace, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_materialSpace.tet");
        }
    }
    else {
        sprintf(filename_targetSurface, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_convexHull.obj");
        sprintf(filename_materialSpace, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_materialSpace.tet");
        sprintf(filename_modelSurface, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_modelSurf.obj");
       // sprintf(filename_fixtureSurface, "%s%s%s", "../DataSet/TET_MODEL/", model_name.c_str(), "_fixtureSurface.obj");
    }

    std::cout << filename_targetSurface << std::endl;
    std::cout << filename_materialSpace << std::endl;
    

    // input mesh -- machining space

    QMeshPatch* patch2 = new QMeshPatch;
    patch2->SetIndexNo(materialSpace->GetMeshList().GetCount()); //index begin from 0
    materialSpace->GetMeshList().AddTail(patch2);
    patch2->inputTETFile(filename_materialSpace, false);
    double SectionMins[6];
    patch2->ComputeBoundingBox(SectionMins);
    SectionMin_X = SectionMins[0];
    SectionMax_X = SectionMins[1];
    SectionMin_Z = SectionMins[4];
    SectionMax_Z = SectionMins[5];



    
        // input mesh -- target surface (ConvexHull)
    
    QMeshPatch* patch1 = new QMeshPatch;
    patch1->SetIndexNo(targetSurface->GetMeshList().GetCount()); //index begin from 0
    targetSurface->GetMeshList().AddTail(patch1);
    patch1->inputOBJFile(filename_targetSurface, false);

    



    // input mesh -- model surface and fixture surface
    if (true) {
        QMeshPatch* patch3 = new QMeshPatch;
        patch3->SetIndexNo(modelSurface->GetMeshList().GetCount()); //index begin from 0
        modelSurface->GetMeshList().AddTail(patch3);
        patch3->inputOBJFile(filename_modelSurface, false);

        
          
            for (GLKPOSITION pos = patch3->GetEdgeList().GetHeadPosition(); pos;) {
                QMeshEdge* edge = (QMeshEdge*)patch3->GetEdgeList().GetNext(pos);

                QMeshFace* faceL = edge->GetLeftFace();
                QMeshFace* faceR = edge->GetRightFace();

                if (!faceL) continue;
                if (!faceR) continue;
                
                Eigen::Vector3d normL, normR;
                faceL->CalPlaneEquation();
                faceR->CalPlaneEquation();

                faceL->GetNormal(normL[0], normL[1], normL[2]);
                faceR->GetNormal(normR[0], normR[1], normR[2]);

                normL.stableNormalize();
                normR.stableNormalize();

                double checkVal = normL.dot(normR);

                if (checkVal < 0.8) edge->selected = true;
              }
    
/*
        QMeshPatch* patch4 = new QMeshPatch;
        patch4->SetIndexNo(fixtureSurface->GetMeshList().GetCount()); //index begin from 0
        fixtureSurface->GetMeshList().AddTail(patch4);
        patch4->inputOBJFile(filename_fixtureSurface, false);
*/
    }
    
 


    /*meshOperation* meshOperator = new meshOperation;
    meshOperator->tetMeshGeneration_outerSkin_Chamber(patch2, patch1, patch2);*/

    setBoundaryNodes(materialSpace);

    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();

    std::cout << "finish input the machining space and target surafce!" << endl;
   
}

void MainWindow::setBoundaryNodes(PolygenMesh* polymesh) {
    //This needs to be improved on priority!!
    for (GLKPOSITION PatchPos = polymesh->GetMeshList().GetHeadPosition(); PatchPos;) {
        QMeshPatch* Patch = (QMeshPatch*) polymesh->GetMeshList().GetNext(PatchPos);
        for (GLKPOSITION pos = Patch->GetNodeList().GetHeadPosition(); pos;) {
            QMeshNode* node = (QMeshNode*) Patch->GetNodeList().GetNext(pos);
            int num = node->GetFaceNumber();
            for (int i = 0; i < num; i++) {
                QMeshFace* face = (QMeshFace*)node->GetFaceRecordPtr(i + 1);
                if(face->GetLeftTetra()==NULL || face->GetRightTetra()==NULL){
                    node->isBoundary = true;
                    face->isBoundary = true;
                    face->dispFace = true;
                }
            }
        }
    }

  
}

void MainWindow::initialise_data() {

    std::cout << "New RoughMachining class building ... \n" << std::endl;
    RoughMach = new RoughMachining();

    PolygenMesh* targetSurface = _detectPolygenMesh(SURFACE_MESH);
    PolygenMesh* materialSpace = _detectPolygenMesh(TET_MODEL);
    PolygenMesh* modelSurface = _detectPolygenMesh(MODEL_MESH);
    PolygenMesh* fixtureSurface = _detectPolygenMesh(FIXTURE_MESH);

    if (targetSurface == NULL || materialSpace == NULL) {
        std::cerr << "There is no PolygenMesh!" << std::endl; return;
    }

    

    if (true) {
        RoughMach->initial(materialSpace, targetSurface, modelSurface, fixtureSurface, ui->IsIn_radioButton->isChecked());
        RoughMach->classifyBoundary();
        
    }
    else
        RoughMach->initial(materialSpace, targetSurface, NULL, NULL, ui->IsIn_radioButton->isChecked());
    std::cout << "Finish building and initialization.\n" << std::endl;

    if (false)
    {
        RoughMach->convexHull_Boundary_Detection();
        std::cout << "Finish detection of boundary face of convexHull.\n" << std::endl;

        RoughMach->initialIndex();
        std::cout << "Finish rearrangement of indices.\n" << std::endl;
    }
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::initiate_vector_on_bound()
{
    std::cout << "Initiating vector field...\n";
    RoughMach->initialIndex();
    RoughMach->initiateVectorOnBound(ui->checkBox_inBoundGuess->isChecked(), ui->checkBox_outBoundGuess->isChecked(), ui->checkBox_chGuess->isChecked());
    ui->pushButton_intr_Vector_Node2Tets->setEnabled(true);

    /***********/
    PolygenMesh* vectorMesh = _detectPolygenMesh(VECTOR_FIELD_MESH);
    if (vectorMesh) {
        vectorMesh->ClearAll();
        vectorMesh->DeleteGLList();
        pGLK->DelDisplayObj(vectorMesh);
        polygenMeshList.Remove(vectorMesh);
        delete(vectorMesh);
        updateTree();
    }

    vectorMesh = _buildPolygenMesh(VECTOR_FIELD_MESH, "vector_field_mesh");
    QMeshPatch* vectorPatch = new QMeshPatch;
    vectorPatch->SetIndexNo(vectorMesh->GetMeshList().GetCount());
    vectorMesh->GetMeshList().AddTail(vectorPatch);
    RoughMach->createVectorPatch(vectorPatch, false);
    /**********/

    std::cout << "...Initiating vector field Complete\n";

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();


}

void MainWindow::set_guide_field()
{
    std::cout << "Setting guide field...\n";
    RoughMach->initiateGuideVectors();
    /***********/
    PolygenMesh* vectorMesh = _detectPolygenMesh(VECTOR_FIELD_MESH);
    if (vectorMesh) {
      
        vectorMesh->DeleteGLList();
        pGLK->DelDisplayObj(vectorMesh);
        polygenMeshList.Remove(vectorMesh);
    
        updateTree();
    }

    vectorMesh = _buildPolygenMesh(VECTOR_FIELD_MESH, "vector_field_mesh");
    QMeshPatch* vectorPatch = new QMeshPatch;
    vectorPatch->SetIndexNo(vectorMesh->GetMeshList().GetCount());
    vectorMesh->GetMeshList().AddTail(vectorPatch);
    RoughMach->createVectorPatch(vectorPatch);
    /**********/
    std::cout << "...Setting guide field complete\n";

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();

}

void MainWindow::propagate_vector_field()
{
    std::cout << "Propagating field...\n";

    RoughMach->propagateMag = ui->doubleSpinBox_pivotMag->value();
    clock_t t;
    t = clock();
    RoughMach->propagateVectorField();
    t = clock() - t;
    std::cout << "Time for optimisation: " << ((double)t) / CLOCKS_PER_SEC << std::endl;
    /***********/
    PolygenMesh* vectorMesh = _detectPolygenMesh(VECTOR_FIELD_MESH);
    if (vectorMesh) {
      //  vectorMesh->ClearAll();
        vectorMesh->DeleteGLList();
        pGLK->DelDisplayObj(vectorMesh);
        polygenMeshList.Remove(vectorMesh);
       // delete(vectorMesh);
        updateTree();
    }

    vectorMesh = _buildPolygenMesh(VECTOR_FIELD_MESH, "vector_field_mesh");
    QMeshPatch* vectorPatch = new QMeshPatch;
    vectorPatch->SetIndexNo(vectorMesh->GetMeshList().GetCount());
    vectorMesh->GetMeshList().AddTail(vectorPatch);
    RoughMach->createVectorPatch(vectorPatch);
    update_Section_view();
    /**********/
    std::cout << "...Propagating field complete\n";
    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::detect_singularity()
{
    std::cout << "Detecting singularity\n";
    RoughMach->detectSingularity();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "...Operation Completed\n";
}

void MainWindow::correct_singularity()
{
    std::cout << "Correcting singularity\n";
    RoughMach->correctSingularity();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "...Operation Completed\n";
}

void MainWindow::compute_scalar_field()
{
    std::cout << "Computing scalar field\n";

    RoughMach->computeScalarField();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "...Operation Completed\n";
}

void MainWindow::compute_field_difference()
{
    std::cout << "Computing Field Difference\n";
    RoughMach->checkDifference();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "...Operation Completed\n";
}

void MainWindow::convert_2_GradField()
{
    std::cout << "Converting to Grad field\n";
    for (int i = 0; i < 10; i++) {
        RoughMach->convert2GradientField();
      //  std::cout << "******Iter " << i << std::endl;
        RoughMach->checkDifference();
    }
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "...Operation Completed\n";
}


double dir_[3];
bool isDoneOnce = false;

void MainWindow::correct_Planar_Singularity()
{

    std::cout << "Correcting planar singularity\n";
    
    if (!isDoneOnce) {
        if (ui->checkBox_Flip->isChecked()) RoughMach->correctPlanarSingularity(dir_, true);
        else
            RoughMach->correctPlanarSingularity(dir_);

        RoughMach->computeScalarField(false);
        isDoneOnce = true;
    }
    //std::cout << dir_[0] << " " << dir_[1] << " " << dir_[2] << std::endl;
  
    if(ui->checkBox_forceIndex->isChecked())
        _extr_layer(ui->doubleSpinBox_forcingIndex->value(), dir_);
    else
        _extr_layer(RoughMach->extractionIndex, dir_);

    ui->spinBox_ShowLayerIndex->setMaximum(isoLayerSet->GetMeshList().GetCount() - 1);
    
    pGLK->refresh(true);
    std::cout << "...Operation Completed\n";
}


void MainWindow::create_heatField() {

    PolygenMesh* materialSpace = _detectPolygenMesh(TET_MODEL);
    if (materialSpace == NULL) {
        std::cerr << "There is no PolygenMesh!" << std::endl; return;
    }

    std::cout << "Intitialising Signed Distance Field on Mesh...\n";

    //RoughMach = new RoughMachining();
    int boundaryVal = 0.5;//ui->spinBox_BC->value();
    if (false) {
        RoughMach->createHeatField(materialSpace, boundaryVal);
    }
    else
        RoughMach->createDistanceField();
   // RoughMach->_cal_heatValue1(materialSpace);

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    
    std::cout << "Intitialising Signed Distance Field on Mesh is COMPLETE\n";
}


QMeshPatch* createSubface(QMeshPatch* sourcePatch);

void MainWindow::curvedLayer_Generation() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* tempMesh = this->_detectPolygenMesh(CURVED_LAYER);
    if (tempMesh) {
        tempMesh->DeleteGLList();
        pGLK->DelDisplayObj(tempMesh);
        polygenMeshList.Remove(tempMesh);
        updateTree();
    }

    isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, "isoLayers");

    IsoLayerGeneration* convexMachiningSlicer = new IsoLayerGeneration(model);
    convexMachiningSlicer->max_distance = RoughMach->max_distance;
    convexMachiningSlicer->min_distance = RoughMach->min_distance;
    convexMachiningSlicer->min_SingularDistance = RoughMach->min_SingularPlane;
    convexMachiningSlicer->max_SingularDistance = RoughMach->max_SingularPlane;
    convexMachiningSlicer->generateIsoSurface(isoLayerSet, ui->doubleSpinBox_isoLayerNumber->value());
    
   //char fileLoc[] = "C:/Users/neelo/5AXISMAKER/MultiAxis_3DP_MotionPlanning/DataSet/Sorce/cupNew/modelSurf.obj";
    //char fileLoc[] = "C:/Users/neelo/5AXISMAKER/MultiAxis_3DP_MotionPlanning/DataSet/Sorce/mannequin/modelSurfRem.obj";
   // char fileLoc[] = "C:/Users/neelo/5AXISMAKER/MultiAxis_3DP_MotionPlanning/DataSet/Sorce/mannequin/modelSurfRem.obj";
   //  char fileLoc[] = "C:/Users/neelo/5AXISMAKER/MultiAxis_3DP_MotionPlanning/DataSet/Sorce/mannequin/modelSurfRem.obj";
    //char fileLoc[] = "C:/Users/neelo/5AXISMAKER/remesh_bat/man2Solid/stage20Final.obj";
   // for (int i = 20; i <= 20; i++) {
     //   QMeshPatch* newLayerPatch = new QMeshPatch;
     //   newLayerPatch->SetIndexNo(isoLayerSet->GetMeshList().GetCount());
     //  newLayerPatch->SetIndexNo(isoLayerSet->GetMeshList().GetCount());
     //  char fileLoc[99];
     // sprintf(fileLoc, "%s%d%s", "C:/Users/neelo/5AXISMAKER/VectorFieldCAM/DataSet/CURVED_LAYER/OUT/", i, ".obj");
     //   sprintf(fileLoc, "%s%d%s", "C:/Users/neelo/5AXISMAKER/remesh_bat/manLayers/", i, ".obj");
     //  bool isSuccess = newLayerPatch->inputOBJFile(fileLoc);
        
     //  if (!isSuccess) std::cout << "Wrong!!!!!!!!!\n";
      // newLayerPatch = createSubface(newLayerPatch);
      // convexMachiningSlicer->planeCutSurfaceMesh_delete(newLayerPatch, 0, 1);
    //    //convexMachiningSlicer->planeCutSurfaceMesh_delete(newLayerPatch, -85, 3);
    //    //convexMachiningSlicer->planeCutSurfaceMesh_delete(newLayerPatch, 115, 2);
    //    //convexMachiningSlicer->planeCutSurfaceMesh_delete(newLayerPatch, 2, 5);
    //  //  convexMachiningSlicer->planeCutSurfaceMesh_delete(newLayerPatch, 145, 2);
     //   isoLayerSet->GetMeshList().AddTail(newLayerPatch);
    //}
   RoughMach->layerInSingularZone = convexMachiningSlicer->isInSingularZone;
   RoughMach->extractionIndex = convexMachiningSlicer->singularZoneIndex;
    ui->spinBox_ShowLayerIndex->setMinimum((int)0);

    for (GLKPOSITION posMesh = isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* layer = (QMeshPatch*)isoLayerSet->GetMeshList().GetNext(posMesh);
                //convexMachiningSlicer->planeCutSurfaceMesh_delete(layer, 30, 2);
               //convexMachiningSlicer->planeCutSurfaceMesh_delete(layer, -8.0, 3);
                //convexMachiningSlicer->planeCutSurfaceMesh_delete(layer, -0.0, 1);
     //   convexMachiningSlicer->planeCutSurfaceMesh_delete(layer);
    }

  // for (int i = 0; i < isoLayerSet->GetMeshList().GetCount(); i++) convexMachiningSlicer->smoothingIsoSurface(isoLayerSet);

    int F_Offset = 0;
    if (ui->IsIn_radioButton->isChecked()) F_Offset = 1;
    convexMachiningSlicer->outputSurfaceMesh(isoLayerSet, ui->checkBox_outputLayer->isChecked(),ui->lineEdit_SorceDataDir->text().toStdString(),F_Offset);

    ui->spinBox_ShowLayerIndex->setMaximum(isoLayerSet->GetMeshList().GetCount() - 1);
    std::cout << "Finish generating curved layer of convexMachining in the materialSpace.\n" << std::endl;

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}


QMeshPatch* createSubface(QMeshPatch* sourcePatch) {
    QMeshPatch* newPatch = new QMeshPatch;
    int faceCount = 0, nodeCount = 0;
    //std::cout << "1\n";
    for (GLKPOSITION facePos = sourcePatch->GetFaceList().GetHeadPosition(); facePos;) {
        QMeshFace* face = (QMeshFace*)sourcePatch->GetFaceList().GetNext(facePos);

        face->CalPlaneEquation();
        Eigen::Vector3d faceNorm;
        face->GetNormal(faceNorm[0], faceNorm[1], faceNorm[2]);
        Eigen::Vector3d upVec = Eigen::Vector3d(0, 1, 0);
        double checkVal = faceNorm.dot(upVec);
        if (checkVal <= 1e-2) {
            face->visible = false;
        }
        else {
            face->visible = true;
            faceCount++;
            for (int i = 0; i < 3; i++) {
                QMeshNode* faceNode = face->GetNodeRecordPtr(i);
                if (!faceNode->visited) {
                    nodeCount++;
                }
                faceNode->visited = true;
            }
        }
    }

    //std::cout << "2\n";
    float* nodeList = (float*)malloc(3 * nodeCount * sizeof(float));
    unsigned int* faceList = (unsigned int*)malloc(3 * faceCount * sizeof(unsigned int));
    int entryCount = 0;
    for (GLKPOSITION nodePos = sourcePatch->GetNodeList().GetHeadPosition(); nodePos;) {
        QMeshNode* node = (QMeshNode*)sourcePatch->GetNodeList().GetNext(nodePos);
        if (node->visited) {
            double x, y, z;
            node->GetCoord3D(x, y, z);
            nodeList[entryCount * 3] = (float)x;
            nodeList[entryCount * 3 + 1] = (float)y;
            nodeList[entryCount * 3 + 2] = (float)z;
            node->scalarField = entryCount;
            entryCount++;
        }
    }
    assert(entryCount == nodeCount);
    entryCount = 0;
    //std::cout << "3\n";
    for (GLKPOSITION facePos = sourcePatch->GetFaceList().GetHeadPosition(); facePos;) {
        QMeshFace* face = (QMeshFace*)sourcePatch->GetFaceList().GetNext(facePos);
        if (face->visible) {
            for (int i = 0; i < 3; i++) {
                QMeshNode* faceNode = face->GetNodeRecordPtr(i);
                faceList[3*entryCount + i] = (unsigned int)faceNode->scalarField;
            }
            entryCount++;
        }
    }
    assert(entryCount = faceCount);
    //std::cout << "4\n";
    newPatch->constructionFromVerFaceTable(nodeCount, nodeList, faceCount, faceList);
    //std::cout << "5\n";
    return newPatch;
}


void MainWindow::toolPath_Generation() {

    PolygenMesh* toolpathSet = this->_buildPolygenMesh(TOOL_PATH, "toolPath");
    toolpathGeneration* ToolPathComp_layer = new toolpathGeneration(isoLayerSet, toolpathSet,
        ui->doubleSpinBox_toolPathWidth->value(), ui->doubleSpinBox_toolPathDistance->value());

    ToolPathComp_layer->generate_all_toolPath();

    int F_Offset = 0;
    if (ui->IsIn_radioButton->isChecked()) F_Offset = 1;
    ToolPathComp_layer->output_toolpath(toolpathSet, ui->checkBox_outputToolpath->isChecked(),ui->lineEdit_SorceDataDir->text().toStdString(),F_Offset);

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "Finish generating toolpath of convexMachining in the materialSpace.\n" << std::endl;
}

void MainWindow::correct_Along_Inner_Bound()
{
    std::cout << "Correcting vector along inner boundary...\n";

    RoughMach->correctAlongInnerBoundary();
    pGLK->refresh(true);
}


