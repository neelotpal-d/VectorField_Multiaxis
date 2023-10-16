/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionFront;
    QAction *actionBack;
    QAction *actionTop;
    QAction *actionBottom;
    QAction *actionLeft;
    QAction *actionRight;
    QAction *actionIsometric;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *actionZoom_All;
    QAction *actionZoom_Window;
    QAction *actionShade;
    QAction *actionMesh;
    QAction *actionNode;
    QAction *actionSave;
    QAction *actionSelectNode;
    QAction *actionSelectFace;
    QAction *actionShifttoOrigin;
    QAction *actionProfile;
    QAction *actionFaceNormal;
    QAction *actionNodeNormal;
    QAction *actionSelectEdge;
    QAction *actionGenerate;
    QAction *actionTest_1;
    QAction *actionSelectFix;
    QAction *actionSelectHandle;
    QAction *actionSaveSelection;
    QAction *actionReadSelection;
    QAction *actionSelectChamber;
    QAction *actionExport_to_Abaqus_model;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QToolBar *navigationToolBar;
    QStatusBar *statusBar;
    QToolBar *selectionToolBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QLabel *label_MANY_3DP_CNC_CAM;
    QCheckBox *boxDeselect;
    QFrame *line;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_16;
    QLabel *label_inputFile;
    QLineEdit *lineEdit_SorceDataDir;
    QPushButton *pushButton_readData;
    QRadioButton *IsIn_radioButton;
    QPushButton *pushButton_initialise;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_10;
    QCheckBox *checkBox_inBoundGuess;
    QCheckBox *checkBox_outBoundGuess;
    QCheckBox *checkBox_chGuess;
    QPushButton *pushButton_setBoundaryVectors;
    QPushButton *pushButton_setGuideFields;
    QDoubleSpinBox *doubleSpinBox_pivotMag;
    QPushButton *pushButton_propagateField;
    QPushButton *pushButton_generateScalarField;
    QFrame *line_3;
    QPushButton *pushButton_detectSingularity;
    QPushButton *pushButton_correctSingularity;
    QPushButton *pushButton_checkFieldDifference;
    QPushButton *pushButton_convert2GradField;
    QHBoxLayout *horizontalLayout_7;
    QCheckBox *checkBox_Flip;
    QCheckBox *checkBox_forceIndex;
    QDoubleSpinBox *doubleSpinBox_forcingIndex;
    QPushButton *pushButton_correctPlanarSingularity;
    QPushButton *pushButton_correctInnerBound;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *pushButton_isoLayerGeneration;
    QLabel *label_6;
    QDoubleSpinBox *doubleSpinBox_isoLayerNumber;
    QFrame *line_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QDoubleSpinBox *doubleSpinBox_toolPathWidth;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBox_toolPathDistance;
    QPushButton *pushButton_toolPathGeneration;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_4;
    QCheckBox *checkBox_outputLayer;
    QCheckBox *checkBox_outputToolpath;
    QPushButton *pushButton_clearAll;
    QWidget *Interactive;
    QHBoxLayout *horizontalLayout_9;
    QVBoxLayout *verticalLayout_4;
    QPushButton *pushButton_vecDisplay;
    QPushButton *pushButton_intr_editVectorDirection;
    QLabel *label_5;
    QHBoxLayout *horizontalLayout_6;
    QDoubleSpinBox *doubleSpinBox_intr_VecEdit_nx;
    QDoubleSpinBox *doubleSpinBox_intr_VecEdit_ny;
    QDoubleSpinBox *doubleSpinBox_intr_VecEdit_nz;
    QPushButton *pushButton_intr_setVectorDirection;
    QPushButton *pushButton_intr_getVectorDirection;
    QPushButton *pushButton_intr_Vector_Node2Tets;
    QPushButton *pushButton_deSelectNode;
    QPushButton *pushButton_facePatchSelect;
    QCheckBox *checkBox_faceMode;
    QSpacerItem *verticalSpacer_2;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_2;
    QSpinBox *spinBox_ShowLayerIndex;
    QCheckBox *checkBox_EachLayerSwitch;
    QPushButton *pushButton_ShowAllLayers;
    QHBoxLayout *horizontalLayout_8;
    QRadioButton *radioButton_SectionY;
    QRadioButton *radioButton_SectionZ;
    QRadioButton *radioButton_SectionX;
    QSlider *horizontalSlider_Section;
    QPushButton *pushButton_updateSection;
    QCheckBox *checkBox_colorPivots;
    QHBoxLayout *horizontalLayout_12;
    QDoubleSpinBox *doubleSpinBox_colorR;
    QDoubleSpinBox *doubleSpinBox_colorG;
    QDoubleSpinBox *doubleSpinBox_colorB;
    QPushButton *pushButton_updateVecViz;
    QHBoxLayout *horizontalLayout_11;
    QPushButton *pushButton_extractLayer;
    QDoubleSpinBox *doubleSpinBox_ExtractLayerIndex;
    QSpacerItem *verticalSpacer;
    QPushButton *pushButton_FixtureField;
    QTreeView *treeView;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuSelect;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1244, 1147);
        MainWindow->setMinimumSize(QSize(0, 0));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        MainWindow->setFont(font);
        MainWindow->setMouseTracking(true);
        MainWindow->setFocusPolicy(Qt::StrongFocus);
        MainWindow->setAcceptDrops(true);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/Open Folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionFront = new QAction(MainWindow);
        actionFront->setObjectName(QString::fromUtf8("actionFront"));
        actionFront->setCheckable(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/Front View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFront->setIcon(icon1);
        actionBack = new QAction(MainWindow);
        actionBack->setObjectName(QString::fromUtf8("actionBack"));
        actionBack->setCheckable(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/Back View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBack->setIcon(icon2);
        actionTop = new QAction(MainWindow);
        actionTop->setObjectName(QString::fromUtf8("actionTop"));
        actionTop->setCheckable(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/Top View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionTop->setIcon(icon3);
        actionBottom = new QAction(MainWindow);
        actionBottom->setObjectName(QString::fromUtf8("actionBottom"));
        actionBottom->setCheckable(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/Bottom View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottom->setIcon(icon4);
        actionLeft = new QAction(MainWindow);
        actionLeft->setObjectName(QString::fromUtf8("actionLeft"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/Left View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeft->setIcon(icon5);
        actionRight = new QAction(MainWindow);
        actionRight->setObjectName(QString::fromUtf8("actionRight"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/Right View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRight->setIcon(icon6);
        actionIsometric = new QAction(MainWindow);
        actionIsometric->setObjectName(QString::fromUtf8("actionIsometric"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/Isometric View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionIsometric->setIcon(icon7);
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/resource/Zoom In.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon8);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/resource/Zoom Out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon9);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QString::fromUtf8("actionZoom_All"));
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/resource/Zoom All.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon10);
        actionZoom_Window = new QAction(MainWindow);
        actionZoom_Window->setObjectName(QString::fromUtf8("actionZoom_Window"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/resource/Zoom Window.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Window->setIcon(icon11);
        actionShade = new QAction(MainWindow);
        actionShade->setObjectName(QString::fromUtf8("actionShade"));
        actionShade->setCheckable(true);
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/resource/Shade.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShade->setIcon(icon12);
        actionMesh = new QAction(MainWindow);
        actionMesh->setObjectName(QString::fromUtf8("actionMesh"));
        actionMesh->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/resource/Mesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMesh->setIcon(icon13);
        actionNode = new QAction(MainWindow);
        actionNode->setObjectName(QString::fromUtf8("actionNode"));
        actionNode->setCheckable(true);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/resource/Node.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNode->setIcon(icon14);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/resource/Save as.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon15);
        actionSelectNode = new QAction(MainWindow);
        actionSelectNode->setObjectName(QString::fromUtf8("actionSelectNode"));
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/resource/selectNode.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectNode->setIcon(icon16);
        actionSelectFace = new QAction(MainWindow);
        actionSelectFace->setObjectName(QString::fromUtf8("actionSelectFace"));
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/resource/selectFace.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFace->setIcon(icon17);
        actionShifttoOrigin = new QAction(MainWindow);
        actionShifttoOrigin->setObjectName(QString::fromUtf8("actionShifttoOrigin"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QString::fromUtf8("actionProfile"));
        actionProfile->setCheckable(true);
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/resource/Profile.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionProfile->setIcon(icon18);
        actionFaceNormal = new QAction(MainWindow);
        actionFaceNormal->setObjectName(QString::fromUtf8("actionFaceNormal"));
        actionFaceNormal->setCheckable(true);
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/resource/FaceNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFaceNormal->setIcon(icon19);
        actionNodeNormal = new QAction(MainWindow);
        actionNodeNormal->setObjectName(QString::fromUtf8("actionNodeNormal"));
        actionNodeNormal->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/resource/NodeNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNodeNormal->setIcon(icon20);
        actionSelectEdge = new QAction(MainWindow);
        actionSelectEdge->setObjectName(QString::fromUtf8("actionSelectEdge"));
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/resource/selectEdge.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectEdge->setIcon(icon21);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QString::fromUtf8("actionGenerate"));
        actionTest_1 = new QAction(MainWindow);
        actionTest_1->setObjectName(QString::fromUtf8("actionTest_1"));
        actionSelectFix = new QAction(MainWindow);
        actionSelectFix->setObjectName(QString::fromUtf8("actionSelectFix"));
        QIcon icon22;
        icon22.addFile(QString::fromUtf8(":/resource/selectFix.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFix->setIcon(icon22);
        actionSelectHandle = new QAction(MainWindow);
        actionSelectHandle->setObjectName(QString::fromUtf8("actionSelectHandle"));
        QIcon icon23;
        icon23.addFile(QString::fromUtf8(":/resource/selectHandle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectHandle->setIcon(icon23);
        actionSaveSelection = new QAction(MainWindow);
        actionSaveSelection->setObjectName(QString::fromUtf8("actionSaveSelection"));
        QIcon icon24;
        icon24.addFile(QString::fromUtf8(":/resource/SaveSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSaveSelection->setIcon(icon24);
        actionReadSelection = new QAction(MainWindow);
        actionReadSelection->setObjectName(QString::fromUtf8("actionReadSelection"));
        QIcon icon25;
        icon25.addFile(QString::fromUtf8(":/resource/InputSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionReadSelection->setIcon(icon25);
        actionSelectChamber = new QAction(MainWindow);
        actionSelectChamber->setObjectName(QString::fromUtf8("actionSelectChamber"));
        actionExport_to_Abaqus_model = new QAction(MainWindow);
        actionExport_to_Abaqus_model->setObjectName(QString::fromUtf8("actionExport_to_Abaqus_model"));
        actionExport_to_Abaqus_model->setCheckable(false);
        QIcon icon26;
        icon26.addFile(QString::fromUtf8(":/resource/abaqus logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport_to_Abaqus_model->setIcon(icon26);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(true);
        centralWidget->setAcceptDrops(true);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        MainWindow->setCentralWidget(centralWidget);
        navigationToolBar = new QToolBar(MainWindow);
        navigationToolBar->setObjectName(QString::fromUtf8("navigationToolBar"));
        navigationToolBar->setMovable(false);
        navigationToolBar->setIconSize(QSize(25, 25));
        navigationToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, navigationToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        selectionToolBar = new QToolBar(MainWindow);
        selectionToolBar->setObjectName(QString::fromUtf8("selectionToolBar"));
        selectionToolBar->setMovable(false);
        selectionToolBar->setIconSize(QSize(25, 25));
        selectionToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, selectionToolBar);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget->sizePolicy().hasHeightForWidth());
        dockWidget->setSizePolicy(sizePolicy);
        dockWidget->setMinimumSize(QSize(350, 1056));
        dockWidget->setMaximumSize(QSize(350, 524287));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_MANY_3DP_CNC_CAM = new QLabel(dockWidgetContents);
        label_MANY_3DP_CNC_CAM->setObjectName(QString::fromUtf8("label_MANY_3DP_CNC_CAM"));
        QFont font1;
        font1.setPointSize(10);
        label_MANY_3DP_CNC_CAM->setFont(font1);

        verticalLayout->addWidget(label_MANY_3DP_CNC_CAM);

        boxDeselect = new QCheckBox(dockWidgetContents);
        boxDeselect->setObjectName(QString::fromUtf8("boxDeselect"));

        verticalLayout->addWidget(boxDeselect);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setMinimumSize(QSize(328, 0));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_2 = new QVBoxLayout(tab);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setSpacing(6);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        label_inputFile = new QLabel(tab);
        label_inputFile->setObjectName(QString::fromUtf8("label_inputFile"));
        QFont font2;
        font2.setPointSize(8);
        font2.setBold(true);
        font2.setWeight(75);
        label_inputFile->setFont(font2);

        horizontalLayout_16->addWidget(label_inputFile);

        lineEdit_SorceDataDir = new QLineEdit(tab);
        lineEdit_SorceDataDir->setObjectName(QString::fromUtf8("lineEdit_SorceDataDir"));

        horizontalLayout_16->addWidget(lineEdit_SorceDataDir);

        pushButton_readData = new QPushButton(tab);
        pushButton_readData->setObjectName(QString::fromUtf8("pushButton_readData"));
        QFont font3;
        font3.setPointSize(9);
        font3.setBold(true);
        font3.setItalic(false);
        font3.setUnderline(true);
        font3.setWeight(75);
        pushButton_readData->setFont(font3);
        pushButton_readData->setStyleSheet(QString::fromUtf8("color: rgb(0, 80, 0);"));

        horizontalLayout_16->addWidget(pushButton_readData);

        IsIn_radioButton = new QRadioButton(tab);
        IsIn_radioButton->setObjectName(QString::fromUtf8("IsIn_radioButton"));
        IsIn_radioButton->setAutoExclusive(false);

        horizontalLayout_16->addWidget(IsIn_radioButton);


        verticalLayout_2->addLayout(horizontalLayout_16);

        pushButton_initialise = new QPushButton(tab);
        pushButton_initialise->setObjectName(QString::fromUtf8("pushButton_initialise"));
        pushButton_initialise->setFont(font);

        verticalLayout_2->addWidget(pushButton_initialise);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(-1, 5, -1, -1);
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        horizontalLayout_10->setContentsMargins(-1, 3, -1, 3);
        checkBox_inBoundGuess = new QCheckBox(tab);
        checkBox_inBoundGuess->setObjectName(QString::fromUtf8("checkBox_inBoundGuess"));
        QFont font4;
        font4.setPointSize(7);
        checkBox_inBoundGuess->setFont(font4);

        horizontalLayout_10->addWidget(checkBox_inBoundGuess);

        checkBox_outBoundGuess = new QCheckBox(tab);
        checkBox_outBoundGuess->setObjectName(QString::fromUtf8("checkBox_outBoundGuess"));
        checkBox_outBoundGuess->setFont(font4);

        horizontalLayout_10->addWidget(checkBox_outBoundGuess);


        verticalLayout_6->addLayout(horizontalLayout_10);

        checkBox_chGuess = new QCheckBox(tab);
        checkBox_chGuess->setObjectName(QString::fromUtf8("checkBox_chGuess"));

        verticalLayout_6->addWidget(checkBox_chGuess);

        pushButton_setBoundaryVectors = new QPushButton(tab);
        pushButton_setBoundaryVectors->setObjectName(QString::fromUtf8("pushButton_setBoundaryVectors"));

        verticalLayout_6->addWidget(pushButton_setBoundaryVectors);


        verticalLayout_2->addLayout(verticalLayout_6);

        pushButton_setGuideFields = new QPushButton(tab);
        pushButton_setGuideFields->setObjectName(QString::fromUtf8("pushButton_setGuideFields"));
        pushButton_setGuideFields->setFont(font);

        verticalLayout_2->addWidget(pushButton_setGuideFields);

        doubleSpinBox_pivotMag = new QDoubleSpinBox(tab);
        doubleSpinBox_pivotMag->setObjectName(QString::fromUtf8("doubleSpinBox_pivotMag"));
        doubleSpinBox_pivotMag->setMinimum(-9.000000000000000);
        doubleSpinBox_pivotMag->setMaximum(9.000000000000000);
        doubleSpinBox_pivotMag->setValue(9.000000000000000);

        verticalLayout_2->addWidget(doubleSpinBox_pivotMag);

        pushButton_propagateField = new QPushButton(tab);
        pushButton_propagateField->setObjectName(QString::fromUtf8("pushButton_propagateField"));
        pushButton_propagateField->setFont(font);

        verticalLayout_2->addWidget(pushButton_propagateField);

        pushButton_generateScalarField = new QPushButton(tab);
        pushButton_generateScalarField->setObjectName(QString::fromUtf8("pushButton_generateScalarField"));

        verticalLayout_2->addWidget(pushButton_generateScalarField);

        line_3 = new QFrame(tab);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_3);

        pushButton_detectSingularity = new QPushButton(tab);
        pushButton_detectSingularity->setObjectName(QString::fromUtf8("pushButton_detectSingularity"));

        verticalLayout_2->addWidget(pushButton_detectSingularity);

        pushButton_correctSingularity = new QPushButton(tab);
        pushButton_correctSingularity->setObjectName(QString::fromUtf8("pushButton_correctSingularity"));

        verticalLayout_2->addWidget(pushButton_correctSingularity);

        pushButton_checkFieldDifference = new QPushButton(tab);
        pushButton_checkFieldDifference->setObjectName(QString::fromUtf8("pushButton_checkFieldDifference"));

        verticalLayout_2->addWidget(pushButton_checkFieldDifference);

        pushButton_convert2GradField = new QPushButton(tab);
        pushButton_convert2GradField->setObjectName(QString::fromUtf8("pushButton_convert2GradField"));

        verticalLayout_2->addWidget(pushButton_convert2GradField);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(-1, 4, -1, -1);
        checkBox_Flip = new QCheckBox(tab);
        checkBox_Flip->setObjectName(QString::fromUtf8("checkBox_Flip"));

        horizontalLayout_7->addWidget(checkBox_Flip);

        checkBox_forceIndex = new QCheckBox(tab);
        checkBox_forceIndex->setObjectName(QString::fromUtf8("checkBox_forceIndex"));

        horizontalLayout_7->addWidget(checkBox_forceIndex);


        verticalLayout_2->addLayout(horizontalLayout_7);

        doubleSpinBox_forcingIndex = new QDoubleSpinBox(tab);
        doubleSpinBox_forcingIndex->setObjectName(QString::fromUtf8("doubleSpinBox_forcingIndex"));
        doubleSpinBox_forcingIndex->setMinimum(-1000000.000000000000000);
        doubleSpinBox_forcingIndex->setMaximum(1000000.000000000000000);
        doubleSpinBox_forcingIndex->setSingleStep(0.010000000000000);

        verticalLayout_2->addWidget(doubleSpinBox_forcingIndex);

        pushButton_correctPlanarSingularity = new QPushButton(tab);
        pushButton_correctPlanarSingularity->setObjectName(QString::fromUtf8("pushButton_correctPlanarSingularity"));

        verticalLayout_2->addWidget(pushButton_correctPlanarSingularity);

        pushButton_correctInnerBound = new QPushButton(tab);
        pushButton_correctInnerBound->setObjectName(QString::fromUtf8("pushButton_correctInnerBound"));

        verticalLayout_2->addWidget(pushButton_correctInnerBound);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        pushButton_isoLayerGeneration = new QPushButton(tab);
        pushButton_isoLayerGeneration->setObjectName(QString::fromUtf8("pushButton_isoLayerGeneration"));
        pushButton_isoLayerGeneration->setFont(font);

        horizontalLayout_2->addWidget(pushButton_isoLayerGeneration);

        label_6 = new QLabel(tab);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_2->addWidget(label_6);

        doubleSpinBox_isoLayerNumber = new QDoubleSpinBox(tab);
        doubleSpinBox_isoLayerNumber->setObjectName(QString::fromUtf8("doubleSpinBox_isoLayerNumber"));
        doubleSpinBox_isoLayerNumber->setValue(5.000000000000000);

        horizontalLayout_2->addWidget(doubleSpinBox_isoLayerNumber);


        verticalLayout_2->addLayout(horizontalLayout_2);

        line_2 = new QFrame(tab);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font);

        horizontalLayout_3->addWidget(label);

        doubleSpinBox_toolPathWidth = new QDoubleSpinBox(tab);
        doubleSpinBox_toolPathWidth->setObjectName(QString::fromUtf8("doubleSpinBox_toolPathWidth"));
        doubleSpinBox_toolPathWidth->setValue(2.000000000000000);

        horizontalLayout_3->addWidget(doubleSpinBox_toolPathWidth);

        label_3 = new QLabel(tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setFont(font);

        horizontalLayout_3->addWidget(label_3);

        doubleSpinBox_toolPathDistance = new QDoubleSpinBox(tab);
        doubleSpinBox_toolPathDistance->setObjectName(QString::fromUtf8("doubleSpinBox_toolPathDistance"));
        doubleSpinBox_toolPathDistance->setValue(2.000000000000000);

        horizontalLayout_3->addWidget(doubleSpinBox_toolPathDistance);


        verticalLayout_2->addLayout(horizontalLayout_3);

        pushButton_toolPathGeneration = new QPushButton(tab);
        pushButton_toolPathGeneration->setObjectName(QString::fromUtf8("pushButton_toolPathGeneration"));
        pushButton_toolPathGeneration->setFont(font);

        verticalLayout_2->addWidget(pushButton_toolPathGeneration);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_4 = new QLabel(tab);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setFont(font);

        horizontalLayout_5->addWidget(label_4);

        checkBox_outputLayer = new QCheckBox(tab);
        checkBox_outputLayer->setObjectName(QString::fromUtf8("checkBox_outputLayer"));
        QFont font5;
        font5.setBold(false);
        font5.setWeight(50);
        checkBox_outputLayer->setFont(font5);

        horizontalLayout_5->addWidget(checkBox_outputLayer);

        checkBox_outputToolpath = new QCheckBox(tab);
        checkBox_outputToolpath->setObjectName(QString::fromUtf8("checkBox_outputToolpath"));
        checkBox_outputToolpath->setFont(font5);

        horizontalLayout_5->addWidget(checkBox_outputToolpath);


        verticalLayout_2->addLayout(horizontalLayout_5);

        pushButton_clearAll = new QPushButton(tab);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));

        verticalLayout_2->addWidget(pushButton_clearAll);

        tabWidget->addTab(tab, QString());
        Interactive = new QWidget();
        Interactive->setObjectName(QString::fromUtf8("Interactive"));
        horizontalLayout_9 = new QHBoxLayout(Interactive);
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        pushButton_vecDisplay = new QPushButton(Interactive);
        pushButton_vecDisplay->setObjectName(QString::fromUtf8("pushButton_vecDisplay"));

        verticalLayout_4->addWidget(pushButton_vecDisplay);

        pushButton_intr_editVectorDirection = new QPushButton(Interactive);
        pushButton_intr_editVectorDirection->setObjectName(QString::fromUtf8("pushButton_intr_editVectorDirection"));

        verticalLayout_4->addWidget(pushButton_intr_editVectorDirection);

        label_5 = new QLabel(Interactive);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_4->addWidget(label_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(5);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(-1, 0, -1, -1);
        doubleSpinBox_intr_VecEdit_nx = new QDoubleSpinBox(Interactive);
        doubleSpinBox_intr_VecEdit_nx->setObjectName(QString::fromUtf8("doubleSpinBox_intr_VecEdit_nx"));
        doubleSpinBox_intr_VecEdit_nx->setMinimum(-99.989999999999995);

        horizontalLayout_6->addWidget(doubleSpinBox_intr_VecEdit_nx);

        doubleSpinBox_intr_VecEdit_ny = new QDoubleSpinBox(Interactive);
        doubleSpinBox_intr_VecEdit_ny->setObjectName(QString::fromUtf8("doubleSpinBox_intr_VecEdit_ny"));
        doubleSpinBox_intr_VecEdit_ny->setMinimum(-99.989999999999995);

        horizontalLayout_6->addWidget(doubleSpinBox_intr_VecEdit_ny);

        doubleSpinBox_intr_VecEdit_nz = new QDoubleSpinBox(Interactive);
        doubleSpinBox_intr_VecEdit_nz->setObjectName(QString::fromUtf8("doubleSpinBox_intr_VecEdit_nz"));
        doubleSpinBox_intr_VecEdit_nz->setMinimum(-100.000000000000000);

        horizontalLayout_6->addWidget(doubleSpinBox_intr_VecEdit_nz);

        pushButton_intr_setVectorDirection = new QPushButton(Interactive);
        pushButton_intr_setVectorDirection->setObjectName(QString::fromUtf8("pushButton_intr_setVectorDirection"));
        pushButton_intr_setVectorDirection->setIconSize(QSize(10, 20));

        horizontalLayout_6->addWidget(pushButton_intr_setVectorDirection);

        pushButton_intr_getVectorDirection = new QPushButton(Interactive);
        pushButton_intr_getVectorDirection->setObjectName(QString::fromUtf8("pushButton_intr_getVectorDirection"));

        horizontalLayout_6->addWidget(pushButton_intr_getVectorDirection);


        verticalLayout_4->addLayout(horizontalLayout_6);

        pushButton_intr_Vector_Node2Tets = new QPushButton(Interactive);
        pushButton_intr_Vector_Node2Tets->setObjectName(QString::fromUtf8("pushButton_intr_Vector_Node2Tets"));
        pushButton_intr_Vector_Node2Tets->setEnabled(false);

        verticalLayout_4->addWidget(pushButton_intr_Vector_Node2Tets);

        pushButton_deSelectNode = new QPushButton(Interactive);
        pushButton_deSelectNode->setObjectName(QString::fromUtf8("pushButton_deSelectNode"));

        verticalLayout_4->addWidget(pushButton_deSelectNode);

        pushButton_facePatchSelect = new QPushButton(Interactive);
        pushButton_facePatchSelect->setObjectName(QString::fromUtf8("pushButton_facePatchSelect"));

        verticalLayout_4->addWidget(pushButton_facePatchSelect);

        checkBox_faceMode = new QCheckBox(Interactive);
        checkBox_faceMode->setObjectName(QString::fromUtf8("checkBox_faceMode"));

        verticalLayout_4->addWidget(checkBox_faceMode);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_2);


        horizontalLayout_9->addLayout(verticalLayout_4);

        tabWidget->addTab(Interactive, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_3 = new QVBoxLayout(tab_2);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_2 = new QLabel(tab_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_4->addWidget(label_2);

        spinBox_ShowLayerIndex = new QSpinBox(tab_2);
        spinBox_ShowLayerIndex->setObjectName(QString::fromUtf8("spinBox_ShowLayerIndex"));

        horizontalLayout_4->addWidget(spinBox_ShowLayerIndex);

        checkBox_EachLayerSwitch = new QCheckBox(tab_2);
        checkBox_EachLayerSwitch->setObjectName(QString::fromUtf8("checkBox_EachLayerSwitch"));

        horizontalLayout_4->addWidget(checkBox_EachLayerSwitch);

        pushButton_ShowAllLayers = new QPushButton(tab_2);
        pushButton_ShowAllLayers->setObjectName(QString::fromUtf8("pushButton_ShowAllLayers"));

        horizontalLayout_4->addWidget(pushButton_ShowAllLayers);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        radioButton_SectionY = new QRadioButton(tab_2);
        radioButton_SectionY->setObjectName(QString::fromUtf8("radioButton_SectionY"));

        horizontalLayout_8->addWidget(radioButton_SectionY);

        radioButton_SectionZ = new QRadioButton(tab_2);
        radioButton_SectionZ->setObjectName(QString::fromUtf8("radioButton_SectionZ"));

        horizontalLayout_8->addWidget(radioButton_SectionZ);

        radioButton_SectionX = new QRadioButton(tab_2);
        radioButton_SectionX->setObjectName(QString::fromUtf8("radioButton_SectionX"));

        horizontalLayout_8->addWidget(radioButton_SectionX);


        verticalLayout_3->addLayout(horizontalLayout_8);

        horizontalSlider_Section = new QSlider(tab_2);
        horizontalSlider_Section->setObjectName(QString::fromUtf8("horizontalSlider_Section"));
        horizontalSlider_Section->setMaximum(100);
        horizontalSlider_Section->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(horizontalSlider_Section);

        pushButton_updateSection = new QPushButton(tab_2);
        pushButton_updateSection->setObjectName(QString::fromUtf8("pushButton_updateSection"));

        verticalLayout_3->addWidget(pushButton_updateSection);

        checkBox_colorPivots = new QCheckBox(tab_2);
        checkBox_colorPivots->setObjectName(QString::fromUtf8("checkBox_colorPivots"));

        verticalLayout_3->addWidget(checkBox_colorPivots);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        horizontalLayout_12->setContentsMargins(0, 10, -1, -1);
        doubleSpinBox_colorR = new QDoubleSpinBox(tab_2);
        doubleSpinBox_colorR->setObjectName(QString::fromUtf8("doubleSpinBox_colorR"));
        doubleSpinBox_colorR->setMaximum(1.000000000000000);
        doubleSpinBox_colorR->setSingleStep(0.010000000000000);

        horizontalLayout_12->addWidget(doubleSpinBox_colorR);

        doubleSpinBox_colorG = new QDoubleSpinBox(tab_2);
        doubleSpinBox_colorG->setObjectName(QString::fromUtf8("doubleSpinBox_colorG"));
        doubleSpinBox_colorG->setMaximum(1.000000000000000);
        doubleSpinBox_colorG->setSingleStep(0.010000000000000);

        horizontalLayout_12->addWidget(doubleSpinBox_colorG);

        doubleSpinBox_colorB = new QDoubleSpinBox(tab_2);
        doubleSpinBox_colorB->setObjectName(QString::fromUtf8("doubleSpinBox_colorB"));
        doubleSpinBox_colorB->setMaximum(1.000000000000000);
        doubleSpinBox_colorB->setSingleStep(0.010000000000000);

        horizontalLayout_12->addWidget(doubleSpinBox_colorB);


        verticalLayout_3->addLayout(horizontalLayout_12);

        pushButton_updateVecViz = new QPushButton(tab_2);
        pushButton_updateVecViz->setObjectName(QString::fromUtf8("pushButton_updateVecViz"));

        verticalLayout_3->addWidget(pushButton_updateVecViz);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        horizontalLayout_11->setContentsMargins(-1, 4, -1, -1);
        pushButton_extractLayer = new QPushButton(tab_2);
        pushButton_extractLayer->setObjectName(QString::fromUtf8("pushButton_extractLayer"));

        horizontalLayout_11->addWidget(pushButton_extractLayer);

        doubleSpinBox_ExtractLayerIndex = new QDoubleSpinBox(tab_2);
        doubleSpinBox_ExtractLayerIndex->setObjectName(QString::fromUtf8("doubleSpinBox_ExtractLayerIndex"));
        doubleSpinBox_ExtractLayerIndex->setMinimum(-100000.000000000000000);
        doubleSpinBox_ExtractLayerIndex->setMaximum(100000.000000000000000);
        doubleSpinBox_ExtractLayerIndex->setSingleStep(0.100000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_ExtractLayerIndex);


        verticalLayout_3->addLayout(horizontalLayout_11);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        tabWidget->addTab(tab_2, QString());

        verticalLayout->addWidget(tabWidget);

        pushButton_FixtureField = new QPushButton(dockWidgetContents);
        pushButton_FixtureField->setObjectName(QString::fromUtf8("pushButton_FixtureField"));

        verticalLayout->addWidget(pushButton_FixtureField);

        treeView = new QTreeView(dockWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        verticalLayout->addWidget(treeView);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1244, 26));
        menuBar->setLayoutDirection(Qt::LeftToRight);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuSelect = new QMenu(menuBar);
        menuSelect->setObjectName(QString::fromUtf8("menuSelect"));
        MainWindow->setMenuBar(menuBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setMovable(false);
        toolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        navigationToolBar->addAction(actionFront);
        navigationToolBar->addAction(actionBack);
        navigationToolBar->addAction(actionTop);
        navigationToolBar->addAction(actionBottom);
        navigationToolBar->addAction(actionLeft);
        navigationToolBar->addAction(actionRight);
        navigationToolBar->addAction(actionIsometric);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionZoom_In);
        navigationToolBar->addAction(actionZoom_Out);
        navigationToolBar->addAction(actionZoom_All);
        navigationToolBar->addAction(actionZoom_Window);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionShade);
        navigationToolBar->addAction(actionMesh);
        navigationToolBar->addAction(actionNode);
        navigationToolBar->addAction(actionProfile);
        navigationToolBar->addAction(actionFaceNormal);
        navigationToolBar->addAction(actionNodeNormal);
        selectionToolBar->addAction(actionSaveSelection);
        selectionToolBar->addAction(actionReadSelection);
        selectionToolBar->addSeparator();
        selectionToolBar->addAction(actionSelectNode);
        selectionToolBar->addAction(actionSelectEdge);
        selectionToolBar->addAction(actionSelectFace);
        selectionToolBar->addAction(actionSelectFix);
        selectionToolBar->addAction(actionSelectHandle);
        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuSelect->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSaveSelection);
        menuFile->addAction(actionReadSelection);
        menuView->addAction(actionFront);
        menuView->addAction(actionBack);
        menuView->addAction(actionTop);
        menuView->addAction(actionBottom);
        menuView->addAction(actionLeft);
        menuView->addAction(actionRight);
        menuView->addAction(actionIsometric);
        menuView->addSeparator();
        menuView->addAction(actionZoom_In);
        menuView->addAction(actionZoom_Out);
        menuView->addAction(actionZoom_All);
        menuView->addAction(actionZoom_Window);
        menuView->addSeparator();
        menuView->addAction(actionShade);
        menuView->addAction(actionMesh);
        menuView->addAction(actionNode);
        menuView->addAction(actionProfile);
        menuView->addSeparator();
        menuView->addAction(actionShifttoOrigin);
        menuSelect->addAction(actionSelectNode);
        menuSelect->addAction(actionSelectEdge);
        menuSelect->addAction(actionSelectFace);
        menuSelect->addSeparator();
        menuSelect->addAction(actionSelectFix);
        menuSelect->addAction(actionSelectHandle);
        menuSelect->addSeparator();
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", nullptr));
        actionFront->setText(QApplication::translate("MainWindow", "Front", nullptr));
        actionBack->setText(QApplication::translate("MainWindow", "Back", nullptr));
        actionTop->setText(QApplication::translate("MainWindow", "Top", nullptr));
        actionBottom->setText(QApplication::translate("MainWindow", "Bottom", nullptr));
        actionLeft->setText(QApplication::translate("MainWindow", "Left", nullptr));
        actionRight->setText(QApplication::translate("MainWindow", "Right", nullptr));
        actionIsometric->setText(QApplication::translate("MainWindow", "Isometric", nullptr));
        actionZoom_In->setText(QApplication::translate("MainWindow", "Zoom In", nullptr));
        actionZoom_Out->setText(QApplication::translate("MainWindow", "Zoom Out", nullptr));
        actionZoom_All->setText(QApplication::translate("MainWindow", "Zoom All", nullptr));
        actionZoom_Window->setText(QApplication::translate("MainWindow", "Zoom Window", nullptr));
        actionShade->setText(QApplication::translate("MainWindow", "Shade", nullptr));
        actionMesh->setText(QApplication::translate("MainWindow", "Mesh", nullptr));
        actionNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSave->setText(QApplication::translate("MainWindow", "Save", nullptr));
        actionSelectNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSelectFace->setText(QApplication::translate("MainWindow", "Face", nullptr));
        actionShifttoOrigin->setText(QApplication::translate("MainWindow", "Shift to Origin", nullptr));
        actionProfile->setText(QApplication::translate("MainWindow", "Profile", nullptr));
        actionFaceNormal->setText(QApplication::translate("MainWindow", "FaceNormal", nullptr));
        actionNodeNormal->setText(QApplication::translate("MainWindow", "NodeNormal", nullptr));
        actionSelectEdge->setText(QApplication::translate("MainWindow", "Edge", nullptr));
        actionGenerate->setText(QApplication::translate("MainWindow", "Generate", nullptr));
        actionTest_1->setText(QApplication::translate("MainWindow", "Test_1", nullptr));
        actionSelectFix->setText(QApplication::translate("MainWindow", "Fix", nullptr));
        actionSelectHandle->setText(QApplication::translate("MainWindow", "Handle & Rigid", nullptr));
        actionSaveSelection->setText(QApplication::translate("MainWindow", "Save selection", nullptr));
        actionReadSelection->setText(QApplication::translate("MainWindow", "Read selection", nullptr));
        actionSelectChamber->setText(QApplication::translate("MainWindow", "Select Chamber (SORO)", nullptr));
        actionExport_to_Abaqus_model->setText(QApplication::translate("MainWindow", "Export to Abaqus model", nullptr));
        navigationToolBar->setWindowTitle(QApplication::translate("MainWindow", "navigationToolBar", nullptr));
        selectionToolBar->setWindowTitle(QApplication::translate("MainWindow", "selectionToolBar", nullptr));
        label_MANY_3DP_CNC_CAM->setText(QApplication::translate("MainWindow", "5AM Rough Machining", nullptr));
        boxDeselect->setText(QApplication::translate("MainWindow", "Select", nullptr));
#ifndef QT_NO_WHATSTHIS
        tabWidget->setWhatsThis(QApplication::translate("MainWindow", "<html><head/><body><p><br/></p></body></html>", nullptr));
#endif // QT_NO_WHATSTHIS
        label_inputFile->setText(QApplication::translate("MainWindow", "File:", nullptr));
        lineEdit_SorceDataDir->setText(QApplication::translate("MainWindow", "<Model_Name>", nullptr));
        pushButton_readData->setText(QApplication::translate("MainWindow", "Read Data", nullptr));
        IsIn_radioButton->setText(QApplication::translate("MainWindow", "In", nullptr));
        pushButton_initialise->setText(QApplication::translate("MainWindow", "Initialise Meshes", nullptr));
        checkBox_inBoundGuess->setText(QApplication::translate("MainWindow", "Inner Boun. Guess", nullptr));
        checkBox_outBoundGuess->setText(QApplication::translate("MainWindow", "Outer Bound. Guess", nullptr));
        checkBox_chGuess->setText(QApplication::translate("MainWindow", "Convex Hull Guess", nullptr));
        pushButton_setBoundaryVectors->setText(QApplication::translate("MainWindow", "Initialise Vectors Field", nullptr));
        pushButton_setGuideFields->setText(QApplication::translate("MainWindow", "Set Guide Fields", nullptr));
        pushButton_propagateField->setText(QApplication::translate("MainWindow", "Propagete Field", nullptr));
        pushButton_generateScalarField->setText(QApplication::translate("MainWindow", "Generate  Scalar Field", nullptr));
        pushButton_detectSingularity->setText(QApplication::translate("MainWindow", "Detect Singularity", nullptr));
        pushButton_correctSingularity->setText(QApplication::translate("MainWindow", "Correct Singularity", nullptr));
        pushButton_checkFieldDifference->setText(QApplication::translate("MainWindow", "Check Difference", nullptr));
        pushButton_convert2GradField->setText(QApplication::translate("MainWindow", "Convert2GradientField", nullptr));
        checkBox_Flip->setText(QApplication::translate("MainWindow", "Flip?", nullptr));
        checkBox_forceIndex->setText(QApplication::translate("MainWindow", "Force Index?", nullptr));
        pushButton_correctPlanarSingularity->setText(QApplication::translate("MainWindow", "Correct Planar Singularity", nullptr));
        pushButton_correctInnerBound->setText(QApplication::translate("MainWindow", "Correct  Inner Bound", nullptr));
        pushButton_isoLayerGeneration->setText(QApplication::translate("MainWindow", "IsoLayer Generation", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Depth :", nullptr));
        label->setText(QApplication::translate("MainWindow", "Width", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Dist", nullptr));
        pushButton_toolPathGeneration->setText(QApplication::translate("MainWindow", "Toolpath Generation", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Output", nullptr));
        checkBox_outputLayer->setText(QApplication::translate("MainWindow", "layer", nullptr));
        checkBox_outputToolpath->setText(QApplication::translate("MainWindow", "toolpath", nullptr));
        pushButton_clearAll->setText(QApplication::translate("MainWindow", "Clear All", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Calculation", nullptr));
        pushButton_vecDisplay->setText(QApplication::translate("MainWindow", "Display Vector", nullptr));
        pushButton_intr_editVectorDirection->setText(QApplication::translate("MainWindow", "Edit Vector Direction", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "(nx,ny,nz):", nullptr));
        pushButton_intr_setVectorDirection->setText(QApplication::translate("MainWindow", "Set", nullptr));
        pushButton_intr_getVectorDirection->setText(QApplication::translate("MainWindow", "Update", nullptr));
        pushButton_intr_Vector_Node2Tets->setText(QApplication::translate("MainWindow", "Translate Vector to Tets.", nullptr));
        pushButton_deSelectNode->setText(QApplication::translate("MainWindow", "deSelect Node", nullptr));
        pushButton_facePatchSelect->setText(QApplication::translate("MainWindow", "select adjacent face patch", nullptr));
        checkBox_faceMode->setText(QApplication::translate("MainWindow", "Face Mode", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(Interactive), QApplication::translate("MainWindow", "Interact", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Show:", nullptr));
        checkBox_EachLayerSwitch->setText(QApplication::translate("MainWindow", "single", nullptr));
        pushButton_ShowAllLayers->setText(QApplication::translate("MainWindow", "All", nullptr));
        radioButton_SectionY->setText(QApplication::translate("MainWindow", "Section Y", nullptr));
        radioButton_SectionZ->setText(QApplication::translate("MainWindow", "Section Z", nullptr));
        radioButton_SectionX->setText(QApplication::translate("MainWindow", "Section X", nullptr));
        pushButton_updateSection->setText(QApplication::translate("MainWindow", "Update Section", nullptr));
        checkBox_colorPivots->setText(QApplication::translate("MainWindow", "Colour Pivots", nullptr));
        pushButton_updateVecViz->setText(QApplication::translate("MainWindow", "Update Vector Visualisation", nullptr));
        pushButton_extractLayer->setText(QApplication::translate("MainWindow", "Extract Layer", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Visualization", nullptr));
        pushButton_FixtureField->setText(QApplication::translate("MainWindow", "Fixture Field", nullptr));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QApplication::translate("MainWindow", "View", nullptr));
        menuSelect->setTitle(QApplication::translate("MainWindow", "Select", nullptr));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
