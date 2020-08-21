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
#include <QtGui/QIcon>
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
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
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
    QAction *actionShape_Up_Mesh_Deformation;
    QAction *actionInputFEM;
    QAction *actionFaceNormalShade;
    QAction *actionNodeNormalShade;
    QAction *actionGeoField_boundary;
    QAction *actionGeoField_directional;
    QAction *actionGeoField_selection;
    QAction *action_GenerateTrajectory;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QToolBar *navigationToolBar;
    QStatusBar *statusBar;
    QToolBar *selectionToolBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_16;
    QCheckBox *boxDeselect;
    QCheckBox *checkBox_spaseVectorField;
    QCheckBox *checkBox_3DCompute;
    QTabWidget *tabWidget;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_3;
    QPushButton *pushButton_compPrincipleStressField;
    QHBoxLayout *horizontalLayout_12;
    QLabel *Tensile;
    QLineEdit *tensileRegionRatio;
    QLabel *Compress;
    QLineEdit *compressRegionRatio;
    QPushButton *pushButton_changeTensileandCompressRegion;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *pushButton_CompGuideField;
    QCheckBox *checkBox_drawScalarGradient;
    QHBoxLayout *horizontalLayout_15;
    QPushButton *pushButton_CompVectorField;
    QPushButton *pushButton_VectorFieldFlipNormal;
    QPushButton *pushButton_VectorFieldDeleteRegion;
    QPushButton *pushButton_CompScalarField;
    QLabel *label_20;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *pushButtonBuildIsoSurface;
    QLabel *label_11;
    QSpinBox *isoLayerNumber;
    QHBoxLayout *horizontalLayout_13;
    QCheckBox *checkBox_UniformThickness;
    QDoubleSpinBox *doubleSpinBox_LayerThickMin;
    QDoubleSpinBox *doubleSpinBox_LayerThickMax;
    QHBoxLayout *horizontalLayout_33;
    QPushButton *pushButton_compFieldonIsoSurface;
    QPushButton *pushButton_changeOrder;
    QHBoxLayout *horizontalLayout_5;
    QLabel *LayerIndexLabel;
    QSpinBox *IsoLayerIndex;
    QRadioButton *VisualSingleLayerButtom;
    QPushButton *pushButton_viewallLayerandOffset;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_outputIsoLayer;
    QSpinBox *outputLayerIndexNum;
    QCheckBox *checkBox_outputSingleLayer;
    QCheckBox *checkBox_outputILayer_splitMode;
    QCheckBox *checkBox_outputILayer_OFFMode;
    QWidget *tab_5;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_27;
    QPushButton *pushButton_fabricationDirection;
    QCheckBox *checkBox_3DrotateInverse;
    QHBoxLayout *horizontalLayout_31;
    QCheckBox *checkBox_computeFabricationDirection;
    QSpinBox *spinBox_ComputedAnlge;
    QDoubleSpinBox *spinBox_ComputedAnlgeAlpha;
    QDoubleSpinBox *spinBox_ComputedAnlgeBeta;
    QHBoxLayout *horizontalLayout_21;
    QPushButton *pushButton_supportingStructureDetection;
    QPushButton *pushButton_buildSupportNode;
    QCheckBox *checkBox_saveSupportNode;
    QHBoxLayout *horizontalLayout_23;
    QPushButton *pushButton_selectSupportRegion;
    QCheckBox *checkBox_deselectSupportNode;
    QCheckBox *checkBox_deselectSupportNode_byNode;
    QHBoxLayout *horizontalLayout_28;
    QPushButton *pushButton_layerThickCompandDraw;
    QHBoxLayout *horizontalLayout_32;
    QPushButton *pushButton_checkCollision;
    QCheckBox *checkBox_CollisionUpdateLayer;
    QLabel *label_23;
    QHBoxLayout *horizontalLayout_26;
    QPushButton *pushButton_InputIsoLayerwithSupport;
    QPushButton *pushButton_cutLayerbyConvexHull;
    QPushButton *pushButton_objtooff;
    QHBoxLayout *horizontalLayout_22;
    QPushButton *pushButton_fabricationSupportTetMesh;
    QPushButton *pushButton_fabricationSupportSurface;
    QHBoxLayout *horizontalLayout_25;
    QPushButton *pushButton_fabricationEffectSupportSurfaceDetection;
    QDoubleSpinBox *doubleSpinBox_supportNodeBoxSize;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBox_toolpathOffsetValue;
    QLabel *label_2;
    QSpinBox *spinBox_bToolPath_Num;
    QHBoxLayout *horizontalLayout_17;
    QLabel *label_4;
    QDoubleSpinBox *doubleSpinBox_maxGapDistance;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label_19;
    QDoubleSpinBox *doubleSpinBox_zigzagBoundConnectDistance;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_12;
    QDoubleSpinBox *doubleSpinBox_boundShrinkValue;
    QLabel *label_22;
    QPushButton *pushButton_runHeatMethod;
    QHBoxLayout *horizontalLayout_29;
    QPushButton *pushButton_generateZigZagToolPath;
    QPushButton *pushButton_generateStressFieldToolPath;
    QHBoxLayout *horizontalLayout_30;
    QPushButton *pushButton_outputToolPath;
    QPushButton *pushButton_directToolPathGeneration;
    QLabel *label_21;
    QPushButton *pushButton_toolpathGenerationChecking;
    QHBoxLayout *horizontalLayout_14;
    QPushButton *pushButton_allLayerToolpathGenerate;
    QCheckBox *checkBox_cpuSpeedUpToolpathGeneration;
    QWidget *tab;
    QVBoxLayout *verticalLayout_4;
    QPushButton *pushButton_deselectAllinModelTree;
    QHBoxLayout *horizontalLayout_24;
    QPushButton *pushButton_scaleModelSize;
    QDoubleSpinBox *doubleSpinBox_modelScaleFactor;
    QPushButton *pushButton_heatMethodTest;
    QHBoxLayout *horizontalLayout_3;
    QLabel *maxValue;
    QSlider *maximumValue;
    QHBoxLayout *horizontalLayout_2;
    QLabel *minValue;
    QSlider *minimumValue;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_14;
    QLineEdit *LaplaceWeight;
    QPushButton *pushButton_tetraDelete;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_5;
    QPushButton *pushButton_volumetoSurface;
    QHBoxLayout *horizontalLayout_20;
    QLabel *label_9;
    QDoubleSpinBox *doubleSpinBox_voxelWidth;
    QLabel *label_10;
    QSpinBox *spinBox_expandingLayer;
    QPushButton *pushButton_GenerateVoxel;
    QLabel *label_8;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *pushButton_voxelLayerGeneration;
    QSpinBox *spinBox_VoxelGenerateMethod;
    QCheckBox *checkBox_loadPlatform;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_18;
    QSpinBox *spinBox_voxelLayer;
    QPushButton *pushButton_saveVoxelOrder;
    QPushButton *pushButton_loadVoxelOrder;
    QLabel *label_6;
    QPushButton *pushButton_isoFacefromVoxelLayer;
    QPushButton *pushButton_translateTetrahedraltoVoxel;
    QFrame *frame_modelTree;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QTreeView *treeView;
    QLabel *label_5;
    QTextEdit *SystemDialog;
    QPushButton *pushButton_clearAll;
    QPushButton *pushButton_closewindow;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuFace_Node_shade;
    QMenu *menuSelect;
    QMenu *menuFunction;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1450, 887);
        MainWindow->setMouseTracking(true);
        MainWindow->setFocusPolicy(Qt::StrongFocus);
        MainWindow->setAcceptDrops(true);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/icon.jpg"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/Open Folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon1);
        actionFront = new QAction(MainWindow);
        actionFront->setObjectName(QString::fromUtf8("actionFront"));
        actionFront->setCheckable(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/Front View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFront->setIcon(icon2);
        actionBack = new QAction(MainWindow);
        actionBack->setObjectName(QString::fromUtf8("actionBack"));
        actionBack->setCheckable(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/Back View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBack->setIcon(icon3);
        actionTop = new QAction(MainWindow);
        actionTop->setObjectName(QString::fromUtf8("actionTop"));
        actionTop->setCheckable(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/Top View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionTop->setIcon(icon4);
        actionBottom = new QAction(MainWindow);
        actionBottom->setObjectName(QString::fromUtf8("actionBottom"));
        actionBottom->setCheckable(false);
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/Bottom View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottom->setIcon(icon5);
        actionLeft = new QAction(MainWindow);
        actionLeft->setObjectName(QString::fromUtf8("actionLeft"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/Left View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeft->setIcon(icon6);
        actionRight = new QAction(MainWindow);
        actionRight->setObjectName(QString::fromUtf8("actionRight"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/Right View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRight->setIcon(icon7);
        actionIsometric = new QAction(MainWindow);
        actionIsometric->setObjectName(QString::fromUtf8("actionIsometric"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/resource/Isometric View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionIsometric->setIcon(icon8);
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/resource/Zoom In.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon9);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/resource/Zoom Out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon10);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QString::fromUtf8("actionZoom_All"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/resource/Zoom All.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon11);
        actionZoom_Window = new QAction(MainWindow);
        actionZoom_Window->setObjectName(QString::fromUtf8("actionZoom_Window"));
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/resource/Zoom Window.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Window->setIcon(icon12);
        actionShade = new QAction(MainWindow);
        actionShade->setObjectName(QString::fromUtf8("actionShade"));
        actionShade->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/resource/Shade.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShade->setIcon(icon13);
        actionMesh = new QAction(MainWindow);
        actionMesh->setObjectName(QString::fromUtf8("actionMesh"));
        actionMesh->setCheckable(true);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/resource/Mesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMesh->setIcon(icon14);
        actionNode = new QAction(MainWindow);
        actionNode->setObjectName(QString::fromUtf8("actionNode"));
        actionNode->setCheckable(true);
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/resource/Node.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNode->setIcon(icon15);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/resource/Save as.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon16);
        actionSelectNode = new QAction(MainWindow);
        actionSelectNode->setObjectName(QString::fromUtf8("actionSelectNode"));
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/resource/selectNode.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectNode->setIcon(icon17);
        actionSelectFace = new QAction(MainWindow);
        actionSelectFace->setObjectName(QString::fromUtf8("actionSelectFace"));
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/resource/selectFace.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFace->setIcon(icon18);
        actionShifttoOrigin = new QAction(MainWindow);
        actionShifttoOrigin->setObjectName(QString::fromUtf8("actionShifttoOrigin"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QString::fromUtf8("actionProfile"));
        actionProfile->setCheckable(true);
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/resource/Profile.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionProfile->setIcon(icon19);
        actionFaceNormal = new QAction(MainWindow);
        actionFaceNormal->setObjectName(QString::fromUtf8("actionFaceNormal"));
        actionFaceNormal->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/resource/FaceNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFaceNormal->setIcon(icon20);
        actionNodeNormal = new QAction(MainWindow);
        actionNodeNormal->setObjectName(QString::fromUtf8("actionNodeNormal"));
        actionNodeNormal->setCheckable(true);
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/resource/NodeNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNodeNormal->setIcon(icon21);
        actionSelectEdge = new QAction(MainWindow);
        actionSelectEdge->setObjectName(QString::fromUtf8("actionSelectEdge"));
        QIcon icon22;
        icon22.addFile(QString::fromUtf8(":/resource/selectEdge.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectEdge->setIcon(icon22);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QString::fromUtf8("actionGenerate"));
        actionTest_1 = new QAction(MainWindow);
        actionTest_1->setObjectName(QString::fromUtf8("actionTest_1"));
        actionSelectFix = new QAction(MainWindow);
        actionSelectFix->setObjectName(QString::fromUtf8("actionSelectFix"));
        QIcon icon23;
        icon23.addFile(QString::fromUtf8(":/resource/selectFix.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFix->setIcon(icon23);
        actionSelectHandle = new QAction(MainWindow);
        actionSelectHandle->setObjectName(QString::fromUtf8("actionSelectHandle"));
        QIcon icon24;
        icon24.addFile(QString::fromUtf8(":/resource/selectHandle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectHandle->setIcon(icon24);
        actionSaveSelection = new QAction(MainWindow);
        actionSaveSelection->setObjectName(QString::fromUtf8("actionSaveSelection"));
        QIcon icon25;
        icon25.addFile(QString::fromUtf8(":/resource/SaveSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSaveSelection->setIcon(icon25);
        actionReadSelection = new QAction(MainWindow);
        actionReadSelection->setObjectName(QString::fromUtf8("actionReadSelection"));
        QIcon icon26;
        icon26.addFile(QString::fromUtf8(":/resource/InputSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionReadSelection->setIcon(icon26);
        actionSelectChamber = new QAction(MainWindow);
        actionSelectChamber->setObjectName(QString::fromUtf8("actionSelectChamber"));
        actionExport_to_Abaqus_model = new QAction(MainWindow);
        actionExport_to_Abaqus_model->setObjectName(QString::fromUtf8("actionExport_to_Abaqus_model"));
        actionExport_to_Abaqus_model->setCheckable(false);
        QIcon icon27;
        icon27.addFile(QString::fromUtf8(":/resource/abaqus logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport_to_Abaqus_model->setIcon(icon27);
        actionShape_Up_Mesh_Deformation = new QAction(MainWindow);
        actionShape_Up_Mesh_Deformation->setObjectName(QString::fromUtf8("actionShape_Up_Mesh_Deformation"));
        actionInputFEM = new QAction(MainWindow);
        actionInputFEM->setObjectName(QString::fromUtf8("actionInputFEM"));
        QIcon icon28;
        icon28.addFile(QString::fromUtf8(":/resource/FEM.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionInputFEM->setIcon(icon28);
        actionFaceNormalShade = new QAction(MainWindow);
        actionFaceNormalShade->setObjectName(QString::fromUtf8("actionFaceNormalShade"));
        actionNodeNormalShade = new QAction(MainWindow);
        actionNodeNormalShade->setObjectName(QString::fromUtf8("actionNodeNormalShade"));
        actionGeoField_boundary = new QAction(MainWindow);
        actionGeoField_boundary->setObjectName(QString::fromUtf8("actionGeoField_boundary"));
        actionGeoField_directional = new QAction(MainWindow);
        actionGeoField_directional->setObjectName(QString::fromUtf8("actionGeoField_directional"));
        actionGeoField_selection = new QAction(MainWindow);
        actionGeoField_selection->setObjectName(QString::fromUtf8("actionGeoField_selection"));
        action_GenerateTrajectory = new QAction(MainWindow);
        action_GenerateTrajectory->setObjectName(QString::fromUtf8("action_GenerateTrajectory"));
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
        dockWidget->setMinimumSize(QSize(350, 800));
        dockWidget->setMaximumSize(QSize(350, 800));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setSpacing(6);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        boxDeselect = new QCheckBox(dockWidgetContents);
        boxDeselect->setObjectName(QString::fromUtf8("boxDeselect"));
        boxDeselect->setLayoutDirection(Qt::LeftToRight);
        boxDeselect->setChecked(false);

        horizontalLayout_16->addWidget(boxDeselect);

        checkBox_spaseVectorField = new QCheckBox(dockWidgetContents);
        checkBox_spaseVectorField->setObjectName(QString::fromUtf8("checkBox_spaseVectorField"));
        checkBox_spaseVectorField->setChecked(false);

        horizontalLayout_16->addWidget(checkBox_spaseVectorField);

        checkBox_3DCompute = new QCheckBox(dockWidgetContents);
        checkBox_3DCompute->setObjectName(QString::fromUtf8("checkBox_3DCompute"));
        checkBox_3DCompute->setChecked(true);

        horizontalLayout_16->addWidget(checkBox_3DCompute);


        verticalLayout_2->addLayout(horizontalLayout_16);

        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setMinimumSize(QSize(100, 350));
        tabWidget->setMaximumSize(QSize(2000, 350));
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabWidget->setElideMode(Qt::ElideNone);
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_3 = new QVBoxLayout(tab_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        pushButton_compPrincipleStressField = new QPushButton(tab_3);
        pushButton_compPrincipleStressField->setObjectName(QString::fromUtf8("pushButton_compPrincipleStressField"));

        verticalLayout_3->addWidget(pushButton_compPrincipleStressField);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        Tensile = new QLabel(tab_3);
        Tensile->setObjectName(QString::fromUtf8("Tensile"));

        horizontalLayout_12->addWidget(Tensile);

        tensileRegionRatio = new QLineEdit(tab_3);
        tensileRegionRatio->setObjectName(QString::fromUtf8("tensileRegionRatio"));
        tensileRegionRatio->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_12->addWidget(tensileRegionRatio);

        Compress = new QLabel(tab_3);
        Compress->setObjectName(QString::fromUtf8("Compress"));

        horizontalLayout_12->addWidget(Compress);

        compressRegionRatio = new QLineEdit(tab_3);
        compressRegionRatio->setObjectName(QString::fromUtf8("compressRegionRatio"));
        compressRegionRatio->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_12->addWidget(compressRegionRatio);

        pushButton_changeTensileandCompressRegion = new QPushButton(tab_3);
        pushButton_changeTensileandCompressRegion->setObjectName(QString::fromUtf8("pushButton_changeTensileandCompressRegion"));

        horizontalLayout_12->addWidget(pushButton_changeTensileandCompressRegion);


        verticalLayout_3->addLayout(horizontalLayout_12);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        pushButton_CompGuideField = new QPushButton(tab_3);
        pushButton_CompGuideField->setObjectName(QString::fromUtf8("pushButton_CompGuideField"));
        pushButton_CompGuideField->setEnabled(false);

        horizontalLayout_4->addWidget(pushButton_CompGuideField);

        checkBox_drawScalarGradient = new QCheckBox(tab_3);
        checkBox_drawScalarGradient->setObjectName(QString::fromUtf8("checkBox_drawScalarGradient"));
        checkBox_drawScalarGradient->setChecked(true);

        horizontalLayout_4->addWidget(checkBox_drawScalarGradient);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        pushButton_CompVectorField = new QPushButton(tab_3);
        pushButton_CompVectorField->setObjectName(QString::fromUtf8("pushButton_CompVectorField"));

        horizontalLayout_15->addWidget(pushButton_CompVectorField);

        pushButton_VectorFieldFlipNormal = new QPushButton(tab_3);
        pushButton_VectorFieldFlipNormal->setObjectName(QString::fromUtf8("pushButton_VectorFieldFlipNormal"));

        horizontalLayout_15->addWidget(pushButton_VectorFieldFlipNormal);

        pushButton_VectorFieldDeleteRegion = new QPushButton(tab_3);
        pushButton_VectorFieldDeleteRegion->setObjectName(QString::fromUtf8("pushButton_VectorFieldDeleteRegion"));

        horizontalLayout_15->addWidget(pushButton_VectorFieldDeleteRegion);

        pushButton_CompScalarField = new QPushButton(tab_3);
        pushButton_CompScalarField->setObjectName(QString::fromUtf8("pushButton_CompScalarField"));

        horizontalLayout_15->addWidget(pushButton_CompScalarField);


        verticalLayout_3->addLayout(horizontalLayout_15);

        label_20 = new QLabel(tab_3);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setMaximumSize(QSize(16777215, 10));

        verticalLayout_3->addWidget(label_20);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        pushButtonBuildIsoSurface = new QPushButton(tab_3);
        pushButtonBuildIsoSurface->setObjectName(QString::fromUtf8("pushButtonBuildIsoSurface"));
        pushButtonBuildIsoSurface->setMinimumSize(QSize(0, 0));

        horizontalLayout_6->addWidget(pushButtonBuildIsoSurface);

        label_11 = new QLabel(tab_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setMaximumSize(QSize(40, 16777215));

        horizontalLayout_6->addWidget(label_11);

        isoLayerNumber = new QSpinBox(tab_3);
        isoLayerNumber->setObjectName(QString::fromUtf8("isoLayerNumber"));
        isoLayerNumber->setMaximum(1000);
        isoLayerNumber->setValue(100);

        horizontalLayout_6->addWidget(isoLayerNumber);


        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        checkBox_UniformThickness = new QCheckBox(tab_3);
        checkBox_UniformThickness->setObjectName(QString::fromUtf8("checkBox_UniformThickness"));
        checkBox_UniformThickness->setChecked(false);

        horizontalLayout_13->addWidget(checkBox_UniformThickness);

        doubleSpinBox_LayerThickMin = new QDoubleSpinBox(tab_3);
        doubleSpinBox_LayerThickMin->setObjectName(QString::fromUtf8("doubleSpinBox_LayerThickMin"));
        doubleSpinBox_LayerThickMin->setValue(0.100000000000000);

        horizontalLayout_13->addWidget(doubleSpinBox_LayerThickMin);

        doubleSpinBox_LayerThickMax = new QDoubleSpinBox(tab_3);
        doubleSpinBox_LayerThickMax->setObjectName(QString::fromUtf8("doubleSpinBox_LayerThickMax"));
        doubleSpinBox_LayerThickMax->setValue(1.500000000000000);

        horizontalLayout_13->addWidget(doubleSpinBox_LayerThickMax);


        verticalLayout_3->addLayout(horizontalLayout_13);

        horizontalLayout_33 = new QHBoxLayout();
        horizontalLayout_33->setSpacing(6);
        horizontalLayout_33->setObjectName(QString::fromUtf8("horizontalLayout_33"));
        pushButton_compFieldonIsoSurface = new QPushButton(tab_3);
        pushButton_compFieldonIsoSurface->setObjectName(QString::fromUtf8("pushButton_compFieldonIsoSurface"));

        horizontalLayout_33->addWidget(pushButton_compFieldonIsoSurface);

        pushButton_changeOrder = new QPushButton(tab_3);
        pushButton_changeOrder->setObjectName(QString::fromUtf8("pushButton_changeOrder"));

        horizontalLayout_33->addWidget(pushButton_changeOrder);


        verticalLayout_3->addLayout(horizontalLayout_33);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        LayerIndexLabel = new QLabel(tab_3);
        LayerIndexLabel->setObjectName(QString::fromUtf8("LayerIndexLabel"));

        horizontalLayout_5->addWidget(LayerIndexLabel);

        IsoLayerIndex = new QSpinBox(tab_3);
        IsoLayerIndex->setObjectName(QString::fromUtf8("IsoLayerIndex"));
        IsoLayerIndex->setMaximum(300);

        horizontalLayout_5->addWidget(IsoLayerIndex);

        VisualSingleLayerButtom = new QRadioButton(tab_3);
        VisualSingleLayerButtom->setObjectName(QString::fromUtf8("VisualSingleLayerButtom"));

        horizontalLayout_5->addWidget(VisualSingleLayerButtom);

        pushButton_viewallLayerandOffset = new QPushButton(tab_3);
        pushButton_viewallLayerandOffset->setObjectName(QString::fromUtf8("pushButton_viewallLayerandOffset"));

        horizontalLayout_5->addWidget(pushButton_viewallLayerandOffset);


        verticalLayout_3->addLayout(horizontalLayout_5);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        pushButton_outputIsoLayer = new QPushButton(tab_3);
        pushButton_outputIsoLayer->setObjectName(QString::fromUtf8("pushButton_outputIsoLayer"));

        horizontalLayout_7->addWidget(pushButton_outputIsoLayer);

        outputLayerIndexNum = new QSpinBox(tab_3);
        outputLayerIndexNum->setObjectName(QString::fromUtf8("outputLayerIndexNum"));
        outputLayerIndexNum->setAccelerated(true);
        outputLayerIndexNum->setMaximum(999);
        outputLayerIndexNum->setDisplayIntegerBase(10);

        horizontalLayout_7->addWidget(outputLayerIndexNum);

        checkBox_outputSingleLayer = new QCheckBox(tab_3);
        checkBox_outputSingleLayer->setObjectName(QString::fromUtf8("checkBox_outputSingleLayer"));

        horizontalLayout_7->addWidget(checkBox_outputSingleLayer);

        checkBox_outputILayer_splitMode = new QCheckBox(tab_3);
        checkBox_outputILayer_splitMode->setObjectName(QString::fromUtf8("checkBox_outputILayer_splitMode"));
        checkBox_outputILayer_splitMode->setChecked(true);

        horizontalLayout_7->addWidget(checkBox_outputILayer_splitMode);

        checkBox_outputILayer_OFFMode = new QCheckBox(tab_3);
        checkBox_outputILayer_OFFMode->setObjectName(QString::fromUtf8("checkBox_outputILayer_OFFMode"));

        horizontalLayout_7->addWidget(checkBox_outputILayer_OFFMode);


        verticalLayout_3->addLayout(horizontalLayout_7);

        tabWidget->addTab(tab_3, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        verticalLayout_7 = new QVBoxLayout(tab_5);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_27 = new QHBoxLayout();
        horizontalLayout_27->setSpacing(6);
        horizontalLayout_27->setObjectName(QString::fromUtf8("horizontalLayout_27"));
        pushButton_fabricationDirection = new QPushButton(tab_5);
        pushButton_fabricationDirection->setObjectName(QString::fromUtf8("pushButton_fabricationDirection"));

        horizontalLayout_27->addWidget(pushButton_fabricationDirection);

        checkBox_3DrotateInverse = new QCheckBox(tab_5);
        checkBox_3DrotateInverse->setObjectName(QString::fromUtf8("checkBox_3DrotateInverse"));

        horizontalLayout_27->addWidget(checkBox_3DrotateInverse);


        verticalLayout_7->addLayout(horizontalLayout_27);

        horizontalLayout_31 = new QHBoxLayout();
        horizontalLayout_31->setSpacing(6);
        horizontalLayout_31->setObjectName(QString::fromUtf8("horizontalLayout_31"));
        checkBox_computeFabricationDirection = new QCheckBox(tab_5);
        checkBox_computeFabricationDirection->setObjectName(QString::fromUtf8("checkBox_computeFabricationDirection"));

        horizontalLayout_31->addWidget(checkBox_computeFabricationDirection);

        spinBox_ComputedAnlge = new QSpinBox(tab_5);
        spinBox_ComputedAnlge->setObjectName(QString::fromUtf8("spinBox_ComputedAnlge"));
        spinBox_ComputedAnlge->setMaximum(360);
        spinBox_ComputedAnlge->setValue(295);

        horizontalLayout_31->addWidget(spinBox_ComputedAnlge);

        spinBox_ComputedAnlgeAlpha = new QDoubleSpinBox(tab_5);
        spinBox_ComputedAnlgeAlpha->setObjectName(QString::fromUtf8("spinBox_ComputedAnlgeAlpha"));
        spinBox_ComputedAnlgeAlpha->setMinimum(-1000.000000000000000);
        spinBox_ComputedAnlgeAlpha->setMaximum(1000.000000000000000);
        spinBox_ComputedAnlgeAlpha->setValue(230.000000000000000);

        horizontalLayout_31->addWidget(spinBox_ComputedAnlgeAlpha);

        spinBox_ComputedAnlgeBeta = new QDoubleSpinBox(tab_5);
        spinBox_ComputedAnlgeBeta->setObjectName(QString::fromUtf8("spinBox_ComputedAnlgeBeta"));
        spinBox_ComputedAnlgeBeta->setMinimum(-10000.000000000000000);
        spinBox_ComputedAnlgeBeta->setMaximum(10000.000000000000000);
        spinBox_ComputedAnlgeBeta->setValue(130.000000000000000);

        horizontalLayout_31->addWidget(spinBox_ComputedAnlgeBeta);


        verticalLayout_7->addLayout(horizontalLayout_31);

        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setSpacing(6);
        horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
        pushButton_supportingStructureDetection = new QPushButton(tab_5);
        pushButton_supportingStructureDetection->setObjectName(QString::fromUtf8("pushButton_supportingStructureDetection"));

        horizontalLayout_21->addWidget(pushButton_supportingStructureDetection);

        pushButton_buildSupportNode = new QPushButton(tab_5);
        pushButton_buildSupportNode->setObjectName(QString::fromUtf8("pushButton_buildSupportNode"));

        horizontalLayout_21->addWidget(pushButton_buildSupportNode);

        checkBox_saveSupportNode = new QCheckBox(tab_5);
        checkBox_saveSupportNode->setObjectName(QString::fromUtf8("checkBox_saveSupportNode"));

        horizontalLayout_21->addWidget(checkBox_saveSupportNode);


        verticalLayout_7->addLayout(horizontalLayout_21);

        horizontalLayout_23 = new QHBoxLayout();
        horizontalLayout_23->setSpacing(6);
        horizontalLayout_23->setObjectName(QString::fromUtf8("horizontalLayout_23"));
        pushButton_selectSupportRegion = new QPushButton(tab_5);
        pushButton_selectSupportRegion->setObjectName(QString::fromUtf8("pushButton_selectSupportRegion"));

        horizontalLayout_23->addWidget(pushButton_selectSupportRegion);

        checkBox_deselectSupportNode = new QCheckBox(tab_5);
        checkBox_deselectSupportNode->setObjectName(QString::fromUtf8("checkBox_deselectSupportNode"));
        checkBox_deselectSupportNode->setChecked(true);

        horizontalLayout_23->addWidget(checkBox_deselectSupportNode);

        checkBox_deselectSupportNode_byNode = new QCheckBox(tab_5);
        checkBox_deselectSupportNode_byNode->setObjectName(QString::fromUtf8("checkBox_deselectSupportNode_byNode"));
        checkBox_deselectSupportNode_byNode->setChecked(true);

        horizontalLayout_23->addWidget(checkBox_deselectSupportNode_byNode);


        verticalLayout_7->addLayout(horizontalLayout_23);

        horizontalLayout_28 = new QHBoxLayout();
        horizontalLayout_28->setSpacing(6);
        horizontalLayout_28->setObjectName(QString::fromUtf8("horizontalLayout_28"));
        pushButton_layerThickCompandDraw = new QPushButton(tab_5);
        pushButton_layerThickCompandDraw->setObjectName(QString::fromUtf8("pushButton_layerThickCompandDraw"));

        horizontalLayout_28->addWidget(pushButton_layerThickCompandDraw);


        verticalLayout_7->addLayout(horizontalLayout_28);

        horizontalLayout_32 = new QHBoxLayout();
        horizontalLayout_32->setSpacing(6);
        horizontalLayout_32->setObjectName(QString::fromUtf8("horizontalLayout_32"));
        pushButton_checkCollision = new QPushButton(tab_5);
        pushButton_checkCollision->setObjectName(QString::fromUtf8("pushButton_checkCollision"));

        horizontalLayout_32->addWidget(pushButton_checkCollision);

        checkBox_CollisionUpdateLayer = new QCheckBox(tab_5);
        checkBox_CollisionUpdateLayer->setObjectName(QString::fromUtf8("checkBox_CollisionUpdateLayer"));

        horizontalLayout_32->addWidget(checkBox_CollisionUpdateLayer);


        verticalLayout_7->addLayout(horizontalLayout_32);

        label_23 = new QLabel(tab_5);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        verticalLayout_7->addWidget(label_23);

        horizontalLayout_26 = new QHBoxLayout();
        horizontalLayout_26->setSpacing(6);
        horizontalLayout_26->setObjectName(QString::fromUtf8("horizontalLayout_26"));
        pushButton_InputIsoLayerwithSupport = new QPushButton(tab_5);
        pushButton_InputIsoLayerwithSupport->setObjectName(QString::fromUtf8("pushButton_InputIsoLayerwithSupport"));

        horizontalLayout_26->addWidget(pushButton_InputIsoLayerwithSupport);

        pushButton_cutLayerbyConvexHull = new QPushButton(tab_5);
        pushButton_cutLayerbyConvexHull->setObjectName(QString::fromUtf8("pushButton_cutLayerbyConvexHull"));

        horizontalLayout_26->addWidget(pushButton_cutLayerbyConvexHull);

        pushButton_objtooff = new QPushButton(tab_5);
        pushButton_objtooff->setObjectName(QString::fromUtf8("pushButton_objtooff"));

        horizontalLayout_26->addWidget(pushButton_objtooff);


        verticalLayout_7->addLayout(horizontalLayout_26);

        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setSpacing(6);
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        pushButton_fabricationSupportTetMesh = new QPushButton(tab_5);
        pushButton_fabricationSupportTetMesh->setObjectName(QString::fromUtf8("pushButton_fabricationSupportTetMesh"));

        horizontalLayout_22->addWidget(pushButton_fabricationSupportTetMesh);

        pushButton_fabricationSupportSurface = new QPushButton(tab_5);
        pushButton_fabricationSupportSurface->setObjectName(QString::fromUtf8("pushButton_fabricationSupportSurface"));

        horizontalLayout_22->addWidget(pushButton_fabricationSupportSurface);


        verticalLayout_7->addLayout(horizontalLayout_22);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setSpacing(6);
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        pushButton_fabricationEffectSupportSurfaceDetection = new QPushButton(tab_5);
        pushButton_fabricationEffectSupportSurfaceDetection->setObjectName(QString::fromUtf8("pushButton_fabricationEffectSupportSurfaceDetection"));

        horizontalLayout_25->addWidget(pushButton_fabricationEffectSupportSurfaceDetection);

        doubleSpinBox_supportNodeBoxSize = new QDoubleSpinBox(tab_5);
        doubleSpinBox_supportNodeBoxSize->setObjectName(QString::fromUtf8("doubleSpinBox_supportNodeBoxSize"));
        doubleSpinBox_supportNodeBoxSize->setValue(1.500000000000000);

        horizontalLayout_25->addWidget(doubleSpinBox_supportNodeBoxSize);


        verticalLayout_7->addLayout(horizontalLayout_25);

        tabWidget->addTab(tab_5, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_6 = new QVBoxLayout(tab_2);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_3 = new QLabel(tab_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_11->addWidget(label_3);

        doubleSpinBox_toolpathOffsetValue = new QDoubleSpinBox(tab_2);
        doubleSpinBox_toolpathOffsetValue->setObjectName(QString::fromUtf8("doubleSpinBox_toolpathOffsetValue"));
        doubleSpinBox_toolpathOffsetValue->setValue(0.800000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_toolpathOffsetValue);

        label_2 = new QLabel(tab_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_11->addWidget(label_2);

        spinBox_bToolPath_Num = new QSpinBox(tab_2);
        spinBox_bToolPath_Num->setObjectName(QString::fromUtf8("spinBox_bToolPath_Num"));
        spinBox_bToolPath_Num->setValue(3);

        horizontalLayout_11->addWidget(spinBox_bToolPath_Num);


        verticalLayout_6->addLayout(horizontalLayout_11);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
        label_4 = new QLabel(tab_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_17->addWidget(label_4);

        doubleSpinBox_maxGapDistance = new QDoubleSpinBox(tab_2);
        doubleSpinBox_maxGapDistance->setObjectName(QString::fromUtf8("doubleSpinBox_maxGapDistance"));
        doubleSpinBox_maxGapDistance->setValue(4.000000000000000);

        horizontalLayout_17->addWidget(doubleSpinBox_maxGapDistance);


        verticalLayout_6->addLayout(horizontalLayout_17);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        label_19 = new QLabel(tab_2);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        horizontalLayout_18->addWidget(label_19);

        doubleSpinBox_zigzagBoundConnectDistance = new QDoubleSpinBox(tab_2);
        doubleSpinBox_zigzagBoundConnectDistance->setObjectName(QString::fromUtf8("doubleSpinBox_zigzagBoundConnectDistance"));
        doubleSpinBox_zigzagBoundConnectDistance->setValue(1.700000000000000);

        horizontalLayout_18->addWidget(doubleSpinBox_zigzagBoundConnectDistance);


        verticalLayout_6->addLayout(horizontalLayout_18);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        label_12 = new QLabel(tab_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_19->addWidget(label_12);

        doubleSpinBox_boundShrinkValue = new QDoubleSpinBox(tab_2);
        doubleSpinBox_boundShrinkValue->setObjectName(QString::fromUtf8("doubleSpinBox_boundShrinkValue"));
        doubleSpinBox_boundShrinkValue->setValue(0.100000000000000);

        horizontalLayout_19->addWidget(doubleSpinBox_boundShrinkValue);


        verticalLayout_6->addLayout(horizontalLayout_19);

        label_22 = new QLabel(tab_2);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        verticalLayout_6->addWidget(label_22);

        pushButton_runHeatMethod = new QPushButton(tab_2);
        pushButton_runHeatMethod->setObjectName(QString::fromUtf8("pushButton_runHeatMethod"));

        verticalLayout_6->addWidget(pushButton_runHeatMethod);

        horizontalLayout_29 = new QHBoxLayout();
        horizontalLayout_29->setSpacing(6);
        horizontalLayout_29->setObjectName(QString::fromUtf8("horizontalLayout_29"));
        pushButton_generateZigZagToolPath = new QPushButton(tab_2);
        pushButton_generateZigZagToolPath->setObjectName(QString::fromUtf8("pushButton_generateZigZagToolPath"));

        horizontalLayout_29->addWidget(pushButton_generateZigZagToolPath);

        pushButton_generateStressFieldToolPath = new QPushButton(tab_2);
        pushButton_generateStressFieldToolPath->setObjectName(QString::fromUtf8("pushButton_generateStressFieldToolPath"));

        horizontalLayout_29->addWidget(pushButton_generateStressFieldToolPath);


        verticalLayout_6->addLayout(horizontalLayout_29);

        horizontalLayout_30 = new QHBoxLayout();
        horizontalLayout_30->setSpacing(6);
        horizontalLayout_30->setObjectName(QString::fromUtf8("horizontalLayout_30"));
        pushButton_outputToolPath = new QPushButton(tab_2);
        pushButton_outputToolPath->setObjectName(QString::fromUtf8("pushButton_outputToolPath"));

        horizontalLayout_30->addWidget(pushButton_outputToolPath);

        pushButton_directToolPathGeneration = new QPushButton(tab_2);
        pushButton_directToolPathGeneration->setObjectName(QString::fromUtf8("pushButton_directToolPathGeneration"));

        horizontalLayout_30->addWidget(pushButton_directToolPathGeneration);


        verticalLayout_6->addLayout(horizontalLayout_30);

        label_21 = new QLabel(tab_2);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        verticalLayout_6->addWidget(label_21);

        pushButton_toolpathGenerationChecking = new QPushButton(tab_2);
        pushButton_toolpathGenerationChecking->setObjectName(QString::fromUtf8("pushButton_toolpathGenerationChecking"));

        verticalLayout_6->addWidget(pushButton_toolpathGenerationChecking);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        pushButton_allLayerToolpathGenerate = new QPushButton(tab_2);
        pushButton_allLayerToolpathGenerate->setObjectName(QString::fromUtf8("pushButton_allLayerToolpathGenerate"));

        horizontalLayout_14->addWidget(pushButton_allLayerToolpathGenerate);

        checkBox_cpuSpeedUpToolpathGeneration = new QCheckBox(tab_2);
        checkBox_cpuSpeedUpToolpathGeneration->setObjectName(QString::fromUtf8("checkBox_cpuSpeedUpToolpathGeneration"));
        checkBox_cpuSpeedUpToolpathGeneration->setChecked(true);

        horizontalLayout_14->addWidget(checkBox_cpuSpeedUpToolpathGeneration);


        verticalLayout_6->addLayout(horizontalLayout_14);

        tabWidget->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_4 = new QVBoxLayout(tab);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        pushButton_deselectAllinModelTree = new QPushButton(tab);
        pushButton_deselectAllinModelTree->setObjectName(QString::fromUtf8("pushButton_deselectAllinModelTree"));

        verticalLayout_4->addWidget(pushButton_deselectAllinModelTree);

        horizontalLayout_24 = new QHBoxLayout();
        horizontalLayout_24->setSpacing(6);
        horizontalLayout_24->setObjectName(QString::fromUtf8("horizontalLayout_24"));
        pushButton_scaleModelSize = new QPushButton(tab);
        pushButton_scaleModelSize->setObjectName(QString::fromUtf8("pushButton_scaleModelSize"));

        horizontalLayout_24->addWidget(pushButton_scaleModelSize);

        doubleSpinBox_modelScaleFactor = new QDoubleSpinBox(tab);
        doubleSpinBox_modelScaleFactor->setObjectName(QString::fromUtf8("doubleSpinBox_modelScaleFactor"));
        doubleSpinBox_modelScaleFactor->setValue(0.750000000000000);

        horizontalLayout_24->addWidget(doubleSpinBox_modelScaleFactor);


        verticalLayout_4->addLayout(horizontalLayout_24);

        pushButton_heatMethodTest = new QPushButton(tab);
        pushButton_heatMethodTest->setObjectName(QString::fromUtf8("pushButton_heatMethodTest"));

        verticalLayout_4->addWidget(pushButton_heatMethodTest);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        maxValue = new QLabel(tab);
        maxValue->setObjectName(QString::fromUtf8("maxValue"));

        horizontalLayout_3->addWidget(maxValue);

        maximumValue = new QSlider(tab);
        maximumValue->setObjectName(QString::fromUtf8("maximumValue"));
        maximumValue->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(maximumValue);


        verticalLayout_4->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        minValue = new QLabel(tab);
        minValue->setObjectName(QString::fromUtf8("minValue"));
        minValue->setLayoutDirection(Qt::LeftToRight);

        horizontalLayout_2->addWidget(minValue);

        minimumValue = new QSlider(tab);
        minimumValue->setObjectName(QString::fromUtf8("minimumValue"));
        minimumValue->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(minimumValue);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_14 = new QLabel(tab);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout_10->addWidget(label_14);

        LaplaceWeight = new QLineEdit(tab);
        LaplaceWeight->setObjectName(QString::fromUtf8("LaplaceWeight"));

        horizontalLayout_10->addWidget(LaplaceWeight);


        verticalLayout_4->addLayout(horizontalLayout_10);

        pushButton_tetraDelete = new QPushButton(tab);
        pushButton_tetraDelete->setObjectName(QString::fromUtf8("pushButton_tetraDelete"));

        verticalLayout_4->addWidget(pushButton_tetraDelete);

        tabWidget->addTab(tab, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        verticalLayout_5 = new QVBoxLayout(tab_4);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        pushButton_volumetoSurface = new QPushButton(tab_4);
        pushButton_volumetoSurface->setObjectName(QString::fromUtf8("pushButton_volumetoSurface"));

        verticalLayout_5->addWidget(pushButton_volumetoSurface);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        label_9 = new QLabel(tab_4);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_20->addWidget(label_9);

        doubleSpinBox_voxelWidth = new QDoubleSpinBox(tab_4);
        doubleSpinBox_voxelWidth->setObjectName(QString::fromUtf8("doubleSpinBox_voxelWidth"));
        doubleSpinBox_voxelWidth->setSingleStep(0.100000000000000);
        doubleSpinBox_voxelWidth->setValue(1.000000000000000);

        horizontalLayout_20->addWidget(doubleSpinBox_voxelWidth);

        label_10 = new QLabel(tab_4);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_20->addWidget(label_10);

        spinBox_expandingLayer = new QSpinBox(tab_4);
        spinBox_expandingLayer->setObjectName(QString::fromUtf8("spinBox_expandingLayer"));
        spinBox_expandingLayer->setMinimum(-1);
        spinBox_expandingLayer->setMaximum(10);
        spinBox_expandingLayer->setValue(4);

        horizontalLayout_20->addWidget(spinBox_expandingLayer);


        verticalLayout_5->addLayout(horizontalLayout_20);

        pushButton_GenerateVoxel = new QPushButton(tab_4);
        pushButton_GenerateVoxel->setObjectName(QString::fromUtf8("pushButton_GenerateVoxel"));

        verticalLayout_5->addWidget(pushButton_GenerateVoxel);

        label_8 = new QLabel(tab_4);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_5->addWidget(label_8);

        label_15 = new QLabel(tab_4);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        verticalLayout_5->addWidget(label_15);

        label_16 = new QLabel(tab_4);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        verticalLayout_5->addWidget(label_16);

        label_17 = new QLabel(tab_4);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        verticalLayout_5->addWidget(label_17);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        pushButton_voxelLayerGeneration = new QPushButton(tab_4);
        pushButton_voxelLayerGeneration->setObjectName(QString::fromUtf8("pushButton_voxelLayerGeneration"));
        pushButton_voxelLayerGeneration->setEnabled(false);

        horizontalLayout_8->addWidget(pushButton_voxelLayerGeneration);

        spinBox_VoxelGenerateMethod = new QSpinBox(tab_4);
        spinBox_VoxelGenerateMethod->setObjectName(QString::fromUtf8("spinBox_VoxelGenerateMethod"));
        spinBox_VoxelGenerateMethod->setMinimum(1);
        spinBox_VoxelGenerateMethod->setMaximum(6);
        spinBox_VoxelGenerateMethod->setValue(2);

        horizontalLayout_8->addWidget(spinBox_VoxelGenerateMethod);

        checkBox_loadPlatform = new QCheckBox(tab_4);
        checkBox_loadPlatform->setObjectName(QString::fromUtf8("checkBox_loadPlatform"));
        checkBox_loadPlatform->setChecked(true);

        horizontalLayout_8->addWidget(checkBox_loadPlatform);


        verticalLayout_5->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_18 = new QLabel(tab_4);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        horizontalLayout_9->addWidget(label_18);

        spinBox_voxelLayer = new QSpinBox(tab_4);
        spinBox_voxelLayer->setObjectName(QString::fromUtf8("spinBox_voxelLayer"));
        spinBox_voxelLayer->setEnabled(true);
        spinBox_voxelLayer->setAccelerated(false);
        spinBox_voxelLayer->setProperty("showGroupSeparator", QVariant(false));
        spinBox_voxelLayer->setMinimum(-1);
        spinBox_voxelLayer->setMaximum(300);
        spinBox_voxelLayer->setValue(0);
        spinBox_voxelLayer->setDisplayIntegerBase(10);

        horizontalLayout_9->addWidget(spinBox_voxelLayer);

        pushButton_saveVoxelOrder = new QPushButton(tab_4);
        pushButton_saveVoxelOrder->setObjectName(QString::fromUtf8("pushButton_saveVoxelOrder"));

        horizontalLayout_9->addWidget(pushButton_saveVoxelOrder);

        pushButton_loadVoxelOrder = new QPushButton(tab_4);
        pushButton_loadVoxelOrder->setObjectName(QString::fromUtf8("pushButton_loadVoxelOrder"));

        horizontalLayout_9->addWidget(pushButton_loadVoxelOrder);


        verticalLayout_5->addLayout(horizontalLayout_9);

        label_6 = new QLabel(tab_4);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setMinimumSize(QSize(0, 10));
        label_6->setMaximumSize(QSize(16777215, 10));

        verticalLayout_5->addWidget(label_6);

        pushButton_isoFacefromVoxelLayer = new QPushButton(tab_4);
        pushButton_isoFacefromVoxelLayer->setObjectName(QString::fromUtf8("pushButton_isoFacefromVoxelLayer"));

        verticalLayout_5->addWidget(pushButton_isoFacefromVoxelLayer);

        pushButton_translateTetrahedraltoVoxel = new QPushButton(tab_4);
        pushButton_translateTetrahedraltoVoxel->setObjectName(QString::fromUtf8("pushButton_translateTetrahedraltoVoxel"));

        verticalLayout_5->addWidget(pushButton_translateTetrahedraltoVoxel);

        tabWidget->addTab(tab_4, QString());

        verticalLayout_2->addWidget(tabWidget);

        frame_modelTree = new QFrame(dockWidgetContents);
        frame_modelTree->setObjectName(QString::fromUtf8("frame_modelTree"));
        frame_modelTree->setMaximumSize(QSize(16777215, 400));
        frame_modelTree->setFrameShape(QFrame::StyledPanel);
        frame_modelTree->setFrameShadow(QFrame::Raised);
        verticalLayout = new QVBoxLayout(frame_modelTree);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(frame_modelTree);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        treeView = new QTreeView(frame_modelTree);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setMaximumSize(QSize(16777215, 150));
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        verticalLayout->addWidget(treeView);

        label_5 = new QLabel(frame_modelTree);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout->addWidget(label_5);

        SystemDialog = new QTextEdit(frame_modelTree);
        SystemDialog->setObjectName(QString::fromUtf8("SystemDialog"));
        SystemDialog->setMaximumSize(QSize(16777215, 200));
        SystemDialog->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        SystemDialog->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        SystemDialog->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        SystemDialog->setReadOnly(true);

        verticalLayout->addWidget(SystemDialog);


        verticalLayout_2->addWidget(frame_modelTree);

        pushButton_clearAll = new QPushButton(dockWidgetContents);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));

        verticalLayout_2->addWidget(pushButton_clearAll);

        pushButton_closewindow = new QPushButton(dockWidgetContents);
        pushButton_closewindow->setObjectName(QString::fromUtf8("pushButton_closewindow"));

        verticalLayout_2->addWidget(pushButton_closewindow);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(1), dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1450, 21));
        menuBar->setLayoutDirection(Qt::LeftToRight);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuFace_Node_shade = new QMenu(menuView);
        menuFace_Node_shade->setObjectName(QString::fromUtf8("menuFace_Node_shade"));
        menuSelect = new QMenu(menuBar);
        menuSelect->setObjectName(QString::fromUtf8("menuSelect"));
        menuFunction = new QMenu(menuBar);
        menuFunction->setObjectName(QString::fromUtf8("menuFunction"));
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
        menuBar->addAction(menuFunction->menuAction());
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
        menuView->addAction(actionMesh);
        menuView->addAction(actionNode);
        menuView->addAction(actionProfile);
        menuView->addAction(actionShade);
        menuView->addAction(menuFace_Node_shade->menuAction());
        menuView->addSeparator();
        menuView->addAction(actionShifttoOrigin);
        menuFace_Node_shade->addAction(actionFaceNormalShade);
        menuFace_Node_shade->addAction(actionNodeNormalShade);
        menuSelect->addAction(actionSelectNode);
        menuSelect->addAction(actionSelectEdge);
        menuSelect->addAction(actionSelectFace);
        menuSelect->addSeparator();
        menuSelect->addAction(actionSelectFix);
        menuSelect->addAction(actionSelectHandle);
        menuSelect->addSeparator();
        menuFunction->addAction(actionExport_to_Abaqus_model);
        menuFunction->addAction(actionInputFEM);
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);
        toolBar->addAction(actionExport_to_Abaqus_model);
        toolBar->addAction(actionInputFEM);

        retranslateUi(MainWindow);
        QObject::connect(pushButton_closewindow, SIGNAL(clicked()), MainWindow, SLOT(close()));
        QObject::connect(SystemDialog, SIGNAL(textChanged()), MainWindow, SLOT(autoScroll()));

        tabWidget->setCurrentIndex(3);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MeshWorksQT - Stress field Printing", nullptr));
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
        actionSelectHandle->setText(QApplication::translate("MainWindow", "Handle / Rigid", nullptr));
        actionSelectHandle->setIconText(QApplication::translate("MainWindow", "Handle /  Rigid", nullptr));
        actionSaveSelection->setText(QApplication::translate("MainWindow", "Save selection", nullptr));
        actionReadSelection->setText(QApplication::translate("MainWindow", "Read selection", nullptr));
        actionSelectChamber->setText(QApplication::translate("MainWindow", "Select Chamber (SORO)", nullptr));
        actionExport_to_Abaqus_model->setText(QApplication::translate("MainWindow", "Export to Abaqus model", nullptr));
        actionShape_Up_Mesh_Deformation->setText(QApplication::translate("MainWindow", "Shape-Up Mesh Deformation", nullptr));
        actionInputFEM->setText(QApplication::translate("MainWindow", "Input FEM analysis result", nullptr));
        actionFaceNormalShade->setText(QApplication::translate("MainWindow", "Face normal", nullptr));
        actionNodeNormalShade->setText(QApplication::translate("MainWindow", "Node normal", nullptr));
        actionGeoField_boundary->setText(QApplication::translate("MainWindow", "Boundary distance Field", nullptr));
        actionGeoField_directional->setText(QApplication::translate("MainWindow", "Directional distance Field", nullptr));
        actionGeoField_selection->setText(QApplication::translate("MainWindow", "By selection", nullptr));
        action_GenerateTrajectory->setText(QApplication::translate("MainWindow", "Generate Trajectory", nullptr));
        navigationToolBar->setWindowTitle(QApplication::translate("MainWindow", "navigationToolBar", nullptr));
        selectionToolBar->setWindowTitle(QApplication::translate("MainWindow", "selectionToolBar", nullptr));
        boxDeselect->setText(QApplication::translate("MainWindow", "Deselect Mode", nullptr));
        checkBox_spaseVectorField->setText(QApplication::translate("MainWindow", "Sparse vector field", nullptr));
        checkBox_3DCompute->setText(QApplication::translate("MainWindow", "3D Compute", nullptr));
        pushButton_compPrincipleStressField->setText(QApplication::translate("MainWindow", "Input Stress Tensor (FEM) and Compute Principle Stress", nullptr));
        Tensile->setText(QApplication::translate("MainWindow", "Tensile", nullptr));
        tensileRegionRatio->setText(QApplication::translate("MainWindow", "0.1", nullptr));
        Compress->setText(QApplication::translate("MainWindow", "Compress", nullptr));
        compressRegionRatio->setText(QApplication::translate("MainWindow", "0.1", nullptr));
        pushButton_changeTensileandCompressRegion->setText(QApplication::translate("MainWindow", "Change ratio", nullptr));
        pushButton_CompGuideField->setText(QApplication::translate("MainWindow", "Field Computing", nullptr));
        checkBox_drawScalarGradient->setText(QApplication::translate("MainWindow", "Vector / Scalar", nullptr));
        pushButton_CompVectorField->setText(QApplication::translate("MainWindow", "Vector Feild", nullptr));
        pushButton_VectorFieldFlipNormal->setText(QApplication::translate("MainWindow", "Flip normal", nullptr));
        pushButton_VectorFieldDeleteRegion->setText(QApplication::translate("MainWindow", "Delete region", nullptr));
        pushButton_CompScalarField->setText(QApplication::translate("MainWindow", "Scalar Field", nullptr));
        label_20->setText(QApplication::translate("MainWindow", "----------------------------------------------------------------------------", nullptr));
        pushButtonBuildIsoSurface->setText(QApplication::translate("MainWindow", "Curved Layer Slicer", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "Layer #", nullptr));
        checkBox_UniformThickness->setText(QApplication::translate("MainWindow", "Layer (min, max)", nullptr));
        pushButton_compFieldonIsoSurface->setText(QApplication::translate("MainWindow", "Compute Field on Iso-surface", nullptr));
        pushButton_changeOrder->setText(QApplication::translate("MainWindow", "Flip order", nullptr));
        LayerIndexLabel->setText(QApplication::translate("MainWindow", "Draw Layer Below:", nullptr));
        VisualSingleLayerButtom->setText(QApplication::translate("MainWindow", "Draw Single", nullptr));
        pushButton_viewallLayerandOffset->setText(QApplication::translate("MainWindow", "Draw All", nullptr));
        pushButton_outputIsoLayer->setText(QApplication::translate("MainWindow", "Layer Below:", nullptr));
        checkBox_outputSingleLayer->setText(QApplication::translate("MainWindow", "Siglie IO", nullptr));
        checkBox_outputILayer_splitMode->setText(QApplication::translate("MainWindow", "Split", nullptr));
        checkBox_outputILayer_OFFMode->setText(QApplication::translate("MainWindow", ".off", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "IsoSurface", nullptr));
        pushButton_fabricationDirection->setText(QApplication::translate("MainWindow", "Fabrication Direction", nullptr));
        checkBox_3DrotateInverse->setText(QApplication::translate("MainWindow", "Inverse (3d)", nullptr));
        checkBox_computeFabricationDirection->setText(QApplication::translate("MainWindow", "Computed (2D/3D (a,b))", nullptr));
        pushButton_supportingStructureDetection->setText(QApplication::translate("MainWindow", "Support Region Detection", nullptr));
        pushButton_buildSupportNode->setText(QApplication::translate("MainWindow", "Build support node", nullptr));
        checkBox_saveSupportNode->setText(QApplication::translate("MainWindow", "Save", nullptr));
        pushButton_selectSupportRegion->setText(QApplication::translate("MainWindow", "Interactive S-region", nullptr));
        checkBox_deselectSupportNode->setText(QApplication::translate("MainWindow", "deselect SNode", nullptr));
        checkBox_deselectSupportNode_byNode->setText(QApplication::translate("MainWindow", "Init/Support", nullptr));
        pushButton_layerThickCompandDraw->setText(QApplication::translate("MainWindow", "Compute Layer Thickness and Draw", nullptr));
        pushButton_checkCollision->setText(QApplication::translate("MainWindow", "Check Collision", nullptr));
        checkBox_CollisionUpdateLayer->setText(QApplication::translate("MainWindow", "Rebuild Layer", nullptr));
        label_23->setText(QApplication::translate("MainWindow", "--------------- Input support structure support mesh---------------", nullptr));
        pushButton_InputIsoLayerwithSupport->setText(QApplication::translate("MainWindow", "input layer (S)", nullptr));
        pushButton_cutLayerbyConvexHull->setText(QApplication::translate("MainWindow", "split mesh (C-HULL)", nullptr));
        pushButton_objtooff->setText(QApplication::translate("MainWindow", "obj->off", nullptr));
        pushButton_fabricationSupportTetMesh->setText(QApplication::translate("MainWindow", "Support mesh generation", nullptr));
        pushButton_fabricationSupportSurface->setText(QApplication::translate("MainWindow", "Compute support surface", nullptr));
        pushButton_fabricationEffectSupportSurfaceDetection->setText(QApplication::translate("MainWindow", "Delete non-important support region (iso-surface)", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("MainWindow", "Fabrication", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Offset (mm)", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Boundary Circle Num:", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Max ZigZag gap distance (mm)", nullptr));
        label_19->setText(QApplication::translate("MainWindow", "Zigzag-bound connect dis (layer gap #)", nullptr));
        label_12->setText(QApplication::translate("MainWindow", "Boundary ToolPath shrink distance (mm)", nullptr));
        label_22->setText(QApplication::translate("MainWindow", "---------------------------------Parameter-------------------------------", nullptr));
        pushButton_runHeatMethod->setText(QApplication::translate("MainWindow", "Run Heat Method (Boundary + Zigzag)", nullptr));
        pushButton_generateZigZagToolPath->setText(QApplication::translate("MainWindow", "Heat Method tool-path", nullptr));
        pushButton_generateStressFieldToolPath->setText(QApplication::translate("MainWindow", "Stress-Field tool-path", nullptr));
        pushButton_outputToolPath->setText(QApplication::translate("MainWindow", "Output ToolPath", nullptr));
        pushButton_directToolPathGeneration->setText(QApplication::translate("MainWindow", "Multi-toolpath Generation (new)", nullptr));
        label_21->setText(QApplication::translate("MainWindow", "-----------Single \342\206\221------------------------------Multiple \342\206\223---------------", nullptr));
        pushButton_toolpathGenerationChecking->setText(QApplication::translate("MainWindow", "Mesh Checking for Toolpath generation (layer set)", nullptr));
        pushButton_allLayerToolpathGenerate->setText(QApplication::translate("MainWindow", "Generate toolpath for all layer and save", nullptr));
        checkBox_cpuSpeedUpToolpathGeneration->setText(QApplication::translate("MainWindow", "CPU SpeedUp", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Tool Path", nullptr));
        pushButton_deselectAllinModelTree->setText(QApplication::translate("MainWindow", "Deselect all in model tree", nullptr));
        pushButton_scaleModelSize->setText(QApplication::translate("MainWindow", "Scale Model", nullptr));
        pushButton_heatMethodTest->setText(QApplication::translate("MainWindow", "heatMethodBoundaryField", nullptr));
        maxValue->setText(QApplication::translate("MainWindow", "Max Value = 0", nullptr));
        minValue->setText(QApplication::translate("MainWindow", " Min Value = 0", nullptr));
        label_14->setText(QApplication::translate("MainWindow", "Laplace Weight", nullptr));
        LaplaceWeight->setText(QApplication::translate("MainWindow", "0.01", nullptr));
        pushButton_tetraDelete->setText(QApplication::translate("MainWindow", "Delete inner tetrahderal (hollowing)", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Visual", nullptr));
        pushButton_volumetoSurface->setText(QApplication::translate("MainWindow", "Transfer volume mesh to surface mesh", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Voxel Width", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "Voxel Exbanding", nullptr));
        pushButton_GenerateVoxel->setText(QApplication::translate("MainWindow", "Generation Voxel", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "M1: Convex Peeling", nullptr));
        label_15->setText(QApplication::translate("MainWindow", "M2: Peeling-Field Guide Growing (Recommand)", nullptr));
        label_16->setText(QApplication::translate("MainWindow", "M3: Convex-front Growing; M4: with Platform", nullptr));
        label_17->setText(QApplication::translate("MainWindow", "M5: Convex-front Growing + Shadow Prevention", nullptr));
        pushButton_voxelLayerGeneration->setText(QApplication::translate("MainWindow", "Order Generation by Method:", nullptr));
        checkBox_loadPlatform->setText(QApplication::translate("MainWindow", "Input Platform", nullptr));
        label_18->setText(QApplication::translate("MainWindow", "View Voxel Layer:", nullptr));
        pushButton_saveVoxelOrder->setText(QApplication::translate("MainWindow", "Save order", nullptr));
        pushButton_loadVoxelOrder->setText(QApplication::translate("MainWindow", "Load  order", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "-----------------------------------------------------------------------------", nullptr));
        pushButton_isoFacefromVoxelLayer->setText(QApplication::translate("MainWindow", " Voxel Order -> Tetrahedral Scalar Field", nullptr));
        pushButton_translateTetrahedraltoVoxel->setText(QApplication::translate("MainWindow", "Tetrahedral Scalar Field ->  Voxel Order", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "Voxel", nullptr));
        label->setText(QApplication::translate("MainWindow", "Model Tree", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "System Dialog", nullptr));
        pushButton_clearAll->setText(QApplication::translate("MainWindow", "Clear All", nullptr));
        pushButton_closewindow->setText(QApplication::translate("MainWindow", "Close program", nullptr));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QApplication::translate("MainWindow", "View", nullptr));
        menuFace_Node_shade->setTitle(QApplication::translate("MainWindow", "Face / Node shade", nullptr));
        menuSelect->setTitle(QApplication::translate("MainWindow", "Select", nullptr));
        menuFunction->setTitle(QApplication::translate("MainWindow", "Function", nullptr));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
