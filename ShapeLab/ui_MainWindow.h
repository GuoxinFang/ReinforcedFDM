/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.10
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
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
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
    QAction *actionExport2Abaqus;
    QAction *actionShape_Up_Mesh_Deformation;
    QAction *actionInputFEM;
    QAction *actionFaceNormalShade;
    QAction *actionNodeNormalShade;
    QAction *actionGeoField_boundary;
    QAction *actionGeoField_directional;
    QAction *actionGeoField_selection;
    QAction *action_GenerateTrajectory;
    QAction *actionScale_Model_Size;
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
    QCheckBox *checkBox_3DCompute;
    QTabWidget *tabWidget;
    QWidget *tab_3;
    QGridLayout *gridLayout_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_4;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_15;
    QPushButton *pushButton_CompGuideField;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QPushButton *pushButton_CompVectorField;
    QPushButton *pushButton_CompScalarField;
    QPushButton *pushButton_VectorFieldFlipNormal;
    QPushButton *pushButton_VectorFieldDeleteRegion;
    QHBoxLayout *horizontalLayout_33;
    QPushButton *pushButton_compFieldonIsoSurface;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *pushButtonBuildIsoSurface;
    QLabel *label_11;
    QSpinBox *isoLayerNumber;
    QPushButton *pushButton_compPrincipleStressField;
    QHBoxLayout *horizontalLayout_12;
    QLabel *Tensile;
    QLineEdit *tensileRegionRatio;
    QLabel *Compress;
    QLineEdit *compressRegionRatio;
    QPushButton *pushButton_changeTensileandCompressRegion;
    QPushButton *pushButton_directToolPathGeneration;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_outputIsoLayer;
    QSpinBox *outputLayerIndexNum;
    QCheckBox *checkBox_outputSingleLayer;
    QCheckBox *checkBox_outputILayer_splitMode;
    QCheckBox *checkBox_outputILayer_OFFMode;
    QWidget *tab;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_8;
    QCheckBox *checkBox_spaseVectorField;
    QHBoxLayout *horizontalLayout_5;
    QLabel *LayerIndexLabel;
    QSpinBox *IsoLayerIndex;
    QRadioButton *VisualSingleLayerButtom;
    QPushButton *pushButton_viewallLayerandOffset;
    QHBoxLayout *horizontalLayout_9;
    QPushButton *pushButton_deselectAllinModelTree;
    QPushButton *pushButton_heatMethodTest;
    QPushButton *pushButton_changeOrder;
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
        MainWindow->resize(1511, 887);
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
        actionExport2Abaqus = new QAction(MainWindow);
        actionExport2Abaqus->setObjectName(QString::fromUtf8("actionExport2Abaqus"));
        actionExport2Abaqus->setCheckable(false);
        QIcon icon27;
        icon27.addFile(QString::fromUtf8(":/resource/abaqus logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport2Abaqus->setIcon(icon27);
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
        actionScale_Model_Size = new QAction(MainWindow);
        actionScale_Model_Size->setObjectName(QString::fromUtf8("actionScale_Model_Size"));
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
        dockWidget->setMinimumSize(QSize(400, 800));
        dockWidget->setMaximumSize(QSize(400, 800));
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
        gridLayout_2 = new QGridLayout(tab_3);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        groupBox = new QGroupBox(tab_3);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(140, 80));
        groupBox->setMaximumSize(QSize(140, 80));
        horizontalLayout_15 = new QHBoxLayout(groupBox);
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        pushButton_CompGuideField = new QPushButton(groupBox);
        pushButton_CompGuideField->setObjectName(QString::fromUtf8("pushButton_CompGuideField"));
        pushButton_CompGuideField->setEnabled(false);
        pushButton_CompGuideField->setMinimumSize(QSize(0, 30));

        horizontalLayout_15->addWidget(pushButton_CompGuideField);


        horizontalLayout_4->addWidget(groupBox);

        groupBox_2 = new QGroupBox(tab_3);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setMinimumSize(QSize(0, 80));
        groupBox_2->setMaximumSize(QSize(16777215, 80));
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        pushButton_CompVectorField = new QPushButton(groupBox_2);
        pushButton_CompVectorField->setObjectName(QString::fromUtf8("pushButton_CompVectorField"));

        gridLayout->addWidget(pushButton_CompVectorField, 0, 0, 1, 1);

        pushButton_CompScalarField = new QPushButton(groupBox_2);
        pushButton_CompScalarField->setObjectName(QString::fromUtf8("pushButton_CompScalarField"));

        gridLayout->addWidget(pushButton_CompScalarField, 0, 1, 1, 1);

        pushButton_VectorFieldFlipNormal = new QPushButton(groupBox_2);
        pushButton_VectorFieldFlipNormal->setObjectName(QString::fromUtf8("pushButton_VectorFieldFlipNormal"));

        gridLayout->addWidget(pushButton_VectorFieldFlipNormal, 1, 0, 1, 1);

        pushButton_VectorFieldDeleteRegion = new QPushButton(groupBox_2);
        pushButton_VectorFieldDeleteRegion->setObjectName(QString::fromUtf8("pushButton_VectorFieldDeleteRegion"));

        gridLayout->addWidget(pushButton_VectorFieldDeleteRegion, 1, 1, 1, 1);


        horizontalLayout_4->addWidget(groupBox_2);


        verticalLayout_3->addLayout(horizontalLayout_4);


        gridLayout_2->addLayout(verticalLayout_3, 2, 0, 1, 1);

        horizontalLayout_33 = new QHBoxLayout();
        horizontalLayout_33->setSpacing(6);
        horizontalLayout_33->setObjectName(QString::fromUtf8("horizontalLayout_33"));
        pushButton_compFieldonIsoSurface = new QPushButton(tab_3);
        pushButton_compFieldonIsoSurface->setObjectName(QString::fromUtf8("pushButton_compFieldonIsoSurface"));

        horizontalLayout_33->addWidget(pushButton_compFieldonIsoSurface);


        gridLayout_2->addLayout(horizontalLayout_33, 5, 0, 1, 1);

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


        gridLayout_2->addLayout(horizontalLayout_6, 3, 0, 1, 1);

        pushButton_compPrincipleStressField = new QPushButton(tab_3);
        pushButton_compPrincipleStressField->setObjectName(QString::fromUtf8("pushButton_compPrincipleStressField"));

        gridLayout_2->addWidget(pushButton_compPrincipleStressField, 0, 0, 1, 1);

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


        gridLayout_2->addLayout(horizontalLayout_12, 1, 0, 1, 1);

        pushButton_directToolPathGeneration = new QPushButton(tab_3);
        pushButton_directToolPathGeneration->setObjectName(QString::fromUtf8("pushButton_directToolPathGeneration"));

        gridLayout_2->addWidget(pushButton_directToolPathGeneration, 8, 0, 1, 1);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        pushButton_outputIsoLayer = new QPushButton(tab_3);
        pushButton_outputIsoLayer->setObjectName(QString::fromUtf8("pushButton_outputIsoLayer"));

        horizontalLayout_7->addWidget(pushButton_outputIsoLayer);

        outputLayerIndexNum = new QSpinBox(tab_3);
        outputLayerIndexNum->setObjectName(QString::fromUtf8("outputLayerIndexNum"));
        outputLayerIndexNum->setMaximumSize(QSize(60, 16777215));
        outputLayerIndexNum->setAccelerated(true);
        outputLayerIndexNum->setMaximum(999);
        outputLayerIndexNum->setDisplayIntegerBase(10);

        horizontalLayout_7->addWidget(outputLayerIndexNum);

        checkBox_outputSingleLayer = new QCheckBox(tab_3);
        checkBox_outputSingleLayer->setObjectName(QString::fromUtf8("checkBox_outputSingleLayer"));
        checkBox_outputSingleLayer->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_7->addWidget(checkBox_outputSingleLayer);

        checkBox_outputILayer_splitMode = new QCheckBox(tab_3);
        checkBox_outputILayer_splitMode->setObjectName(QString::fromUtf8("checkBox_outputILayer_splitMode"));
        checkBox_outputILayer_splitMode->setMaximumSize(QSize(40, 16777215));
        checkBox_outputILayer_splitMode->setChecked(true);

        horizontalLayout_7->addWidget(checkBox_outputILayer_splitMode);

        checkBox_outputILayer_OFFMode = new QCheckBox(tab_3);
        checkBox_outputILayer_OFFMode->setObjectName(QString::fromUtf8("checkBox_outputILayer_OFFMode"));
        checkBox_outputILayer_OFFMode->setMaximumSize(QSize(40, 16777215));

        horizontalLayout_7->addWidget(checkBox_outputILayer_OFFMode);


        gridLayout_2->addLayout(horizontalLayout_7, 7, 0, 1, 1);

        tabWidget->addTab(tab_3, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_4 = new QVBoxLayout(tab);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        checkBox_spaseVectorField = new QCheckBox(tab);
        checkBox_spaseVectorField->setObjectName(QString::fromUtf8("checkBox_spaseVectorField"));
        checkBox_spaseVectorField->setChecked(false);

        horizontalLayout_8->addWidget(checkBox_spaseVectorField);


        verticalLayout_4->addLayout(horizontalLayout_8);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        LayerIndexLabel = new QLabel(tab);
        LayerIndexLabel->setObjectName(QString::fromUtf8("LayerIndexLabel"));

        horizontalLayout_5->addWidget(LayerIndexLabel);

        IsoLayerIndex = new QSpinBox(tab);
        IsoLayerIndex->setObjectName(QString::fromUtf8("IsoLayerIndex"));
        IsoLayerIndex->setMaximum(300);

        horizontalLayout_5->addWidget(IsoLayerIndex);

        VisualSingleLayerButtom = new QRadioButton(tab);
        VisualSingleLayerButtom->setObjectName(QString::fromUtf8("VisualSingleLayerButtom"));

        horizontalLayout_5->addWidget(VisualSingleLayerButtom);

        pushButton_viewallLayerandOffset = new QPushButton(tab);
        pushButton_viewallLayerandOffset->setObjectName(QString::fromUtf8("pushButton_viewallLayerandOffset"));

        horizontalLayout_5->addWidget(pushButton_viewallLayerandOffset);


        verticalLayout_4->addLayout(horizontalLayout_5);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        pushButton_deselectAllinModelTree = new QPushButton(tab);
        pushButton_deselectAllinModelTree->setObjectName(QString::fromUtf8("pushButton_deselectAllinModelTree"));

        horizontalLayout_9->addWidget(pushButton_deselectAllinModelTree);

        pushButton_heatMethodTest = new QPushButton(tab);
        pushButton_heatMethodTest->setObjectName(QString::fromUtf8("pushButton_heatMethodTest"));

        horizontalLayout_9->addWidget(pushButton_heatMethodTest);


        verticalLayout_4->addLayout(horizontalLayout_9);

        pushButton_changeOrder = new QPushButton(tab);
        pushButton_changeOrder->setObjectName(QString::fromUtf8("pushButton_changeOrder"));

        verticalLayout_4->addWidget(pushButton_changeOrder);

        tabWidget->addTab(tab, QString());

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
        SystemDialog->setMaximumSize(QSize(16777215, 150));
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
        menuBar->setGeometry(QRect(0, 0, 1511, 18));
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
        menuFunction->addAction(actionExport2Abaqus);
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);

        retranslateUi(MainWindow);
        QObject::connect(pushButton_closewindow, SIGNAL(clicked()), MainWindow, SLOT(close()));
        QObject::connect(SystemDialog, SIGNAL(textChanged()), MainWindow, SLOT(autoScroll()));

        tabWidget->setCurrentIndex(0);


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
        actionExport2Abaqus->setText(QApplication::translate("MainWindow", "Export tetrahedral mesh to Abaqus INP file", nullptr));
        actionShape_Up_Mesh_Deformation->setText(QApplication::translate("MainWindow", "Shape-Up Mesh Deformation", nullptr));
        actionInputFEM->setText(QApplication::translate("MainWindow", "Input FEM analysis result", nullptr));
        actionFaceNormalShade->setText(QApplication::translate("MainWindow", "Face normal", nullptr));
        actionNodeNormalShade->setText(QApplication::translate("MainWindow", "Node normal", nullptr));
        actionGeoField_boundary->setText(QApplication::translate("MainWindow", "Boundary distance Field", nullptr));
        actionGeoField_directional->setText(QApplication::translate("MainWindow", "Directional distance Field", nullptr));
        actionGeoField_selection->setText(QApplication::translate("MainWindow", "By selection", nullptr));
        action_GenerateTrajectory->setText(QApplication::translate("MainWindow", "Generate Trajectory", nullptr));
        actionScale_Model_Size->setText(QApplication::translate("MainWindow", "Scale Model Size", nullptr));
        navigationToolBar->setWindowTitle(QApplication::translate("MainWindow", "navigationToolBar", nullptr));
        selectionToolBar->setWindowTitle(QApplication::translate("MainWindow", "selectionToolBar", nullptr));
        boxDeselect->setText(QApplication::translate("MainWindow", "Deselect Mode", nullptr));
        checkBox_3DCompute->setText(QApplication::translate("MainWindow", "3D Compute", nullptr));
        groupBox->setTitle(QApplication::translate("MainWindow", "Step 2: Field Computing\357\274\232", nullptr));
        pushButton_CompGuideField->setText(QApplication::translate("MainWindow", "Guidance Field Computing", nullptr));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Generate Field by step and manual edit", nullptr));
        pushButton_CompVectorField->setText(QApplication::translate("MainWindow", "Vector Feild", nullptr));
        pushButton_CompScalarField->setText(QApplication::translate("MainWindow", "Scalar Field", nullptr));
        pushButton_VectorFieldFlipNormal->setText(QApplication::translate("MainWindow", "Flip normal", nullptr));
        pushButton_VectorFieldDeleteRegion->setText(QApplication::translate("MainWindow", "Delete region", nullptr));
        pushButton_compFieldonIsoSurface->setText(QApplication::translate("MainWindow", "Step 4: Compute Field on Iso-surface", nullptr));
        pushButtonBuildIsoSurface->setText(QApplication::translate("MainWindow", "Step 3: Curved Layer Slicer", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "Layer #", nullptr));
        pushButton_compPrincipleStressField->setText(QApplication::translate("MainWindow", "Step 1: Input FEA Result", nullptr));
        Tensile->setText(QApplication::translate("MainWindow", "Tensile", nullptr));
        tensileRegionRatio->setText(QApplication::translate("MainWindow", "0.1", nullptr));
        Compress->setText(QApplication::translate("MainWindow", "Compress", nullptr));
        compressRegionRatio->setText(QApplication::translate("MainWindow", "0.1", nullptr));
        pushButton_changeTensileandCompressRegion->setText(QApplication::translate("MainWindow", "Change ratio", nullptr));
        pushButton_directToolPathGeneration->setText(QApplication::translate("MainWindow", "Step 6: Multi-toolpath Generation (new)", nullptr));
        pushButton_outputIsoLayer->setText(QApplication::translate("MainWindow", "Step 5: Output Layer Below:", nullptr));
        checkBox_outputSingleLayer->setText(QApplication::translate("MainWindow", "Siglie IO", nullptr));
        checkBox_outputILayer_splitMode->setText(QApplication::translate("MainWindow", "Split", nullptr));
        checkBox_outputILayer_OFFMode->setText(QApplication::translate("MainWindow", ".off", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "Field computing, slcing and toolpath generation", nullptr));
        checkBox_spaseVectorField->setText(QApplication::translate("MainWindow", "Draw sparse vector field", nullptr));
        LayerIndexLabel->setText(QApplication::translate("MainWindow", "Draw Layer Below:", nullptr));
        VisualSingleLayerButtom->setText(QApplication::translate("MainWindow", "Draw Single", nullptr));
        pushButton_viewallLayerandOffset->setText(QApplication::translate("MainWindow", "Draw All", nullptr));
        pushButton_deselectAllinModelTree->setText(QApplication::translate("MainWindow", "Deselect all in model tree", nullptr));
        pushButton_heatMethodTest->setText(QApplication::translate("MainWindow", "Draw Heat Method BoundaryField", nullptr));
        pushButton_changeOrder->setText(QApplication::translate("MainWindow", "Flip layer order", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Visual", nullptr));
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
