#-------------------------------------------------
#
# Project created by QtCreator 2016-07-19T14:47:30
#
#-------------------------------------------------

QT       += opengl

TARGET = QMeshLib
TEMPLATE = lib
CONFIG += staticlib

SOURCES += \
    QMeshEdge.cpp \
    QMeshFace.cpp \
    QMeshNode.cpp \
    QMeshPatch.cpp \
    QMeshTetra.cpp \
    QMeshCluster.cpp \
    PolygenMesh.cpp \
    BSPTree.cpp \
    BSPTreeOperation.cpp\
    BSTree.cpp \
    PMBody.cpp \
    QMeshVoxel.cpp \
    QMeshVoxelOperation.cpp

HEADERS += \
    QMeshEdge.h \
    QMeshFace.h \
    QMeshNode.h \
    QMeshPatch.h \
    QMeshTetra.h \
    QMeshCluster.h \
    PolygenMesh.h \
    BSPTree.h \
    BSPTreeOperation.h\
    BSTree.h \
    PMBody.h \
    QMeshVoxel.h \
    QMeshVoxelOperation.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
