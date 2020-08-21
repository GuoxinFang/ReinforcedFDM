#-------------------------------------------------
#
# Project created by QtCreator 2016-07-19T18:18:17
#
#-------------------------------------------------

QT       += widgets opengl

TARGET = GLKLib
TEMPLATE = lib
CONFIG += staticlib

SOURCES += GLKLib.cpp \
    GLKGeometry.cpp \
    GLKGraph.cpp \
    GLKHeap.cpp \
    GLKMatrixLib.cpp \
    GLKNearestNeighbor.cpp \
    GLKObList.cpp \
    GLKRational.cpp \
    GLKSparseMatrix.cpp \
    GLKSparseMatrixDirectSolver.cpp \
    GLKSparseMatrixIterativeSolver.cpp \
    InteractiveTool.cpp

HEADERS += GLKLib.h \
    GLKGeometry.h \
    GLKGLList.h \
    GLKGraph.h \
    GLKHeap.h \
    GLKMatrixLib.h \
    GLKNearestNeighbor.h \
    GLKObList.h \
    GLKRational.h \
    GLKSparseMatrix.h \
    GLKSparseMatrixDirectSolver.h \
    GLKSparseMatrixIterativeSolver.h \
    GLKCameraTool.h \
    InteractiveTool.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../QMeshLib/release/ -lQMeshLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../QMeshLib/debug/ -lQMeshLib
else:unix: LIBS += -L$$OUT_PWD/../QMeshLib/ -lQMeshLib

INCLUDEPATH += $$PWD/../QMeshLib
DEPENDPATH += $$PWD/../QMeshLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/release/libQMeshLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/debug/libQMeshLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/release/QMeshLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/debug/QMeshLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/libQMeshLib.a
