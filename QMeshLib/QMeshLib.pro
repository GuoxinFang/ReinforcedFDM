#-------------------------------------------------
#
# Project created by QtCreator 2016-07-19T14:47:30
#
#-------------------------------------------------

QT       += opengl

TARGET = QMeshLib
TEMPLATE = lib
CONFIG += staticlib

SOURCES += *.cpp\
	../Library/QHull/*.cpp \
	../Library/PQPLib/*.cpp \

HEADERS += *.h\
    ../Library/QHull/*.h \
	../Library/PQPLib/*.h \

unix {
    target.path = /usr/lib
    INSTALLS += target
}
