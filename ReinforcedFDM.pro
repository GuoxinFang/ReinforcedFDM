TEMPLATE = subdirs

SUBDIRS += \
    QMeshLib \
    GLKLib \
    ShapeLab

LIBS += -lopengl32 \
    -lglu32 \
    -lglut \

QMAKE_CXXFLAGS += -openmp
