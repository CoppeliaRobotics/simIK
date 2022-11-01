TARGET = simExtIK
TEMPLATE = lib
DEFINES -= UNICODE
CONFIG += shared plugin
CONFIG -= core
CONFIG -= gui

DEFINES += SIM_MATH_DOUBLE

*-msvc* {
    QMAKE_CFLAGS += -O2
    QMAKE_CFLAGS += -fp:precise
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -fp:precise
    QMAKE_CFLAGS_WARN_ON = -W3
    QMAKE_CXXFLAGS_WARN_ON = -W3
}

*-g++* {
    QMAKE_CFLAGS += -O3
    QMAKE_CXXFLAGS += -O3
    QMAKE_CFLAGS_WARN_ON = -Wall
    QMAKE_CXXFLAGS_WARN_ON = -Wall
    QMAKE_CFLAGS += -Wno-float-equal
    QMAKE_CXXFLAGS += -Wno-float-equal
}

clang* {
    QMAKE_CFLAGS += -O3
    QMAKE_CXXFLAGS += -O3
    QMAKE_CFLAGS_WARN_ON = -Wall
    QMAKE_CXXFLAGS_WARN_ON = -Wall
    QMAKE_CFLAGS += -Wno-float-equal
    QMAKE_CXXFLAGS += -Wno-float-equal
}

win32 {
    DEFINES += WIN_SIM
}

macx {
    DEFINES += MAC_SIM
    INCLUDEPATH += "/usr/local/include"
}

unix:!macx {
    DEFINES += LIN_SIM
}

INCLUDEPATH += "../include"
INCLUDEPATH += "../common"
INCLUDEPATH += "../simMath"
INCLUDEPATH += "../coppeliaKinematicsRoutines"

HEADERS += simExtIK.h \
    envCont.h \
    ../include/simLib.h \
    ../include/scriptFunctionData.h \
    ../include/scriptFunctionDataItem.h \
    ../coppeliaKinematicsRoutines/ik.h \
    ../coppeliaKinematicsRoutines/environment.h \
    ../coppeliaKinematicsRoutines/serialization.h \
    ../coppeliaKinematicsRoutines/ikGroupContainer.h \
    ../coppeliaKinematicsRoutines/ikGroup.h \
    ../coppeliaKinematicsRoutines/ikElement.h \
    ../coppeliaKinematicsRoutines/objectContainer.h \
    ../coppeliaKinematicsRoutines/sceneObject.h \
    ../coppeliaKinematicsRoutines/dummy.h \
    ../coppeliaKinematicsRoutines/joint.h \
    ../simMath/mathDefines.h \
    ../simMath/MyMath.h \
    ../simMath/3Vector.h \
    ../simMath/4Vector.h \
    ../simMath/7Vector.h \
    ../simMath/3X3Matrix.h \
    ../simMath/4X4Matrix.h \
    ../simMath/MMatrix.h \

SOURCES += simExtIK.cpp \
    envCont.cpp \
    ../common/simLib.cpp \
    ../common/scriptFunctionData.cpp \
    ../common/scriptFunctionDataItem.cpp \
    ../coppeliaKinematicsRoutines/ik.cpp \
    ../coppeliaKinematicsRoutines/environment.cpp \
    ../coppeliaKinematicsRoutines/serialization.cpp \
    ../coppeliaKinematicsRoutines/ikGroupContainer.cpp \
    ../coppeliaKinematicsRoutines/ikGroup.cpp \
    ../coppeliaKinematicsRoutines/ikElement.cpp \
    ../coppeliaKinematicsRoutines/objectContainer.cpp \
    ../coppeliaKinematicsRoutines/sceneObject.cpp \
    ../coppeliaKinematicsRoutines/dummy.cpp \
    ../coppeliaKinematicsRoutines/joint.cpp \
    ../simMath/MyMath.cpp \
    ../simMath/3Vector.cpp \
    ../simMath/4Vector.cpp \
    ../simMath/7Vector.cpp \
    ../simMath/3X3Matrix.cpp \
    ../simMath/4X4Matrix.cpp \
    ../simMath/MMatrix.cpp \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
