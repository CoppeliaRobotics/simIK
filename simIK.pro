include(config.pri)

TARGET = simIK
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
    QMAKE_CXXFLAGS += -fvisibility=hidden
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
INCLUDEPATH += "../coppeliaKinematicsRoutines"

INCLUDEPATH += $$EIGEN_INCLUDEPATH

HEADERS += simIK.h \
    envCont.h \
    ../include/simLib/simLib.h \
    ../include/simLib/scriptFunctionData.h \
    ../include/simLib/scriptFunctionDataItem.h \
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
    ../include/simMath/mathDefines.h \
    ../include/simMath/mathFuncs.h \
    ../include/simMath/3Vector.h \
    ../include/simMath/4Vector.h \
    ../include/simMath/7Vector.h \
    ../include/simMath/3X3Matrix.h \
    ../include/simMath/4X4Matrix.h \
    ../include/simMath/mXnMatrix.h \

SOURCES += simIK.cpp \
    envCont.cpp \
    ../include/simLib/simLib.cpp \
    ../include/simLib/scriptFunctionData.cpp \
    ../include/simLib/scriptFunctionDataItem.cpp \
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
    ../include/simMath/mathFuncs.cpp \
    ../include/simMath/3Vector.cpp \
    ../include/simMath/4Vector.cpp \
    ../include/simMath/7Vector.cpp \
    ../include/simMath/3X3Matrix.cpp \
    ../include/simMath/4X4Matrix.cpp \
    ../include/simMath/mXnMatrix.cpp \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
