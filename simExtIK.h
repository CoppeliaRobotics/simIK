#ifndef SIMEXTIK_H
#define SIMEXTIK_H

#ifdef _WIN32
    #define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#endif
#if defined (__linux) || defined (__APPLE__)
    #define SIM_DLLEXPORT extern "C"
#endif

#include <simLib/simTypes.h>

SIM_DLLEXPORT unsigned char simStart(void*,int);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int,int*,void*,int*);

SIM_DLLEXPORT int ikPlugin_createEnv();
SIM_DLLEXPORT void ikPlugin_eraseEnvironment(int ikEnv);
SIM_DLLEXPORT void ikPlugin_eraseObject(int ikEnv,int objectHandle);
SIM_DLLEXPORT void ikPlugin_setObjectParent(int ikEnv,int objectHandle,int parentObjectHandle);
SIM_DLLEXPORT int ikPlugin_createDummy(int ikEnv);
SIM_DLLEXPORT void ikPlugin_setLinkedDummy(int ikEnv,int dummyHandle,int linkedDummyHandle);
SIM_DLLEXPORT int ikPlugin_createJoint(int ikEnv,int jointType);
SIM_DLLEXPORT void ikPlugin_setJointMode(int ikEnv,int jointHandle,int jointMode);
SIM_DLLEXPORT void ikPlugin_setJointInterval(int ikEnv,int jointHandle,bool cyclic,const double* intervalMinAndRange);
SIM_DLLEXPORT void ikPlugin_setJointScrewPitch(int ikEnv,int jointHandle,double pitch);
SIM_DLLEXPORT void ikPlugin_setJointIkWeight(int ikEnv,int jointHandle,double ikWeight);
SIM_DLLEXPORT void ikPlugin_setJointMaxStepSize(int ikEnv,int jointHandle,double maxStepSize);
SIM_DLLEXPORT void ikPlugin_setJointDependency(int ikEnv,int jointHandle,int dependencyJointHandle,double offset,double mult);
SIM_DLLEXPORT double ikPlugin_getJointPosition(int ikEnv,int jointHandle);
SIM_DLLEXPORT void ikPlugin_setJointPosition(int ikEnv,int jointHandle,double position);
SIM_DLLEXPORT void ikPlugin_getSphericalJointQuaternion(int ikEnv,int jointHandle,double* quaternion);
SIM_DLLEXPORT void ikPlugin_setSphericalJointQuaternion(int ikEnv,int jointHandle,const double* quaternion);
SIM_DLLEXPORT int ikPlugin_createIkGroup(int ikEnv);
SIM_DLLEXPORT void ikPlugin_eraseIkGroup(int ikEnv,int ikGroupHandle);
SIM_DLLEXPORT void ikPlugin_setIkGroupFlags(int ikEnv,int ikGroupHandle,int flags);
SIM_DLLEXPORT void ikPlugin_setIkGroupCalculation(int ikEnv,int ikGroupHandle,int method,double damping,int maxIterations);
SIM_DLLEXPORT int ikPlugin_addIkElement(int ikEnv,int ikGroupHandle,int tipHandle);
SIM_DLLEXPORT void ikPlugin_eraseIkElement(int ikEnv,int ikGroupHandle,int ikElementHandle);
SIM_DLLEXPORT void ikPlugin_setIkElementFlags(int ikEnv,int ikGroupHandle,int ikElementHandle,int flags);
SIM_DLLEXPORT void ikPlugin_setIkElementBase(int ikEnv,int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle);
SIM_DLLEXPORT void ikPlugin_setIkElementConstraints(int ikEnv,int ikGroupHandle,int ikElementHandle,int constraints);
SIM_DLLEXPORT void ikPlugin_setIkElementPrecision(int ikEnv,int ikGroupHandle,int ikElementHandle,double linearPrecision,double angularPrecision);
SIM_DLLEXPORT void ikPlugin_setIkElementWeights(int ikEnv,int ikGroupHandle,int ikElementHandle,double linearWeight,double angularWeight);
SIM_DLLEXPORT int ikPlugin_handleIkGroup(int ikEnv,int ikGroupHandle);
SIM_DLLEXPORT bool ikPlugin_computeJacobian(int ikEnv,int ikGroupHandle,int options);
SIM_DLLEXPORT double* ikPlugin_getJacobian(int ikEnv,int ikGroupHandle,int* matrixSize);
SIM_DLLEXPORT double ikPlugin_getManipulability(int ikEnv,int ikGroupHandle);
SIM_DLLEXPORT char* ikPlugin_getConfigForTipPose(int ikEnv,int ikGroupHandle,int jointCnt,const int* jointHandles,double thresholdDist,int maxIterations,int* result,double* retConfig,const double* metric,bool(*validationCallback)(double*),const int* jointOptions,const double* lowLimits,const double* ranges);
SIM_DLLEXPORT void ikPlugin_getObjectLocalTransformation(int ikEnv,int objectHandle,double* pos,double* quat);
SIM_DLLEXPORT void ikPlugin_setObjectLocalTransformation(int ikEnv,int objectHandle,const double* pos,const double* quat);

#endif
