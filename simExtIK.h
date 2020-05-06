#ifndef SIMEXTIK_H
#define SIMEXTIK_H

#ifdef _WIN32
    #define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#endif
#if defined (__linux) || defined (__APPLE__)
    #define SIM_DLLEXPORT extern "C"
#endif

SIM_DLLEXPORT unsigned char simStart(void*,int);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int,int*,void*,int*);


SIM_DLLEXPORT void ikPlugin_setVerbosity(int level);
SIM_DLLEXPORT int ikPlugin_createEnvironment();
SIM_DLLEXPORT bool ikPlugin_switchEnvironment(int handle);
SIM_DLLEXPORT void ikPlugin_eraseEnvironment();

SIM_DLLEXPORT void ikPlugin_eraseObject(int objectHandle);
SIM_DLLEXPORT void ikPlugin_setObjectParent(int objectHandle,int parentObjectHandle);

SIM_DLLEXPORT int ikPlugin_createDummy();
SIM_DLLEXPORT void ikPlugin_setLinkedDummy(int dummyHandle,int linkedDummyHandle);

SIM_DLLEXPORT int ikPlugin_createJoint(int jointType);
SIM_DLLEXPORT void ikPlugin_setJointMode(int jointHandle,int jointMode);
SIM_DLLEXPORT void ikPlugin_setJointInterval(int jointHandle,bool cyclic,const float* intervalMinAndRange);
SIM_DLLEXPORT void ikPlugin_setJointScrewPitch(int jointHandle,float pitch);
SIM_DLLEXPORT void ikPlugin_setJointIkWeight(int jointHandle,float ikWeight);
SIM_DLLEXPORT void ikPlugin_setJointMaxStepSize(int jointHandle,float maxStepSize);
SIM_DLLEXPORT void ikPlugin_setJointDependency(int jointHandle,int dependencyJointHandle,float offset,float mult);
SIM_DLLEXPORT float ikPlugin_getJointPosition(int jointHandle);
SIM_DLLEXPORT void ikPlugin_setJointPosition(int jointHandle,float position);
SIM_DLLEXPORT void ikPlugin_getSphericalJointQuaternion(int jointHandle,float* quaternion);
SIM_DLLEXPORT void ikPlugin_setSphericalJointQuaternion(int jointHandle,const float* quaternion);

SIM_DLLEXPORT int ikPlugin_createIkGroup();
SIM_DLLEXPORT void ikPlugin_eraseIkGroup(int ikGroupHandle);

SIM_DLLEXPORT void ikPlugin_setIkGroupFlags(int ikGroupHandle,int flags);
SIM_DLLEXPORT void ikPlugin_setIkGroupCalculation(int ikGroupHandle,int method,float damping,int maxIterations);

SIM_DLLEXPORT int ikPlugin_addIkElement(int ikGroupHandle,int tipHandle);
SIM_DLLEXPORT void ikPlugin_eraseIkElement(int ikGroupHandle,int ikElementHandle);
SIM_DLLEXPORT void ikPlugin_setIkElementFlags(int ikGroupHandle,int ikElementHandle,int flags);
SIM_DLLEXPORT void ikPlugin_setIkElementBase(int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle);
SIM_DLLEXPORT void ikPlugin_setIkElementConstraints(int ikGroupHandle,int ikElementHandle,int constraints);
SIM_DLLEXPORT void ikPlugin_setIkElementPrecision(int ikGroupHandle,int ikElementHandle,float linearPrecision,float angularPrecision);
SIM_DLLEXPORT void ikPlugin_setIkElementWeights(int ikGroupHandle,int ikElementHandle,float linearWeight,float angularWeight);

SIM_DLLEXPORT int ikPlugin_handleIkGroup(int ikGroupHandle);
SIM_DLLEXPORT bool ikPlugin_computeJacobian(int ikGroupHandle,int options);
SIM_DLLEXPORT float* ikPlugin_getJacobian(int ikGroupHandle,int* matrixSize);
SIM_DLLEXPORT float ikPlugin_getManipulability(int ikGroupHandle);

SIM_DLLEXPORT char* ikPlugin_getConfigForTipPose(int ikGroupHandle,int jointCnt,const int* jointHandles,float thresholdDist,int maxIterations,int* result,float* retConfig,const float* metric,bool(*validationCallback)(float*),const int* jointOptions,const float* lowLimits,const float* ranges);

SIM_DLLEXPORT void ikPlugin_getObjectLocalTransformation(int objectHandle,float* pos,float* quat);
SIM_DLLEXPORT void ikPlugin_setObjectLocalTransformation(int objectHandle,const float* pos,const float* quat);

#endif
