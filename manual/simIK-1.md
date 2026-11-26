# IK-1 plugin API reference

API functions for creating kinematics tasks. All units, unless otherwise indicated, are specified in meters and radians.

## Alphabetical list

- [simIK.addElement](#addElement)
- [simIK.addElementFromScene](#addElementFromScene)
- [simIK.applyIkEnvironmentToScene](#applyIkEnvironmentToScene)
- [simIK.applySceneToIkEnvironment](#applySceneToIkEnvironment)
- [simIK.computeGroupJacobian](#computeGroupJacobian)
- [simIK.computeJacobian](#computeJacobian)
- [simIK.createDebugOverlay](#createDebugOverlay)
- [simIK.createDummy](#createDummy)
- [simIK.createEnvironment](#createEnvironment)
- [simIK.createGroup](#createGroup)
- [simIK.createJoint](#createJoint)
- [simIK.doesGroupExist](#doesGroupExist)
- [simIK.doesObjectExist](#doesObjectExist)
- [simIK.duplicateEnvironment](#duplicateEnvironment)
- [simIK.eraseDebugOverlay](#eraseDebugOverlay)
- [simIK.eraseEnvironment](#eraseEnvironment)
- [simIK.eraseObject](#eraseObject)
- [simIK.findConfig](#findConfig)
- [simIK.findConfigs](#findConfigs)
- [simIK.generatePath](#generatePath)
- [simIK.getAlternateConfigs](#getAlternateConfigs)
- [simIK.getConfigForTipPose](#getConfigForTipPose)
- [simIK.getElementBase](#getElementBase)
- [simIK.getElementConstraints](#getElementConstraints)
- [simIK.getElementFlags](#getElementFlags)
- [simIK.getElementPrecision](#getElementPrecision)
- [simIK.getElementWeights](#getElementWeights)
- [simIK.getGroupCalculation](#getGroupCalculation)
- [simIK.getGroupFlags](#getGroupFlags)
- [simIK.getGroupHandle](#getGroupHandle)
- [simIK.getGroupJointLimitHits](#getGroupJointLimitHits)
- [simIK.getGroupJoints](#getGroupJoints)
- [simIK.getJointDependency](#getJointDependency)
- [simIK.getJointInterval](#getJointInterval)
- [simIK.getJointLimitMargin](#getJointLimitMargin)
- [simIK.getJointMatrix](#getJointMatrix)
- [simIK.getJointMaxStepSize](#getJointMaxStepSize)
- [simIK.getJointMode](#getJointMode)
- [simIK.getJointPosition](#getJointPosition)
- [simIK.getJointScrewLead](#getJointScrewLead)
- [simIK.getJointTransformation](#getJointTransformation)
- [simIK.getJointType](#getJointType)
- [simIK.getJointWeight](#getJointWeight)
- [simIK.getLinkedDummy](#getLinkedDummy)
- [simIK.getObjectHandle](#getObjectHandle)
- [simIK.getObjectMatrix](#getObjectMatrix)
- [simIK.getObjectParent](#getObjectParent)
- [simIK.getObjectPose](#getObjectPose)
- [simIK.getObjects](#getObjects)
- [simIK.getObjectTransformation](#getObjectTransformation)
- [simIK.getObjectType](#getObjectType)
- [simIK.getTargetDummy](#getTargetDummy)
- [simIK.handleGroup](#handleGroup)
- [simIK.handleGroups](#handleGroups)
- [simIK.load](#load)
- [simIK.save](#save)
- [simIK.setElementBase](#setElementBase)
- [simIK.setElementConstraints](#setElementConstraints)
- [simIK.setElementFlags](#setElementFlags)
- [simIK.setElementPrecision](#setElementPrecision)
- [simIK.setElementWeights](#setElementWeights)
- [simIK.setGroupCalculation](#setGroupCalculation)
- [simIK.setGroupFlags](#setGroupFlags)
- [simIK.setJointDependency](#setJointDependency)
- [simIK.setJointInterval](#setJointInterval)
- [simIK.setJointLimitMargin](#setJointLimitMargin)
- [simIK.setJointMaxStepSize](#setJointMaxStepSize)
- [simIK.setJointMode](#setJointMode)
- [simIK.setJointPosition](#setJointPosition)
- [simIK.setJointScrewLead](#setJointScrewLead)
- [simIK.setJointWeight](#setJointWeight)
- [simIK.setLinkedDummy](#setLinkedDummy)
- [simIK.setObjectMatrix](#setObjectMatrix)
- [simIK.setObjectParent](#setObjectParent)
- [simIK.setObjectPose](#setObjectPose)
- [simIK.setObjectTransformation](#setObjectTransformation)
- [simIK.setSphericalJointMatrix](#setSphericalJointMatrix)
- [simIK.setSphericalJointRotation](#setSphericalJointRotation)
- [simIK.setTargetDummy](#setTargetDummy)
- [simIK.syncFromSim](#syncFromSim)
- [simIK.syncToSim](#syncToSim)

## Environment functions and helpers

- [simIK.createEnvironment](#createEnvironment)
- [simIK.eraseEnvironment](#eraseEnvironment)
- [simIK.duplicateEnvironment](#duplicateEnvironment)
- [simIK.createDebugOverlay](#createDebugOverlay)
- [simIK.eraseDebugOverlay](#eraseDebugOverlay)
- [simIK.save](#save)
- [simIK.load](#load)
- [simIK.addElementFromScene](#addElementFromScene)
- [simIK.syncToSim](#syncToSim)
- [simIK.syncFromSim](#syncFromSim)

## Objects

- [simIK.getObjects](#getObjects)
- [simIK.getObjectHandle](#getObjectHandle)
- [simIK.doesObjectExist](#doesObjectExist)
- [simIK.eraseObject](#eraseObject)
- [simIK.getObjectParent](#getObjectParent)
- [simIK.setObjectParent](#setObjectParent)
- [simIK.getObjectPose](#getObjectPose)
- [simIK.setObjectPose](#setObjectPose)
- [simIK.getObjectTransformation](#getObjectTransformation)
- [simIK.setObjectTransformation](#setObjectTransformation)
- [simIK.getObjectType](#getObjectType)
- [simIK.getObjectMatrix](#getObjectMatrix)
- [simIK.setObjectMatrix](#setObjectMatrix)

## Dummies

- [simIK.createDummy](#createDummy)
- [simIK.eraseObject](#eraseObject)
- [simIK.getTargetDummy](#getTargetDummy)
- [simIK.setTargetDummy](#setTargetDummy)

## Joints

- [simIK.createJoint](#createJoint)
- [simIK.eraseObject](#eraseObject)
- [simIK.getJointPosition](#getJointPosition)
- [simIK.setJointPosition](#setJointPosition)
- [simIK.getJointMode](#getJointMode)
- [simIK.setJointMode](#setJointMode)
- [simIK.getJointInterval](#getJointInterval)
- [simIK.setJointInterval](#setJointInterval)
- [simIK.getJointDependency](#getJointDependency)
- [simIK.setJointDependency](#setJointDependency)
- [simIK.getJointWeight](#getJointWeight)
- [simIK.setJointWeight](#setJointWeight)
- [simIK.getJointLimitMargin](#getJointLimitMargin)
- [simIK.setJointLimitMargin](#setJointLimitMargin)
- [simIK.getJointMaxStepSize](#getJointMaxStepSize)
- [simIK.setJointMaxStepSize](#setJointMaxStepSize)
- [simIK.getJointScrewLead](#getJointScrewLead)
- [simIK.setJointScrewLead](#setJointScrewLead)
- [simIK.getJointTransformation](#getJointTransformation)
- [simIK.setSphericalJointRotation](#setSphericalJointRotation)
- [simIK.getJointType](#getJointType)
- [simIK.getJointMatrix](#getJointMatrix)
- [simIK.setSphericalJointMatrix](#setSphericalJointMatrix)
- [simIK.getGroupJoints](#getGroupJoints)

## IK groups

- [simIK.createGroup](#createGroup)
- [simIK.getGroupHandle](#getGroupHandle)
- [simIK.doesGroupExist](#doesGroupExist)
- [simIK.getGroupFlags](#getGroupFlags)
- [simIK.setGroupFlags](#setGroupFlags)
- [simIK.getGroupJointLimitHits](#getGroupJointLimitHits)
- [simIK.getGroupJoints](#getGroupJoints)
- [simIK.getGroupCalculation](#getGroupCalculation)
- [simIK.setGroupCalculation](#setGroupCalculation)

## IK elements

- [simIK.addElement](#addElement)
- [simIK.addElementFromScene](#addElementFromScene)
- [simIK.getElementFlags](#getElementFlags)
- [simIK.setElementFlags](#setElementFlags)
- [simIK.getElementConstraints](#getElementConstraints)
- [simIK.setElementConstraints](#setElementConstraints)
- [simIK.getElementBase](#getElementBase)
- [simIK.setElementBase](#setElementBase)
- [simIK.getElementPrecision](#getElementPrecision)
- [simIK.setElementPrecision](#setElementPrecision)
- [simIK.getElementWeights](#getElementWeights)
- [simIK.setElementWeights](#setElementWeights)

## IK calculation

- [simIK.handleGroup](#handleGroup)
- [simIK.handleGroups](#handleGroups)
- [simIK.findConfigs](#findConfigs)
- [simIK.getAlternateConfigs](#getAlternateConfigs)
- [simIK.computeGroupJacobian](#computeGroupJacobian)
- [simIK.computeJacobian](#computeJacobian)
- [simIK.generatePath](#generatePath)
- [simIK.syncToSim](#syncToSim)
- [simIK.syncFromSim](#syncFromSim)

## Commands

### <a name="addElement" id="addElement"></a> <a name="simIK.addElement" id="simIK.addElement"></a> simIK.addElement

Adds a new IK element to an IK group.

#### Synopsis:

```lua
int elementHandle=simIK.addElement(int environmentHandle,int ikGroupHandle,int tipDummyHandle)
```

```python
int elementHandle=simIK.addElement(int environmentHandle,int ikGroupHandle,int tipDummyHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **tipDummyHandle** (int): the handle of the dummy object that should act as the tip in the IK element.

#### Return values:

- **elementHandle** (int): the IK element handle in the IK group

#### See also:

[createEnvironment](#createEnvironment), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle), [addElementFromScene](#addElementFromScene)

### <a name="addElementFromScene" id="addElementFromScene"></a> <a name="simIK.addElementFromScene" id="simIK.addElementFromScene"></a> simIK.addElementFromScene

Convenience function to quickly generate an IK element from a kinematic chain in the scene

#### Synopsis:

```lua
int ikElement,map simToIkObjectMap,map ikToSimObjectMap=simIK.addElementFromScene(int environmentHandle,int ikGroup,int baseHandle,int tipHandle,int targetHandle,int constraints)
```

```python
int ikElement,list simToIkObjectMap,list ikToSimObjectMap=simIK.addElementFromScene(int environmentHandle,int ikGroup,int baseHandle,int tipHandle,int targetHandle,int constraints)
```

#### Parameters:

- **environmentHandle** (int): the handle of the IK environment.
- **ikGroup** (int): the handle of the IK group, where the IK element should be added.
- **baseHandle** (int): the handle of the base object in the scene. Can be -1 if the base of the kinematic chain is not moving nor changing orientation.
- **tipHandle** (int): the handle of the tip object in the scene.
- **targetHandle** (int): the handle of the target object in the scene.
- **constraints** (int): the constraints for the IK element that should be created. Bit-combine following: simIK.constraint_x, simIK.constraint_y, simIK.constraint_z, simIK.constraint_alpha_beta, simIK.constraint_gamma (simIK.constraint_gamma should only be set if simIK.constraint_alpha_beta is also set). For convenience we also have simIK.constraint_position=simIK.constraint_x|simIK.constraint_y|simIK.constraint_z, simIK.constraint_orientation=simIK.constraint_alpha_beta|simIK.constraint_gamma, and simIK.constraint_pose=simIK.constraint_position|simIK.constraint_orientation.

#### Return values:

- **ikElement** (int): the handle of the created IK element
- **simToIkObjectMap** (map): the mapping of scene objects to IK environment objects
- **ikToSimObjectMap** (map): the mapping of IK environment objects to scene objects

#### See also:

[addElement](#addElement)

### <a name="applyIkEnvironmentToScene" id="applyIkEnvironmentToScene"></a> <a name="simIK.applyIkEnvironmentToScene" id="simIK.applyIkEnvironmentToScene"></a> simIK.applyIkEnvironmentToScene

Deprecated. Use [syncToSim](#syncToSim).

#### Synopsis:

```lua
int result,int reason=simIK.applyIkEnvironmentToScene(int environmentHandle,int ikGroup,bool applyOnlyWhenSuccessful=false)
```

```python
int result=simIK.applyIkEnvironmentToScene(int environmentHandle,int ikGroup,bool applyOnlyWhenSuccessful=False)
```

#### Parameters:

- **environmentHandle** (int): the handle of the IK environment.
- **ikGroup** (int): the handle of the IK group.
- **applyOnlyWhenSuccessful** (bool, default: false): whether computed values should only be applied if computation result is simIK.result_success.

#### Return values:

- **result** (int): simIK.result_success if successfull

### <a name="applySceneToIkEnvironment" id="applySceneToIkEnvironment"></a> <a name="simIK.applySceneToIkEnvironment" id="simIK.applySceneToIkEnvironment"></a> simIK.applySceneToIkEnvironment

Deprecated. Use [syncFromSim](#syncFromSim) instead

#### Synopsis:

```lua
simIK.applySceneToIkEnvironment(int environmentHandle,int ikGroup)
```

```python
simIK.applySceneToIkEnvironment(int environmentHandle,int ikGroup)
```

#### Parameters:

- **environmentHandle** (int): the handle of the IK environment.
- **ikGroup** (int): the handle of the IK group.

#### Return values:

No return values

### <a name="computeGroupJacobian" id="computeGroupJacobian"></a> <a name="simIK.computeGroupJacobian" id="simIK.computeGroupJacobian"></a> simIK.computeGroupJacobian

Computes the Jacobian and error vector for an IK group

#### Synopsis:

```lua
float[] jacobian,float[] errorVector=simIK.computeGroupJacobian(int environmentHandle,int ikGroupHandle)
```

```python
list jacobian,list errorVector=simIK.computeGroupJacobian(int environmentHandle,int ikGroupHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment
- **ikGroupHandle** (int): the handle of an IK group

#### Return values:

- **jacobian** (float[]): the jacobian (row major order). Each row corresponds to one constraint in following order: x, y, z, alpha, beta, gamma. Each column corresponds to one joint, from base to tip
- **errorVector** (float[]): the error vector between the target and the tip

#### See also:

[createEnvironment](#createEnvironment), [handleGroup](#handleGroup), [computeJacobian](#computeJacobian)

### <a name="computeJacobian" id="computeJacobian"></a> <a name="simIK.computeJacobian" id="simIK.computeJacobian"></a> simIK.computeJacobian

Computes the Jacobian and error vector for a kinematic chain

#### Synopsis:

```lua
float[] jacobian,float[] errorVector=simIK.computeJacobian(int environmentHandle,int baseObject,int lastJoint,int constraints,float[7..12] tipMatrix,float[7..12] targetMatrix=nil,float[7..12] constrBaseMatrix=nil)
```

```python
list jacobian,list errorVector=simIK.computeJacobian(int environmentHandle,int baseObject,int lastJoint,int constraints,list tipMatrix,list targetMatrix=None,list constrBaseMatrix=None)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **baseObject** (int): the handle of the base of the kinematic chain, or -1 for the world
- **lastJoint** (int): the handle of the last joint in the kinematic chain, when going from base to tip
- **constraints** (int): a combination of following is possible: simIK.constraint_x, simIK.constraint_y, simIK.constraint_z, simIK.constraint_alpha_beta, simIK.constraint_gamma. For convenience we also have simIK.constraint_position=simIK.constraint_x|simIK.constraint_y|simIK.constraint_z, simIK.constraint_orientation=simIK.constraint_alpha_beta|simIK.constraint_gamma, and simIK.constraint_pose=simIK.constraint_position|simIK.constraint_orientation
- **tipMatrix** (float[7..12]): the transformation matrix or pose of the tip object, relative to the world
- **targetMatrix** (float[7..12], default: nil): the transformation matrix or pose of the target object, relative to the world. Can be nil, in which case the targetPose is the same as the tipPose
- **constrMaseMatrix** (float[7..12], default: nil): the transformation matrix or pose that should serve as the reference frame for positional constraints, or nil to use baseHandle as reference frame

#### Return values:

- **jacobian** (float[]): the jacobian (row major order). Each row corresponds to one constraint in following order: x, y, z, alpha, beta, gamma. Each column corresponds to one joint, from base to tip
- **errorVector** (float[]): the error vector between the target and the tip

#### See also:

[createEnvironment](#createEnvironment), [handleGroup](#handleGroup), [computeGroupJacobian](#computeGroupJacobian)

### <a name="createDebugOverlay" id="createDebugOverlay"></a> <a name="simIK.createDebugOverlay" id="simIK.createDebugOverlay"></a> simIK.createDebugOverlay

Creates a visual representation of an IK chain.

#### Synopsis:

```lua
int objectHandle=simIK.createDebugOverlay(int environmentHandle,int tipHandle,int baseHandle=-1)
```

```python
int objectHandle=simIK.createDebugOverlay(int environmentHandle,int tipHandle,int baseHandle=-1)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **tipHandle** (int): the handle of the tip dummy.
- **baseHandle** (int, default: -1): the handle of the base object.

#### Return values:

- **objectHandle** (int): the handle of the visual overlay object.

#### See also:

[eraseDebugOverlay](#eraseDebugOverlay)

### <a name="createDummy" id="createDummy"></a> <a name="simIK.createDummy" id="simIK.createDummy"></a> simIK.createDummy

Creates a dummy object.

#### Synopsis:

```lua
int dummyHandle=simIK.createDummy(int environmentHandle,string dummyName='')
```

```python
int dummyHandle=simIK.createDummy(int environmentHandle,string dummyName='')
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **dummyName** (string, default: ''): the name of the dummy.

#### Return values:

- **dummyHandle** (int): the handle of the dummy.

#### See also:

[createEnvironment](#createEnvironment), [doesObjectExist](#doesObjectExist), [createJoint](#createJoint), [eraseObject](#eraseObject)

### <a name="createEnvironment" id="createEnvironment"></a> <a name="simIK.createEnvironment" id="simIK.createEnvironment"></a> simIK.createEnvironment

Creates an new IK environment.

#### Synopsis:

```lua
int environmentHandle=simIK.createEnvironment(int flags=0)
```

```python
int environmentHandle=simIK.createEnvironment()
```

#### Parameters:

- **flags** (int, default: 0): reserved, keep at 0.

#### Return values:

- **environmentHandle** (int): the handle of the newly created environment.

#### See also:

[eraseEnvironment](#eraseEnvironment), [load](#load)

### <a name="createGroup" id="createGroup"></a> <a name="simIK.createGroup" id="simIK.createGroup"></a> simIK.createGroup

Creates an IK group.

#### Synopsis:

```lua
int ikGroupHandle=simIK.createGroup(int environmentHandle,string ikGroupName='')
```

```python
int ikGroupHandle=simIK.createGroup(int environmentHandle,string ikGroupName='')
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupName** (string, default: ''): the name of the IK group.

#### Return values:

- **ikGroupHandle** (int): the handle of the IK group.

#### See also:

[createEnvironment](#createEnvironment), [doesGroupExist](#doesGroupExist)

### <a name="createJoint" id="createJoint"></a> <a name="simIK.createJoint" id="simIK.createJoint"></a> simIK.createJoint

Creates a joint object.

#### Synopsis:

```lua
int jointHandle=simIK.createJoint(int environmentHandle,string jointName='')
```

```python
int jointHandle=simIK.createJoint(int environmentHandle,string jointName='')
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointName** (string, default: ''): the name of the joint.
- **jointType**: the type of the joint. Supported types are simIK.jointtype_revolute, simIK.jointtype_prismatic and simIK.jointtype_spherical.

#### Return values:

- **jointHandle** (int): the handle of the joint.

#### See also:

[createEnvironment](#createEnvironment), [doesObjectExist](#doesObjectExist), [createDummy](#createJoint), [eraseObject](#eraseObject)

### <a name="doesGroupExist" id="doesGroupExist"></a> <a name="simIK.doesGroupExist" id="simIK.doesGroupExist"></a> simIK.doesGroupExist

Checks whether an IK group exists, based on its name.

#### Synopsis:

```lua
bool result=simIK.doesGroupExist(int environmentHandle,string ikGroupName)
```

```python
bool result=simIK.doesGroupExist(int environmentHandle,string ikGroupName)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupName** (string): the name of the IK group.

#### Return values:

- **result** (bool): true if the IK group exists.

#### See also:

[createEnvironment](#createEnvironment), [getGroupHandle](#getGroupHandle)

### <a name="doesObjectExist" id="doesObjectExist"></a> <a name="simIK.doesObjectExist" id="simIK.doesObjectExist"></a> simIK.doesObjectExist

Checks whether an object exists, based on its name.

#### Synopsis:

```lua
bool result=simIK.doesObjectExist(int environmentHandle,string objectName)
```

```python
bool result=simIK.doesObjectExist(int environmentHandle,string objectName)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectName** (string): the name of the object.

#### Return values:

- **result** (bool): true if the object exists.

#### See also:

[createEnvironment](#createEnvironment), [getObjectHandle](#getObjectHandle)

### <a name="duplicateEnvironment" id="duplicateEnvironment"></a> <a name="simIK.duplicateEnvironment" id="simIK.duplicateEnvironment"></a> simIK.duplicateEnvironment

Duplicates an IK environment. Useful when operating on an environment while leaving the original environment unchanged.

#### Synopsis:

```lua
int newEnvironmentHandle=simIK.duplicateEnvironment(int environmentHandle)
```

```python
int newEnvironmentHandle=simIK.duplicateEnvironment(int environmentHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.

#### Return values:

- **newEnvironmentHandle** (int): the handle of the duplicated environment.

#### See also:

[eraseEnvironment](#eraseEnvironment), [createEnvironment](#createEnvironment)

### <a name="eraseDebugOverlay" id="eraseDebugOverlay"></a> <a name="simIK.eraseDebugOverlay" id="simIK.eraseDebugOverlay"></a> simIK.eraseDebugOverlay

Removes the visual representation of an IK chain.

#### Synopsis:

```lua
simIK.eraseDebugOverlay(int objectHandle)
```

```python
simIK.eraseDebugOverlay(int objectHandle)
```

#### Parameters:

- **objectHandle** (int): the handle of the visual overlay object.

#### Return values:

No return values

#### See also:

[createDebugOverlay](#createDebugOverlay)

### <a name="eraseEnvironment" id="eraseEnvironment"></a> <a name="simIK.eraseEnvironment" id="simIK.eraseEnvironment"></a> simIK.eraseEnvironment

Erases an IK environment.

#### Synopsis:

```lua
simIK.eraseEnvironment(int environmentHandle)
```

```python
simIK.eraseEnvironment(int environmentHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment)

### <a name="eraseObject" id="eraseObject"></a> <a name="simIK.eraseObject" id="simIK.eraseObject"></a> simIK.eraseObject

Erases an object.

#### Synopsis:

```lua
simIK.eraseObject(int environmentHandle,int objectHandle)
```

```python
simIK.eraseObject(int environmentHandle,int objectHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): handle of the object.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getObjectHandle](#getObjectHandle), [createDummy](#createDummy), [createJoint](#createJoint)

### <a name="findConfig" id="findConfig"></a> <a name="simIK.findConfig" id="simIK.findConfig"></a> simIK.findConfig

Deprecated. See simIK.findConfigs instead.

#### Synopsis:

```lua
float[] jointPositions=simIK.findConfig(int environmentHandle,int ikGroupHandle,int[] jointHandles,float thresholdDist=0.1,float maxTime=0.5,float[4] metric={1,1,1,0.1},func/string validationCallback=nil,auxData=nil)
```

```python
list jointPositions=simIK.findConfig(int environmentHandle,int ikGroupHandle,list jointHandles,float thresholdDist=0.1,float maxTime=0.5,list metric=[1,1,1,0.1],function/string validationCallback=None,auxData=None)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **jointHandles** (int[]): a table that specifies the joint handles for the joints we wish to retrieve the values calculated by the IK.
- **thresholdDist** (float, default: 0.1): a distance indicating when IK should be computed in order to try to bring the tip onto the target: since the search algorithm proceeds by generating random configurations, many of them produce a tip pose that is too far from the target pose to run IK successfully. Choosing a large value will result in slow calculations, choosing a small value might produce a smaller subset of solutions. Distance between two poses is calculated using a metric (see metric argument below).
- **maxTime** (float, default: 0.5): the upper time limit, in seconds, after which the function returns.
- **metric** (float[4], default: {1,1,1,0.1}): a table to 4 values indicating a metric used to compute pose-pose distances: distance=sqrt((dx*metric[1])^2+(dy*metric[2])^2+(dz*metric[3])^2+(angle*metric[4])^2).
- **validationCallback** (func/string, default: nil): an optional callback function expressed as a function or a string. The callback function takes as input arguments the proposed joint values (i.e. a configuration) and **auxData**, and as return value whether the configuration is valid (e.g. is not colliding).
- **auxData** (default: nil): auxiliary data that will be handed to the validation callback.

#### Return values:

- **jointPositions** (float[]): a table that contains the IK calculated joint values, as specified by the jointHandles table, if a valid configuration was found.

#### See also:

[findConfigs](#findConfigs)

### <a name="findConfigs" id="findConfigs"></a> <a name="simIK.findConfigs" id="simIK.findConfigs"></a> simIK.findConfigs

Searches for one or several manipulator configurations that match the target dummy/dummies position/orientation in space. Search is randomized. The IK environment remains unchanged.

#### Synopsis:

```lua
float[][] configs=simIK.findConfigs(int environmentHandle,int ikGroupHandle,int[] jointHandles,map params={}, float[][] configs={})
```

```python
list[] configs=simIK.findConfigs(int environmentHandle,int ikGroupHandle,list jointHandles,dict params={}, list[] configs=[])
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **jointHandles** (int[]): a table that specifies the joint handles for the joints we wish to retrieve the values calculated by the IK.
- **params** (map, default: {}): optional parameters:
- **configs** (float[][], default: {}): configurations previously calculated, that should be added to the newly calculated configurations, as return value

#### Return values:

- **configs** (float[][]): an array of configurations, sorted from closest to farthest from the current configuration (using the metric **cMetric**)

#### See also:

[createEnvironment](#createEnvironment), [duplicateEnvironment](#duplicateEnvironment), [computeJacobian](#computeJacobian)

### <a name="generatePath" id="generatePath"></a> <a name="simIK.generatePath" id="simIK.generatePath"></a> simIK.generatePath

Generates a path that drives the IK tip onto its IK target, in a straight line (i.e. shortest path in Cartesian space). The function returns a path in the configuration space if the operation was successful. A reason for a non-successful operation can be: there are some forbidden poses/configurations on the way, or some of the configuration points cannot be reached (e.g. out of reach, or due to joint limits). The IK environment remains unchanged.

#### Synopsis:

```lua
float[] configurationList=simIK.generatePath(int environmentHandle,int ikGroupHandle,int[] jointHandles,int tipHandle,int pathPointCount,function/string validationCallback=nil,auxData=nil)
```

```python
list configurationList=simIK.generatePath(int environmentHandle,int ikGroupHandle,list jointHandles,int tipHandle,int pathPointCount,function/string validationCallback=None,auxData=None)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the IK group handle.
- **jointHandles** (int[]): a table that specifies the joint handles for the joints we wish to retrieve the values calculated by the IK.
- **tipHandle** (int): the handle of the tip object.
- **pathPointCount** (int): the desired number of path points. Each path point contains a joint configuration. A minimum of two path points is required.
- **validationCallback** (function/string, default: nil): an optional callback function, expressed as a function or string. The callback function takes as input arguments proposed joint values (i.e. a configuration) and **auxData**, and as return value whether the configuration is valid (e.g. is not colliding).
- **auxData** (default: nil): auxiliary data that will be handed to the validation callback.

#### Return values:

- **configurationList** (float[]): a table that contains configurations (in row-major order) that will bring the IK tip onto its IK target.

#### See also:

[duplicateEnvironment](#duplicateEnvironment)

### <a name="getAlternateConfigs" id="getAlternateConfigs"></a> <a name="simIK.getAlternateConfigs" id="simIK.getAlternateConfigs"></a> simIK.getAlternateConfigs

Generates alternative manipulator configurations, for a same end-effector pose, for a manipulator that has revolute joints with a range larger than 360 degrees. The original submitted configuration will be part of the returned configurations. The IK environment remains unchanged.

#### Synopsis:

```lua
float[] configurations=simIK.getAlternateConfigs(int environmentHandle,int[] jointHandles,float[] lowLimits=nil,float[] ranges=nil)
```

```python
list configurations=simIK.getAlternateConfigs(int environmentHandle,list jointHandles,list lowLimits=None,list ranges=None)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandles** (int[]): a table with the handles of the manipulator joints.
- **lowLimits** (float[], default: nil): a table with joint low limit values for each specified joint. This can be useful when you wish to explore a sub-set of the joint's intervals.
- **ranges** (float[], default: nil): a table with joint range values for each specified joint. This can be useful when you wish to explore a sub-set of the joint's intervals. If the range value is 0, then the lowLimit and range values are taken from the joint's properties. If the range value is negative, then the search interval will be centered around the current linear/angular joint position, with an extent of (-range).

#### Return values:

- **configurations** (float[]): a table containing configurations (in row-major order) that are equivalent to the specified inputConfig, in terms of end-effector pose.

#### See also:

[createEnvironment](#createEnvironment)

### <a name="getConfigForTipPose" id="getConfigForTipPose"></a> <a name="simIK.getConfigForTipPose" id="simIK.getConfigForTipPose"></a> simIK.getConfigForTipPose

Deprecated. See simIK.findConfigs instead.

#### Synopsis:

```lua
float[] jointPositions=simIK.getConfigForTipPose(int environmentHandle,int ikGroupHandle,int[] jointHandles,float thresholdDist=0.1,float maxTime=0.5,float[4] metric={1,1,1,0.1},function validationCallback=nil,auxData=nil,int[] jointOptions={},float[] lowLimits={},float[] ranges={})
```

```python
list jointPositions=simIK.getConfigForTipPose(int environmentHandle,int ikGroupHandle,list jointHandles,float thresholdDist=0.1,float maxTime=0.5,list metric=[1,1,1,0.1],function validationCallback=None,auxData=None,list jointOptions=[],list lowLimits=[],list ranges=[])
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **jointHandles** (int[]): a table that specifies the joint handles for the joints we wish to retrieve the values calculated by the IK.
- **thresholdDist** (float, default: 0.1): a distance indicating when IK should be computed in order to try to bring the tip onto the target: since the search algorithm proceeds by generating random configurations, many of them produce a tip pose that is too far from the target pose to run IK successfully. Choosing a large value will result in slow calculations, choosing a small value might produce a smaller subset of solutions. Distance between two poses is calculated using a metric (see metric argument below).
- **maxTime** (float, default: 0.5): the upper time limit, in seconds, after which the function returns.
- **metric** (float[4], default: {1,1,1,0.1}): a table to 4 values indicating a metric used to compute pose-pose distances: distance=sqrt((dx*metric[1])^2+(dy*metric[2])^2+(dz*metric[3])^2+(angle*metric[4])^2).
- **validationCallback** (function, default: nil): an optional callback function. The callback function takes as input arguments the proposed joint values (i.e. a configuration) and **auxData**, and as return value whether the configuration is valid (e.g. is not colliding).
- **auxData** (default: nil): auxiliary data that will be handed to the validation callback.
- **jointOptions** (int[], default: {}): a table with bit-coded values corresponding to each specified joint handle. Bit 0 (i.e. 1) indicates the corresponding joint is dependent of another joint.
- **lowLimits** (float[], default: {}): a table with joint low limit values for each specified joint. This can be useful when you wish to explore a sub-set of the joint's intervals.
- **ranges** (float[], default: {}): a table with joint range values for each specified joint. This can be useful when you wish to explore a sub-set of the joint's intervals. If the range value is 0, then the lowLimit and range values are taken from the joint's properties. If the range value is negative, then the search interval will be centered around the current linear/angular joint position, with an extent of (-range).

#### Return values:

- **jointPositions** (float[]): a table that contains the IK calculated joint values, as specified by the jointHandles table, if a valid configuration was found.

#### See also:

[createEnvironment](#createEnvironment), [duplicateEnvironment](#duplicateEnvironment), [getAlternateConfigs](#getAlternateConfigs), [computeJacobian](#computeJacobian)

### <a name="getElementBase" id="getElementBase"></a> <a name="simIK.getElementBase" id="simIK.getElementBase"></a> simIK.getElementBase

Retrieves the base object of an IK element.

#### Synopsis:

```lua
int baseHandle,int constraintsBaseHandle=simIK.getElementBase(int environmentHandle,int ikGroupHandle,int elementHandle)
```

```python
int baseHandle,int constraintsBaseHandle=simIK.getElementBase(int environmentHandle,int ikGroupHandle,int elementHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).

#### Return values:

- **baseHandle** (int): the handle of the base object, or -1 if the world is the base.
- **constraintsBaseHandle** (int): the handle of the constraints base object, relative to which the constraints are specified. Returns -1 if the constraints are relative to the base object.

#### See also:

[createEnvironment](#createEnvironment), [setElementBase](#setElementBase), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="getElementConstraints" id="getElementConstraints"></a> <a name="simIK.getElementConstraints" id="simIK.getElementConstraints"></a> simIK.getElementConstraints

Retrieves the constraints of an IK element.

#### Synopsis:

```lua
int constraints=simIK.getElementConstraints(int environmentHandle,int ikGroupHandle,int elementHandle)
```

```python
int constraints=simIK.getElementConstraints(int environmentHandle,int ikGroupHandle,int elementHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).

#### Return values:

- **constraints** (int): the constraints. A bit-combination of following is possible: simIK.constraint_x, simIK.constraint_y, simIK.constraint_z, simIK.constraint_alpha_beta, simIK.constraint_gamma.

#### See also:

[createEnvironment](#createEnvironment), [setElementConstraints](#setElementConstraints), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="getElementFlags" id="getElementFlags"></a> <a name="simIK.getElementFlags" id="simIK.getElementFlags"></a> simIK.getElementFlags

Retrieves various flags of an IK element.

#### Synopsis:

```lua
int flags=simIK.getElementFlags(int environmentHandle,int ikGroupHandle,int elementHandle)
```

```python
int flags=simIK.getElementFlags(int environmentHandle,int ikGroupHandle,int elementHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).

#### Return values:

- **flags** (int): bit-coded flags: bit0 set(1)=the enabled state of the ik element.

#### See also:

[createEnvironment](#createEnvironment), [setElementFlags](#setElementFlags), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="getElementPrecision" id="getElementPrecision"></a> <a name="simIK.getElementPrecision" id="simIK.getElementPrecision"></a> simIK.getElementPrecision

Retrieves the precision settings of an IK element.

#### Synopsis:

```lua
float[2] precision=simIK.getElementPrecision(int environmentHandle,int ikGroupHandle,int elementHandle)
```

```python
list precision=simIK.getElementPrecision(int environmentHandle,int ikGroupHandle,int elementHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).

#### Return values:

- **precision** (float[2]): a table with the linear and angular precision.

#### See also:

[createEnvironment](#createEnvironment), [setElementPrecision](#setElementPrecision), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="getElementWeights" id="getElementWeights"></a> <a name="simIK.getElementWeights" id="simIK.getElementWeights"></a> simIK.getElementWeights

Retrieves the desired linear and angular resolution weights of an IK element.

#### Synopsis:

```lua
float[3] weights=simIK.getElementWeights(int environmentHandle,int ikGroupHandle,int elementHandle)
```

```python
list weights=simIK.getElementWeights(int environmentHandle,int ikGroupHandle,int elementHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).

#### Return values:

- **weights** (float[3]): a table with the linear, angular and overall resolution weights.

#### See also:

[createEnvironment](#createEnvironment), [setElementWeights](#setElementWeights), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="getGroupCalculation" id="getGroupCalculation"></a> <a name="simIK.getGroupCalculation" id="simIK.getGroupCalculation"></a> simIK.getGroupCalculation

Retrieves calculation properties for an IK group.

#### Synopsis:

```lua
int method,float damping,int maxIterations=simIK.getGroupCalculation(int environmentHandle,int ikGroupHandle)
```

```python
int method,float damping,int maxIterations=simIK.getGroupCalculation(int environmentHandle,int ikGroupHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.

#### Return values:

- **method** (int): the resolution method. Possible values are simIK.method_pseudo_inverse (features a tiny bit of hard-coded damping), simIK.method_undamped_pseudo_inverse, simIK.method_damped_least_squares and simIK.method_jacobian_transpose.
- **damping** (float): the damping, in case the resolution method is simIK.method_damped_least_squares.
- **maxIterations** (int): the maximum number of iterations.

#### See also:

[createEnvironment](#createEnvironment), [setGroupCalculation](#setGroupCalculation), [getGroupHandle](#getGroupHandle)

### <a name="getGroupFlags" id="getGroupFlags"></a> <a name="simIK.getGroupFlags" id="simIK.getGroupFlags"></a> simIK.getGroupFlags

Retrieves flags of an IK group.

#### Synopsis:

```lua
int flags=simIK.getGroupFlags(int environmentHandle,int ikGroupHandle)
```

```python
int flags=simIK.getGroupFlags(int environmentHandle,int ikGroupHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.

#### Return values:

- **flags** (int): the flags of the IK group, bit-coded:
  - simIK.group_enabled: the group's enabled state
  - simIK.group_ignoremaxsteps: the joints' max step sizes are ignored. Otherwise, when detected, an interpolation factor (originally at 1.0) is successively divided by 2.0 until the max. step sizes are respected
  - simIK.group_restoreonbadlintol: the IK world state is left untouched if the linear precision is not reached
  - simIK.group_restoreonbadangtol: the IK world state is left untouched if the angular precision is not reached
  - simIK.group_avoidlimits: joint limits are actively avoided
  - simIK.group_stoponlimithit: calculation ends when a joint limit is hit

#### See also:

[createEnvironment](#createEnvironment), [setGroupFlags](#setGroupFlags), [getGroupHandle](#getGroupHandle)

### <a name="getGroupHandle" id="getGroupHandle"></a> <a name="simIK.getGroupHandle" id="simIK.getGroupHandle"></a> simIK.getGroupHandle

Retrieves the handle of an IK group based on its name.

#### Synopsis:

```lua
int ikGroupHandle=simIK.getGroupHandle(int environmentHandle,string ikGroupName)
```

```python
int ikGroupHandle=simIK.getGroupHandle(int environmentHandle,string ikGroupName)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupName** (string): the name of the IK group.

#### Return values:

- **ikGroupHandle** (int): the handle of the IK group.

#### See also:

[createEnvironment](#createEnvironment), [doesGroupExist](#doesGroupExist), [createGroup](#createGroup)

### <a name="getGroupJointLimitHits" id="getGroupJointLimitHits"></a> <a name="simIK.getGroupJointLimitHits" id="simIK.getGroupJointLimitHits"></a> simIK.getGroupJointLimitHits

Checks which joints of an IK group hit a limit last time that IK group was handled

#### Synopsis:

```lua
int[] jointHandles,float[] underOrOvershots=simIK.getGroupJointLimitHits(int environmentHandle,int ikGroupHandle)
```

```python
list jointHandles,list underOrOvershots=simIK.getGroupJointLimitHits(int environmentHandle,int ikGroupHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.

#### Return values:

- **jointHandles** (int[]): the joint handles that that had joint limits under- or overshot
- **underOrOvershots** (float[]): the corresponding under- or overshots, for each of the jointHandles

#### See also:

[setGroupFlags](#setGroupFlags)

### <a name="getGroupJoints" id="getGroupJoints"></a> <a name="simIK.getGroupJoints" id="simIK.getGroupJoints"></a> simIK.getGroupJoints

Returns the joint handles involved in the IK group calculation, i.e. one handle per Jacobian column (except with revolute joints that have 3 corresponding Jacobian columns)

#### Synopsis:

```lua
int[] jointHandles=simIK.getGroupJoints(int environmentHandle,int ikGroupHandle)
```

```python
list jointHandles=simIK.getGroupJoints(int environmentHandle,int ikGroupHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment
- **ikGroupHandle** (int): the handle of the IK group

#### Return values:

- **jointHandles** (int[]): the joint handles

### <a name="getJointDependency" id="getJointDependency"></a> <a name="simIK.getJointDependency" id="simIK.getJointDependency"></a> simIK.getJointDependency

Retrieves information about a possible joint dependency.

#### Synopsis:

```lua
int depJointHandle,float offset,float mult=simIK.getJointDependency(int environmentHandle,int jointHandle)
```

```python
int depJointHandle,float offset,float mult=simIK.getJointDependency(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **depJointHandle** (int): the handle of the dependency joint.
- **offset** (float): the offset. We have linear/angular joint position = dependency linear/angular joint position * mult + offset.
- **mult** (float): the multiplication factor. We have linear/angular joint position = dependency linear/angular joint position * mult + offset.

#### See also:

[createEnvironment](#createEnvironment), [setJointDependency](#setJointDependency), [getObjectHandle](#getObjectHandle)

### <a name="getJointInterval" id="getJointInterval"></a> <a name="simIK.getJointInterval" id="simIK.getJointInterval"></a> simIK.getJointInterval

Retrieves the joint limits.

#### Synopsis:

```lua
bool cyclic,float[2] interval=simIK.getJointInterval(int environmentHandle,int jointHandle)
```

```python
bool cyclic,list interval=simIK.getJointInterval(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **cyclic** (bool): whether the joint is cyclic (has no limits).
- **interval** (float[2]): a table with two values: the joint lower limit, and the joint range (i.e. joint upper limit = joint lower limit + joint range)

#### See also:

[createEnvironment](#createEnvironment), [setJointInterval](#setJointInterval), [getObjectHandle](#getObjectHandle)

### <a name="getJointLimitMargin" id="getJointLimitMargin"></a> <a name="simIK.getJointLimitMargin" id="simIK.getJointLimitMargin"></a> simIK.getJointLimitMargin

Retrieves the limit margin of a joint, i.e. the threshold that will be used to counteract on joint limit violation during IK resolution, if the appropriate IK group flag was set

#### Synopsis:

```lua
float margin=simIK.getJointLimitMargin(int environmentHandle,int jointHandle)
```

```python
float margin=simIK.getJointLimitMargin(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **margin** (float): the margin

#### See also:

[createEnvironment](#createEnvironment), [setJointLimitMargin](#setJointLimitMargin), [getGroupFlags](#getGroupFlags)

### <a name="getJointMatrix" id="getJointMatrix"></a> <a name="simIK.getJointMatrix" id="simIK.getJointMatrix"></a> simIK.getJointMatrix

Retrieves the intrinsic transformation matrix of a joint.

#### Synopsis:

```lua
float[12] matrix=simIK.getJointMatrix(int environmentHandle,int jointHandle)
```

```python
list matrix=simIK.getJointMatrix(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **matrix** (float[12]): a table of 12 numbers representing the transformation matrix (the last row of the 4x4 matrix (0,0,0,1) is not returned).

#### See also:

[createEnvironment](#createEnvironment), [setSphericalJointMatrix](#setSphericalJointMatrix), [getJointPosition](#getJointPosition), [getJointTransformation](#getJointTransformation), [getObjectHandle](#getObjectHandle)

### <a name="getJointMaxStepSize" id="getJointMaxStepSize"></a> <a name="simIK.getJointMaxStepSize" id="simIK.getJointMaxStepSize"></a> simIK.getJointMaxStepSize

Retrieves the maximum step size of a joint.

#### Synopsis:

```lua
float stepSize=simIK.getJointMaxStepSize(int environmentHandle,int jointHandle)
```

```python
float stepSize=simIK.getJointMaxStepSize(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **stepSize** (float): the maximum step size.

#### See also:

[createEnvironment](#createEnvironment), [setJointMaxStepSize](#setJointMaxStepSize), [getObjectHandle](#getObjectHandle)

### <a name="getJointMode" id="getJointMode"></a> <a name="simIK.getJointMode" id="simIK.getJointMode"></a> simIK.getJointMode

Retrieves the joint mode.

#### Synopsis:

```lua
int jointMode=simIK.getJointMode(int environmentHandle,int jointHandle)
```

```python
int jointMode=simIK.getJointMode(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **jointMode** (int): the joint mode. Possible values are: simIK.jointmode_passive, simIK.jointmode_ik

#### See also:

[createEnvironment](#createEnvironment), [setJointMode](#setJointMode), [getObjectHandle](#getObjectHandle)

### <a name="getJointPosition" id="getJointPosition"></a> <a name="simIK.getJointPosition" id="simIK.getJointPosition"></a> simIK.getJointPosition

Retrieves the position (linear or angular) of a joint.

#### Synopsis:

```lua
float position=simIK.getJointPosition(int environmentHandle,int jointHandle)
```

```python
float position=simIK.getJointPosition(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **position** (float): the position.

#### See also:

[createEnvironment](#createEnvironment), [setJointPosition](#setJointPosition), [getJointMatrix](#getJointMatrix), [getJointTransformation](#getJointTransformation), [getObjectHandle](#getObjectHandle)

### <a name="getJointScrewLead" id="getJointScrewLead"></a> <a name="simIK.getJointScrewLead" id="simIK.getJointScrewLead"></a> simIK.getJointScrewLead

Retrieves the screw lead of a revolute joint.

#### Synopsis:

```lua
float lead=simIK.getJointScrewLead(int environmentHandle,int jointHandle)
```

```python
float lead=simIK.getJointScrewLead(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **lead** (float): the screw lead of the joint. A lead value of zero represents a revolute joint, a value different from zero represents a screw.

#### See also:

[createEnvironment](#createEnvironment), [setJointScrewLead](#setJointScrewLead), [getObjectHandle](#getObjectHandle)

### <a name="getJointTransformation" id="getJointTransformation"></a> <a name="simIK.getJointTransformation" id="simIK.getJointTransformation"></a> simIK.getJointTransformation

Retrieves the intrinsic transformation of a joint.

#### Synopsis:

```lua
float[3] position,float[4] quaternion,float[3] euler=simIK.getJointTransformation(int environmentHandle,int jointHandle)
```

```python
list position,list quaternion,list euler=simIK.getJointTransformation(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **position** (float[3]): the translation of the joint.
- **quaternion** (float[4]): the rotation of the joint, expressed as quaternion (x,y,z,w).
- **euler** (float[3]): the rotation of the joint, expressed as Euler angles (alpha,beta,gamma).

#### See also:

[createEnvironment](#createEnvironment), [setSphericalJointRotation](#setSphericalJointRotation), [getJointPosition](#getJointPosition), [getJointMatrix](#getJointMatrix), [getObjectHandle](#getObjectHandle)

### <a name="getJointType" id="getJointType"></a> <a name="simIK.getJointType" id="simIK.getJointType"></a> simIK.getJointType

Retrieves the joint type.

#### Synopsis:

```lua
int jointType=simIK.getJointType(int environmentHandle,int jointHandle)
```

```python
int jointType=simIK.getJointType(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **jointType** (int): the joint type. Possible values are: simIK.jointtype_revolute, simIK.jointtype_prismatic and simIK.jointtype_spherical

#### See also:

[createJoint](#createJoint)

### <a name="getJointWeight" id="getJointWeight"></a> <a name="simIK.getJointWeight" id="simIK.getJointWeight"></a> simIK.getJointWeight

Retrieves the IK weight of a joint, i.e. the weight it has during IK resolution.

#### Synopsis:

```lua
float weight=simIK.getJointWeight(int environmentHandle,int jointHandle)
```

```python
float weight=simIK.getJointWeight(int environmentHandle,int jointHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.

#### Return values:

- **weight** (float): the IK weight.

#### See also:

[createEnvironment](#createEnvironment), [setJointWeight](#setJointWeight), [getObjectHandle](#getObjectHandle)

### <a name="getLinkedDummy" id="getLinkedDummy"></a> <a name="simIK.getLinkedDummy" id="simIK.getLinkedDummy"></a> simIK.getLinkedDummy

Deprecated. Use [getTargetDummy](#getTargetDummy) instead

#### Synopsis:

```lua
int linkedDummyHandle=simIK.getLinkedDummy(int environmentHandle,int dummyHandle)
```

```python
int linkedDummyHandle=simIK.getLinkedDummy(int environmentHandle,int dummyHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **dummyHandle** (int): the handle of the dummy object.

#### Return values:

- **linkedDummyHandle** (int): the handle of the linked dummy object. Is -1 if no dummy object is linked to this one.

#### See also:

[createEnvironment](#createEnvironment), [setLinkedDummy](#setLinkedDummy), [getObjectHandle](#getObjectHandle)

### <a name="getObjectHandle" id="getObjectHandle"></a> <a name="simIK.getObjectHandle" id="simIK.getObjectHandle"></a> simIK.getObjectHandle

Retrieves the handle of an object based on its name.

#### Synopsis:

```lua
int objectHandle=simIK.getObjectHandle(int environmentHandle,string objectName)
```

```python
int objectHandle=simIK.getObjectHandle(int environmentHandle,string objectName)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectName** (string): the name of the object.

#### Return values:

- **objectHandle** (int): the object handle.

#### See also:

[createEnvironment](#createEnvironment), [doesObjectExist](#doesObjectExist), [createDummy](#createDummy), [createJoint](#createJoint)

### <a name="getObjectMatrix" id="getObjectMatrix"></a> <a name="simIK.getObjectMatrix" id="simIK.getObjectMatrix"></a> simIK.getObjectMatrix

Retrieves the transformation matrix of an object. If the object is a joint object, the matrix does not include the joint's intrinsic transformation.

#### Synopsis:

```lua
float[12] matrix=simIK.getObjectMatrix(int environmentHandle,int objectHandle,int relativeToObjectHandle=simIK.handle_world)
```

```python
list matrix=simIK.getObjectMatrix(int environmentHandle,int objectHandle,int relativeToObjectHandle=simIK.handle_world)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.
- **relativeToObjectHandle** (int, default: simIK.handle_world): the handle of an object relative to which we want the matrix expressed. Otherwise, specify simIK.handle_world if you want the absolute matrix, or simIK.handle_parent if you want the matrix relative to the parent object.

#### Return values:

- **matrix** (float[12]): a table of 12 numbers representing the transformation matrix (the last row of the 4x4 matrix (0,0,0,1) is not returned)

#### See also:

[createEnvironment](#createEnvironment), [setObjectMatrix](#setObjectMatrix), [getObjectPose](#getObjectPose), [getObjectTransformation](#getObjectTransformation), [getJointTransformation](#getJointTransformation), [getObjectHandle](#getObjectHandle)

### <a name="getObjectParent" id="getObjectParent"></a> <a name="simIK.getObjectParent" id="simIK.getObjectParent"></a> simIK.getObjectParent

Retrieves an object's parent handle.

#### Synopsis:

```lua
int parentObjectHandle=simIK.getObjectParent(int environmentHandle,int objectHandle)
```

```python
int parentObjectHandle=simIK.getObjectParent(int environmentHandle,int objectHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.

#### Return values:

- **parentObjectHandle** (int): the returned handle of the parent, or -1 if the object has no parent.

#### See also:

[createEnvironment](#createEnvironment), [setObjectParent](#setObjectParent), [getObjectHandle](#getObjectHandle)

### <a name="getObjectPose" id="getObjectPose"></a> <a name="simIK.getObjectPose" id="simIK.getObjectPose"></a> simIK.getObjectPose

Retrieves the pose (position and quaternion) of an object. If the object is a joint object, the pose does not include the joint's intrinsic transformation.

#### Synopsis:

```lua
float[7] pose=simIK.getObjectPose(int environmentHandle,int objectHandle,int relativeToObjectHandle=simIK.handle_world)
```

```python
list pose=simIK.getObjectPose(int environmentHandle,int objectHandle,int relativeToObjectHandle=simIK.handle_world)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.
- **relativeToObjectHandle** (int, default: simIK.handle_world): the handle of an object relative to which we want the pose expressed. Otherwise, specify simIK.handle_world if you want the absolute pose, or simIK.handle_parent if you want the pose relative to the parent object.

#### Return values:

- **pose** (float[7]): the position and quaternion of the object (x,y,z,qx,qy,qz,qw).

#### See also:

[createEnvironment](#createEnvironment), [setObjectPose](#setObjectPose), [getObjectTransformation](#getObjectTransformation), [getObjectMatrix](#getObjectMatrix), [getJointTransformation](#getJointTransformation), [getObjectHandle](#getObjectHandle)

### <a name="getObjects" id="getObjects"></a> <a name="simIK.getObjects" id="simIK.getObjects"></a> simIK.getObjects

Allows to loop through all objects in the environment.

#### Synopsis:

```lua
int objectHandle,string objectName,bool isJoint,int jointType=simIK.getObjects(int environmentHandle,int index)
```

```python
int objectHandle,string objectName,bool isJoint,int jointType=simIK.getObjects(int environmentHandle,int index)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **index** (int): the zero-based index. Start at 0, and increment until there is no return values anymore, in order to loop through all objects in the environment.

#### Return values:

- **objectHandle** (int): the handle of the object, or nil if there is no object at the specified index.
- **objectName** (string): the name of the object.
- **isJoint** (bool): whether the object is a joint.
- **jointType** (int): the type of joint, if the object at the specified index is a joint. Possible values are simIK.jointtype_revolute, simIK.jointtype_prismatic or simIK.jointtype_spherical.

#### See also:

[getObjectHandle](#getObjectHandle), [doesObjectExist](#doesObjectExist)

### <a name="getObjectTransformation" id="getObjectTransformation"></a> <a name="simIK.getObjectTransformation" id="simIK.getObjectTransformation"></a> simIK.getObjectTransformation

Retrieves the transformation (position and quaternion/euler angles) of an object. If the object is a joint object, the transformation does not include the joint's intrinsic transformation.

#### Synopsis:

```lua
float[3] position,float[4] quaternion,float[3] euler=simIK.getObjectTransformation(int environmentHandle,int objectHandle,int relativeToObjectHandle=simIK.handle_world)
```

```python
list position,list quaternion,list euler=simIK.getObjectTransformation(int environmentHandle,int objectHandle,int relativeToObjectHandle=simIK.handle_world)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.
- **relativeToObjectHandle** (int, default: simIK.handle_world): the handle of an object relative to which we want the transformation expressed. Otherwise, specify simIK.handle_world if you want the absolute transformation, or simIK.handle_parent if you want the transformation relative to the parent object.

#### Return values:

- **position** (float[3]): the position of the object.
- **quaternion** (float[4]): the orientation of the object, expressed as quaternion (x,y,z,w).
- **euler** (float[3]): the orientation of the object, expressed as Euler angles (alpha,beta,gamma).

#### See also:

[createEnvironment](#createEnvironment), [setObjectTransformation](#setObjectTransformation), [getObjectPose](#getObjectPose), [getObjectMatrix](#getObjectMatrix), [getJointTransformation](#getJointTransformation), [getObjectHandle](#getObjectHandle)

### <a name="getObjectType" id="getObjectType"></a> <a name="simIK.getObjectType" id="simIK.getObjectType"></a> simIK.getObjectType

Retrieves the type of an object.

#### Synopsis:

```lua
int objectType=simIK.getObjectType(int environmentHandle,int objectHandle)
```

```python
int objectType=simIK.getObjectType(int environmentHandle,int objectHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.

#### Return values:

- **objectType** (int): the returned type of the object.

### <a name="getTargetDummy" id="getTargetDummy"></a> <a name="simIK.getTargetDummy" id="simIK.getTargetDummy"></a> simIK.getTargetDummy

Retrieves the handle of the target dummy associated with a tip dummy

#### Synopsis:

```lua
int targetDummyHandle=simIK.getTargetDummy(int environmentHandle,int tipDummyHandle)
```

```python
int targetDummyHandle=simIK.getTargetDummy(int environmentHandle,int tipDummyHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment
- **tipDummyHandle** (int): the handle of the tip dummy

#### Return values:

- **targetDummyHandle** (int): the handle of the target dummy. Is -1 if no target dummy is associated with the tip dummy

#### See also:

[createEnvironment](#createEnvironment), [setTargetDummy](#setTargetDummy), [getObjectHandle](#getObjectHandle)

### <a name="handleGroup" id="handleGroup"></a> <a name="simIK.handleGroup" id="simIK.handleGroup"></a> simIK.handleGroup

Handles (i.e. computes/resolves) an IK group. Convenience function for simIK.handleIkGroups(ikEnv,{ikGroupHandle},..)

#### Synopsis:

```lua
int result,int flags,float[2] precision=simIK.handleGroup(int environmentHandle,int ikGroupHandle,map options={})
```

```python
int result,int flags,list precision=simIK.handleGroup(int environmentHandle,int ikGroupHandle,map options=None)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment
- **ikGroupHandle** (int): the handle of the IK group
- **options** (map, default: {}): options:
  - options.syncWorlds: if true, then calculation will be preceeded by simIK.syncFromSim and followed by simIK.syncToSim
  - options.allowError: if true, and options.syncWorlds is true too, then calculation result will be applied to the scene, even if tip/target pairs are not within tolerance
  - options.debug: bit0 is set, then a visual representation of the IK group will be made
  - options.callback: a callback function that allows to inspect and manipulate the Jacobian. It also allows to directly perform joint valiation calculations while skipping internal computations:
  - outData=callbackFunction(inData)
    - inData.jacobian: a Matrix object representing the Jacobian
    - inData.e: a Vector object representing the error vector
    - inData.rows: a table describing the jacobian rows
    - inData.cols: a table describing the jacobian cols
    - outData.jacobian: an optional Matrix object representing the Jacobian to use in subsequent calculations. If not provided, then the original inData.jacobian is used
    - outData.e: an optional Vector object representing the error vector to use in subsequent calculations. If not provided, then the original inData.e is used
    - outData.dq: an optional Vector object representing the joint variation to apply. If provided, then internal Jacobian calculations (i.e. Jacobian pseudo-inverse) will be skipped)
    - outData.jacobianPinv: an optional Matrix object representing the jacobian pseudo-inverse. If provided (and dq is not provided), then internal Jacobian calculations will be skipped and following executed instead: dq=outData.jacobianPinv*e)

#### Return values:

- **result** (int): simIK.result_success, if successful
- **reason** (int): bit-coded flags: simIK.calc_notperformed, simIK.calc_cannotinvert, simIK.calc_notwithintolerance, simIK.calc_stepstoobig, simIK.calc_limithit
- **precision** (float[2]): 2 values indicating the largest linear and angular distance between all tip-target pairs

#### See also:

[handleGroups](#handleGroups), [createEnvironment](#createEnvironment), [computeJacobian](#computeJacobian), [findConfigs](#findConfigs)

### <a name="handleGroups" id="handleGroups"></a> <a name="simIK.handleGroups" id="simIK.handleGroups"></a> simIK.handleGroups

Handles (i.e. computes/resolves) one or several IK groups

#### Synopsis:

```lua
int result,int flags,float[2] precision=simIK.handleGroups(int environmentHandle,int[] ikGroupHandles,map options={})
```

```python
int result,int flags,list precision=simIK.handleGroups(int environmentHandle,int[] ikGroupHandles,map options=None)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment
- **ikGroupHandles** (int[]): the handles of the IK groups. If one handle is specified, then a simple IK calculation is performed. If more than one handle is specified, then a projected IK computation is performed, where each group calculation result is projected into the previous group's null space, i.e. dq=dq0+NP0*(dq1+NP1*(dq2+NP2*(...))), where dqi is the result of i-th task's IK (i.e. typically J#*e), and NPi is the i-th task's I-J#*J
- **options** (map, default: {}): options:
  - options.syncWorlds: if true, then calculation will be preceeded by simIK.syncFromSim and followed by simIK.syncToSim
  - options.allowError: if true, and options.syncWorlds is true too, then calculation result will be applied to the scene, even if tip/target pairs are not within tolerance
  - options.debug: if bit0 is set, then a visual representation of the IK groups will be made
  - options.callback: a callback function that allows to inspect and manipulate the Jacobian. It also allows to directly perform joint valiation calculations while skipping internal computations:
  - outData=callbackFunction(inData)
    - inData.jacobian: a Matrix object representing the Jacobian
    - inData.e: a Vector object representing the error vector
    - inData.rows: a table describing the jacobian rows
    - inData.cols: a table describing the jacobian cols
    - outData.jacobian: an optional Matrix object representing the Jacobian to use in subsequent calculations. If not provided, then the original inData.jacobian is used
    - outData.e: an optional Vector object representing the error vector to use in subsequent calculations. If not provided, then the original inData.e is used
    - outData.dq: an optional Vector object representing the joint variation to apply. If provided, then internal Jacobian calculations (i.e. Jacobian pseudo-inverse) will be skipped)
    - outData.jacobianPinv: an optional Matrix object representing the jacobian pseudo-inverse. If provided (and dq is not provided), then internal Jacobian calculations will be skipped and following executed instead: dq=outData.jacobianPinv*e)

#### Return values:

- **result** (int): simIK.result_success, if successful
- **reason** (int): bit-coded flags: simIK.calc_notperformed, simIK.calc_cannotinvert, simIK.calc_notwithintolerance, simIK.calc_stepstoobig, simIK.calc_limithit
- **precision** (float[2]): 2 values indicating the largest linear and angular distance between all tip-target pairs

#### See also:

[handleGroup](#handleGroup), [createEnvironment](#createEnvironment), [computeJacobian](#computeJacobian), [findConfigs](#findConfigs)

### <a name="load" id="load"></a> <a name="simIK.load" id="simIK.load"></a> simIK.load

Loads kinematic content previously exported in the CoppeliaSim application. Make sure that the environment is empty before calling this function.

#### Synopsis:

```lua
simIK.load(int environmentHandle,buffer data)
```

```python
simIK.load(int environmentHandle,bytes data)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **data** (buffer): a buffer with the kinematic content.

#### Return values:

No return values

#### See also:

[save](#save), [createEnvironment](#createEnvironment), [eraseEnvironment](#eraseEnvironment)

### <a name="save" id="save"></a> <a name="simIK.save" id="simIK.save"></a> simIK.save

Saves the kinematic content of an IK environment.

#### Synopsis:

```lua
buffer data=simIK.save(int environmentHandle)
```

```python
bytes data=simIK.save(int environmentHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.

#### Return values:

- **data** (buffer): the exported data.

#### See also:

[load](#load)

### <a name="setElementBase" id="setElementBase"></a> <a name="simIK.setElementBase" id="simIK.setElementBase"></a> simIK.setElementBase

Sets the base object of an IK element.

#### Synopsis:

```lua
simIK.setElementBase(int environmentHandle,int ikGroupHandle,int elementHandle,int baseHandle,int constraintsBaseHandle=-1)
```

```python
simIK.setElementBase(int environmentHandle,int ikGroupHandle,int elementHandle,int baseHandle,int constraintsBaseHandle=-1)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **baseHandle** (int): the handle of the base object, or -1 if the world is the base.
- **constraintsBaseHandle** (int, default: -1): the handle of the constraints base object, relative to which the constraints are specified. Set to -1 to have the constraints relative to the base object.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getElementBase](#getElementBase), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="setElementConstraints" id="setElementConstraints"></a> <a name="simIK.setElementConstraints" id="simIK.setElementConstraints"></a> simIK.setElementConstraints

Sets the constraints of an IK element.

#### Synopsis:

```lua
simIK.setElementConstraints(int environmentHandle,int ikGroupHandle,int elementHandle,int constraints)
```

```python
simIK.setElementConstraints(int environmentHandle,int ikGroupHandle,int elementHandle,int constraints)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **constraints** (int): the constraints. Bit-combine following: simIK.constraint_x, simIK.constraint_y, simIK.constraint_z, simIK.constraint_alpha_beta, simIK.constraint_gamma (simIK.constraint_gamma should only be set if simIK.constraint_alpha_beta is also set). For convenience we also have simIK.constraint_position=simIK.constraint_x|simIK.constraint_y|simIK.constraint_z, simIK.constraint_orientation=simIK.constraint_alpha_beta|simIK.constraint_gamma, and simIK.constraint_pose=simIK.constraint_position|simIK.constraint_orientation.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getElementConstraints](#getElementConstraints), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="setElementFlags" id="setElementFlags"></a> <a name="simIK.setElementFlags" id="simIK.setElementFlags"></a> simIK.setElementFlags

Sets various flags of an IK element.

#### Synopsis:

```lua
simIK.setElementFlags(int environmentHandle,int ikGroupHandle,int elementHandle,int flags)
```

```python
simIK.setElementFlags(int environmentHandle,int ikGroupHandle,int elementHandle,int flags)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **flags** (int): bit-coded flags: bit0 set (1)=the enabled state of the IK element.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getElementFlags](#getElementFlags), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="setElementPrecision" id="setElementPrecision"></a> <a name="simIK.setElementPrecision" id="simIK.setElementPrecision"></a> simIK.setElementPrecision

Sets the desired precision of an IK element.

#### Synopsis:

```lua
simIK.setElementPrecision(int environmentHandle,int ikGroupHandle,int elementHandle,float[2] precision)
```

```python
simIK.setElementPrecision(int environmentHandle,int ikGroupHandle,int elementHandle,list precision)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **precision** (float[2]): a table with two values for the linear and angular precision.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getElementPrecision](#getElementPrecision), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="setElementWeights" id="setElementWeights"></a> <a name="simIK.setElementWeights" id="simIK.setElementWeights"></a> simIK.setElementWeights

Sets the desired linear and angular resolution weights of an IK element.

#### Synopsis:

```lua
simIK.setElementWeights(int environmentHandle,int ikGroupHandle,int elementHandle,float[3] weights)
```

```python
simIK.setElementWeights(int environmentHandle,int ikGroupHandle,int elementHandle,list weights)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **elementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy+simIK.handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **weights** (float[3]): a table with three values for the linear, angular and overall resolution weight.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getElementWeights](#getElementWeights), [getGroupHandle](#getGroupHandle), [getObjectHandle](#getObjectHandle)

### <a name="setGroupCalculation" id="setGroupCalculation"></a> <a name="simIK.setGroupCalculation" id="simIK.setGroupCalculation"></a> simIK.setGroupCalculation

Sets calculation properties for an IK group.

#### Synopsis:

```lua
simIK.setGroupCalculation(int environmentHandle,int ikGroupHandle,int method,float damping,float maxIterations)
```

```python
simIK.setGroupCalculation(int environmentHandle,int ikGroupHandle,int method,float damping,float maxIterations)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **method** (int): the resolution method. Possible values are simIK.method_pseudo_inverse (features a tiny bit of hard-coded damping), simIK.method_undamped_pseudo_inverse, simIK.method_damped_least_squares and simIK.method_jacobian_transpose.
- **damping** (float): the damping, in case the resolution method is simIK.method_damped_least_squares.
- **maxIterations** (float): the maximum number of iterations.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getGroupCalculation](#getGroupCalculation), [getGroupHandle](#getGroupHandle)

### <a name="setGroupFlags" id="setGroupFlags"></a> <a name="simIK.setGroupFlags" id="simIK.setGroupFlags"></a> simIK.setGroupFlags

Sets flags of an IK group.

#### Synopsis:

```lua
simIK.setGroupFlags(int environmentHandle,int ikGroupHandle,int flags)
```

```python
simIK.setGroupFlags(int environmentHandle,int ikGroupHandle,int flags)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **ikGroupHandle** (int): the handle of the IK group.
- **flags** (int): the flags of the IK group, bit-coded:
  - simIK.group_enabled: the group's enabled state
  - simIK.group_ignoremaxsteps: the joints' max step sizes are ignored. Otherwise, when detected, an interpolation factor (originally at 1.0) is successively divided by 2.0 until the max. step sizes are respected
  - simIK.group_restoreonbadlintol: the IK world state is left untouched if the linear precision is not reached
  - simIK.group_restoreonbadangtol: the IK world state is left untouched if the angular precision is not reached
  - simIK.group_avoidlimits: joint limits are actively avoided
  - simIK.group_stoponlimithit: calculation ends when a joint limit is hit

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getGroupFlags](#getGroupFlags), [getGroupHandle](#getGroupHandle)

### <a name="setJointDependency" id="setJointDependency"></a> <a name="simIK.setJointDependency" id="simIK.setJointDependency"></a> simIK.setJointDependency

Sets information about a possible dependent joint.

#### Synopsis:

```lua
simIK.setJointDependency(int environmentHandle,int jointHandle,int depJointHandle,float offset=0,float mult=1,func callback=nil)
```

```python
simIK.setJointDependency(int environmentHandle,int jointHandle,int depJointHandle,float offset=0,float mult=1,string callback=None)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **depJointHandle** (int): the handle of the joint, this joint is dependent of. -1 to disable.
- **offset** (float, default: 0): the offset. We have linear/angular joint position = dependency linear/angular joint position * mult + offset
- **mult** (float, default: 1): the multiplication factor. We have linear/angular joint position = dependency linear/angular joint position * mult + offset
- **callback** (func, default: nil): an optional callback that allows to provide a custom dependency function:
  - float result=callbackFunction(int ikEnv,int slaveJointHandle,float masterJointPosition)

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointDependency](#getJointDependency), [getObjectHandle](#getObjectHandle)

### <a name="setJointInterval" id="setJointInterval"></a> <a name="simIK.setJointInterval" id="simIK.setJointInterval"></a> simIK.setJointInterval

Sets the joint limits.

#### Synopsis:

```lua
simIK.setJointInterval(int environmentHandle,int jointHandle,bool cyclic,float[2] interval={})
```

```python
simIK.setJointInterval(int environmentHandle,int jointHandle,bool cyclic,list interval=[])
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **cyclic** (bool): whether the joint is cyclic (has no limits). Only revolute joints can be cyclic.
- **interval** (float[2], default: {}): a table with two values: the joint lower limit, and the joint range (i.e. joint upper limit = joint lower limit + joint range)

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointInterval](#getJointInterval), [getObjectHandle](#getObjectHandle)

### <a name="setJointLimitMargin" id="setJointLimitMargin"></a> <a name="simIK.setJointLimitMargin" id="simIK.setJointLimitMargin"></a> simIK.setJointLimitMargin

Sets the limit margin of a joint, i.e. the threshold that will be used to counteract on joint limit violation during IK resolution, if the appropriate IK group flag was set

#### Synopsis:

```lua
simIK.setJointLimitMargin(int environmentHandle,int jointHandle,float margin)
```

```python
simIK.setJointLimitMargin(int environmentHandle,int jointHandle,float margin)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **margin** (float): the margin

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointLimitMargin](#getJointLimitMargin), [getGroupFlags](#getGroupFlags)

### <a name="setJointMaxStepSize" id="setJointMaxStepSize"></a> <a name="simIK.setJointMaxStepSize" id="simIK.setJointMaxStepSize"></a> simIK.setJointMaxStepSize

Sets the maximum step size of a joint.

#### Synopsis:

```lua
simIK.setJointMaxStepSize(int environmentHandle,int jointHandle,float stepSize)
```

```python
simIK.setJointMaxStepSize(int environmentHandle,int jointHandle,float stepSize)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **stepSize** (float): the maximum step size.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointMaxStepSize](#getJointMaxStepSize), [getObjectHandle](#getObjectHandle)

### <a name="setJointMode" id="setJointMode"></a> <a name="simIK.setJointMode" id="simIK.setJointMode"></a> simIK.setJointMode

Sets the joint mode.

#### Synopsis:

```lua
simIK.setJointMode(int environmentHandle,int jointHandle,int jointMode)
```

```python
simIK.setJointMode(int environmentHandle,int jointHandle,int jointMode)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **jointMode** (int): the joint mode. Allowed values are: simIK.jointmode_passive, simIK.jointmode_ik

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointMode](#getJointMode), [getObjectHandle](#getObjectHandle)

### <a name="setJointPosition" id="setJointPosition"></a> <a name="simIK.setJointPosition" id="simIK.setJointPosition"></a> simIK.setJointPosition

Sets the position (linear or angular) of a joint.

#### Synopsis:

```lua
simIK.setJointPosition(int environmentHandle,int jointHandle,float position)
```

```python
simIK.setJointPosition(int environmentHandle,int jointHandle,float position)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **position** (float): the position.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointPosition](#getJointPosition), [setSphericalJointMatrix](#setSphericalJointMatrix), [setSphericalJointRotation](#setSphericalJointRotation), [getObjectHandle](#getObjectHandle)

### <a name="setJointScrewLead" id="setJointScrewLead"></a> <a name="simIK.setJointScrewLead" id="simIK.setJointScrewLead"></a> simIK.setJointScrewLead

Sets the screw lead, in case of a revolute joint.

#### Synopsis:

```lua
simIK.setJointScrewLead(int environmentHandle,int jointHandle,float lead)
```

```python
simIK.setJointScrewLead(int environmentHandle,int jointHandle,float lead)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **lead** (float): the screw lead of the joint. A lead value of zero represents a revolute joint, a value different from zero represents a screw.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointScrewLead](#getJointScrewLead), [getObjectHandle](#getObjectHandle)

### <a name="setJointWeight" id="setJointWeight"></a> <a name="simIK.setJointWeight" id="simIK.setJointWeight"></a> simIK.setJointWeight

Sets the IK weight of a joint, i.e. the weight it has during IK resolution.

#### Synopsis:

```lua
simIK.setJointWeight(int environmentHandle,int jointHandle,float weight)
```

```python
simIK.setJointWeight(int environmentHandle,int jointHandle,float weight)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **weight** (float): the IK weight.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointWeight](#getJointWeight), [getObjectHandle](#getObjectHandle)

### <a name="setLinkedDummy" id="setLinkedDummy"></a> <a name="simIK.setLinkedDummy" id="simIK.setLinkedDummy"></a> simIK.setLinkedDummy

Deprecated. Use [setTargetDummy instead](#setTargetDummy)

#### Synopsis:

```lua
simIK.setLinkedDummy(int environmentHandle,int dummyHandle,int linkedDummyHandle)
```

```python
simIK.setLinkedDummy(int environmentHandle,int dummyHandle,int linkedDummyHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **dummyHandle** (int): the handle of the dummy object.
- **linkedDummyHandle** (int): the handle of the dummy object to be linked, or -1 to detach this dummy object from a linked dummy object.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getLinkedDummy](#getLinkedDummy), [getObjectHandle](#getObjectHandle)

### <a name="setObjectMatrix" id="setObjectMatrix"></a> <a name="simIK.setObjectMatrix" id="simIK.setObjectMatrix"></a> simIK.setObjectMatrix

Sets the transformation matrix of an object. If the object is a joint object, the matrix does not include the joint's intrinsic transformation.

#### Synopsis:

```lua
simIK.setObjectMatrix(int environmentHandle,int objectHandle,float[12] matrix,int relativeToObjectHandle=simIK.handle_world)
```

```python
simIK.setObjectMatrix(int environmentHandle,int objectHandle,list matrix,int relativeToObjectHandle=simIK.handle_world)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.
- **matrix** (float[12]): a table of 12 values representing the transformation matrix (the last row of the 4x4 matrix (0,0,0,1) is omitted)
- **relativeToObjectHandle** (int, default: simIK.handle_world): the handle of an object relative to which the matrix is expressed. Otherwise, specify simIK.handle_world if you specify the absolute matrix, or simIK.handle_parent if you specify the matrix relative to the parent object.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getObjectMatrix](#getObjectMatrix), [setObjectPose](#setObjectPose), [setObjectTransformation](#setObjectTransformation), [setJointPosition](#setJointPosition), [getObjectHandle](#getObjectHandle)

### <a name="setObjectParent" id="setObjectParent"></a> <a name="simIK.setObjectParent" id="simIK.setObjectParent"></a> simIK.setObjectParent

Sets the parent of an object.

#### Synopsis:

```lua
simIK.setObjectParent(int environmentHandle,int objectHandle,int parentObjectHandle, bool keepInPlace=true)
```

```python
simIK.setObjectParent(int environmentHandle,int objectHandle,int parentObjectHandle, bool keepInPlace=True)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.
- **parentObjectHandle** (int): the desired parent object, set -1 for no parent.
- **keepInPlace** (bool, default: true): if true, the object will stay in place, otherwise, it will keep its local transformation.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getObjectParent](#getObjectParent), [getObjectHandle](#getObjectHandle)

### <a name="setObjectPose" id="setObjectPose"></a> <a name="simIK.setObjectPose" id="simIK.setObjectPose"></a> simIK.setObjectPose

Sets the pose (position and quaternion) of an object. If the object is a joint object, the pose does not include the joint's intrinsic transformation.

#### Synopsis:

```lua
simIK.setObjectPose(int environmentHandle,int objectHandle,float[7] pose,int relativeToObjectHandle=simIK.handle_world)
```

```python
simIK.setObjectPose(int environmentHandle,int objectHandle,list pose,int relativeToObjectHandle=simIK.handle_world)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.
- **pose** (float[7]): the position and quaternion (x,y,z,qx,qy,qz,qw).
- **relativeToObjectHandle** (int, default: simIK.handle_world): the handle of an object relative to which the pose is expressed. Otherwise, specify simIK.handle_world if you specify the absolute pose, or simIK.handle_parent if you specify the pose relative to the parent object.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getObjectPose](#getObjectPose), [setObjectTransformation](#setObjectTransformation), [setObjectMatrix](#setObjectMatrix), [setJointPosition](#setJointPosition), [getObjectHandle](#getObjectHandle)

### <a name="setObjectTransformation" id="setObjectTransformation"></a> <a name="simIK.setObjectTransformation" id="simIK.setObjectTransformation"></a> simIK.setObjectTransformation

Sets the transformation (position and quaternion/Euler angles) of an object. If the object is a joint object, the transformation does not include the joint's intrinsic transformation.

#### Synopsis:

```lua
simIK.setObjectTransformation(int environmentHandle,int objectHandle,float[3] position,float[] eulerOrQuaternion,int relativeToObjectHandle=simIK.handle_world)
```

```python
simIK.setObjectTransformation(int environmentHandle,int objectHandle,list position,list eulerOrQuaternion,int relativeToObjectHandle=simIK.handle_world)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **objectHandle** (int): the handle of the object.
- **position** (float[3]): the position component of the transformation.
- **eulerOrQuaternion** (float[]): the orientation component of the transformation. If 3 values are specified, then they will be interpreted as Euler angles (alpha,beta,gamma). If 4 values are specified, they will be interpreted as quaternion values (x,y,z,w).
- **relativeToObjectHandle** (int, default: simIK.handle_world): the handle of an object relative to which the transformation is expressed. Otherwise, specify simIK.handle_world if you specify the absolute transformation, or simIK.handle_parent if you specify the transformation relative to the parent object.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getObjectTransformation](#getObjectTransformation), [setObjectPose](#setObjectPose), [setObjectMatrix](#setObjectMatrix), [setJointPosition](#setJointPosition), [getObjectHandle](#getObjectHandle)

### <a name="setSphericalJointMatrix" id="setSphericalJointMatrix"></a> <a name="simIK.setSphericalJointMatrix" id="simIK.setSphericalJointMatrix"></a> simIK.setSphericalJointMatrix

Sets the rotation transformation matrix of a spherical joint.

#### Synopsis:

```lua
simIK.setSphericalJointMatrix(int environmentHandle,int jointHandle,float[12] matrix)
```

```python
simIK.setSphericalJointMatrix(int environmentHandle,int jointHandle,list matrix)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **matrix** (float[12]): a table of 12 values representing the transformation matrix (the last row of the 4x4 matrix (0,0,0,1) is omitted). The translation part in the matrix will be ignored.

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointMatrix](#getJointMatrix), [setJointPosition](#setJointPosition), [setSphericalJointRotation](#setSphericalJointRotation), [getObjectHandle](#getObjectHandle)

### <a name="setSphericalJointRotation" id="setSphericalJointRotation"></a> <a name="simIK.setSphericalJointRotation" id="simIK.setSphericalJointRotation"></a> simIK.setSphericalJointRotation

Sets the rotation transformation of a spherical joint.

#### Synopsis:

```lua
simIK.setSphericalJointRotation(int environmentHandle,int jointHandle,float[] eulerOrQuaternion)
```

```python
simIK.setSphericalJointRotation(int environmentHandle,int jointHandle,list eulerOrQuaternion)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment.
- **jointHandle** (int): the handle of the joint.
- **eulerOrQuaternion** (float[]): the rotation transformation to apply. If 3 values are specified, they will be interpreted as Euler angles (alpha,beta,gamma). If 4 values are specified, they will be interpreted as quaternion values (x,y,z,w).

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getJointTransformation](#getJointTransformation), [setSphericalJointMatrix](#setSphericalJointMatrix), [setJointPosition](#setJointPosition), [getObjectHandle](#getObjectHandle)

### <a name="setTargetDummy" id="setTargetDummy"></a> <a name="simIK.setTargetDummy" id="simIK.setTargetDummy"></a> simIK.setTargetDummy

Associates a tip dummy with a target dummy, or removes that association. If the tip dummy is already associated with another target dummy, then first remove that association before setting another one

#### Synopsis:

```lua
simIK.setTargetDummy(int environmentHandle,int tipDummyHandle,int targetDummyHandle)
```

```python
simIK.setTargetDummy(int environmentHandle,int tipDummyHandle,int targetDummyHandle)
```

#### Parameters:

- **environmentHandle** (int): the handle of the environment
- **tipDummyHandle** (int): the handle of the tip dummy
- **targetdDummyHandle** (int): the handle of the new target dummy, or -1 to remove a previous target dummy association from the tip dummy

#### Return values:

No return values

#### See also:

[createEnvironment](#createEnvironment), [getTargetDummy](#getTargetDummy), [getObjectHandle](#getObjectHandle)

### <a name="syncFromSim" id="syncFromSim"></a> <a name="simIK.syncFromSim" id="simIK.syncFromSim"></a> simIK.syncFromSim

Convenience function to apply the scene state to its ik environment counterpart. Use together with [addElementFromScene](#addElementFromScene).

#### Synopsis:

```lua
simIK.syncFromSim(int environmentHandle,int[] ikGroups)
```

```python
simIK.syncFromSim(int environmentHandle,int[] ikGroups)
```

#### Parameters:

- **environmentHandle** (int): the handle of the IK environment
- **ikGroups** (int[]): the handle of one or several IK groups

#### Return values:

No return values

#### See also:

[addElementFromScene](#addElementFromScene), [syncToSim](#syncToSim)

### <a name="syncToSim" id="syncToSim"></a> <a name="simIK.syncToSim" id="simIK.syncToSim"></a> simIK.syncToSim

Convenience function to apply inverse kinematic values computed in the IK world, to the scene. Use together with [addElementFromScene](#addElementFromScene).

#### Synopsis:

```lua
simIK.syncToSim(int environmentHandle,int[] ikGroups)
```

```python
simIK.syncToSim(int environmentHandle,int[] ikGroups)
```

#### Parameters:

- **environmentHandle** (int): the handle of the IK environment
- **ikGroups** (int[]): the handles of one or several IK groups

#### Return values:

No return values

#### See also:

[addElementFromScene](#addElementFromScene), [syncFromSim](#syncFromSim)
