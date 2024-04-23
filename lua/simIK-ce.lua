local codeEditorInfos = [[
int elementHandle = simIK.addElement(int environmentHandle, int ikGroupHandle, int tipDummyHandle)
int ikElement, map simToIkMap, map ikToSimMap = simIK.addElementFromScene(int environmentHandle, int ikGroup, int baseHandle, int tipHandle, int targetHandle, int constraints)
float[] jacobian, float[] errorVector = simIK.computeGroupJacobian(int environmentHandle, int ikGroupHandle)
float[] jacobian, float[] errorVector = simIK.computeJacobian(int environmentHandle, int baseObject, int lastJoint, int constraints, float[7..12] tipMatrix, float[7..12] targetMatrix=nil, float[7..12] constrBaseMatrix=nil)
int debugObject = simIK.createDebugOverlay(int environmentHandle, int tipHandle, int baseHandle=-1)
int dummyHandle = simIK.createDummy(int environmentHandle, string dummyName='')
int environmentHandle = simIK.createEnvironment(int flags=0)
int ikGroupHandle = simIK.createGroup(int environmentHandle, string ikGroupName='')
int jointHandle = simIK.createJoint(int environmentHandle, int jointType, string jointName='')
bool result = simIK.doesGroupExist(int environmentHandle, string ikGroupName)
bool result = simIK.doesObjectExist(int environmentHandle, string objectName)
int duplicateEnvHandle = simIK.duplicateEnvironment(int environmentHandle)
simIK.eraseDebugOverlay(int debugObject)
simIK.eraseEnvironment(int environmentHandle)
simIK.eraseObject(int environmentHandle, int objectHandle)
any[] configs = simIK.findConfigs(int envHandle, int ikGroupHandle, int[] jointHandles, map params={}, any[] configs={})
float[] path = simIK.generatePath(int environmentHandle, int ikGroupHandle, int[] jointHandles, int tipHandle, int pathPointCount, func validationCallback=nil, any auxData=nil)
float[] configs = simIK.getAlternateConfigs(int environmentHandle, int[] jointHandles, float[] lowLimits=nil, float[] ranges=nil)
int baseHandle, int constraintsBaseHandle = simIK.getElementBase(int environmentHandle, int ikGroupHandle, int elementHandle)
int constraints = simIK.getElementConstraints(int environmentHandle, int ikGroupHandle, int elementHandle)
int flags = simIK.getElementFlags(int environmentHandle, int ikGroupHandle, int elementHandle)
float[2] precision = simIK.getElementPrecision(int environmentHandle, int ikGroupHandle, int elementHandle)
float[2] weights = simIK.getElementWeights(int environmentHandle, int ikGroupHandle, int elementHandle)
string description = simIK.getFailureDescription(int reason)
int method, float damping, int maxIterations = simIK.getGroupCalculation(int environmentHandle, int ikGroupHandle)
int flags = simIK.getGroupFlags(int environmentHandle, int ikGroupHandle)
int ikGroupHandle = simIK.getGroupHandle(int environmentHandle, string ikGroupName)
int[] jointHandles, float[] underOrOvershots = simIK.getGroupJointLimitHits(int environmentHandle, int ikGroupHandle)
int[] jointHandles = simIK.getGroupJoints(int environmentHandle, int ikGroupHandle)
int depJointHandle, float offset, float mult = simIK.getJointDependency(int environmentHandle, int jointHandle)
bool cyclic, float[2] interval = simIK.getJointInterval(int environmentHandle, int jointHandle)
float margin = simIK.getJointLimitMargin(int environmentHandle, int jointHandle)
float[12] matrix = simIK.getJointMatrix(int environmentHandle, int jointHandle)
float stepSize = simIK.getJointMaxStepSize(int environmentHandle, int jointHandle)
int jointMode = simIK.getJointMode(int environmentHandle, int jointHandle)
float position = simIK.getJointPosition(int environmentHandle, int jointHandle)
float lead = simIK.getJointScrewLead(int environmentHandle, int jointHandle)
float[3] position, float[4] quaternion, float[3] euler = simIK.getJointTransformation(int environmentHandle, int jointHandle)
int jointType = simIK.getJointType(int environmentHandle, int jointHandle)
float weight = simIK.getJointWeight(int environmentHandle, int jointHandle)
int objectHandle = simIK.getObjectHandle(int environmentHandle, string objectName)
float[12] matrix = simIK.getObjectMatrix(int environmentHandle, int objectHandle, int relativeToObjectHandle=simIK.handle_world)
int parentObjectHandle = simIK.getObjectParent(int environmentHandle, int objectHandle)
float[7] pose = simIK.getObjectPose(int environmentHandle, int objectHandle, int relativeToObjectHandle=simIK.handle_world)
float[3] position, float[4] quaternion, float[3] euler = simIK.getObjectTransformation(int environmentHandle, int objectHandle, int relativeToObjectHandle=simIK.handle_world)
int objectType = simIK.getObjectType(int environmentHandle, int objectHandle)
int objectHandle, string objectName, bool isJoint, int jointType = simIK.getObjects(int environmentHandle, int index)
int targetDummyHandle = simIK.getTargetDummy(int environmentHandle, int dummyHandle)
int success, int flags, float[2] precision = simIK.handleGroup(int environmentHandle, int ikGroup, map options={})
int success, int flags, float[2] precision = simIK.handleGroups(int environmentHandle, int[] ikGroups, map options={})
simIK.load(int environmentHandle, string data)
string data = simIK.save(int environmentHandle)
simIK.setElementBase(int environmentHandle, int ikGroupHandle, int elementHandle, int baseHandle, int constraintsBaseHandle=-1)
simIK.setElementConstraints(int environmentHandle, int ikGroupHandle, int elementHandle, int constraints)
simIK.setElementFlags(int environmentHandle, int ikGroupHandle, int elementHandle, int flags)
simIK.setElementPrecision(int environmentHandle, int ikGroupHandle, int elementHandle, float[2] precision)
simIK.setElementWeights(int environmentHandle, int ikGroupHandle, int elementHandle, float[2] weights)
simIK.setGroupCalculation(int environmentHandle, int ikGroupHandle, int method, float damping, int maxIterations)
simIK.setGroupFlags(int environmentHandle, int ikGroupHandle, int flags)
simIK.setJointDependency(int environmentHandle, int jointHandle, int masterJointHandle, float offset=0.0, float mult=1.0, func callback=nil)
simIK.setJointInterval(int environmentHandle, int jointHandle, bool cyclic, float[2] interval={})
simIK.setJointLimitMargin(int environmentHandle, int jointHandle, float margin)
simIK.setJointMaxStepSize(int environmentHandle, int jointHandle, float stepSize)
simIK.setJointMode(int environmentHandle, int jointHandle, int jointMode)
simIK.setJointPosition(int environmentHandle, int jointHandle, float position)
simIK.setJointScrewLead(int environmentHandle, int jointHandle, float lead)
simIK.setJointWeight(int environmentHandle, int jointHandle, float weight)
simIK.setObjectMatrix(int environmentHandle, int objectHandle, float[12] matrix, int relativeToObjectHandle=simIK.handle_world)
simIK.setObjectParent(int environmentHandle, int objectHandle, int parentObjectHandle, bool keepInPlace=true)
simIK.setObjectPose(int environmentHandle, int objectHandle, float[7] pose, int relativeToObjectHandle=simIK.handle_world)
simIK.setObjectTransformation(int environmentHandle, int objectHandle, float[3] position, float[] eulerOrQuaternion, int relativeToObjectHandle=simIK.handle_world)
simIK.setSphericalJointMatrix(int environmentHandle, int jointHandle, float[12] matrix)
simIK.setSphericalJointRotation(int environmentHandle, int jointHandle, float[] eulerOrQuaternion)
simIK.setTargetDummy(int environmentHandle, int dummyHandle, int targetDummyHandle)
simIK.syncFromSim(int environmentHandle, int[] ikGroups)
simIK.syncToSim(int environmentHandle, int[] ikGroups)
simIK.calc_cannotinvert
simIK.calc_invalidcallbackdata
simIK.calc_limithit
simIK.calc_notperformed
simIK.calc_notwithintolerance
simIK.calc_stepstoobig
simIK.constraint_alpha_beta
simIK.constraint_gamma
simIK.constraint_orientation
simIK.constraint_pose
simIK.constraint_position
simIK.constraint_x
simIK.constraint_y
simIK.constraint_z
simIK.group_avoidlimits
simIK.group_enabled
simIK.group_ignoremaxsteps
simIK.group_restoreonbadangtol
simIK.group_restoreonbadlintol
simIK.group_stoponlimithit
simIK.handle_all
simIK.handle_parent
simIK.handle_world
simIK.handleflag_tipdummy
simIK.jointmode_ik
simIK.jointmode_passive
simIK.jointtype_prismatic
simIK.jointtype_revolute
simIK.jointtype_spherical
simIK.method_damped_least_squares
simIK.method_jacobian_transpose
simIK.method_pseudo_inverse
simIK.method_undamped_pseudo_inverse
simIK.objecttype_dummy
simIK.objecttype_joint
simIK.result_fail
simIK.result_not_performed
simIK.result_success
]]

registerCodeEditorInfos("simIK", codeEditorInfos)
