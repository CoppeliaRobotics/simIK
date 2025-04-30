local simIK = loadPlugin('simIK')

_S.simIKgetObjectTransformation = simIK.getObjectTransformation
function simIK.getObjectTransformation(env, handle, rel)
    rel = rel or simIK.handle_world
    return _S.simIKgetObjectTransformation(env, handle, rel)
end

_S.simIKsetObjectTransformation = simIK.setObjectTransformation
function simIK.setObjectTransformation(env, handle, pos, orient, rel)
    if type(pos) == 'table' then
        rel = rel or simIK.handle_world
        return _S.simIKsetObjectTransformation(env, handle, pos, orient, rel)
    else
        return _S.simIKsetObjectTransformation(env, handle, orient, rel, pos) -- deprecated
    end
end

_S.simIKgetObjectMatrix = simIK.getObjectMatrix
function simIK.getObjectMatrix(env, handle, rel)
    rel = rel or simIK.handle_world
    return _S.simIKgetObjectMatrix(env, handle, rel)
end

_S.simIKsetObjectMatrix = simIK.setObjectMatrix
function simIK.setObjectMatrix(env, handle, matr, rel)
    if type(matr) == 'table' then
        rel = rel or simIK.handle_world
        return _S.simIKsetObjectMatrix(env, handle, matr, rel)
    else
        return _S.simIKsetObjectMatrix(env, handle, rel, matr) -- deprecated
    end
end

function simIK.getObjectPose(ikEnv, obj, relObj)
    local pos, quat = simIK.getObjectTransformation(ikEnv, obj, relObj)
    return {pos[1], pos[2], pos[3], quat[1], quat[2], quat[3], quat[4]}
end

function simIK.setObjectPose(ikEnv, obj, pose, relObj)
    if type(pose) == 'table' then
        simIK.setObjectTransformation(
            ikEnv, obj, {pose[1], pose[2], pose[3]}, {pose[4], pose[5], pose[6], pose[7]}, relObj
        )
    else
        simIK.setObjectTransformation(
            ikEnv, obj, {relObj[1], relObj[2], relObj[3]},
            {relObj[4], relObj[5], relObj[6], relObj[7]}, pose
        ) -- deprecated
    end
end

function _S.simIKLoopThroughAltConfigSolutions(ikEnvironment, jointHandles, desiredPose, confS, x,
                                               index)
    if index > #jointHandles then
        return {sim.unpackDoubleTable(sim.packDoubleTable(confS))} -- copy the table
    else
        local c = {}
        for i = 1, #jointHandles, 1 do c[i] = confS[i] end
        local solutions = {}
        while c[index] <= x[index][2] do
            local s = _S.simIKLoopThroughAltConfigSolutions(
                          ikEnvironment, jointHandles, desiredPose, c, x, index + 1)
            for i = 1, #s, 1 do solutions[#solutions + 1] = s[i] end
            c[index] = c[index] + math.pi * 2
        end
        return solutions
    end
end

function simIK.getAlternateConfigs(...)
    local ikEnv, jointHandles, lowLimits, ranges = checkargs({
        {type = 'int'},
        {type = 'table', size = '1..*', item_type = 'int'},
        {type = 'table', size = '1..*', item_type = 'float', default = NIL, nullable = true},
        {type = 'table', size = '1..*', item_type = 'float', default = NIL, nullable = true},
    }, ...)

    local dof = #jointHandles
    if (lowLimits and dof ~= #lowLimits) or (ranges and dof ~= #ranges) then
        error("Bad table size.")
    end

    local lb = sim.setStepping(true)

    local retVal = {}
    local x = {}
    local confS = {}
    local err = false
    local inputConfig = {}
    for i = 1, #jointHandles, 1 do
        inputConfig[i] = simIK.getJointPosition(ikEnv, jointHandles[i])
        local c, interv = simIK.getJointInterval(ikEnv, jointHandles[i])
        local t = simIK.getJointType(ikEnv, jointHandles[i])
        local sp = simIK.getJointScrewLead(ikEnv, jointHandles[i])
        if t == simIK.jointtype_revolute and not c then
            if sp == 0 then
                if inputConfig[i] - math.pi * 2 >= interv[1] or inputConfig[i] + math.pi * 2 <=
                    interv[1] + interv[2] then
                    -- We use the low and range values from the joint's settings
                    local y = inputConfig[i]
                    while y - math.pi * 2 >= interv[1] do y = y - math.pi * 2 end
                    x[i] = {y, interv[1] + interv[2]}
                end
            end
        end
        if x[i] then
            if lowLimits and ranges then
                -- the user specified low and range values. Use those instead:
                local l = lowLimits[i]
                local r = ranges[i]
                if r ~= 0 then
                    if r > 0 then
                        if l < interv[1] then
                            -- correct for user bad input
                            r = r - (interv[1] - l)
                            l = interv[1]
                        end
                        if l > interv[1] + interv[2] then
                            -- bad user input. No alternative position for this joint
                            x[i] = {inputConfig[i], inputConfig[i]}
                            err = true
                        else
                            if l + r > interv[1] + interv[2] then
                                -- correct for user bad input
                                r = interv[1] + interv[2] - l
                            end
                            if inputConfig[i] - math.pi * 2 >= l or inputConfig[i] + math.pi * 2 <=
                                l + r then
                                local y = inputConfig[i]
                                while y < l do y = y + math.pi * 2 end
                                while y - math.pi * 2 >= l do
                                    y = y - math.pi * 2
                                end
                                x[i] = {y, l + r}
                            else
                                -- no alternative position for this joint
                                x[i] = {inputConfig[i], inputConfig[i]}
                                err = (inputConfig[i] < l) or (inputConfig[i] > l + r)
                            end
                        end
                    else
                        r = -r
                        l = inputConfig[i] - r * 0.5
                        if l < x[i][1] then l = x[i][1] end
                        local u = inputConfig[i] + r * 0.5
                        if u > x[i][2] then u = x[i][2] end
                        x[i] = {l, u}
                    end
                end
            end
        else
            -- there's no alternative position for this joint
            x[i] = {inputConfig[i], inputConfig[i]}
        end
        confS[i] = x[i][1]
    end
    local configs = {}
    if not err then
        local desiredPose = 0
        configs = _S.simIKLoopThroughAltConfigSolutions(
                      ikEnv, jointHandles, desiredPose, confS, x, 1
                  )
    end
    sim.setStepping(lb)

    if next(configs) ~= nil then
        configs = Matrix:fromtable(configs)
        configs = configs:data()
    end
    return configs
end

function simIK.syncFromSim(...)
    local ikEnv, ikGroups = checkargs({
        {type = 'int'},
        {type = 'table'},
    }, ...)

    local lb = sim.setStepping(true)
    for g = 1, #ikGroups, 1 do
        local groupData = _S.ikEnvs[ikEnv].ikGroups[ikGroups[g]]
        for k, v in pairs(groupData.joints) do
            if sim.isHandle(k) then
                if sim.getJointType(k) == sim.joint_spherical then
                    simIK.setSphericalJointMatrix(ikEnv, v, sim.getJointMatrix(k))
                else
                    simIK.setJointPosition(ikEnv, v, sim.getJointPosition(k))
                end
            else
                -- that is probably a joint in a dependency relation, that was removed
                simIK.eraseObject(ikEnv, v)
                groupData.joints[k] = nil
                _S.ikEnvs[ikEnv].simToIkMap[k] = nil
                _S.ikEnvs[ikEnv].ikToSimMap[v] = nil
            end
        end
        for i = 1, #groupData.targetTipBaseTriplets, 1 do
            -- Make sure target relative to base is in sync too:
            simIK.setObjectMatrix(
                ikEnv, groupData.targetTipBaseTriplets[i][4], sim.getObjectMatrix(
                    groupData.targetTipBaseTriplets[i][1], groupData.targetTipBaseTriplets[i][3]
                ), groupData.targetTipBaseTriplets[i][6]
            )
        end
    end
    sim.setStepping(lb)
end

function simIK.syncToSim(...)
    local ikEnv, ikGroups = checkargs({
        {type = 'int'},
        {type = 'table'},
    }, ...)

    local lb = sim.setStepping(true)
    for g = 1, #ikGroups, 1 do
        local groupData = _S.ikEnvs[ikEnv].ikGroups[ikGroups[g]]
        for k, v in pairs(groupData.joints) do
            if sim.isHandle(k) then
                if sim.getJointType(k) == sim.joint_spherical then
                    if sim.getJointMode(k) ~= sim.jointmode_force or not sim.isDynamicallyEnabled(k) then
                        sim.setSphericalJointMatrix(k, simIK.getJointMatrix(ikEnv, v))
                    end
                else
                    if sim.getJointMode(k) == sim.jointmode_force and sim.isDynamicallyEnabled(k) then
                        sim.setJointTargetPosition(k, simIK.getJointPosition(ikEnv, v))
                    else
                        sim.setJointPosition(k, simIK.getJointPosition(ikEnv, v))
                    end
                end
            else
                -- that is probably a joint in a dependency relation, that was removed
                simIK.eraseObject(ikEnv, v)
                groupData.joints[k] = nil
                _S.ikEnvs[ikEnv].simToIkMap[k] = nil
                _S.ikEnvs[ikEnv].ikToSimMap[v] = nil
            end
        end
    end
    sim.setStepping(lb)
end

function simIK.debugGroupIfNeeded(ikEnv, ikGroup, debugFlags)
    if not _S.ikEnvs then _S.ikEnvs = {} end

    if _S.ikEnvs[ikEnv] then -- when an IK environment is duplicated, it does not appear in _S.ikEnvs...
        local p = sim.getNamedInt32Param('simIK.debug_world')
        if (p and (p & 1) ~= 0) or ((debugFlags & 1) ~= 0) then
            local lb = sim.setStepping(true)
            local groupData = _S.ikEnvs[ikEnv].ikGroups[ikGroup]
            groupData.visualDebug = {}
            for i = 1, #groupData.targetTipBaseTriplets, 1 do
                groupData.visualDebug[i] = simIK.createDebugOverlay(
                                               ikEnv, groupData.targetTipBaseTriplets[i][5],
                                               groupData.targetTipBaseTriplets[i][6]
                                           )
            end
            sim.setStepping(lb)
        else
            local groupData = _S.ikEnvs[ikEnv].ikGroups[ikGroup]
            if groupData.visualDebug then
                for i = 1, #groupData.visualDebug, 1 do
                    simIK.eraseDebugOverlay(groupData.visualDebug[i])
                end
            end
            groupData.visualDebug = {}
        end
    end
end

function simIK.addElementFromScene(...)
    local ikEnv, ikGroup, simBase, simTip, simTarget, constraints = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'int'},
        {type = 'int'},
        {type = 'int'},
        {type = 'int'},
    }, ...)

    local lb = sim.setStepping(true)

    if not _S.ikEnvs then _S.ikEnvs = {} end
    if not _S.ikEnvs[ikEnv] then _S.ikEnvs[ikEnv] = {} end
    if not _S.ikEnvs[ikEnv].ikGroups then
        _S.ikEnvs[ikEnv].ikGroups = {}
        _S.ikEnvs[ikEnv].simToIkMap = {}
        _S.ikEnvs[ikEnv].ikToSimMap = {}
    end
    local groupData = _S.ikEnvs[ikEnv].ikGroups[ikGroup]
    -- simToIkMap and ikToSimMap, need to be scoped by ik env, and not ik group,
    -- otherwise we may have duplicates:
    local simToIkMap = _S.ikEnvs[ikEnv].simToIkMap
    local ikToSimMap = _S.ikEnvs[ikEnv].ikToSimMap
    if not groupData then
        groupData = {}
        groupData.joints = {}
        groupData.targetTipBaseTriplets = {}
        _S.ikEnvs[ikEnv].ikGroups[ikGroup] = groupData
    end

    function createIkJointFromSimJoint(ikEnv, simJoint)
        local t = sim.getJointType(simJoint)
        local ikJoint = simIK.createJoint(ikEnv, t)
        local c, interv = sim.getJointInterval(simJoint)
        simIK.setJointInterval(ikEnv, ikJoint, c, interv)
        local sp = sim.getObjectFloatParam(simJoint, sim.jointfloatparam_screwlead)
        simIK.setJointScrewLead(ikEnv, ikJoint, sp)
        local sp = sim.getObjectFloatParam(simJoint, sim.jointfloatparam_step_size)
        simIK.setJointMaxStepSize(ikEnv, ikJoint, sp)
        local sp = sim.getObjectFloatParam(simJoint, sim.jointfloatparam_ik_weight)
        simIK.setJointWeight(ikEnv, ikJoint, sp)
        if t == sim.joint_spherical then
            simIK.setSphericalJointMatrix(ikEnv, ikJoint, sim.getJointMatrix(simJoint))
        else
            simIK.setJointPosition(ikEnv, ikJoint, sim.getJointPosition(simJoint))
        end
        return ikJoint
    end

    function iterateAndAdd(theTip, theBase)
        local ikPrevIterator = -1
        local simIterator = theTip
        while true do
            local ikIterator = -1
            if simToIkMap[simIterator] then
                -- object already added (but maybe parenting not done yet, e.g. with master joints in dependency relationship)
                ikIterator = simToIkMap[simIterator]
            else
                if sim.getObjectType(simIterator) ~= sim.sceneobject_joint then
                    ikIterator = simIK.createDummy(ikEnv)
                else
                    ikIterator = createIkJointFromSimJoint(ikEnv, simIterator)
                end
                simToIkMap[simIterator] = ikIterator
                ikToSimMap[ikIterator] = simIterator
                simIK.setObjectMatrix(ikEnv, ikIterator, sim.getObjectMatrix(simIterator))
            end
            if sim.getObjectType(simIterator) == sim.sceneobject_joint then
                groupData.joints[simIterator] = ikIterator
            end
            if ikPrevIterator ~= -1 then
                simIK.setObjectParent(ikEnv, ikPrevIterator, ikIterator)
            end
            local newSimIterator = sim.getObjectParent(simIterator)
            if simIterator == theBase or newSimIterator == -1 then break end
            simIterator = newSimIterator
            ikPrevIterator = ikIterator
        end
    end

    iterateAndAdd(simTip, -1) -- need to add the whole chain down to world, otherwise subtle bugs!
    local ikTip = simToIkMap[simTip]
    local ikBase = -1;
    if simBase ~= -1 then ikBase = simToIkMap[simBase] end
    iterateAndAdd(simTarget, -1) -- need to add the whole target chain down to world too, otherwise subtle bugs!
    local ikTarget = simToIkMap[simTarget]
    simIK.setTargetDummy(ikEnv, ikTip, ikTarget)
    groupData.targetTipBaseTriplets[#groupData.targetTipBaseTriplets + 1] = {
        simTarget, simTip, simBase, ikTarget, ikTip, ikBase,
    }

    -- Now handle joint dependencies. Consider slave0 --> slave1 --> .. --> master
    -- We add all master and slave joints, even if not in the IK world (for simplification, since we could have complex daisy chains)
    -- We will however have to be careful, and always first check if a joint is still valid (e.g. we could remove a joint from the scene
    -- in an unrelated model)
    local simJoints = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_joint)
    local slaves = {}
    local masters = {}
    local passives = {}
    for i = 1, #simJoints, 1 do
        local jo = simJoints[i]
        if sim.getJointMode(jo) == sim.jointmode_dependent then
            local dep, off, mult = sim.getJointDependency(jo)
            if dep ~= -1 then
                slaves[#slaves + 1] = jo
                masters[#masters + 1] = dep
            else
                passives[#passives + 1] = jo
            end
        end
    end
    for i = 1, #passives, 1 do
        local ikJo = simToIkMap[passives[i]]
        if ikJo then simIK.setJointMode(ikEnv, ikJo, simIK.jointmode_passive) end
    end
    for i = 1, #slaves, 1 do
        local slave = slaves[i]
        local master = masters[i]
        local ikJo_s = simToIkMap[slave]
        local ikJo_m = simToIkMap[master]
        if ikJo_s == nil then
            ikJo_s = createIkJointFromSimJoint(ikEnv, slave)
            simToIkMap[slave] = ikJo_s
            ikToSimMap[ikJo_s] = slave
            simIK.setObjectMatrix(ikEnv, ikJo_s, sim.getObjectMatrix(slave))
            groupData.joints[slave] = ikJo_s
        end
        if ikJo_m == nil then
            ikJo_m = createIkJointFromSimJoint(ikEnv, master)
            simToIkMap[master] = ikJo_m
            ikToSimMap[ikJo_m] = master
            simIK.setObjectMatrix(ikEnv, ikJo_m, sim.getObjectMatrix(master))
            groupData.joints[master] = ikJo_m
        end
        local dep, off, mult = sim.getJointDependency(slave)
        simIK.setJointDependency(ikEnv, ikJo_s, ikJo_m, off, mult)
    end

    local ikElement = simIK.addElement(ikEnv, ikGroup, ikTip)
    simIK.setElementBase(ikEnv, ikGroup, ikElement, ikBase, -1)
    simIK.setElementConstraints(ikEnv, ikGroup, ikElement, constraints)
    sim.setStepping(lb)
    return ikElement, simToIkMap, ikToSimMap
end

function simIK.eraseEnvironment(...)
    local ikEnv = checkargs({
        {type = 'int'},
    }, ...)

    local lb = sim.setStepping(true)

    if _S.ikEnvs and _S.ikEnvs[ikEnv] then
        local env = _S.ikEnvs[ikEnv]
        if env.ikGroups then
            for k, v in pairs(env.ikGroups) do
                if v.visualDebug then
                    for i = 1, #v.visualDebug do
                        simIK.eraseDebugOverlay(v.visualDebug[i])
                    end
                end
            end
        end
        _S.ikEnvs[ikEnv] = nil
    end
    simIK._eraseEnvironment(ikEnv)
    sim.setStepping(lb)
end

function simIK.findConfig(...)
    -- deprecated. Use simIK.findConfigs instead
    local ikEnv, ikGroup, joints, thresholdDist, maxTime, metric, callback, auxData = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'table', size = '1..*', item_type = 'int'},
        {type = 'float', default = 0.1},
        {type = 'float', default = 0.5},
        {type = 'table', size = 4, item_type = 'float', default = {1, 1, 1, 0.1}, nullable = true},
        {type = 'any', default = NIL, nullable = true},
        {type = 'any', default = NIL, nullable = true},
    }, ...)

    local dof = #joints
    local lb = sim.setStepping(true)

    -- local env=simIK.duplicateEnvironment(ikEnv)
    local env = ikEnv
    if metric == nil then metric = {1, 1, 1, 0.1} end
    local __callback
    function __ikcb(config)
        local fun = _G
        if string.find(__callback, "%.") then
            for w in __callback:gmatch("[^%.]+") do -- handle cases like sim.func or similar too
                if fun[w] then fun = fun[w] end
            end
        else
            fun = fun[__callback]
        end
        if type(fun) == 'function' then
            return fun(config, auxData)
        end
    end
    local funcNm, t
    if callback then
        __callback = reify(callback)
        funcNm = '__ikcb'
        t = sim.getScript(sim.handle_self)
    end
    local retVal = simIK._findConfig(env, ikGroup, joints, thresholdDist, maxTime * 1000, metric, funcNm, t)
    -- simIK.eraseEnvironment(env)
    sim.setStepping(lb)
    return retVal
end

function simIK.handleGroup(...) -- convenience function
    local ikEnv, ikGroup, options = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'table', default = {}},
    }, ...)

    return simIK.handleGroups(ikEnv, {ikGroup}, options)
end

function simIK.debugJacobianDisplay(inData)
    local groupData = _S.ikEnvs[_S.currentIkEnv].ikGroups[inData.groupHandle]
    local groupIdStr = string.format('env:%d/group:%d', _S.currentIkEnv, inData.groupHandle)
    pcall(
        function()
            simQML = require 'simQML'
        end
    )
    if simQML then
        if groupData.jacobianDebug == nil then
            groupData.jacobianDebug = {qmlEngine = simQML.createEngine()}
            function jacobianDebugClicked()
                jacobianDebugPrint = true
            end
            simQML.setEventHandler(groupData.jacobianDebug.qmlEngine, 'dispatchEventsToFunctions')
            simQML.loadData(
                groupData.jacobianDebug.qmlEngine, [[
                import QtQuick 2.12
                import QtQuick.Window 2.12
                import CoppeliaSimPlugin 1.0

                PluginWindow {
                    id: mainWindow
                    width: cellSize * cols
                    height: cellSize * rows
                    title: "Jacobian ]] .. groupIdStr .. [["

                    readonly property string groupId: "]] .. groupIdStr .. [["

                    property int rows: 6
                    property int cols: 12
                    readonly property int cellSize: 15

                    property real absMin: 0
                    property real absMax: 0.001
                    property bool initMax: true

                    property var jacobianData: {
                        var _t = []
                        for(var iy = 0; iy < rows; iy++) {
                            var _r = []
                            for(var ix = 0; ix < cols; ix++) {
                                var z = iy/rows - ix/cols
                                z = Math.sign(z) * Math.pow(10, 3 * Math.abs(z))
                                _r.push(z)
                            }
                            _t.push(_r)
                        }
                        return _t
                    }

                    function colorMap(value) {
                        var min = Math.log10(Math.max(0.00001, absMin))
                        var max = Math.log10(absMax)
                        var sign = Math.sign(value)
                        value = Math.max(0, Math.min(1, (Math.log10(Math.abs(value)) - min) / Math.max(1e-9, max - min)))
                        var c = x => Math.min(Math.max(x, 0), 1)
                        var r = c(sign * value)
                        var b = c(-sign * value)
                        return Qt.rgba(1 - b, 1 - 0.6 * (r + b), 1 - r)
                    }

                    Column {
                        Repeater {
                            model: mainWindow.rows
                            Row {
                                readonly property int i: index
                                readonly property var rowData: mainWindow.jacobianData[i] || new Array(mainWindow.cols).fill(0)
                                Repeater {
                                    model: mainWindow.cols
                                    Rectangle {
                                        readonly property int j: index
                                        readonly property real elemData: rowData[j] || 0
                                        width: mainWindow.width / mainWindow.cols
                                        height: mainWindow.height / mainWindow.rows
                                        color: colorMap(elemData)
                                        border.color: Qt.rgba(0, 0, 0, 0.03)
                                        opacity: 0.8
                                        Text {
                                            anchors.fill: parent
                                            text: Number(elemData).toLocaleString(Qt.locale("en_US"), 'f', width / font.pixelSize)
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                            readonly property real k: 0.33
                                            font.pixelSize: Math.min(k * parent.width, k * parent.height, 14)
                                            opacity: Math.min(font.pixelSize / 11, 1)
                                        }
                                    }
                                }
                            }
                        }
                    }

                    MouseArea {
                        anchors.fill: parent
                        onPressed: simBridge.sendEvent('jacobianDebugClicked', {})
                    }

                    Component.onCompleted: {
                        x = Screen.width - width - 5
                        y = Screen.height - height - 180
                    }

                    function setData(info) {
                        if(info.groupId !== mainWindow.groupId) return
                        var d = info.jacobian
                        var _t = []
                        if(mainWindow.initMax && d.length > 0 && d[0].length > 0) {
                            mainWindow.initMax = false
                            mainWindow.absMin = Math.abs(d[0][0])
                            mainWindow.absMax = Math.abs(d[0][0])
                        }
                        for(var iy = 0; iy < d.length; iy++) {
                            var _r = []
                            for(var ix = 0; ix < d[0].length; ix++) {
                                mainWindow.absMin = Math.min(mainWindow.absMin, Math.abs(d[iy][ix]))
                                mainWindow.absMax = Math.max(mainWindow.absMax, Math.abs(d[iy][ix]))
                                _r.push(d[iy][ix])
                            }
                            _t.push(_r)
                        }
                        mainWindow.jacobianData = _t
                        mainWindow.rows = d.length
                        mainWindow.cols = d[0].length
                    }
                }
            ]]
            )
        end
        simQML.sendEvent(
            groupData.jacobianDebug.qmlEngine, 'setData',
            {groupId = groupIdStr, jacobian = inData.jacobian:totable()}
        )
    end
    if jacobianDebugPrint then
        jacobianDebugPrint = false
        inData.jacobian:print 'J'
    end
end

function simIK.handleGroups(...)
    local ikEnv, ikGroups, options = checkargs({
        {type = 'int'},
        {type = 'table'},
        {type = 'table', default = {}},
    }, ...)

    local lb = sim.setStepping(true)
    _S.currentIkEnv = ikEnv
    local debugFlags = 0
    if options.debug then debugFlags = options.debug end
    local p = sim.getNamedInt32Param('simIK.debug_world')
    local debugJacobian = (((debugFlags & 2) ~= 0) or (p and (p & 2) ~= 0)) and _S.ikEnvs[ikEnv] -- when an IK environment is duplicated, it does not appear in _S.ikEnvs...
    local pythonCallback = false
    function __cb(rows_constr, rows_ikEl, cols_handles, cols_dofIndex, jacobian, errorVect, groupId,
                  iteration)
        local data = {}
        data.rows = {}
        data.cols = {}
        if pythonCallback then
            data.jacobian = jacobian
            data.e = errorVect
        else
            data.jacobian = Matrix(#rows_constr, #cols_handles, jacobian)
            data.e = Matrix(#rows_constr, 1, errorVect)
        end
        for i = 1, #rows_constr, 1 do
            data.rows[i] = {constraint = rows_constr[i], element = rows_ikEl[i]}
        end
        for i = 1, #cols_handles, 1 do
            data.cols[i] = {joint = cols_handles[i], dofIndex = cols_dofIndex[i]}
        end
        data.groupHandle = groupId
        data.iteration = iteration
        if debugJacobian then simIK.debugJacobianDisplay(data) end
        local j = {}
        local e = {}
        local dq = {}
        local jpinv = {}
        if options.callback then
            local outData
            if type(options.callback) == 'string' then
                outData = _G[options.callback](data, options.auxData)
            else
                outData = options.callback(data, options.auxData)
            end
            if outData then
                if pythonCallback then
                    if outData.jacobian then
                        if #outData.jacobian == #cols_handles * #rows_constr then
                            j = outData.jacobian
                        else
                            error("invalid jacobian matrix size")
                        end
                    end
                    if outData.e then
                        if #outData.e == #rows_constr then
                            e = outData.e
                        else
                            error("invalid e vector size")
                        end
                    end
                    if outData.dq then
                        if #outData.dq == #cols_handles then
                            dq = outData.dq
                        else
                            error("invalid dq vector size")
                        end
                    end
                    if outData.jacobianPinv then
                        if #outData.jacobianPinv == #cols_handles * #rows_constr then
                            jpinv = outData.jacobianPinv
                        else
                            error("invalid jacobian pseudo-inverse matrix size")
                        end
                    end
                else
                    if outData.jacobian then
                        if outData.jacobian:cols() == #cols_handles and outData.jacobian:rows() ==
                            #rows_constr then
                            j = outData.jacobian:data()
                        else
                            error("invalid jacobian matrix size")
                        end
                    end
                    if outData.e then
                        if outData.e:rows() == #rows_constr and outData.e:cols() == 1 then
                            e = outData.e:data()
                        else
                            error("invalid e vector size")
                        end
                    end
                    if outData.dq then
                        if outData.dq:rows() == #cols_handles and outData.dq:cols() == 1 then
                            dq = outData.dq:data()
                        else
                            error("invalid dq vector size")
                        end
                    end
                    if outData.jacobianPinv then
                        if outData.jacobianPinv:rows() == #cols_handles and outData.jacobianPinv:cols() ==
                            #rows_constr then
                            jpinv = outData.jacobianPinv:data()
                        else
                            error("invalid jacobian pseudo-inverse matrix size")
                        end
                    end
                end
            end
        end
        return j, e, dq, jpinv
    end
    local funcNm, t
    if options.callback or debugJacobian then
        funcNm = '__cb'
        t = sim.getScript(sim.handle_self)
        if _S.pythonCallbacks and _S.pythonCallbacks[options.callback] then
            pythonCallback = true
        end
    end
    if options.syncWorlds then simIK.syncFromSim(ikEnv, ikGroups) end
    local retVal, reason, prec = simIK._handleGroups(ikEnv, ikGroups, funcNm, t)
    if options.syncWorlds then
        if (reason & simIK.calc_notwithintolerance) == 0 or options.allowError then
            simIK.syncToSim(ikEnv, ikGroups)
        end
    end
    for i = 1, #ikGroups, 1 do simIK.debugGroupIfNeeded(ikEnv, ikGroups[i], debugFlags) end
    sim.setStepping(lb)
    return retVal, reason, prec
end

function simIK.getFailureDescription(reason)
    local d = {}
    for _, k in ipairs {
        'notperformed', 'cannotinvert', 'notwithintolerance', 'stepstoobig', 'limithit',
    } do
        local f = 'calc_' .. k
        if reason & simIK[f] > 0 then
            reason = reason & ~simIK[f]
            table.insert(d, k)
        end
    end
    if reason ~= 0 then table.insert(d, tostring(reason)) end
    return table.tostring(d)
end

function simIK.setJointDependency(...)
    local ikEnv, slaveJoint, masterJoint, offset, mult, callback = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'int'},
        {type = 'float', default = 0.0},
        {type = 'float', default = 1.0},
        {type = 'any', default = NIL, nullable = true},
    }, ...)

    function __depcb(ikEnv, slaveJoint, masterPos)
        if type(callback) == 'string' then
            return _G[callback](ikEnv, slaveJoint, masterPos)
        else
            return callback(ikEnv, slaveJoint, masterPos)
        end
    end
    local funcNm, t
    if callback then
        funcNm = '__depcb'
        t = sim.getScript(sim.handle_self)
    end
    simIK._setJointDependency(ikEnv, slaveJoint, masterJoint, offset, mult, funcNm, t)
    return retVal
end

function simIK.generatePath(...)
    local ikEnv, ikGroup, ikJoints, tip, ptCnt, callback, auxData = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'table', size = '1..*', item_type = 'int'},
        {type = 'int'},
        {type = 'int'},
        {type = 'any', default = NIL, nullable = true},
        {type = 'any', default = NIL},
    }, ...)

    local lb = sim.setStepping(true)

    local env = simIK.duplicateEnvironment(ikEnv)
    local targetHandle = simIK.getTargetDummy(env, tip)
    local startMatrix = simIK.getObjectMatrix(env, tip)
    local goalMatrix = simIK.getObjectMatrix(env, targetHandle)
    local retPath = {{}}
    for i = 1, #ikJoints, 1 do retPath[1][i] = simIK.getJointPosition(env, ikJoints[i]) end
    local success = true
    if callback then
        if type(callback) == 'string' then
            success = _G[callback](retPath[1], auxData)
        else
            success = callback(retPath[1], auxData)
        end
    end
    if success then
        for j = 1, ptCnt - 1, 1 do
            local t = j / (ptCnt - 1)
            local m = sim.interpolateMatrices(startMatrix, goalMatrix, t)
            simIK.setObjectMatrix(env, targetHandle, m)
            success = simIK.handleGroups(env, {ikGroup}) == simIK.result_success
            if not success then break end
            retPath[j + 1] = {}
            for i = 1, #ikJoints, 1 do
                retPath[j + 1][i] = simIK.getJointPosition(env, ikJoints[i])
            end
            if callback then
                if type(callback) == 'string' then
                    success = _G[callback](retPath[j + 1], auxData)
                else
                    success = callback(retPath[j + 1], auxData)
                end
            end
            if not success then break end
        end
    end
    simIK.eraseEnvironment(env)
    sim.setStepping(lb)
    if not success then
        retPath = {}
    else
        retPath = table.collapse(retPath)
    end
    return retPath
end

function simIK.createDebugOverlay(...)
    local ikEnv, ikTip, ikBase = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'int', default = -1},
    }, ...)

    if not _S.ikDebug then _S.ikDebug = {ikEnv = {}, ikDebugId = 0} end
    if not _S.ikDebug.ikEnv[ikEnv] then _S.ikDebug.ikEnv[ikEnv] = {} end
    if not _S.ikDebug.ikEnv[ikEnv][ikTip] then
        _S.ikDebug.ikEnv[ikEnv][ikTip] = {drawingConts = {}, id = _S.ikDebug.ikDebugId}
        _S.ikDebug.ikDebugId = _S.ikDebug.ikDebugId + 1
    end
    local drawingConts = _S.ikDebug.ikEnv[ikEnv][ikTip].drawingConts

    local ikTarget = simIK.getTargetDummy(ikEnv, ikTip)
    if drawingConts.targetCont == nil then
        drawingConts.targetCont = sim.addDrawingObject(
                                      sim.drawing_spherepts | sim.drawing_overlay |
                                          sim.drawing_cyclic, 0.012, 0, -1, 1, {1, 0, 0}
                                  )
    end
    sim.addDrawingObjectItem(drawingConts.targetCont, simIK.getObjectPose(ikEnv, ikTarget))
    if drawingConts.tipCont == nil then
        drawingConts.tipCont = sim.addDrawingObject(
                                   sim.drawing_spherepts | sim.drawing_overlay | sim.drawing_cyclic,
                                   0.01, 0, -1, 1, {0, 1, 0}
                               )
    end
    sim.addDrawingObjectItem(drawingConts.tipCont, simIK.getObjectPose(ikEnv, ikTip))
    if drawingConts.linkCont == nil then
        drawingConts.linkCont = sim.addDrawingObject(
                                    sim.drawing_lines | sim.drawing_overlay, 2, 0, -1, 0, {0, 0, 0}
                                )
    else
        sim.addDrawingObjectItem(drawingConts.linkCont, nil)
    end
    if drawingConts.linkContN == nil then
        drawingConts.linkContN = sim.addDrawingObject(
                                     sim.drawing_spherepts | sim.drawing_overlay, 0.01, 0, -1, 0,
                                     {1, 1, 1}
                                 )
    else
        sim.addDrawingObjectItem(drawingConts.linkContN, nil)
    end
    if drawingConts.baseCont == nil then
        drawingConts.baseCont = sim.addDrawingObject(
                                    sim.drawing_cubepts | sim.drawing_overlay | sim.drawing_cyclic,
                                    0.01, 0, -1, 1, {1, 0, 1}
                                )
    end
    local w = {0, 0, 0, 1, 0, 0, 0}
    if ikBase ~= -1 then w = simIK.getObjectPose(ikEnv, ikBase) end
    sim.addDrawingObjectItem(drawingConts.baseCont, w)
    if drawingConts.jointCont == nil then
        drawingConts.jointCont = {}
        drawingConts.jointCont[1] = sim.addDrawingObject(
                                        sim.drawing_lines | sim.drawing_overlay, 4, 0, -1, 0,
                                        {1, 0.5, 0}
                                    )
        drawingConts.jointCont[2] = sim.addDrawingObject(
                                        sim.drawing_lines | sim.drawing_overlay, 2, 0, -1, 0,
                                        {0, 0.5, 1}
                                    )
        drawingConts.jointCont[3] = sim.addDrawingObject(
                                        sim.drawing_lines | sim.drawing_overlay, 4, 0, -1, 0,
                                        {0.5, 0.5, 0.5}
                                    )
        drawingConts.jointCont[4] = sim.addDrawingObject(
                                        sim.drawing_spherepts | sim.drawing_overlay, 0.012, 0, -1,
                                        0, {1, 0.5, 0}
                                    )
        drawingConts.jointCont[5] = -1
        drawingConts.jointCont[6] = sim.addDrawingObject(
                                        sim.drawing_spherepts | sim.drawing_overlay, 0.012, 0, -1,
                                        0, {0.5, 0.5, 0.5}
                                    )
    else
        sim.addDrawingObjectItem(drawingConts.jointCont[1], nil)
        sim.addDrawingObjectItem(drawingConts.jointCont[2], nil)
        sim.addDrawingObjectItem(drawingConts.jointCont[3], nil)
        sim.addDrawingObjectItem(drawingConts.jointCont[4], nil)
        sim.addDrawingObjectItem(drawingConts.jointCont[6], nil)
    end

    local obj = ikTip
    local prevObj = obj
    while obj ~= -1 and obj ~= ikBase do
        local t = simIK.getObjectType(ikEnv, obj)
        if t == simIK.objecttype_joint then
            local p = simIK.getObjectPose(ikEnv, prevObj)
            local m1 = simIK.getObjectMatrix(ikEnv, obj)
            local m2 = simIK.getJointMatrix(ikEnv, obj)
            m1 = sim.multiplyMatrices(m1, m2)
            p[4] = m1[4]
            p[5] = m1[8]
            p[6] = m1[12]
            sim.addDrawingObjectItem(drawingConts.linkCont, p)
            local spherical = (simIK.getJointType(ikEnv, obj) == simIK.jointtype_spherical)
            local m = simIK.getJointMode(ikEnv, obj) -- simIK.jointmode_passive or simIK.jointmode_ik
            local d = simIK.getJointDependency(ikEnv, obj)
            local drt = sim.drawing_lines
            local hs = 0.025
            local ind1 = 0
            local ind2 = 1
            if spherical then ind1 = 3 end
            if d >= 0 then
                ind2 = 2
                hs = 0.05
            elseif m == simIK.jointmode_passive then
                ind2 = 3
            end
            local m = simIK.getObjectMatrix(ikEnv, obj)
            if spherical then
                sim.addDrawingObjectItem(drawingConts.jointCont[ind1 + ind2], {m[4], m[8], m[12]})
            else
                sim.addDrawingObjectItem(
                    drawingConts.jointCont[ind1 + ind2], {
                        m[4] - m[3] * hs, m[8] - m[7] * hs, m[12] - m[11] * hs, m[4] + m[3] * hs,
                        m[8] + m[7] * hs, m[12] + m[11] * hs,
                    }
                )
            end
        else
            if prevObj ~= obj then
                local p = simIK.getObjectPose(ikEnv, obj)
                local p2 = simIK.getObjectPose(ikEnv, prevObj)
                p[4] = p2[1]
                p[5] = p2[2]
                p[6] = p2[3]
                sim.addDrawingObjectItem(drawingConts.linkCont, p)
                sim.addDrawingObjectItem(drawingConts.linkContN, p)
            end
        end
        prevObj = obj
        obj = simIK.getObjectParent(ikEnv, obj)
    end

    local p = simIK.getObjectPose(ikEnv, prevObj)
    p[4] = 0
    p[5] = 0
    p[6] = 0
    if ikBase ~= -1 then
        local p2 = simIK.getObjectPose(ikEnv, ikBase)
        p[4] = p2[1]
        p[5] = p2[2]
        p[6] = p2[3]
    end
    sim.addDrawingObjectItem(drawingConts.linkCont, p)

    return _S.ikDebug.ikEnv[ikEnv][ikTip].id
end

function simIK.eraseDebugOverlay(...)
    local id = checkargs({
        {type = 'int'},
    }, ...)

    if _S.ikDebug then
        for k, env in pairs(_S.ikDebug.ikEnv) do
            for l, tip in pairs(env) do
                if tip.id == id then
                    if tip.drawingConts.targetCont ~= nil then
                        sim.removeDrawingObject(tip.drawingConts.targetCont)
                        tip.drawingConts.targetCont = nil
                    end
                    if tip.drawingConts.tipCont ~= nil then
                        sim.removeDrawingObject(tip.drawingConts.tipCont)
                        tip.drawingConts.tipCont = nil
                    end
                    if tip.drawingConts.linkCont ~= nil then
                        sim.removeDrawingObject(tip.drawingConts.linkCont)
                        tip.drawingConts.linkCont = nil
                    end
                    if tip.drawingConts.linkContN ~= nil then
                        sim.removeDrawingObject(tip.drawingConts.linkContN)
                        tip.drawingConts.linkContN = nil
                    end
                    if tip.drawingConts.baseCont ~= nil then
                        sim.removeDrawingObject(tip.drawingConts.baseCont)
                        tip.drawingConts.baseCont = nil
                    end
                    if tip.drawingConts.jointCont ~= nil then
                        sim.removeDrawingObject(tip.drawingConts.jointCont[1])
                        sim.removeDrawingObject(tip.drawingConts.jointCont[2])
                        sim.removeDrawingObject(tip.drawingConts.jointCont[3])
                        sim.removeDrawingObject(tip.drawingConts.jointCont[4])
                        sim.removeDrawingObject(tip.drawingConts.jointCont[6])
                        tip.drawingConts.jointCont = nil
                    end
                    return
                end
            end
        end
    end
end

function simIK.solvePath(...)
    -- undocumented (for now) function
    -- simPath can be a Path object handle, or the path data itself
    -- ikPath a dummy with a pose and parent consistent with simPath
    local ikEnv, ikGroup, ikTarget, ikJoints, simJoints, ikPath, simPath, collisionPairs, opts =
        checkargs({
            {type = 'int'},
            {type = 'int'},
            {type = 'int'},
            {type = 'table', size = '1..*', item_type = 'int'},
            {type = 'table', size = '1..*', item_type = 'int'},
            {type = 'int'},
            {union = {{type = 'handle'}, {type = 'table'}}},
            {type = 'table', default = {}},
            {type = 'table', default = {}},
        }, ...)

    collisionPairs = collisionPairs or {}
    local delta = opts.delta or 0.005
    local errorCallback = opts.errorCallback or function(e)
    end
    local function reportError(...)
        errorCallback(string.format(...))
    end

    local function callStepCb(failed)
        if opts.stepCallback then opts.stepCallback(failed) end
    end

    local pathData = simPath
    if math.type(simPath) == 'integer' and sim.isHandle(simPath) then
        -- read path data inside path object:
        pathData = sim.readCustomBufferData(simPath, 'PATH')
        assert(pathData ~= nil and #pathData > 0, 'object does not contain PATH data')
        pathData = sim.unpackDoubleTable(pathData)
    end
    local m = Matrix(#pathData // 7, 7, pathData)
    local pathPositions = m:slice(1, 1, m:rows(), 3):data()
    local pathQuaternions = m:slice(1, 4, m:rows(), 7):data()
    local pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)

    local moveIkTarget = opts.moveIkTarget or function(posAlongPath)
        local pose = sim.getPathInterpolatedConfig(
                         pathData, pathLengths, posAlongPath, nil, {0, 0, 0, 2, 2, 2, 2}
                     )
        -- re-fetch path pose, should path be part of the IK chain:
        if ikPath ~= -1 then
            local pathPose = simIK.getObjectPose(ikEnv, ikPath)
            pose = sim.multiplyPoses(pathPose, pose)
        end
        simIK.setObjectPose(ikEnv, ikTarget, pose)
    end
    local getConfig = opts.getConfig or partial(map, sim.getJointPosition, simJoints)
    local setConfig = opts.setConfig or partial(foreach, sim.setJointPosition, simJoints)
    local getIkConfig = opts.getIkConfig or
                            partial(map, partial(simIK.getJointPosition, ikEnv), ikJoints)
    local setIkConfig = opts.setIkConfig or
                            partial(foreach, partial(simIK.setJointPosition, ikEnv), ikJoints)

    -- save current robot config:
    local origIkCfg = getIkConfig()

    local cfgs = {}
    local posAlongPath = 0
    local finished = false

    -- find initial config:
    moveIkTarget(0)
    local cfg = simIK.findConfigs(ikEnv, ikGroup, ikJoints)
    if #cfg == 0 then
        reportError('Failed to find initial config')
        goto fail
    else
        cfg = cfg[1]
    end

    -- apply config in ik world:
    setIkConfig(cfg)

    -- follow path via IK solver:
    while not finished do
        if math.abs(posAlongPath - totalLength) < 1e-6 then finished = true end
        -- move target to next position:
        moveIkTarget(posAlongPath)
        -- if IK failed, return failure:
        local ikResult, failureCode = simIK.handleGroups(
                                          ikEnv, {ikGroup}, {callback = opts.jacobianCallback}
                                      )
        if ikResult ~= simIK.result_success then
            reportError(
                'Failed to perform IK step at t=%.2f (reason: %s)', posAlongPath / totalLength,
                simIK.getFailureDescription(failureCode)
            )
            goto fail
        end
        -- if collidableHandle given, and there is a collision, return failure:
        if #collisionPairs > 0 then
            local origSimCfg = getConfig()
            setConfig(getIkConfig())
            for i = 1, #collisionPairs, 2 do
                if sim.checkCollision(collisionPairs[i], collisionPairs[i + 1]) ~= 0 then
                    local function getObjectAlias(h)
                        if h == sim.handle_all then return '[[all]]' end
                        local r, a = pcall(sim.getObjectAlias, h)
                        return r and a or h
                    end
                    reportError(
                        'Failed due to collision %s/%s at t=%.2f',
                        getObjectAlias(collisionPairs[i]), getObjectAlias(collisionPairs[i + 1]),
                        posAlongPath / totalLength
                    )
                    setConfig(origSimCfg)
                    goto fail
                end
            end
            setConfig(origSimCfg)
        end
        callStepCb(false)
        -- otherwise store config and continue:
        table.insert(cfgs, getIkConfig())
        -- move position on path forward:
        posAlongPath = math.min(posAlongPath + delta, totalLength)
    end

    if cfgs then
        setIkConfig(origIkCfg)
        return cfgs
    end

    ::fail::
    callStepCb(true)
    setIkConfig(origIkCfg)
end

function simIK.addIkElementFromScene(...)
    -- deprecated
    return simIK.addElementFromScene(...)
end

function simIK.handleIkGroup(...)
    -- deprecated
    return simIK.handleGroup(...)
end

function simIK.applySceneToIkEnvironment(...)
    -- deprecated
    local ikEnv, ikGroup = checkargs({
        {type = 'int'},
        {type = 'int'},
    }, ...)

    return simIK.syncFromSim(ikEnv, {ikGroup})
end

function simIK.applyIkEnvironmentToScene(...)
    -- deprecated
    local ikEnv, ikGroup, applyOnlyWhenSuccessful = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'bool', default = false},
    }, ...)

    local lb = sim.setStepping(true)

    simIK.syncFromSim(ikEnv, {ikGroup})
    local groupData = _S.ikEnvs[ikEnv].ikGroups[ikGroup]
    local res, reason, prec = simIK.handleGroups(ikEnv, {ikGroup})
    if res == simIK.result_success or not applyOnlyWhenSuccessful then
        simIK.syncToSim(ikEnv, {ikGroup})
    end
    sim.setStepping(lb)
    return res, reason, prec
end

function simIK.getConfigForTipPose(...)
    -- deprecated
    local ikEnv, ikGroup, joints, thresholdDist, maxTime, metric, callback, auxData, jointOptions,
          lowLimits, ranges = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'table', size = '1..*', item_type = 'int'},
        {type = 'float', default = 0.1},
        {type = 'float', default = 0.5},
        {type = 'table', size = 4, item_type = 'float', default = {1, 1, 1, 0.1}, nullable = true},
        {type = 'any', default = NIL, nullable = true},
        {type = 'any', default = NIL, nullable = true},
        {type = 'table', size = '1..*', item_type = 'int', default = NIL, nullable = true},
        {type = 'table', size = '1..*', item_type = 'float', default = NIL, nullable = true},
        {type = 'table', size = '1..*', item_type = 'float', default = NIL, nullable = true},
    }, ...)

    local dof = #joints

    if (jointOptions and dof ~= #jointOptions) or (lowLimits and dof ~= #lowLimits) or
        (ranges and dof ~= #ranges) then error("Bad table size.") end

    local lb = sim.setStepping(true)

    local env = simIK.duplicateEnvironment(ikEnv)
    if metric == nil then metric = {1, 1, 1, 0.1} end
    if jointOptions == nil then jointOptions = {} end
    if lowLimits == nil then lowLimits = {} end
    if ranges == nil then ranges = {} end
    local retVal
    if type(callback) == 'string' then
        -- deprecated
        retVal = simIK._getConfigForTipPose(
                     env, ikGroup, joints, thresholdDist, maxTime, metric, callback, auxData,
                     jointOptions, lowLimits, ranges
                 )
    else
        if maxTime < 0 then
            maxTime = -maxTime / 1000 -- probably calling the function the old way 
        end
        if maxTime > 2 then maxTime = 2 end
        function __cb(config)
            return callback(config, auxData)
        end
        local funcNm, t
        if callback then
            funcNm = '__cb'
            t = sim.getScript(sim.handle_self)
        end
        retVal = simIK._getConfigForTipPose(
                     env, ikGroup, joints, thresholdDist, -maxTime * 1000, metric, funcNm, t,
                     jointOptions, lowLimits, ranges
                 )
    end
    simIK.eraseEnvironment(env)
    sim.setStepping(lb)
    return retVal
end

function simIK.findConfigs(...)
    local ikEnv, ikGroup, ikJoints, params, otherConfigs = checkargs({
        {type = 'int'},
        {type = 'int'},
        {type = 'table', size = '1..*', item_type = 'int'},
        {type = 'table', default = NIL, nullable = true},
        {type = 'table', size = '1..*', item_type = 'int', default = NIL, nullable = true},
    }, ...)
    params = params or {}
    -- params.findMultiple 
    if params.findAlt == nil then params.findAlt = true end 
    params.maxDist = params.maxDist or 0.3
    params.maxTime = params.maxTime or 0.2
    params.pMetric = params.pMetric or {1.0, 1.0, 1.0, 0.1}
    params.cMetric = params.cMetric or table.rep(1.0, #ikJoints)
    otherConfigs = otherConfigs or {}
    local lb = sim.setStepping(true)
    local retConfs = {}
    
    local st = sim.getSystemTime()
    while true do
        local ct = sim.getSystemTime()
        local conf = simIK.findConfig(ikEnv, ikGroup, ikJoints, params.maxDist, math.max(params.maxTime - (ct - st), 0.01), params.pMetric, params.cb, params.auxData)
        if conf then
            if #retConfs == 0 then
                retConfs = otherConfigs
            end
            local altConfigs = {}
            if params.findAlt then
                -- find all alternate, valid configs:
                altConfigs = _S.simIKGetAltConfigs(ikEnv, ikJoints, conf)
                if params.cb then
                    local cnt = 1
                    while cnt <= #altConfigs do
                        if params.cb(altConfigs[cnt], params.auxData) then
                            cnt = cnt + 1
                        else
                            table.remove(altConfigs, cnt)
                        end
                    end
                end
            end
            retConfs[#retConfs + 1] = conf
            for i = 1, #altConfigs do
                retConfs[#retConfs + 1] = altConfigs[i]
            end
        end
        if (not params.findMultiple) or (ct - st >= params.maxTime) then
            break
        end
    end
        
    if #retConfs > 1 then
        -- Order configs according to proximity to current config:
        local cc = simIK.getConfig(ikEnv, ikJoints)
        local configs = {}
        local dists = {}
        for i = 1, #retConfs do
            local d = sim.getConfigDistance(cc, retConfs[i], params.cMetric)
            dists[i] = d
            configs[d] = retConfs[i]
        end
        retConfs = {}
        table.sort(dists)
        for i = 1, #dists do
            retConfs[#retConfs + 1] = configs[dists[i]]
        end
    end
    sim.setStepping(lb)
    return retConfs
end

function simIK.getConfig(ikEnv, jh)
    local retVal = {}
    for i = 1, #jh do
        retVal[i] = simIK.getJointPosition(ikEnv, jh[i])
    end
    return retVal
end

function simIK.setConfig(ikEnv, jh, config)
    for i = 1, #jh do
        simIK.setJointPosition(ikEnv, jh[i], config[i])
    end
end

function _S.simIKGetAltConfigs(ikEnv, jointHandles, inputConfig)
    local dof = #jointHandles
    local retVal = {}
    local x = {}
    local confS = {}
    local err = false
    for i = 1, #jointHandles, 1 do
        local c, interv = simIK.getJointInterval(ikEnv, jointHandles[i])
        local t = simIK.getJointType(ikEnv, jointHandles[i])
        local sp = simIK.getJointScrewLead(ikEnv, jointHandles[i])
        if t == simIK.jointtype_revolute and not c then
            if sp == 0 then
                if inputConfig[i] - math.pi * 2 >= interv[1] or inputConfig[i] + math.pi * 2 <=
                    interv[1] + interv[2] then
                    -- We use the low and range values from the joint's settings
                    local y = inputConfig[i]
                    while y - math.pi * 2 >= interv[1] do y = y - math.pi * 2 end
                    x[i] = {y, interv[1] + interv[2]}
                end
            end
        end
        if not x[i] then
            -- there's no alternative position for this joint
            x[i] = {inputConfig[i], inputConfig[i]}
        end
        confS[i] = x[i][1]
    end
    local configs = {}
    if not err then
        local desiredPose = 0
        configs = _S.simIKLoopThroughAltConfigSolutions(
                      ikEnv, jointHandles, desiredPose, confS, x, 1)
    end
    
    -- Exclude the input config:
    for j = 1, #configs do
        local distSq = 0.0
        for i = 1, #inputConfig do
            local d = inputConfig[i] - configs[j][i]
            distSq = distSq + d * d
        end
        if distSq < 0.1 then
            table.remove(configs, j)
            break
        end
    end
    return configs
end

return simIK
