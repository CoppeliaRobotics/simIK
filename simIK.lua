local simIK={}

function _S.simIKLoopThroughAltConfigSolutions(ikEnvironment,jointHandles,desiredPose,confS,x,index)
    if index>#jointHandles then
        return {sim.unpackDoubleTable(sim.packDoubleTable(confS))} -- copy the table
    else
        local c={}
        for i=1,#jointHandles,1 do
            c[i]=confS[i]
        end
        local solutions={}
        while c[index]<=x[index][2] do
            local s=_S.simIKLoopThroughAltConfigSolutions(ikEnvironment,jointHandles,desiredPose,c,x,index+1,tipHandle)
            for i=1,#s,1 do
                solutions[#solutions+1]=s[i]
            end
            c[index]=c[index]+math.pi*2
        end
        return solutions
    end
end

function simIK.getAlternateConfigs(...)
    local ikEnvironment,jointHandles,lowLimits,ranges=checkargs({{type='int'},{type='table',size='1..*',item_type='int'},{type='table',size='1..*',item_type='float',default=NIL,nullable=true},{type='table',size='1..*',item_type='float',default=NIL,nullable=true}},...)
    local dof=#jointHandles
    if (lowLimits and dof~=#lowLimits) or (ranges and dof~=#ranges) then
        error("Bad table size.")
    end

    local lb=sim.setThreadAutomaticSwitch(false)

    local retVal={}
    local ikEnv=simIK.duplicateEnvironment(ikEnvironment)
    local x={}
    local confS={}
    local err=false
    local inputConfig={}
    for i=1,#jointHandles,1 do
        inputConfig[i]=simIK.getJointPosition(ikEnv,jointHandles[i])
        local c,interv=simIK.getJointInterval(ikEnv,jointHandles[i])
        local t=simIK.getJointType(ikEnv,jointHandles[i])
        local sp=simIK.getJointScrewLead(ikEnv,jointHandles[i])
        if t==simIK.jointtype_revolute and not c then
            if sp==0 then
                if inputConfig[i]-math.pi*2>=interv[1] or inputConfig[i]+math.pi*2<=interv[1]+interv[2] then
                    -- We use the low and range values from the joint's settings
                    local y=inputConfig[i]
                    while y-math.pi*2>=interv[1] do
                        y=y-math.pi*2
                    end
                    x[i]={y,interv[1]+interv[2]}
                end
            end
        end
        if x[i] then
            if lowLimits and ranges then
                -- the user specified low and range values. Use those instead:
                local l=lowLimits[i]
                local r=ranges[i]
                if r~=0 then
                    if r>0 then
                        if l<interv[1] then
                            -- correct for user bad input
                            r=r-(interv[1]-l)
                            l=interv[1] 
                        end
                        if l>interv[1]+interv[2] then
                            -- bad user input. No alternative position for this joint
                            x[i]={inputConfig[i],inputConfig[i]}
                            err=true
                        else
                            if l+r>interv[1]+interv[2] then
                                -- correct for user bad input
                                r=interv[1]+interv[2]-l
                            end
                            if inputConfig[i]-math.pi*2>=l or inputConfig[i]+math.pi*2<=l+r then
                                local y=inputConfig[i]
                                while y<l do
                                    y=y+math.pi*2
                                end
                                while y-math.pi*2>=l do
                                    y=y-math.pi*2
                                end
                                x[i]={y,l+r}
                            else
                                -- no alternative position for this joint
                                x[i]={inputConfig[i],inputConfig[i]}
                                err=(inputConfig[i]<l) or (inputConfig[i]>l+r)
                            end
                        end
                    else
                        r=-r
                        l=inputConfig[i]-r*0.5
                        if l<x[i][1] then
                            l=x[i][1]
                        end
                        local u=inputConfig[i]+r*0.5
                        if u>x[i][2] then
                            u=x[i][2]
                        end
                        x[i]={l,u}
                    end
                end
            end
        else
            -- there's no alternative position for this joint
            x[i]={inputConfig[i],inputConfig[i]}
        end
        confS[i]=x[i][1]
    end
    local configs={}
    if not err then
        for i=1,#jointHandles,1 do
            simIK.setJointPosition(ikEnv,jointHandles[i],inputConfig[i])
        end
        local desiredPose=0
        configs=_S.simIKLoopThroughAltConfigSolutions(ikEnv,jointHandles,desiredPose,confS,x,1)
    end
    simIK.eraseEnvironment(ikEnv)
    sim.setThreadAutomaticSwitch(lb)
    
    if next(configs)~=nil then
        configs=Matrix:fromtable(configs)
        configs=configs:data()
    end
    return configs
end

function simIK.syncFromSim(...)
    local ikEnv,ikGroups=checkargs({{type='int'},{type='table'}},...)
    local lb=sim.setThreadAutomaticSwitch(false)
    for g=1,#ikGroups,1 do
        local groupData=_S.ikEnvs[ikEnv].ikGroups[ikGroups[g]]
        for k,v in pairs(groupData.joints) do
            if sim.isHandle(k) then
                if sim.getJointType(k)==sim.joint_spherical_subtype then
                    simIK.setSphericalJointMatrix(ikEnv,v,sim.getJointMatrix(k))
                else
                    simIK.setJointPosition(ikEnv,v,sim.getJointPosition(k))
                end
            else
                -- that is probably a joint in a dependency relation, that was removed
                simIK.eraseObject(ikEnv,v)
                groupData.joints[k]=nil
                _S.ikEnvs[ikEnv].simToIkMap[k]=nil
                _S.ikEnvs[ikEnv].ikToSimMap[v]=nil
            end
        end
        for i=1,#groupData.targetTipBaseTriplets,1 do
            -- Make sure target relative to base is in sync too:
            simIK.setObjectMatrix(ikEnv,groupData.targetTipBaseTriplets[i][4],groupData.targetTipBaseTriplets[i][6],sim.getObjectMatrix(groupData.targetTipBaseTriplets[i][1],groupData.targetTipBaseTriplets[i][3]))
        end
    end
    sim.setThreadAutomaticSwitch(lb)
end

function simIK.syncToSim(...)
    local ikEnv,ikGroups=checkargs({{type='int'},{type='table'}},...)
    local lb=sim.setThreadAutomaticSwitch(false)
    for g=1,#ikGroups,1 do
        local groupData=_S.ikEnvs[ikEnv].ikGroups[ikGroups[g]]
        for k,v in pairs(groupData.joints) do
            if sim.isHandle(k) then
                if sim.getJointType(k)==sim.joint_spherical_subtype then
                    if sim.getJointMode(k)~=sim.jointmode_force or not sim.isDynamicallyEnabled(k) then
                        sim.setSphericalJointMatrix(k,simIK.getJointMatrix(ikEnv,v))
                    end
                else
                    if sim.getJointMode(k)==sim.jointmode_force and sim.isDynamicallyEnabled(k) then
                        sim.setJointTargetPosition(k,simIK.getJointPosition(ikEnv,v))
                    else    
                        sim.setJointPosition(k,simIK.getJointPosition(ikEnv,v))
                    end
                end
            else
                -- that is probably a joint in a dependency relation, that was removed
                simIK.eraseObject(ikEnv,v)
                groupData.joints[k]=nil
                _S.ikEnvs[ikEnv].simToIkMap[k]=nil
                _S.ikEnvs[ikEnv].ikToSimMap[v]=nil
            end
        end
    end
    sim.setThreadAutomaticSwitch(lb)
end

function simIK.debugGroupIfNeeded(ikEnv,ikGroup,debugFlags)
    if _S.ikEnvs[ikEnv] then -- when an IK environment is duplicated, it does not appear in _S.ikEnvs...
        if sim.getNamedBoolParam('simIK.debug_world') or ((debugFlags&1)~=0) then
            local lb=sim.setThreadAutomaticSwitch(false)
            local groupData=_S.ikEnvs[ikEnv].ikGroups[ikGroup]
            if groupData.visualDebug then
                for i=1,#groupData.visualDebug,1 do
                    simIK.eraseDebugOverlay(groupData.visualDebug[i])
                end
            end
            groupData.visualDebug={}
            for i=1,#groupData.targetTipBaseTriplets,1 do
                groupData.visualDebug[i]=simIK.createDebugOverlay(ikEnv,groupData.targetTipBaseTriplets[i][5],groupData.targetTipBaseTriplets[i][6])
            end
            sim.setThreadAutomaticSwitch(lb)
        else
            local groupData=_S.ikEnvs[ikEnv].ikGroups[ikGroup]
            if groupData.visualDebug then
                for i=1,#groupData.visualDebug,1 do
                    simIK.eraseDebugOverlay(groupData.visualDebug[i])
                end
            end
            groupData.visualDebug={}
        end
    end
end

function simIK.addElementFromScene(...)
    local ikEnv,ikGroup,simBase,simTip,simTarget,constraints=checkargs({{type='int'},{type='int'},{type='int'},{type='int'},{type='int'},{type='int'}},...)
    
    local lb=sim.setThreadAutomaticSwitch(false)
    
    if not _S.ikEnvs then
        _S.ikEnvs={}
    end
    if not _S.ikEnvs[ikEnv] then
        _S.ikEnvs[ikEnv]={}
    end
    if not _S.ikEnvs[ikEnv].ikGroups then
        _S.ikEnvs[ikEnv].ikGroups={}
        _S.ikEnvs[ikEnv].simToIkMap={}
        _S.ikEnvs[ikEnv].ikToSimMap={}
    end
    local groupData=_S.ikEnvs[ikEnv].ikGroups[ikGroup]
    -- simToIkMap and ikToSimMap, need to be scoped by ik env, and not ik group,
    -- otherwise we may have duplicates:
    local simToIkMap=_S.ikEnvs[ikEnv].simToIkMap
    local ikToSimMap=_S.ikEnvs[ikEnv].ikToSimMap
    if not groupData then
        groupData={}
        groupData.joints={}
        groupData.targetTipBaseTriplets={}
        _S.ikEnvs[ikEnv].ikGroups[ikGroup]=groupData
    end

    function createIkJointFromSimJoint(ikEnv,simJoint)
        local t=sim.getJointType(simJoint)
        local ikJoint=simIK.createJoint(ikEnv,t)
        local c,interv=sim.getJointInterval(simJoint)
        simIK.setJointInterval(ikEnv,ikJoint,c,interv)
        local sp=sim.getObjectFloatParam(simJoint,sim.jointfloatparam_screwlead)
        simIK.setJointScrewLead(ikEnv,ikJoint,sp)
        local sp=sim.getObjectFloatParam(simJoint,sim.jointfloatparam_step_size)
        simIK.setJointMaxStepSize(ikEnv,ikJoint,sp)
        local sp=sim.getObjectFloatParam(simJoint,sim.jointfloatparam_ik_weight)
        simIK.setJointWeight(ikEnv,ikJoint,sp)
        if t==sim.joint_spherical_subtype then
            simIK.setSphericalJointMatrix(ikEnv,ikJoint,sim.getJointMatrix(simJoint))
        else
            simIK.setJointPosition(ikEnv,ikJoint,sim.getJointPosition(simJoint))
        end
        return ikJoint
    end
    
    function iterateAndAdd(theTip,theBase)
        local ikPrevIterator=-1
        local simIterator=theTip
        while true do
            local ikIterator=-1
            if simToIkMap[simIterator] then
                -- object already added (but maybe parenting not done yet, e.g. with master joints in dependency relationship)
                ikIterator=simToIkMap[simIterator]
            else
                if sim.getObjectType(simIterator)~=sim.object_joint_type then
                    ikIterator=simIK.createDummy(ikEnv)
                else
                    ikIterator=createIkJointFromSimJoint(ikEnv,simIterator)
                end
                simToIkMap[simIterator]=ikIterator
                ikToSimMap[ikIterator]=simIterator
                simIK.setObjectMatrix(ikEnv,ikIterator,-1,sim.getObjectMatrix(simIterator,-1))
            end 
            if sim.getObjectType(simIterator)==sim.object_joint_type then
                groupData.joints[simIterator]=ikIterator
            end
            if ikPrevIterator~=-1 then
                simIK.setObjectParent(ikEnv,ikPrevIterator,ikIterator)
            end
            local newSimIterator=sim.getObjectParent(simIterator)
            if simIterator==theBase or newSimIterator==-1 then
                break
            end
            simIterator=newSimIterator
            ikPrevIterator=ikIterator
        end
    end
    
    iterateAndAdd(simTip,-1) -- need to add the whole chain down to world, otherwise subtle bugs!
    local ikTip=simToIkMap[simTip]
    local ikBase=-1;
    if simBase~=-1 then
        ikBase=simToIkMap[simBase]
    end
    iterateAndAdd(simTarget,-1) -- need to add the whole target chain down to world too, otherwise subtle bugs!
    local ikTarget=simToIkMap[simTarget]
    simIK.setTargetDummy(ikEnv,ikTip,ikTarget)
    groupData.targetTipBaseTriplets[#groupData.targetTipBaseTriplets+1]={simTarget,simTip,simBase,ikTarget,ikTip,ikBase}
    
    -- Now handle joint dependencies. Consider slave0 --> slave1 --> .. --> master
    -- We add all master and slave joints, even if not in the IK world (for simplification, since we could have complex daisy chains)
    -- We will however have to be careful, and always first check if a joint is still valid (e.g. we could remove a joint from the scene
    -- in an unrelated model)
    local simJoints=sim.getObjectsInTree(sim.handle_scene,sim.object_joint_type)
    local slaves={}
    local masters={}
    local passives={}
    for i=1,#simJoints,1 do
        local jo=simJoints[i]
        if sim.getJointMode(jo)==sim.jointmode_dependent then
            local dep,off,mult=sim.getJointDependency(jo)
            if dep~=-1 then
                slaves[#slaves+1]=jo
                masters[#masters+1]=dep
            else
                passives[#passives+1]=jo
            end
        end
    end
    for i=1,#passives,1 do
        local ikJo=simToIkMap[passives[i]]
        if ikJo then
            simIK.setJointMode(ikEnv,ikJo,simIK.jointmode_passive)
        end
    end
    for i=1,#slaves,1 do
        local slave=slaves[i]
        local master=masters[i]
        local ikJo_s=simToIkMap[slave]
        local ikJo_m=simToIkMap[master]
        if ikJo_s==nil then
            ikJo_s=createIkJointFromSimJoint(ikEnv,slave)
            simToIkMap[slave]=ikJo_s
            ikToSimMap[ikJo_s]=slave
            simIK.setObjectMatrix(ikEnv,ikJo_s,-1,sim.getObjectMatrix(slave,-1))
            groupData.joints[slave]=ikJo_s
        end
        if ikJo_m==nil then
            ikJo_m=createIkJointFromSimJoint(ikEnv,master)
            simToIkMap[master]=ikJo_m
            ikToSimMap[ikJo_m]=master
            simIK.setObjectMatrix(ikEnv,ikJo_m,-1,sim.getObjectMatrix(master,-1))
            groupData.joints[master]=ikJo_m
        end
        local dep,off,mult=sim.getJointDependency(slave)
        simIK.setJointDependency(ikEnv,ikJo_s,ikJo_m,off,mult) 
    end

    local ikElement=simIK.addElement(ikEnv,ikGroup,ikTip)
    simIK.setElementBase(ikEnv,ikGroup,ikElement,ikBase,-1)
    simIK.setElementConstraints(ikEnv,ikGroup,ikElement,constraints)
    sim.setThreadAutomaticSwitch(lb)
    return ikElement,simToIkMap,ikToSimMap
end

function simIK.eraseEnvironment(...)
    local ikEnv=checkargs({{type='int'}},...)
    
    local lb=sim.setThreadAutomaticSwitch(false)
    
    if _S.ikEnvs then
        _S.ikEnvs[ikEnv]=nil
    end
    simIK._eraseEnvironment(ikEnv)
    sim.setThreadAutomaticSwitch(lb)
end

function simIK.findConfig(...)
    local ikEnv,ikGroup,joints,thresholdDist,maxTime,metric,callback,auxData=checkargs({{type='int'},{type='int'},{type='table',size='1..*',item_type='int'},{type='float',default=0.1},{type='float',default=0.5},{type='table',size=4,item_type='float',default={1,1,1,0.1},nullable=true},{type='any',default=NIL,nullable=true},{type='any',default=NIL,nullable=true}},...)
    local dof=#joints
    local lb=sim.setThreadAutomaticSwitch(false)

    --local env=simIK.duplicateEnvironment(ikEnv)
    local env=ikEnv
    if metric==nil then metric={1,1,1,0.1} end
    function __cb(config)
        if type(callback)=='string' then
            return _G[callback](config,auxData)
        else
            return callback(config,auxData)
        end
    end
    local funcNm,t
    if callback then
        funcNm='__cb'
        t=sim.getScriptInt32Param(sim.handle_self,sim.scriptintparam_handle)
    end
    local retVal=simIK._findConfig(env,ikGroup,joints,thresholdDist,maxTime*1000,metric,funcNm,t)
    --simIK.eraseEnvironment(env)
    sim.setThreadAutomaticSwitch(lb)
    return retVal
end

function simIK.handleGroup(...) -- convenience function
    local ikEnv,ikGroup,options=checkargs({{type='int'},{type='int'},{type='table',default={}}},...)
    local ikGroups={ikGroup}
    return simIK.handleGroups(ikEnv,ikGroups,options)
end

function simIK.debugJacobianDisplay(inData)
    local groupData=_S.ikEnvs[_S.currentIkEnv].ikGroups[inData.groupHandle]
    local groupIdStr=string.format('env:%d/group:%d',_S.currentIkEnv,inData.groupHandle)
    if simQML then
        if groupData.jacobianDebug==nil then
            groupData.jacobianDebug={
                qmlEngine=simQML.createEngine()
            }
            function jacobianDebugClicked()
                jacobianDebugPrint=true
            end
            simQML.setEventHandler(groupData.jacobianDebug.qmlEngine,'dispatchEventsToFunctions')
            simQML.loadData(groupData.jacobianDebug.qmlEngine,[[
                import QtQuick 2.12
                import QtQuick.Window 2.12
                import CoppeliaSimPlugin 1.0

                PluginWindow {
                    id: mainWindow
                    width: cellSize * cols
                    height: cellSize * rows
                    title: "Jacobian ]]..groupIdStr..[["

                    readonly property string groupId: "]]..groupIdStr..[["

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
            ]])
        end
        simQML.sendEvent(groupData.jacobianDebug.qmlEngine,'setData',{groupId=groupIdStr,jacobian=inData.jacobian:totable()})
    end
    if jacobianDebugPrint then
        jacobianDebugPrint=false
        inData.jacobian:print'J'
    end
end

function simIK.handleGroups(...)
    local ikEnv,ikGroups,options=checkargs({{type='int'},{type='table'},{type='table',default={}}},...)
    local lb=sim.setThreadAutomaticSwitch(false)
    _S.currentIkEnv=ikEnv
    local debugFlags=0
    if options.debug then
        debugFlags=options.debug
    end
    local debugJacobian=( ((debugFlags&2)~=0) or sim.getNamedBoolParam('simIK.debug_world') ) and _S.ikEnvs[ikEnv] -- when an IK environment is duplicated, it does not appear in _S.ikEnvs...

    function __cb(rows_constr,rows_ikEl,cols_handles,cols_dofIndex,jacobian,errorVect,groupId,iteration)
        local data={}
        data.jacobian=Matrix({data=jacobian,dims={#rows_constr,#cols_handles}})
        data.rows={}
        data.cols={}
        data.e=Matrix({data=errorVect,dims={#rows_constr,1}})
        for i=1,#rows_constr,1 do
            data.rows[i]={constraint=rows_constr[i],element=rows_ikEl[i]}
        end
        for i=1,#cols_handles,1 do
            data.cols[i]={joint=cols_handles[i],dofIndex=cols_dofIndex[i]}
        end
        data.groupHandle=groupId
        data.iteration=iteration
        if debugJacobian then
            simIK.debugJacobianDisplay(data)
        end
        local j={}
        local e={}
        local dq={}
        local jpinv={}
        if options.callback then
            local outData
            if type(options.callback)=='string' then
                outData=_G[options.callback](data,options.auxData)
            else
                outData=options.callback(data,options.auxData)
            end
            if outData then
                if outData.jacobian then
                    if outData.jacobian:cols()==#cols_handles and outData.jacobian:rows()==#rows_constr then
                        j=outData.jacobian:data()
                    else
                        error("invalid jacobian matrix size")
                    end
                end
                if outData.e then
                    if outData.e:rows()==#rows_constr and outData.e:cols()==1 then
                        e=outData.e:data()
                    else
                        error("invalid e vector size")
                    end
                end
                if outData.dq then
                    if outData.dq:rows()==#cols_handles and outData.dq:cols()==1 then
                        dq=outData.dq:data()
                    else
                        error("invalid dq vector size")
                    end
                end
                if outData.jacobianPinv then
                    if outData.jacobianPinv:rows()==#cols_handles and outData.jacobianPinv:cols()==#rows_constr then
                        jpinv=outData.jacobianPinv:data()
                    else
                        error("invalid jacobian pseudo-inverse matrix size")
                    end
                end
            end
        end
        return j,e,dq,jpinv
    end
    local funcNm,t
    if options.callback or debugJacobian then
        funcNm='__cb'
        t=sim.getScriptInt32Param(sim.handle_self,sim.scriptintparam_handle)
    end
    if options.syncWorlds then
        simIK.syncFromSim(ikEnv,ikGroups)
    end
    local retVal,reason,prec=simIK._handleGroups(ikEnv,ikGroups,funcNm,t)
    if options.syncWorlds then
        if (reason&simIK.calc_notwithintolerance)==0 or options.allowError then
            simIK.syncToSim(ikEnv,ikGroups)
        end
    end
    for i=1,#ikGroups,1 do
        simIK.debugGroupIfNeeded(ikEnv,ikGroups[i],debugFlags)
    end
    sim.setThreadAutomaticSwitch(lb)
    return retVal,reason,prec
end

function simIK.getFailureDescription(reason)
    local d={}
    for _,k in ipairs{
        'notperformed',
        'cannotinvert',
        'notwithintolerance',
        'stepstoobig',
        'limithit',
    } do
        local f='calc_'..k
        if reason&simIK[f]>0 then
            reason=reason&~simIK[f]
            table.insert(d,k)
        end
    end
    if reason~=0 then
        table.insert(d,tostring(reason))
    end
    return table.tostring(d)
end

function simIK.setJointDependency(...)
    local ikEnv,slaveJoint,masterJoint,offset,mult,callback=checkargs({{type='int'},{type='int'},{type='int'},{type='float',default=0.0},{type='float',default=1.0},{type='any',default=NIL,nullable=true}},...)
    function __depcb(ikEnv,slaveJoint,masterPos)
        if type(callback)=='string' then
            return _G[callback](ikEnv,slaveJoint,masterPos)
        else
            return callback(ikEnv,slaveJoint,masterPos)
        end
    end
    local funcNm,t
    if callback then
        funcNm='__depcb'
        t=sim.getScriptInt32Param(sim.handle_self,sim.scriptintparam_handle)
    end
    simIK._setJointDependency(ikEnv,slaveJoint,masterJoint,offset,mult,funcNm,t)
    return retVal
end

function simIK.generatePath(...)
    local ikEnv,ikGroup,ikJoints,tip,ptCnt,callback,auxData=checkargs({{type='int'},{type='int'},{type='table',size='1..*',item_type='int'},{type='int'},{type='int'},{type='any',default=NIL,nullable=true},{type='any',default=NIL}},...)

    local lb=sim.setThreadAutomaticSwitch(false)

    local env=simIK.duplicateEnvironment(ikEnv)
    local targetHandle=simIK.getTargetDummy(env,tip)
    local startMatrix=simIK.getObjectMatrix(env,tip,-1)
    local goalMatrix=simIK.getObjectMatrix(env,targetHandle,-1)
    local retPath={{}}
    for i=1,#ikJoints,1 do
        retPath[1][i]=simIK.getJointPosition(env,ikJoints[i])
    end
    local success=true
    if callback then
        if type(callback)=='string' then
            success=_G[callback](retPath[1])
        else
            success=callback(retPath[1])
        end
    end
    if success then
        for j=1,ptCnt-1,1 do
            local t=j/(ptCnt-1)
            local m=sim.interpolateMatrices(startMatrix,goalMatrix,t)
            simIK.setObjectMatrix(env,targetHandle,-1,m)
            success=simIK.handleGroups(env,{ikGroup})==simIK.result_success
            if not success then
                break
            end
            retPath[j+1]={}
            for i=1,#ikJoints,1 do
                retPath[j+1][i]=simIK.getJointPosition(env,ikJoints[i])
            end
            if callback then
                if type(callback)=='string' then
                    success=_G[callback](retPath[j+1])
                else
                    success=callback(retPath[j+1])
                end
            end
            if not success then
                break
            end
        end
    end
    simIK.eraseEnvironment(env)
    sim.setThreadAutomaticSwitch(lb)
    if not success then
        retPath={}
    else
        retPath=Matrix:fromtable(retPath)
        retPath=retPath:data()
    end
    return retPath
end

function simIK.getObjectPose(...)
    local ikEnv,obj,relObj=checkargs({{type='int'},{type='int'},{type='int'}},...)
    local pos,quat=simIK.getObjectTransformation(ikEnv,obj,relObj)
    return {pos[1],pos[2],pos[3],quat[1],quat[2],quat[3],quat[4]}
end

function simIK.setObjectPose(...)
    local ikEnv,obj,relObj,pose=checkargs({{type='int'},{type='int'},{type='int'},{type='table',size=7,item_type='float'}},...)
    simIK.setObjectTransformation(ikEnv,obj,relObj,{pose[1],pose[2],pose[3]},{pose[4],pose[5],pose[6],pose[7]})
end

function simIK.createDebugOverlay(...)
    local ikEnv,ikTip,ikBase=checkargs({{type='int'},{type='int'},{type='int',default=-1}},...)
    if not _S.ikDebug then
        _S.ikDebug={}
        _S.ikDebugId=0
    end
    local drawingConts={}
    _S.ikDebug[_S.ikDebugId]=drawingConts
    _S.ikDebug[#_S.ikDebug+1]=drawingConts
    local ikTarget=simIK.getTargetDummy(ikEnv,ikTip)
    local targetCont=sim.addDrawingObject(sim.drawing_spherepts|sim.drawing_overlay,0.012,0,-1,1,{1,0,0})
    sim.addDrawingObjectItem(targetCont,simIK.getObjectPose(ikEnv,ikTarget,simIK.handle_world))
    drawingConts[#drawingConts+1]=targetCont
    local tipCont=sim.addDrawingObject(sim.drawing_spherepts|sim.drawing_overlay,0.01,0,-1,1,{0,1,0})
    sim.addDrawingObjectItem(tipCont,simIK.getObjectPose(ikEnv,ikTip,simIK.handle_world))
    drawingConts[#drawingConts+1]=tipCont
    local linkCont=sim.addDrawingObject(sim.drawing_lines|sim.drawing_overlay,2,0,-1,0,{0,0,0})
    drawingConts[#drawingConts+1]=linkCont
    local linkContN=sim.addDrawingObject(sim.drawing_spherepts|sim.drawing_overlay,0.01,0,-1,0,{1,1,1})
    drawingConts[#drawingConts+1]=linkContN
    local baseCont=sim.addDrawingObject(sim.drawing_cubepts|sim.drawing_overlay,0.01,0,-1,1,{1,0,1})
    local w={0,0,0,1,0,0,0}
    if ikBase~=-1 then
        w=simIK.getObjectPose(ikEnv,ikBase,simIK.handle_world)
    end
    sim.addDrawingObjectItem(baseCont,w)
    drawingConts[#drawingConts+1]=baseCont
    local obj=ikTip
    local prevObj=obj
    while obj~=-1 and obj~=ikBase do
        local t=simIK.getObjectType(ikEnv,obj)
        if t==simIK.objecttype_joint then
            local p=simIK.getObjectPose(ikEnv,prevObj,sim.handle_world)
            local m1=simIK.getObjectMatrix(ikEnv,obj,sim.handle_world)
            local m2=simIK.getJointMatrix(ikEnv,obj)
            m1=sim.multiplyMatrices(m1,m2)
            p[4]=m1[4]
            p[5]=m1[8]
            p[6]=m1[12]
            sim.addDrawingObjectItem(linkCont,p)
            local spherical=(simIK.getJointType(ikEnv,obj)==simIK.jointtype_spherical)
            local m=simIK.getJointMode(ikEnv,obj) -- simIK.jointmode_passive or simIK.jointmode_ik
            local d=simIK.getJointDependency(ikEnv,obj)
            local drt=sim.drawing_lines
            local s=4
            local hs=0.025
            if spherical then
                drt=sim.drawing_spherepts
                s=0.012
            end
            local c={1,0.5,0}
            if d>=0 then
                c={0,0.5,1}
                s=2
                hs=0.05
            elseif m==simIK.jointmode_passive then
                c={0.5,0.5,0.5}
            end
            local cont=sim.addDrawingObject(drt|sim.drawing_overlay,s,0,-1,0,c)
            drawingConts[#drawingConts+1]=cont
            local m=simIK.getObjectMatrix(ikEnv,obj,simIK.handle_world)
            if spherical then
                sim.addDrawingObjectItem(cont,{m[4],m[8],m[12]})
            else
                sim.addDrawingObjectItem(cont,{m[4]-m[3]*hs,m[8]-m[7]*hs,m[12]-m[11]*hs,m[4]+m[3]*hs,m[8]+m[7]*hs,m[12]+m[11]*hs})
            end
        else
            if prevObj~=obj then
                local p=simIK.getObjectPose(ikEnv,obj,sim.handle_world)
                local p2=simIK.getObjectPose(ikEnv,prevObj,sim.handle_world)
                p[4]=p2[1]
                p[5]=p2[2]
                p[6]=p2[3]
                sim.addDrawingObjectItem(linkCont,p)
                sim.addDrawingObjectItem(linkContN,p)
            end
        end
        prevObj=obj
        obj=simIK.getObjectParent(ikEnv,obj)
    end
    
    local p=simIK.getObjectPose(ikEnv,prevObj,sim.handle_world)
    p[4]=0
    p[5]=0
    p[6]=0
    if ikBase~=-1 then
        local p2=simIK.getObjectPose(ikEnv,ikBase,sim.handle_world)
        p[4]=p2[1]
        p[5]=p2[2]
        p[6]=p2[3]
    end
    sim.addDrawingObjectItem(linkCont,p)
    
    _S.ikDebugId=_S.ikDebugId+1
    return _S.ikDebugId-1
end

function simIK.eraseDebugOverlay(...)
    local id=checkargs({{type='int'}},...)
    local a=_S.ikDebug[id]
    if a then
        for i=1,#a,1 do
            sim.removeDrawingObject(a[i])
        end
        _S.ikDebug[id]=nil
    end
end

function simIK.solvePath(...)
    -- undocumented (for now) function
    -- simPath can be a Path object handle, or the path data itself
    -- ikPath a dummy with a pose and parent consistent with simPath
    local ikEnv,ikGroup,ikTarget,ikJoints,simJoints,ikPath,simPath,collisionPairs,opts=checkargs({{type='int'},{type='int'},{type='int'},{type='table',size='1..*',item_type='int'},{type='table',size='1..*',item_type='int'},{type='int'},{union={{type='handle'},{type='table'}}},{type='table',default={}},{type='table',default={}}},...)

    collisionPairs=collisionPairs or {}
    local delta=opts.delta or 0.005
    local errorCallback=opts.errorCallback or function(e) end
    local function reportError(...) errorCallback(string.format(...)) end

    local function callStepCb(failed) if opts.stepCallback then opts.stepCallback(failed) end end

    local pathData=simPath
    if math.type(simPath)=='integer' and sim.isHandle(simPath) then
        -- read path data inside path object:
        pathData=sim.readCustomDataBlock(simPath,'PATH')
        assert(pathData~=nil,'object does not contain PATH data')
        pathData=sim.unpackDoubleTable(pathData)
    end
    local m=Matrix(#pathData//7,7,pathData)
    local pathPositions=m:slice(1,1,m:rows(),3):data()
    local pathQuaternions=m:slice(1,4,m:rows(),7):data()
    local pathLengths,totalLength=sim.getPathLengths(pathPositions,3)

    local moveIkTarget=opts.moveIkTarget or function(posAlongPath)
        local pose=sim.getPathInterpolatedConfig(pathData,pathLengths,posAlongPath,nil,{0,0,0,2,2,2,2})
        -- re-fetch path pose, should path be part of the IK chain:
        if ikPath~=-1 then
            local pathPose=simIK.getObjectPose(ikEnv,ikPath,-1)
            pose=sim.multiplyPoses(pathPose,pose)
        end
        simIK.setObjectPose(ikEnv,ikTarget,-1,pose)
    end
    local getConfig=opts.getConfig or partial(map,sim.getJointPosition,simJoints)
    local setConfig=opts.setConfig or partial(foreach,sim.setJointPosition,simJoints)
    local getIkConfig=opts.getIkConfig or partial(map,partial(simIK.getJointPosition,ikEnv),ikJoints)
    local setIkConfig=opts.setIkConfig or partial(foreach,partial(simIK.setJointPosition,ikEnv),ikJoints)

    -- save current robot config:
    local origIkCfg=getIkConfig()

    local cfgs={}
    local posAlongPath=0
    local finished=false

    -- find initial config:
    moveIkTarget(0)
    local cfg=simIK.findConfig(ikEnv,ikGroup,ikJoints)
    if not cfg then
        reportError('Failed to find initial config')
        goto fail
    end

    -- apply config in ik world:
    setIkConfig(cfg)

    -- follow path via IK solver:
    while not finished do
        if math.abs(posAlongPath-totalLength)<1e-6 then finished=true end
        -- move target to next position:
        moveIkTarget(posAlongPath)
        -- if IK failed, return failure:
        local ikResult,failureCode=simIK.handleGroups(ikEnv,{ikGroup},{callback=opts.jacobianCallback})
        if ikResult~=simIK.result_success then
            reportError('Failed to perform IK step at t=%.2f (reason: %s)',posAlongPath/totalLength,simIK.getFailureDescription(failureCode))
            goto fail
        end
        -- if collidableHandle given, and there is a collision, return failure:
        if #collisionPairs>0 then
            local origSimCfg=getConfig()
            setConfig(getIkConfig())
            for i=1,#collisionPairs,2 do
                if sim.checkCollision(collisionPairs[i],collisionPairs[i+1])~=0 then
                    local function getObjectAlias(h)
                        if h==sim.handle_all then return '[[all]]' end
                        local r,a=pcall(sim.getObjectAlias,h)
                        return r and a or h
                    end
                    reportError('Failed due to collision %s/%s at t=%.2f',getObjectAlias(collisionPairs[i]),getObjectAlias(collisionPairs[i+1]),posAlongPath/totalLength)
                    setConfig(origSimCfg)
                    goto fail
                end
            end
            setConfig(origSimCfg)
        end
        callStepCb(false)
        -- otherwise store config and continue:
        table.insert(cfgs,getIkConfig())
        -- move position on path forward:
        posAlongPath=math.min(posAlongPath+delta,totalLength)
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
    local ikEnv,ikGroup=checkargs({{type='int'},{type='int'}},...)
    return simIK.syncFromSim(ikEnv,{ikGroup})
end

function simIK.applyIkEnvironmentToScene(...)
    -- deprecated
    local ikEnv,ikGroup,applyOnlyWhenSuccessful=checkargs({{type='int'},{type='int'},{type='bool',default=false}},...)
    local lb=sim.setThreadAutomaticSwitch(false)
    
    simIK.syncFromSim(ikEnv,{ikGroup})
    local groupData=_S.ikEnvs[ikEnv].ikGroups[ikGroup]
    local res,reason,prec=simIK.handleGroups(ikEnv,{ikGroup})
    if res==simIK.result_success or not applyOnlyWhenSuccessful then
        simIK.syncToSim(ikEnv,{ikGroup})
    end
    sim.setThreadAutomaticSwitch(lb)
    return res,reason,prec
end

function simIK.getConfigForTipPose(...)
    -- deprecated
    local ikEnv,ikGroup,joints,thresholdDist,maxTime,metric,callback,auxData,jointOptions,lowLimits,ranges=checkargs({{type='int'},{type='int'},{type='table',size='1..*',item_type='int'},{type='float',default=0.1},{type='float',default=0.5},{type='table',size=4,item_type='float',default={1,1,1,0.1},nullable=true},{type='any',default=NIL,nullable=true},{type='any',default=NIL,nullable=true},{type='table',size='1..*',item_type='int',default=NIL,nullable=true},{type='table',size='1..*',item_type='float',default=NIL,nullable=true},{type='table',size='1..*',item_type='float',default=NIL,nullable=true}},...)
    local dof=#joints

    if (jointOptions and dof~=#jointOptions) or (lowLimits and dof~=#lowLimits) or (ranges and dof~=#ranges) then
        error("Bad table size.")
    end

    local lb=sim.setThreadAutomaticSwitch(false)

    local env=simIK.duplicateEnvironment(ikEnv)
    if metric==nil then metric={1,1,1,0.1} end
    if jointOptions==nil then jointOptions={} end
    if lowLimits==nil then lowLimits={} end
    if ranges==nil then ranges={} end
    local retVal
    if type(callback)=='string' then
        -- deprecated
        retVal=simIK._getConfigForTipPose(env,ikGroup,joints,thresholdDist,maxTime,metric,callback,auxData,jointOptions,lowLimits,ranges)
    else
        if maxTime<0 then 
            maxTime=-maxTime/1000 -- probably calling the function the old way 
        end
        if maxTime>2 then maxTime=2 end
        function __cb(config)
            return callback(config,auxData)
        end
        local funcNm,t
        if callback then
            funcNm='__cb'
            t=sim.getScriptInt32Param(sim.handle_self,sim.scriptintparam_handle)
        end
        retVal=simIK._getConfigForTipPose(env,ikGroup,joints,thresholdDist,-maxTime*1000,metric,funcNm,t,jointOptions,lowLimits,ranges)
    end
    simIK.eraseEnvironment(env)
    sim.setThreadAutomaticSwitch(lb)
    return retVal
end

function simIK.init()
    -- can only be executed once sim.* functions were initialized
    sim.registerScriptFunction('simIK.getAlternateConfigs@simIK','float[] configs=simIK.getAlternateConfigs(int environmentHandle,int[] jointHandles,float[] lowLimits=nil,float[] ranges=nil)')
    sim.registerScriptFunction('simIK.addElementFromScene@simIK','int ikElement,map simToIkMap,map ikToSimMap=simIK.addElementFromScene(int environmentHandle,int ikGroup,int baseHandle,int tipHandle,int targetHandle,int constraints)')
    sim.registerScriptFunction('simIK.syncFromSim@simIK','simIK.syncFromSim(int environmentHandle,int[] ikGroups)')
    sim.registerScriptFunction('simIK.syncToSim@simIK','simIK.syncToSim(int environmentHandle,int[] ikGroups)')
    sim.registerScriptFunction('simIK.handleGroup@simIK','int success,int flags,float[2] precision=simIK.handleGroup(int environmentHandle,int ikGroup,map options={})')
    sim.registerScriptFunction('simIK.handleGroups@simIK','int success,int flags,float[2] precision=simIK.handleGroups(int environmentHandle,int[] ikGroups,map options={})')
    sim.registerScriptFunction('simIK.eraseEnvironment@simIK','simIK.eraseEnvironment(int environmentHandle)')
    sim.registerScriptFunction('simIK.findConfig@simIK','float[] jointPositions=simIK.findConfig(int environmentHandle,int ikGroupHandle,int[] jointHandles,float thresholdDist=0.1,float maxTime=0.5,float[4] metric={1,1,1,0.1},func validationCallback=nil,any auxData=nil)')
    sim.registerScriptFunction('simIK.getFailureDescription@simIK','string description=simIK.getFailureDescription(int reason)')
    sim.registerScriptFunction('simIK.setJointDependency@simIK','simIK.setJointDependency(int environmentHandle,int jointHandle,int masterJointHandle,float offset=0.0,float mult=1.0,func callback=nil)')
    sim.registerScriptFunction('simIK.generatePath@simIK','float[] path=simIK.generatePath(int environmentHandle,int ikGroupHandle,int[] jointHandles,int tipHandle,int pathPointCount,func validationCallback=nil,any auxData=nil)')
    sim.registerScriptFunction('simIK.getObjectPose@simIK','float[7] pose=simIK.getObjectPose(int environmentHandle,int objectHandle,int relativeToObjectHandle)')
    sim.registerScriptFunction('simIK.setObjectPose@simIK','simIK.setObjectPose(int environmentHandle,int objectHandle,int relativeToObjectHandle,float[7] pose)')
    sim.registerScriptFunction('simIK.createDebugOverlay@simIK','int debugObject=simIK.createDebugOverlay(int environmentHandle,int tipHandle,int baseHandle=-1)')
    sim.registerScriptFunction('simIK.eraseDebugOverlay@simIK','simIK.eraseDebugOverlay(int debugObject)')
    simIK.init=nil
end

sim.registerScriptFuncHook('sysCall_init','simIK.init',true)

return simIK
