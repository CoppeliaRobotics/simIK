local simIK={}

function __HIDDEN__.simIKLoopThroughAltConfigSolutions(ikEnvironment,jointHandles,tipHandle,desiredPose,confS,x,index)
    if index>#jointHandles then
        for i=1,#jointHandles,1 do
            simIK.setJointPosition(ikEnvironment,jointHandles[i],confS[i])
        end
        local p=simIK.getObjectMatrix(ikEnvironment,tipHandle,-1)
        local axis,angle=sim.getRotationAxis(desiredPose,p)
        if math.abs(angle)<0.1*180/math.pi then -- checking is needed in case some joints are dependent on others
            return {sim.unpackDoubleTable(sim.packDoubleTable(confS))} -- copy the table
        else
            return {}
        end
    else
        local c={}
        for i=1,#jointHandles,1 do
            c[i]=confS[i]
        end
        local solutions={}
        while c[index]<=x[index][2] do
            local s=__HIDDEN__.simIKLoopThroughAltConfigSolutions(ikEnvironment,jointHandles,tipHandle,desiredPose,c,x,index+1)
            for i=1,#s,1 do
                solutions[#solutions+1]=s[i]
            end
            c[index]=c[index]+math.pi*2
        end
        return solutions
    end
end

function simIK.getAlternateConfigs(ikEnvironment,jointHandles,tipHandle,inputConfig,lowLimits,ranges)
    local retVal={}
    sim.setThreadAutomaticSwitch(false)
    local initConfig={}
    local x={}
    local confS={}
    for i=1,#jointHandles,1 do
        initConfig[i]=simIK.getJointPosition(ikEnvironment,jointHandles[i])
        local c,interv=simIK.getJointInterval(ikEnvironment,jointHandles[i])
        local t=simIK.getJointType(ikEnvironment,jointHandles[i])
        local sp=simIK.getJointScrewPitch(ikEnvironment,jointHandles[i])
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
                if l<interv[1] then
                    -- correct for user bad input
                    r=r-(interv[1]-l)
                    l=interv[1] 
                end
                if l>interv[1]+interv[2] then
                    -- bad user input. No alternative position for this joint
                    x[i]={inputConfig[i],inputConfig[i]}
                else
                    if l+r>interv[1]+interv[2] then
                        -- correct for user bad input
                        r=interv[1]+interv[2]-l
                    end
                    if inputConfig[i]-math.pi*2>=l or inputConfig[i]+math.pi*2<=l+r then
                        local y=inputConfig[i]
                        while y-math.pi*2>=l do
                            y=y-math.pi*2
                        end
                        x[i]={y,l+r}
                    else
                        -- no alternative position for this joint
                        x[i]={inputConfig[i],inputConfig[i]}
                    end
                end
            end
        else
            -- there's no alternative position for this joint
            x[i]={inputConfig[i],inputConfig[i]}
        end
        confS[i]=x[i][1]
    end
    for i=1,#jointHandles,1 do
        simIK.setJointPosition(ikEnvironment,jointHandles[i],inputConfig[i])
    end
    local desiredPose=simIK.getObjectMatrix(ikEnvironment,tipHandle,-1)
    local configs=__HIDDEN__.simIKLoopThroughAltConfigSolutions(ikEnvironment,jointHandles,tipHandle,desiredPose,confS,x,1)
    
    for i=1,#jointHandles,1 do
        simIK.setJointPosition(ikEnvironment,jointHandles[i],initConfig[i])
    end
    sim.setThreadAutomaticSwitch(true)
    return configs
end

function simIK.init()
    -- can only be executed once sim.* functions were initialized
    sim.registerScriptFunction('simIK.getAlternateConfigs@simIK','table configs=simIK.getAlternateConfigs(table jointHandles,\nnumber tipHandle,table inputConfig,table lowLimits=nil,table ranges=nil)')
    simIK.init=nil
end

if not __initFunctions then
    __initFunctions={}
end
__initFunctions[#__initFunctions+1]=simIK.init

return simIK
