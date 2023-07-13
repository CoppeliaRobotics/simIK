#include "simIK.h"
#include "envCont.h"
#include <simLib/simLib.h>
#include <ik.h>
#include <simMath/4X4Matrix.h>
#include <simMath/mathFuncs.h>
#include <iostream>
#include <cstdio>
#include <simLib/scriptFunctionData.h>
#include <algorithm>

#ifdef _WIN32
    #include <Windows.h>
    typedef CRITICAL_SECTION WMutex;
#endif
#if defined (__linux) || defined (__APPLE__)
    #include <pthread.h>
    typedef pthread_mutex_t WMutex;
#endif

static LIBRARY simLib;
static CEnvCont* _allEnvironments;
static std::string _pluginName;

bool _logCallback(int verbosity,const char* funcName,const char* msg)
{
    bool retVal=true;
    int v=sim_verbosity_none;
    if (verbosity==1)
        v=sim_verbosity_scripterrors;
    if (verbosity==2)
        v=sim_verbosity_scriptwarnings;
    if (verbosity==3)
        v=sim_verbosity_scriptinfos;
    if (verbosity==4)
        v=sim_verbosity_debug;
    if (verbosity==5)
        v=sim_verbosity_trace;
    if (verbosity!=1)
    {
        std::string m(funcName);
        m+=": ";
        m+=msg;
        simAddLog(_pluginName.c_str(),v,m.c_str());
    }
    else
        retVal=false;
    return(retVal);
}

struct SJointDependCB
{
    int ikEnv;
    int ikSlave;
    std::string cbString;
    int scriptHandle;
};

static std::vector<SJointDependCB> jointDependInfo;

void _removeJointDependencyCallback(int envId,int slaveJoint)
{
    for (int i=0;i<int(jointDependInfo.size());i++)
    {
        if (jointDependInfo[i].ikEnv==envId)
        {
            if ( (jointDependInfo[i].ikSlave==slaveJoint)||(slaveJoint==-1) )
            {
                jointDependInfo.erase(jointDependInfo.begin()+i);
                i--;
            }
        }
    }
}

// --------------------------------------------------------------------------------------
// simIK.createEnvironment
// --------------------------------------------------------------------------------------
const int inArgs_CREATEENVIRONMENT[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_CREATEENVIRONMENT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool res=false;
    if (D.readDataFromStack(p->stackID,inArgs_CREATEENVIRONMENT,inArgs_CREATEENVIRONMENT[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int flags=0;
        if ( (inData->size()>=1)&&(inData->at(0).int32Data.size()==1) )
            flags=inData->at(0).int32Data[0];
        std::string err;
        {
            res=ikCreateEnvironment(&retVal,flags);
            if (res)
                _allEnvironments->add(retVal,p->scriptID);
            else
                err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (res)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK._eraseEnvironment
// --------------------------------------------------------------------------------------
const int inArgs_ERASEENVIRONMENT[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_ERASEENVIRONMENT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_ERASEENVIRONMENT,inArgs_ERASEENVIRONMENT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                _removeJointDependencyCallback(envId,-1);
                if (ikEraseEnvironment())
                    _allEnvironments->removeFromEnvHandle(envId);
                else
                    err=ikGetLastError();
            }
            else
                err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.duplicateEnvironment
// --------------------------------------------------------------------------------------
const int inArgs_DUPLICATEENVIRONMENT[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_DUPLICATEENVIRONMENT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool res=false;
    if (D.readDataFromStack(p->stackID,inArgs_DUPLICATEENVIRONMENT,inArgs_DUPLICATEENVIRONMENT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                if (ikDuplicateEnvironment(&retVal))
                {
                    _allEnvironments->add(retVal,p->scriptID);
                    res=true;
                }
                else
                    err=ikGetLastError();
            }
            else
                err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (res)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.load
// --------------------------------------------------------------------------------------
const int inArgs_LOAD[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_charbuff,0,
};

void LUA_LOAD_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_LOAD,inArgs_LOAD[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string buff(inData->at(1).stringData[0]);
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                if (!ikLoad((unsigned char*)buff.c_str(),buff.length()))
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.save
// --------------------------------------------------------------------------------------
const int inArgs_SAVE[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SAVE_CALLBACK(SScriptCallBack* p)
{
    std::string retVal;
    bool res=false;
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SAVE,inArgs_SAVE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];

        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                size_t l;
                unsigned char* data=ikSave(&l);
                if (data!=nullptr)
                {
                    res=true;
                    retVal.assign(data,data+l);
                }
                else
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (res)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getObjectHandle
// --------------------------------------------------------------------------------------
const int inArgs_GETOBJECTHANDLE[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_string,0,
};

void LUA_GETOBJECTHANDLE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETOBJECTHANDLE,inArgs_GETOBJECTHANDLE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetObjectHandle(inData->at(1).stringData[0].c_str(),&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.doesObjectExist
// --------------------------------------------------------------------------------------
const int inArgs_DOESOBJECTEXIST[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_string,0,
};

void LUA_DOESOBJECTEXIST_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool retVal=false;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_DOESOBJECTEXIST,inArgs_DOESOBJECTEXIST[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                retVal=ikDoesObjectExist(inData->at(1).stringData[0].c_str());
                result=true;
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.eraseObject
// --------------------------------------------------------------------------------------
const int inArgs_ERASEOBJECT[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_ERASEOBJECT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_ERASEOBJECT,inArgs_ERASEOBJECT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objectHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                _removeJointDependencyCallback(envId,objectHandle);
                if (!ikEraseObject(objectHandle))
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getObjectParent
// --------------------------------------------------------------------------------------
const int inArgs_GETOBJECTPARENT[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETOBJECTPARENT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETOBJECTPARENT,inArgs_GETOBJECTPARENT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objectHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetObjectParent(objectHandle,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setObjectParent
// --------------------------------------------------------------------------------------
const int inArgs_SETOBJECTPARENT[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_bool,0,
};

void LUA_SETOBJECTPARENT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETOBJECTPARENT,inArgs_SETOBJECTPARENT[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objectHandle=inData->at(1).int32Data[0];
        int parentObjectHandle=inData->at(2).int32Data[0];
        bool keepInPlace=true;
        if ( (inData->size()>3)&&(inData->at(3).boolData.size()==1) )
            keepInPlace=inData->at(3).boolData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetObjectParent(objectHandle,parentObjectHandle,keepInPlace);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getObjectType
// --------------------------------------------------------------------------------------
const int inArgs_GETOBJECTTYPE[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETOBJECTTYPE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETOBJECTTYPE,inArgs_GETOBJECTTYPE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objectHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetObjectType(objectHandle,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getObjects
// --------------------------------------------------------------------------------------
const int inArgs_GETOBJECTS[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETOBJECTS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int objectHandle=-1;
    std::string objectName;
    bool isJoint=false;
    int jointType=ik_jointtype_revolute;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETOBJECTS,inArgs_GETOBJECTS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int index=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetObjects(size_t(index),&objectHandle,&objectName,&isJoint,&jointType);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(objectHandle));
        D.pushOutData(CScriptFunctionDataItem(objectName));
        D.pushOutData(CScriptFunctionDataItem(isJoint));
        D.pushOutData(CScriptFunctionDataItem(jointType));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.createDummy
// --------------------------------------------------------------------------------------
const int inArgs_CREATEDUMMY[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_string|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
};

void LUA_CREATEDUMMY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_CREATEDUMMY,inArgs_CREATEDUMMY[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                const char* nm=nullptr;
                if ( (inData->size()>1)&&(inData->at(1).stringData.size()==1)&&(inData->at(1).stringData[0].size()>0) )
                    nm=inData->at(1).stringData[0].c_str();
                result=ikCreateDummy(nm,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getTargetDummy
// --------------------------------------------------------------------------------------
const int inArgs_GETTARGETDUMMY[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETTARGETDUMMY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETTARGETDUMMY,inArgs_GETTARGETDUMMY[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int dummyHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetTargetDummy(dummyHandle,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setTargetDummy
// --------------------------------------------------------------------------------------
const int inArgs_SETTARGETDUMMY[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETTARGETDUMMY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETTARGETDUMMY,inArgs_SETTARGETDUMMY[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int dummyHandle=inData->at(1).int32Data[0];
        int targetDummyHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetTargetDummy(dummyHandle,targetDummyHandle);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getLinkedDummy OLD, use simIK.getTargetDummy instead
// --------------------------------------------------------------------------------------
const int inArgs_GETLINKEDDUMMY[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETLINKEDDUMMY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETLINKEDDUMMY,inArgs_GETLINKEDDUMMY[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int dummyHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetLinkedDummy(dummyHandle,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setLinkedDummy OLD, use simIK.setTargetDummy instead
// --------------------------------------------------------------------------------------
const int inArgs_SETLINKEDDUMMY[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETLINKEDDUMMY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETLINKEDDUMMY,inArgs_SETLINKEDDUMMY[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int dummyHandle=inData->at(1).int32Data[0];
        int linkedDummyHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetLinkedDummy(dummyHandle,linkedDummyHandle);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.createJoint
// --------------------------------------------------------------------------------------
const int inArgs_CREATEJOINT[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_string|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
};

void LUA_CREATEJOINT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_CREATEJOINT,inArgs_CREATEJOINT[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                int jType=inData->at(1).int32Data[0];
                const char* nm=nullptr;
                if ( (inData->size()>2)&&(inData->at(2).stringData.size()==1)&&(inData->at(2).stringData[0].size()>0) )
                    nm=inData->at(2).stringData[0].c_str();
                result=ikCreateJoint(nm,jType,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointType
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTTYPE[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTTYPE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTTYPE,inArgs_GETJOINTTYPE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointType(jointHandle,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointMode
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTMODE[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTMODE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTMODE,inArgs_GETJOINTMODE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointMode(jointHandle,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointMode
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTMODE[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETJOINTMODE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTMODE,inArgs_SETJOINTMODE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        int jointMode=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetJointMode(jointHandle,jointMode);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointInterval
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTINTERVAL[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTINTERVAL_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool cyclic=true;
    double interv[2];
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTINTERVAL,inArgs_GETJOINTINTERVAL[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointInterval(jointHandle,&cyclic,interv);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(cyclic));
        std::vector<double> iv(interv,interv+2);
        D.pushOutData(iv);
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointInterval
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTINTERVAL[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_bool,0,
    sim_script_arg_double|sim_script_arg_table,2,
};

void LUA_SETJOINTINTERVAL_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTINTERVAL,inArgs_SETJOINTINTERVAL[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        bool cyclic=inData->at(2).boolData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                double* interv=nullptr;
                if ( (inData->size()>3)&&(inData->at(3).doubleData.size()>=2) )
                    interv=&inData->at(3).doubleData[0];

                bool result=ikSetJointInterval(jointHandle,cyclic,interv);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointScrewLead
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTSCREWLEAD[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTSCREWLEAD_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double lead=0.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTSCREWLEAD,inArgs_GETJOINTSCREWLEAD[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointScrewLead(jointHandle,&lead);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(lead));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointScrewLead
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTSCREWLEAD[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_SETJOINTSCREWLEAD_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTSCREWLEAD,inArgs_SETJOINTSCREWLEAD[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double lead=inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetJointScrewLead(jointHandle,lead);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// simIK.getJointScrewPitch, deprecated
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTSCREWPITCH[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTSCREWPITCH_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double pitch=0.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTSCREWPITCH,inArgs_GETJOINTSCREWPITCH[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointScrewPitch(jointHandle,&pitch);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(pitch));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointScrewPitch, deprecated
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTSCREWPITCH[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_SETJOINTSCREWPITCH_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTSCREWPITCH,inArgs_SETJOINTSCREWPITCH[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double pitch=inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetJointScrewPitch(jointHandle,pitch);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointWeight
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTIKWEIGHT[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTIKWEIGHT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double weight=1.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTIKWEIGHT,inArgs_GETJOINTIKWEIGHT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointWeight(jointHandle,&weight);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(weight));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointWeight
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTIKWEIGHT[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_SETJOINTIKWEIGHT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTIKWEIGHT,inArgs_SETJOINTIKWEIGHT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double weight=inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetJointWeight(jointHandle,weight);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointLimitMargin
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTLIMITMARGIN[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTLIMITMARGIN_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double weight=1.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTLIMITMARGIN,inArgs_GETJOINTLIMITMARGIN[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointLimitMargin(jointHandle,&weight);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(weight));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointLimitMargin
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTLIMITMARGIN[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_SETJOINTLIMITMARGIN_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTLIMITMARGIN,inArgs_SETJOINTLIMITMARGIN[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double weight=inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetJointLimitMargin(jointHandle,weight);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointMaxStepSize
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTMAXSTEPSIZE[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTMAXSTEPSIZE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double stepSize=0.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTMAXSTEPSIZE,inArgs_GETJOINTMAXSTEPSIZE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointMaxStepSize(jointHandle,&stepSize);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(stepSize));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointMaxStepSize
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTMAXSTEPSIZE[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_SETJOINTMAXSTEPSIZE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTMAXSTEPSIZE,inArgs_SETJOINTMAXSTEPSIZE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double stepSize=inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetJointMaxStepSize(jointHandle,stepSize);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointDependency
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTDEPENDENCY[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTDEPENDENCY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int depJoint=-1;
    double offset=0.0;
    double mult=1.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTDEPENDENCY,inArgs_GETJOINTDEPENDENCY[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointDependency(jointHandle,&depJoint,&offset,&mult);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(depJoint));
        if (depJoint!=-1)
        {
            D.pushOutData(CScriptFunctionDataItem(offset));
            D.pushOutData(CScriptFunctionDataItem(mult));
        }
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

double jointDependencyCallback(int ikEnv,int slaveJoint,double masterPos)
{
    double retVal=0.0;
    int ind=-1;
    for (size_t i=0;i<jointDependInfo.size();i++)
    {
        if ( (jointDependInfo[i].ikEnv==ikEnv)&&(jointDependInfo[i].ikSlave==slaveJoint) )
        {
            ind=i;
            break;
        }
    }
    if (ind!=-1)
    {
        int stack=simCreateStack();
        simPushInt32OntoStack(stack,ikEnv);
        simPushInt32OntoStack(stack,slaveJoint);
        simPushDoubleOntoStack(stack,masterPos);
        if (simCallScriptFunctionEx(jointDependInfo[ind].scriptHandle,jointDependInfo[ind].cbString.c_str(),stack)!=-1)
        {
            while (simGetStackSize(stack)>1)
                simPopStackItem(stack,1);
            if (simGetStackSize(stack)==1)
                simGetStackDoubleValue(stack,&retVal);
        }
        simReleaseStack(stack);
    }
    return(retVal);
}

// --------------------------------------------------------------------------------------
// simIK._setJointDependency
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTDEPENDENCY[]={
    7,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_string|SIM_SCRIPT_ARG_NULL_ALLOWED,0, // cb func name
    sim_script_arg_int32|SIM_SCRIPT_ARG_NULL_ALLOWED,0, // script handle of cb
};

void LUA_SETJOINTDEPENDENCY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTDEPENDENCY,inArgs_SETJOINTDEPENDENCY[0]-4,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        int depJointHandle=inData->at(2).int32Data[0];
        double off=0.0;
        double mult=1.0;
        if ( (inData->size()>3)&&(inData->at(3).doubleData.size()==1) )
            off=inData->at(3).doubleData[0];
        if ( (inData->size()>4)&&(inData->at(4).doubleData.size()==1) )
            mult=inData->at(4).doubleData[0];
        std::string cbString;
        if ( (inData->size()>5)&&(inData->at(5).stringData.size()==1) )
            cbString=inData->at(5).stringData[0];
        int cbScriptHandle=-1;
        if ( (inData->size()>6)&&(inData->at(6).int32Data.size()==1) )
            cbScriptHandle=inData->at(6).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                _removeJointDependencyCallback(envId,jointHandle);
                double(*cb)(int ikEnv,int slaveJoint,double masterPos)=nullptr;
                if ( (cbScriptHandle!=-1)&&(cbString.size()>0) )
                {
                    SJointDependCB a;
                    a.ikEnv=envId;
                    a.ikSlave=jointHandle;
                    a.cbString=cbString;
                    a.scriptHandle=cbScriptHandle;
                    jointDependInfo.push_back(a);
                    cb=jointDependencyCallback;
                }
                bool result=ikSetJointDependency(jointHandle,depJointHandle,off,mult,cb);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointPosition
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTPOSITION[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTPOSITION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double pos=0.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTPOSITION,inArgs_GETJOINTPOSITION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetJointPosition(jointHandle,&pos);
                if (!result)
                    err=ikGetLastError();
            }
            else
                err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(pos));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setJointPosition
// --------------------------------------------------------------------------------------
const int inArgs_SETJOINTPOSITION[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_SETJOINTPOSITION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETJOINTPOSITION,inArgs_SETJOINTPOSITION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double pos=inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetJointPosition(jointHandle,pos);
                if (!result)
                    err=ikGetLastError();
            }
            else
                err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointMatrix
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTMATRIX[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTMATRIX_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double matrix[12];
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTMATRIX,inArgs_GETJOINTMATRIX[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                C7Vector tr;
                result=ikGetJointTransformation(jointHandle,&tr);
                if (result)
                    tr.getMatrix().getData(matrix);
                else
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        std::vector<double> m(matrix,matrix+12);
        D.pushOutData(CScriptFunctionDataItem(m));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setSphericalJointMatrix
// --------------------------------------------------------------------------------------
const int inArgs_SETSPHERICALJOINTMATRIX[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_script_arg_table,12,
};

void LUA_SETSPHERICALJOINTMATRIX_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETSPHERICALJOINTMATRIX,inArgs_SETSPHERICALJOINTMATRIX[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double* m=&inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                C4X4Matrix _m;
                _m.setData(m);
                C4Vector q(_m.M.getQuaternion());
                bool result=ikSetSphericalJointQuaternion(jointHandle,&q);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJointTransformation
// --------------------------------------------------------------------------------------
const int inArgs_GETJOINTTRANSFORMATION[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJOINTTRANSFORMATION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double pos[3];
    double quat[4];
    double e[3];
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETJOINTTRANSFORMATION,inArgs_GETJOINTTRANSFORMATION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                C7Vector tr;
                result=ikGetJointTransformation(jointHandle,&tr);
                if (result)
                {   // CoppeliaSim quaternion, internally: w x y z
                    // CoppeliaSim quaternion, at interfaces: x y z w
                    tr.X.getData(pos);
                    quat[0]=tr.Q(1);
                    quat[1]=tr.Q(2);
                    quat[2]=tr.Q(3);
                    quat[3]=tr.Q(0);
                    tr.Q.getEulerAngles().getData(e);
                }
                else
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        std::vector<double> _p(pos,pos+3);
        D.pushOutData(CScriptFunctionDataItem(_p));
        std::vector<double> _q(quat,quat+4);
        D.pushOutData(CScriptFunctionDataItem(_q));
        std::vector<double> _e(e,e+3);
        D.pushOutData(CScriptFunctionDataItem(_e));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setSphericalJointRotation
// --------------------------------------------------------------------------------------
const int inArgs_SETSPHERICALJOINTROTATION[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_script_arg_table,3,
};

void LUA_SETSPHERICALJOINTROTATION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETSPHERICALJOINTROTATION,inArgs_SETSPHERICALJOINTROTATION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int jointHandle=inData->at(1).int32Data[0];
        double* quat=nullptr;
        double* euler=nullptr;
        if (inData->at(2).doubleData.size()==3)
            euler=&inData->at(2).doubleData[0];
        else
            quat=&inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {   // CoppeliaSim quaternion, internally: w x y z
                // CoppeliaSim quaternion, at interfaces: x y z w
                C4Vector q;
                if (euler!=nullptr)
                    q.setEulerAngles(C3Vector(euler));
                else
                    q=C4Vector(quat[3],quat[0],quat[1],quat[2]);
                bool result=ikSetSphericalJointQuaternion(jointHandle,&q);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getGroupHandle
// --------------------------------------------------------------------------------------
const int inArgs_GETIKGROUPHANDLE[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_string,0,
};

void LUA_GETIKGROUPHANDLE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKGROUPHANDLE,inArgs_GETIKGROUPHANDLE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetGroupHandle(inData->at(1).stringData[0].c_str(),&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.doesGroupExist
// --------------------------------------------------------------------------------------
const int inArgs_DOESIKGROUPEXIST[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_string,0,
};

void LUA_DOESIKGROUPEXIST_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool retVal=false;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_DOESIKGROUPEXIST,inArgs_DOESIKGROUPEXIST[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                retVal=ikDoesGroupExist(inData->at(1).stringData[0].c_str());
                result=true;
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.createGroup
// --------------------------------------------------------------------------------------
const int inArgs_CREATEIKGROUP[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_string|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
};

void LUA_CREATEIKGROUP_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int retVal=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_CREATEIKGROUP,inArgs_CREATEIKGROUP[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                const char* nm=nullptr;
                if ( (inData->size()>1)&&(inData->at(1).stringData.size()==1)&&(inData->at(1).stringData[0].size()>0) )
                    nm=inData->at(1).stringData[0].c_str();
                result=ikCreateGroup(nm,&retVal);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getGroupFlags
// --------------------------------------------------------------------------------------
const int inArgs_GETIKGROUPFLAGS[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKGROUPFLAGS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int flags=0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKGROUPFLAGS,inArgs_GETIKGROUPFLAGS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetGroupFlags(ikGroupHandle,&flags);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(flags));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setGroupFlags
// --------------------------------------------------------------------------------------
const int inArgs_SETIKGROUPFLAGS[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETIKGROUPFLAGS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETIKGROUPFLAGS,inArgs_SETIKGROUPFLAGS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int flags=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetGroupFlags(ikGroupHandle,flags);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getGroupJointLimitHits
// --------------------------------------------------------------------------------------
const int inArgs_GETIKGROUPJOINTLIMITHITS[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKGROUPJOINTLIMITHITS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<int> handles;
    std::vector<double> overshots;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKGROUPJOINTLIMITHITS,inArgs_GETIKGROUPJOINTLIMITHITS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetGroupJointLimitHits(ikGroupHandle,&handles,&overshots);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(handles));
        D.pushOutData(CScriptFunctionDataItem(overshots));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getGroupJoints
// --------------------------------------------------------------------------------------
const int inArgs_GETGROUPJOINTS[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETGROUPJOINTS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<int> handles;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETGROUPJOINTS,inArgs_GETGROUPJOINTS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetGroupJoints(ikGroupHandle,&handles);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(handles));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getGroupCalculation
// --------------------------------------------------------------------------------------
const int inArgs_GETIKGROUPCALCULATION[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKGROUPCALCULATION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int method=-1;
    int iterations=0;
    double damping=0.0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKGROUPCALCULATION,inArgs_GETIKGROUPCALCULATION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetGroupCalculation(ikGroupHandle,&method,&damping,&iterations);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(method));
        D.pushOutData(CScriptFunctionDataItem(damping));
        D.pushOutData(CScriptFunctionDataItem(iterations));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setGroupCalculation
// --------------------------------------------------------------------------------------
const int inArgs_SETIKGROUPCALCULATION[]={
    5,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_int32,0,
};

void LUA_SETIKGROUPCALCULATION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETIKGROUPCALCULATION,inArgs_SETIKGROUPCALCULATION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int method=inData->at(2).int32Data[0];
        double damping=inData->at(3).doubleData[0];
        int iterations=inData->at(4).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetGroupCalculation(ikGroupHandle,method,damping,iterations);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// simIK.addElement
// --------------------------------------------------------------------------------------
const int inArgs_ADDIKELEMENT[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_ADDIKELEMENT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int elementHandle=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_ADDIKELEMENT,inArgs_ADDIKELEMENT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int tipDummyHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikAddElement(ikGroupHandle,tipDummyHandle,&elementHandle);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(elementHandle));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getElementFlags
// --------------------------------------------------------------------------------------
const int inArgs_GETIKELEMENTFLAGS[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKELEMENTFLAGS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int flags=0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKELEMENTFLAGS,inArgs_GETIKELEMENTFLAGS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetElementFlags(ikGroupHandle,ikElementHandle,&flags);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(flags));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setElementFlags
// --------------------------------------------------------------------------------------
const int inArgs_SETIKELEMENTFLAGS[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETIKELEMENTFLAGS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETIKELEMENTFLAGS,inArgs_SETIKELEMENTFLAGS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        int flags=inData->at(3).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetElementFlags(ikGroupHandle,ikElementHandle,flags);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getElementBase
// --------------------------------------------------------------------------------------
const int inArgs_GETIKELEMENTBASE[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKELEMENTBASE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int baseHandle=-1;
    int constrBaseHandle=-1;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKELEMENTBASE,inArgs_GETIKELEMENTBASE[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetElementBase(ikGroupHandle,ikElementHandle,&baseHandle,&constrBaseHandle);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(baseHandle));
        D.pushOutData(CScriptFunctionDataItem(constrBaseHandle));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setElementBase
// --------------------------------------------------------------------------------------
const int inArgs_SETIKELEMENTBASE[]={
    5,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
};

void LUA_SETIKELEMENTBASE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETIKELEMENTBASE,inArgs_SETIKELEMENTBASE[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        int baseHandle=inData->at(3).int32Data[0];
        int constrBaseHandle=-1;
        if ( (inData->size()>4)&&(inData->at(4).int32Data.size()==1) )
            constrBaseHandle=inData->at(4).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetElementBase(ikGroupHandle,ikElementHandle,baseHandle,constrBaseHandle);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getElementConstraints
// --------------------------------------------------------------------------------------
const int inArgs_GETIKELEMENTCONSTRAINTS[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKELEMENTCONSTRAINTS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int constraints=0;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKELEMENTCONSTRAINTS,inArgs_GETIKELEMENTCONSTRAINTS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetElementConstraints(ikGroupHandle,ikElementHandle,&constraints);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        D.pushOutData(CScriptFunctionDataItem(constraints));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setElementConstraints
// --------------------------------------------------------------------------------------
const int inArgs_SETIKELEMENTCONSTRAINTS[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETIKELEMENTCONSTRAINTS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETIKELEMENTCONSTRAINTS,inArgs_SETIKELEMENTCONSTRAINTS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        int constraints=inData->at(3).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetElementConstraints(ikGroupHandle,ikElementHandle,constraints);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getElementPrecision
// --------------------------------------------------------------------------------------
const int inArgs_GETIKELEMENTPRECISION[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKELEMENTPRECISION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double precision[2];
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKELEMENTPRECISION,inArgs_GETIKELEMENTPRECISION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetElementPrecision(ikGroupHandle,ikElementHandle,precision+0,precision+1);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        std::vector<double> v(precision,precision+2);
        D.pushOutData(CScriptFunctionDataItem(v));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setElementPrecision
// --------------------------------------------------------------------------------------
const int inArgs_SETIKELEMENTPRECISION[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_script_arg_table,2,
};

void LUA_SETIKELEMENTPRECISION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETIKELEMENTPRECISION,inArgs_SETIKELEMENTPRECISION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        double* precision=&inData->at(3).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetElementPrecision(ikGroupHandle,ikElementHandle,precision[0],precision[1]);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getElementWeights
// --------------------------------------------------------------------------------------
const int inArgs_GETIKELEMENTWEIGHTS[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETIKELEMENTWEIGHTS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double weights[2];
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETIKELEMENTWEIGHTS,inArgs_GETIKELEMENTWEIGHTS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                result=ikGetElementWeights(ikGroupHandle,ikElementHandle,weights+0,weights+1);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        std::vector<double> v(weights,weights+2);
        D.pushOutData(CScriptFunctionDataItem(v));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setElementWeights
// --------------------------------------------------------------------------------------
const int inArgs_SETIKELEMENTWEIGHTS[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_script_arg_table,2,
};

void LUA_SETIKELEMENTWEIGHTS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETIKELEMENTWEIGHTS,inArgs_SETIKELEMENTWEIGHTS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        int ikElementHandle=inData->at(2).int32Data[0];
        double weights[3];
        weights[0]=inData->at(3).doubleData[0];
        weights[1]=inData->at(3).doubleData[1];
        if (inData->at(3).doubleData.size()>2)
            weights[2]=inData->at(3).doubleData[2];
        else
            weights[2]=1.0;
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                bool result=ikSetElementWeights(ikGroupHandle,ikElementHandle,weights[0],weights[1],weights[2]);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
static std::string jacobianCallback_funcName;
static int jacobianCallback_scriptHandle;
static int jacobianCallback_envId;

int jacobianCallback(const int jacobianSize[2],double* jacobian,const int* rowConstraints,const int* rowIkElements,const int* colHandles,const int* colStages,double* errorVector,double* qVector,double* jacobianPinv,int groupHandle,int iteration)
{
    int retVal=-1; // error, -2 is nan error (ik_calc_invalidcallbackdata)
    int stack=simCreateStack();
    int cols=jacobianSize[1];
    simPushInt32TableOntoStack(stack,rowConstraints,jacobianSize[0]);
    simPushInt32TableOntoStack(stack,rowIkElements,jacobianSize[0]);
    simPushInt32TableOntoStack(stack,colHandles,cols);
    simPushInt32TableOntoStack(stack,colStages,cols);
    simPushDoubleTableOntoStack(stack,jacobian,jacobianSize[0]*jacobianSize[1]);
    simPushDoubleTableOntoStack(stack,errorVector,jacobianSize[0]);
    simPushInt32OntoStack(stack,groupHandle);
    simPushInt32OntoStack(stack,iteration);
    if (simCallScriptFunctionEx(jacobianCallback_scriptHandle,jacobianCallback_funcName.c_str(),stack)!=-1)
    {
        if (simGetStackSize(stack)==4)
        {
            retVal=0;
            // first the Jacobian pseudoinverse, if present:
            int cnt=simGetStackTableInfo(stack,0);
            if (cnt==jacobianSize[1]*jacobianSize[0])
            {
                retVal=retVal|2;
                simGetStackDoubleTable(stack,jacobianPinv,cnt);
                if (!isFloatArrayOk(jacobianPinv,cnt))
                    retVal=-2;
            }
            simPopStackItem(stack,1);

            // now dq, if present:
            cnt=simGetStackTableInfo(stack,0);
            if (cnt==jacobianSize[1])
            {
                retVal=retVal|1;
                simGetStackDoubleTable(stack,qVector,cnt);
                if (!isFloatArrayOk(qVector,cnt))
                    retVal=-2;
            }
            simPopStackItem(stack,1);

            // now e, if present:
            cnt=simGetStackTableInfo(stack,0);
            if (cnt==jacobianSize[0])
                simGetStackDoubleTable(stack,errorVector,cnt);
            simPopStackItem(stack,1);

            // now the Jacobian, if present:
            cnt=simGetStackTableInfo(stack,0);
            if (cnt==jacobianSize[0]*jacobianSize[1])
                simGetStackDoubleTable(stack,jacobian,cnt);
            simPopStackItem(stack,1);
        }
    }
    simReleaseStack(stack);
    ikSwitchEnvironment(jacobianCallback_envId); // actually required to correctly support CoppeliaSim's old GUI-based IK
    return(retVal);
}
// --------------------------------------------------------------------------------------
// simIK._handleGroups
// --------------------------------------------------------------------------------------
const int inArgs_HANDLEIKGROUPS[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_script_arg_table,1,
    sim_script_arg_string|SIM_SCRIPT_ARG_NULL_ALLOWED,0, // cb func name
    sim_script_arg_int32|SIM_SCRIPT_ARG_NULL_ALLOWED,0, // script handle of cb
};

void LUA_HANDLEIKGROUPS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int ikRes=ik_result_not_performed;
    bool result=false;
    double precision[2]={0.0,0.0};
    if (D.readDataFromStack(p->stackID,inArgs_HANDLEIKGROUPS,inArgs_HANDLEIKGROUPS[0]-3,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        std::vector<int>* ikGroupHandles=nullptr; // means handle all
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                int(*cb)(const int*,double*,const int*,const int*,const int*,const int*,double*,double*,double*,int,int)=nullptr;
                if ( (inData->size()>1)&&(inData->at(1).int32Data.size()>=1) )
                    ikGroupHandles=&inData->at(1).int32Data;
                if ( (inData->size()>3)&&(inData->at(2).stringData.size()==1)&&(inData->at(2).stringData[0].size()>0)&&(inData->at(3).int32Data.size()==1) )
                {
                    jacobianCallback_funcName=inData->at(2).stringData[0];
                    jacobianCallback_scriptHandle=inData->at(3).int32Data[0];
                    jacobianCallback_envId=envId;
                    cb=jacobianCallback;
                }
                result=ikHandleGroups(ikGroupHandles,&ikRes,precision,cb);
                if (!result)
                    err=ikGetLastError();
            }
            else
                err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        int r=ikRes;
        if ( (r&ik_calc_notperformed)!=0 )
            r=0; // ik_result_not_performed
        else if ( (r&(ik_calc_cannotinvert|ik_calc_notwithintolerance))!=0 )
            r=2; // previously ik_result_fail
        else r=1; // previously ik_result_success
        D.pushOutData(CScriptFunctionDataItem(r));
        D.pushOutData(CScriptFunctionDataItem(ikRes));
        std::vector<double> prec(precision,precision+2);
        D.pushOutData(CScriptFunctionDataItem(prec));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

static std::string validationCallback_funcNameAtScriptName;
static int validationCallback_scriptType;
static int validationCallback_envId;
static size_t validationCallback_jointCnt;

bool validationCallback(double* conf)
{
    bool retVal=1;
    int stack=simCreateStack();
    simPushDoubleTableOntoStack(stack,conf,int(validationCallback_jointCnt));
    if (simCallScriptFunctionEx(validationCallback_scriptType,validationCallback_funcNameAtScriptName.c_str(),stack)!=-1)
        simGetStackBoolValue(stack,&retVal);
    simReleaseStack(stack);
    ikSwitchEnvironment(validationCallback_envId); // actually required to correctly support CoppeliaSim's old GUI-based IK
    return(retVal!=0);
}

// --------------------------------------------------------------------------------------
// simIK._getConfigForTipPose // deprecated
// --------------------------------------------------------------------------------------
const int inArgs_GETCONFIGFORTIPPOSE[]={
    11,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_script_arg_table,0,
    sim_script_arg_double|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_int32|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_double|sim_script_arg_table|SIM_SCRIPT_ARG_NULL_ALLOWED,4,
    sim_script_arg_string|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_int32|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_int32|sim_script_arg_table|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_double|sim_script_arg_table|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_double|sim_script_arg_table|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
};

void LUA_GETCONFIGFORTIPPOSE_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int calcResult=-1;
    double* retConfig=nullptr;
    size_t jointCnt=0;
    if (D.readDataFromStack(p->stackID,inArgs_GETCONFIGFORTIPPOSE,inArgs_GETCONFIGFORTIPPOSE[0]-8,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                jointCnt=inData->at(2).int32Data.size();
                if (jointCnt>0)
                {
                    retConfig=new double[jointCnt];
                    double thresholdDist=double(0.1);
                    int iterations=1000;
                    double* metric=nullptr;
                    int* jointOptions=nullptr;
                    double* lowLimits=nullptr;
                    double* ranges=nullptr;
                    bool(*cb)(double*)=nullptr;
                    int scriptType=sim_scripttype_childscript;
                    if ( (inData->size()>4)&&(inData->at(4).int32Data.size()==1) )
                        iterations=inData->at(4).int32Data[0];
                    if ( (inData->size()>8)&&(inData->at(8).int32Data.size()>=jointCnt) )
                        jointOptions=&inData->at(8).int32Data[0];
                    if ( (inData->size()>7)&&(inData->at(7).int32Data.size()==1) )
                        scriptType=inData->at(7).int32Data[0];
                    if ( (inData->size()>6)&&(inData->at(6).stringData.size()==1)&&(inData->at(6).stringData[0].size()>0) )
                    {
                        validationCallback_funcNameAtScriptName=inData->at(6).stringData[0];
                        validationCallback_scriptType=scriptType;
                        validationCallback_jointCnt=jointCnt;
                        validationCallback_envId=envId;
                        cb=validationCallback;
                    }
                    if ( (inData->size()>3)&&(inData->at(3).doubleData.size()==1) )
                        thresholdDist=inData->at(3).doubleData[0];
                    if ( (inData->size()>5)&&(inData->at(5).doubleData.size()>=4) )
                        metric=&inData->at(5).doubleData[0];
                    if ( (inData->size()>9)&&(inData->at(9).doubleData.size()>=jointCnt) )
                        lowLimits=&inData->at(9).doubleData[0];
                    if ( (inData->size()>10)&&(inData->at(10).doubleData.size()>=jointCnt) )
                        ranges=&inData->at(10).doubleData[0];
                    calcResult=ikGetConfigForTipPose(ikGroupHandle,jointCnt,&inData->at(2).int32Data[0],thresholdDist,iterations,retConfig,metric,cb,jointOptions,lowLimits,ranges);
                    if (calcResult==-1)
                         err=ikGetLastError();
                }
                else
                    err="invalid joint handles";
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (calcResult==1)
    {
        std::vector<double> v(retConfig,retConfig+jointCnt);
        D.pushOutData(CScriptFunctionDataItem(v));
        D.writeDataToStack(p->stackID);
    }
    if (retConfig!=nullptr)
        delete[] retConfig;
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK._findConfig
// --------------------------------------------------------------------------------------
const int inArgs_FINDCONFIG[]={
    8,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_script_arg_table,0,
    sim_script_arg_double|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_int32|SIM_SCRIPT_ARG_NULL_ALLOWED,0,
    sim_script_arg_double|sim_script_arg_table|SIM_SCRIPT_ARG_NULL_ALLOWED,4,
    sim_script_arg_string|SIM_SCRIPT_ARG_NULL_ALLOWED,0, // cb func name
    sim_script_arg_int32|SIM_SCRIPT_ARG_NULL_ALLOWED,0, // script handle of cb
};

void LUA_FINDCONFIG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int calcResult=-1;
    double* retConfig=nullptr;
    size_t jointCnt=0;
    if (D.readDataFromStack(p->stackID,inArgs_FINDCONFIG,inArgs_FINDCONFIG[0]-5,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                jointCnt=inData->at(2).int32Data.size();
                if (jointCnt>0)
                {
                    retConfig=new double[jointCnt];
                    double thresholdDist=double(0.1);
                    int timeInMs=100;
                    double* metric=nullptr;
                    bool(*cb)(double*)=nullptr;
                    int scriptType=sim_scripttype_childscript;
                    if ( (inData->size()>4)&&(inData->at(4).int32Data.size()==1) )
                        timeInMs=inData->at(4).int32Data[0];
                    if ( (inData->size()>7)&&(inData->at(7).int32Data.size()==1) )
                        scriptType=inData->at(7).int32Data[0];
                    if ( (inData->size()>6)&&(inData->at(6).stringData.size()==1)&&(inData->at(6).stringData[0].size()>0) )
                    {
                        validationCallback_funcNameAtScriptName=inData->at(6).stringData[0];
                        validationCallback_scriptType=scriptType;
                        validationCallback_jointCnt=jointCnt;
                        validationCallback_envId=envId;
                        cb=validationCallback;
                    }
                    if ( (inData->size()>3)&&(inData->at(3).doubleData.size()==1) )
                        thresholdDist=inData->at(3).doubleData[0];
                    if ( (inData->size()>5)&&(inData->at(5).doubleData.size()>=4) )
                        metric=&inData->at(5).doubleData[0];
                    calcResult=ikFindConfig(ikGroupHandle,jointCnt,&inData->at(2).int32Data[0],thresholdDist,timeInMs,retConfig,metric,cb);
                    if (calcResult==-1)
                         err=ikGetLastError();
                }
                else
                    err="invalid joint handles";
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (calcResult==1)
    {
        std::vector<double> v(retConfig,retConfig+jointCnt);
        D.pushOutData(CScriptFunctionDataItem(v));
        D.writeDataToStack(p->stackID);
    }
    if (retConfig!=nullptr)
        delete[] retConfig;
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getObjectTransformation
// --------------------------------------------------------------------------------------
const int inArgs_GETOBJECTTRANSFORMATION[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETOBJECTTRANSFORMATION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double pos[3];
    double q[4];
    double e[3];
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETOBJECTTRANSFORMATION,inArgs_GETOBJECTTRANSFORMATION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objHandle=inData->at(1).int32Data[0];
        int relHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                C7Vector tr;
                result=ikGetObjectTransformation(objHandle,relHandle,&tr);
                if (result)
                {   // CoppeliaSim quaternion, internally: w x y z
                    // CoppeliaSim quaternion, at interfaces: x y z w
                    tr.X.getData(pos);
                    q[0]=tr.Q(1);
                    q[1]=tr.Q(2);
                    q[2]=tr.Q(3);
                    q[3]=tr.Q(0);
                    tr.Q.getEulerAngles().getData(e);
                }
                else
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        std::vector<double> _pos(pos,pos+3);
        D.pushOutData(CScriptFunctionDataItem(_pos));
        std::vector<double> _q(q,q+4);
        D.pushOutData(CScriptFunctionDataItem(_q));
        std::vector<double> _e(e,e+3);
        D.pushOutData(CScriptFunctionDataItem(_e));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setObjectTransformation
// --------------------------------------------------------------------------------------
const int inArgs_SETOBJECTTRANSFORMATION[]={
    5,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_script_arg_table,3,
    sim_script_arg_double|sim_script_arg_table,3,
    sim_script_arg_int32,0,
};

void LUA_SETOBJECTTRANSFORMATION_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETOBJECTTRANSFORMATION,inArgs_SETOBJECTTRANSFORMATION[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objHandle=inData->at(1).int32Data[0];
        int relHandle=inData->at(4).int32Data[0];
        double* quat=nullptr;
        double* euler=nullptr;
        double* pos=&inData->at(2).doubleData[0];
        if (inData->at(3).doubleData.size()==3)
            euler=&inData->at(3).doubleData[0];
        else
            quat=&inData->at(3).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {   // CoppeliaSim quaternion, internally: w x y z
                // CoppeliaSim quaternion, at interfaces: x y z w
                C7Vector tr;
                tr.X=C3Vector(pos);
                if (euler!=nullptr)
                    tr.Q.setEulerAngles(C3Vector(euler));
                if (quat!=nullptr)
                    tr.Q=C4Vector(quat[3],quat[0],quat[1],quat[2]);
                bool result=ikSetObjectTransformation(objHandle,relHandle,&tr);
                if (!result)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getObjectMatrix
// --------------------------------------------------------------------------------------
const int inArgs_GETOBJECTMATRIX[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETOBJECTMATRIX_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double matr[12];
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETOBJECTMATRIX,inArgs_GETOBJECTMATRIX[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objHandle=inData->at(1).int32Data[0];
        int relHandle=inData->at(2).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                C7Vector tr;
                result=ikGetObjectTransformation(objHandle,relHandle,&tr);
                if (result)
                {
                    C4X4Matrix m(tr.getMatrix());
                    m.getData(matr);
                }
                else
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (result)
    {
        std::vector<double> _matr(matr,matr+12);
        D.pushOutData(CScriptFunctionDataItem(_matr));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.setObjectMatrix
// --------------------------------------------------------------------------------------
const int inArgs_SETOBJECTMATRIX[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_script_arg_table,12,
    sim_script_arg_int32,0,
};

void LUA_SETOBJECTMATRIX_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SETOBJECTMATRIX,inArgs_SETOBJECTMATRIX[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int objHandle=inData->at(1).int32Data[0];
        int relHandle=inData->at(3).int32Data[0];
        double* m=&inData->at(2).doubleData[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                C4X4Matrix _m;
                _m.setData(m);
                C7Vector tr(_m.getTransformation());
                bool result=ikSetObjectTransformation(objHandle,relHandle,&tr);
                if (!result)
                    err=ikGetLastError();
            }
            else
                err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.computeJacobian
// --------------------------------------------------------------------------------------
const int inArgs_COMPUTEJACOBIAN[]={
    7,
    sim_script_arg_int32,0, // Ik env
    sim_script_arg_int32,0, // base handle (prev. ik group handle)
    sim_script_arg_int32,0, // last joint handle (prev. options)
    sim_script_arg_int32,0, // constraints
    sim_script_arg_double|sim_script_arg_table,7, // tip pose
    sim_script_arg_double|sim_script_arg_table,7, // target pose, optional
    sim_script_arg_double|sim_script_arg_table,7, // altBase pose, optional
};

void LUA_COMPUTEJACOBIAN_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_COMPUTEJACOBIAN,inArgs_COMPUTEJACOBIAN[0]-4,nullptr))
    {
        std::string err;
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        if (inData->size()>=4)
        {
            int baseHandle=inData->at(1).int32Data[0];
            int jointHandle=inData->at(2).int32Data[0];
            if ( (inData->size()>=5)&&(inData->at(3).int32Data.size()==1)&&(inData->at(4).doubleData.size()>=7) )
            {
                int constraints=inData->at(3).int32Data[0];
                C7Vector tipPose;
                if (inData->at(4).doubleData.size()<12)
                    tipPose.setData(inData->at(4).doubleData.data(),true);
                else
                {
                    C4X4Matrix m;
                    m.setData(inData->at(4).doubleData.data());
                    tipPose=m.getTransformation();
                }
                C7Vector targetPose(tipPose);
                C7Vector altBase;
                C7Vector* _altBase=nullptr;
                bool ok=true;
                if (inData->size()>=6)
                {
                    if (inData->at(5).doubleData.size()>=7)
                    {
                        if (inData->at(5).doubleData.size()<12)
                            targetPose.setData(inData->at(5).doubleData.data(),true);
                        else
                        {
                            C4X4Matrix m;
                            m.setData(inData->at(5).doubleData.data());
                            targetPose=m.getTransformation();
                        }
                        if (inData->size()>=7)
                        {

                            if (inData->at(6).doubleData.size()>=7)
                            {
                                if (inData->at(6).doubleData.size()<12)
                                    altBase.setData(inData->at(6).doubleData.data(),true);
                                else
                                {
                                    C4X4Matrix m;
                                    m.setData(inData->at(6).doubleData.data());
                                    altBase=m.getTransformation();
                                }
                                _altBase=&altBase;
                            }
                            else
                                ok=false;
                        }
                    }
                    else
                        ok=false;
                }
                if (ok)
                {
                    std::vector<double> jacobian;
                    std::vector<double> errorVect;
                    if (ikSwitchEnvironment(envId))
                    {
                        if (ikComputeJacobian(baseHandle,jointHandle,constraints,&tipPose,&targetPose,_altBase,&jacobian,&errorVect))
                        {
                            D.pushOutData(CScriptFunctionDataItem(jacobian));
                            D.pushOutData(CScriptFunctionDataItem(errorVect));
                            D.writeDataToStack(p->stackID);
                        }
                        else
                            err=ikGetLastError();
                    }
                    else
                        err=ikGetLastError();
                }
                else
                    err="invalid arguments";
            }
            else
                err="invalid arguments";
        }
        else
        { // old, for backw. compatibility
            if (ikSwitchEnvironment(envId))
            {
                int ikGroupHandle=inData->at(1).int32Data[0];
                int options=inData->at(2).int32Data[0];
                bool retVal=false;
                bool success=false;
                success=ikComputeJacobian_old(ikGroupHandle,options,&retVal); // old back compatibility function
                if (success)
                {
                    D.pushOutData(CScriptFunctionDataItem(retVal));
                    D.writeDataToStack(p->stackID);
                }
                else
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.computeJacobian
// --------------------------------------------------------------------------------------
const int inArgs_COMPUTEGROUPJACOBIAN[]={
    2,
    sim_script_arg_int32,0, // Ik env
    sim_script_arg_int32,0, // group handle
};

void LUA_COMPUTEGROUPJACOBIAN_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_COMPUTEGROUPJACOBIAN,inArgs_COMPUTEGROUPJACOBIAN[0],nullptr))
    {
        std::string err;
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int groupHandle=inData->at(1).int32Data[0];
        std::vector<double> jacobian;
        std::vector<double> errorVect;
        if (ikSwitchEnvironment(envId))
        {
            if (ikComputeGroupJacobian(groupHandle,&jacobian,&errorVect))
            {
                D.pushOutData(CScriptFunctionDataItem(jacobian));
                D.pushOutData(CScriptFunctionDataItem(errorVect));
                D.writeDataToStack(p->stackID);
            }
            else
                err=ikGetLastError();
        }
        else
            err=ikGetLastError();
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getJacobian, deprecated on 25.10.2022
// --------------------------------------------------------------------------------------
const int inArgs_GETJACOBIAN[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETJACOBIAN_CALLBACK(SScriptCallBack* p)
{ // deprecated on 25.10.2022
    CScriptFunctionData D;
    double* matr=nullptr;
    size_t matrSize[2];
    if (D.readDataFromStack(p->stackID,inArgs_GETJACOBIAN,inArgs_GETJACOBIAN[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
                matr=ikGetJacobian_old(ikGroupHandle,matrSize);
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (matr!=nullptr)
    {
        std::vector<double> v1(matr,matr+matrSize[0]*matrSize[1]);
        D.pushOutData(CScriptFunctionDataItem(v1));
        std::vector<int> v2;
        v2.push_back(int(matrSize[0]));
        v2.push_back(int(matrSize[1]));
        D.pushOutData(CScriptFunctionDataItem(v2));
        D.writeDataToStack(p->stackID);
        ikReleaseBuffer(matr);
    }
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simIK.getManipulability, deprecated
// --------------------------------------------------------------------------------------
const int inArgs_GETMANIPULABILITY[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETMANIPULABILITY_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    double retVal=0.0;
    bool success=false;
    if (D.readDataFromStack(p->stackID,inArgs_GETMANIPULABILITY,inArgs_GETMANIPULABILITY[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int envId=inData->at(0).int32Data[0];
        int ikGroupHandle=inData->at(1).int32Data[0];
        std::string err;
        {
            if (ikSwitchEnvironment(envId))
            {
                success=ikGetManipulability_old(ikGroupHandle,&retVal);
                if (!success)
                     err=ikGetLastError();
            }
            else
                 err=ikGetLastError();
        }
        if (err.size()>0)
            simSetLastError(nullptr,err.c_str());
    }
    if (success)
    {
        D.pushOutData(CScriptFunctionDataItem(retVal));
        D.writeDataToStack(p->stackID);
    }
}
// --------------------------------------------------------------------------------------


SIM_DLLEXPORT int simInit(SSimInit* info)
{
    simLib=loadSimLibrary(info->coppeliaSimLibPath);
    _pluginName=info->pluginName;
    if (simLib==NULL)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find or correctly load the CoppeliaSim library. Cannot start the plugin.");
        return(0);
    }
    if (getSimProcAddresses(simLib)==0)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return(0);
    }

    // Register the new Lua commands:
    simRegisterScriptCallbackFunction("createEnvironment",nullptr,LUA_CREATEENVIRONMENT_CALLBACK);
    simRegisterScriptCallbackFunction("_eraseEnvironment",nullptr,LUA_ERASEENVIRONMENT_CALLBACK);
    simRegisterScriptCallbackFunction("duplicateEnvironment",nullptr,LUA_DUPLICATEENVIRONMENT_CALLBACK);
    simRegisterScriptCallbackFunction("load",nullptr,LUA_LOAD_CALLBACK);
    simRegisterScriptCallbackFunction("save",nullptr,LUA_SAVE_CALLBACK);
    simRegisterScriptCallbackFunction("getObjects",nullptr,LUA_GETOBJECTS_CALLBACK);
    simRegisterScriptCallbackFunction("getObjectHandle",nullptr,LUA_GETOBJECTHANDLE_CALLBACK);
    simRegisterScriptCallbackFunction("doesObjectExist",nullptr,LUA_DOESOBJECTEXIST_CALLBACK);
    simRegisterScriptCallbackFunction("eraseObject",nullptr,LUA_ERASEOBJECT_CALLBACK);
    simRegisterScriptCallbackFunction("getObjectParent",nullptr,LUA_GETOBJECTPARENT_CALLBACK);
    simRegisterScriptCallbackFunction("setObjectParent",nullptr,LUA_SETOBJECTPARENT_CALLBACK);
    simRegisterScriptCallbackFunction("getObjectType",nullptr,LUA_GETOBJECTTYPE_CALLBACK);
    simRegisterScriptCallbackFunction("createDummy",nullptr,LUA_CREATEDUMMY_CALLBACK);
    simRegisterScriptCallbackFunction("getTargetDummy",nullptr,LUA_GETTARGETDUMMY_CALLBACK);
    simRegisterScriptCallbackFunction("setTargetDummy",nullptr,LUA_SETTARGETDUMMY_CALLBACK);
    simRegisterScriptCallbackFunction("createJoint",nullptr,LUA_CREATEJOINT_CALLBACK);
    simRegisterScriptCallbackFunction("getJointType",nullptr,LUA_GETJOINTTYPE_CALLBACK);
    simRegisterScriptCallbackFunction("getJointMode",nullptr,LUA_GETJOINTMODE_CALLBACK);
    simRegisterScriptCallbackFunction("setJointMode",nullptr,LUA_SETJOINTMODE_CALLBACK);
    simRegisterScriptCallbackFunction("getJointInterval",nullptr,LUA_GETJOINTINTERVAL_CALLBACK);
    simRegisterScriptCallbackFunction("setJointInterval",nullptr,LUA_SETJOINTINTERVAL_CALLBACK);
    simRegisterScriptCallbackFunction("getJointScrewLead",nullptr,LUA_GETJOINTSCREWLEAD_CALLBACK);
    simRegisterScriptCallbackFunction("setJointScrewLead",nullptr,LUA_SETJOINTSCREWLEAD_CALLBACK);
    simRegisterScriptCallbackFunction("getJointWeight",nullptr,LUA_GETJOINTIKWEIGHT_CALLBACK);
    simRegisterScriptCallbackFunction("setJointWeight",nullptr,LUA_SETJOINTIKWEIGHT_CALLBACK);
    simRegisterScriptCallbackFunction("getJointLimitMargin",nullptr,LUA_GETJOINTLIMITMARGIN_CALLBACK);
    simRegisterScriptCallbackFunction("setJointLimitMargin",nullptr,LUA_SETJOINTLIMITMARGIN_CALLBACK);
    simRegisterScriptCallbackFunction("getJointMaxStepSize",nullptr,LUA_GETJOINTMAXSTEPSIZE_CALLBACK);
    simRegisterScriptCallbackFunction("setJointMaxStepSize",nullptr,LUA_SETJOINTMAXSTEPSIZE_CALLBACK);
    simRegisterScriptCallbackFunction("getJointDependency",nullptr,LUA_GETJOINTDEPENDENCY_CALLBACK);
    simRegisterScriptCallbackFunction("_setJointDependency",nullptr,LUA_SETJOINTDEPENDENCY_CALLBACK);
    simRegisterScriptCallbackFunction("getJointPosition",nullptr,LUA_GETJOINTPOSITION_CALLBACK);
    simRegisterScriptCallbackFunction("setJointPosition",nullptr,LUA_SETJOINTPOSITION_CALLBACK);
    simRegisterScriptCallbackFunction("getJointMatrix",nullptr,LUA_GETJOINTMATRIX_CALLBACK);
    simRegisterScriptCallbackFunction("setSphericalJointMatrix",nullptr,LUA_SETSPHERICALJOINTMATRIX_CALLBACK);
    simRegisterScriptCallbackFunction("getJointTransformation",nullptr,LUA_GETJOINTTRANSFORMATION_CALLBACK);
    simRegisterScriptCallbackFunction("setSphericalJointRotation",nullptr,LUA_SETSPHERICALJOINTROTATION_CALLBACK);
    simRegisterScriptCallbackFunction("getGroupHandle",nullptr,LUA_GETIKGROUPHANDLE_CALLBACK);
    simRegisterScriptCallbackFunction("doesGroupExist",nullptr,LUA_DOESIKGROUPEXIST_CALLBACK);
    simRegisterScriptCallbackFunction("createGroup",nullptr,LUA_CREATEIKGROUP_CALLBACK);
    simRegisterScriptCallbackFunction("getGroupFlags",nullptr,LUA_GETIKGROUPFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("setGroupFlags",nullptr,LUA_SETIKGROUPFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("getGroupCalculation",nullptr,LUA_GETIKGROUPCALCULATION_CALLBACK);
    simRegisterScriptCallbackFunction("setGroupCalculation",nullptr,LUA_SETIKGROUPCALCULATION_CALLBACK);
    simRegisterScriptCallbackFunction("getGroupJointLimitHits",nullptr,LUA_GETIKGROUPJOINTLIMITHITS_CALLBACK);
    simRegisterScriptCallbackFunction("getGroupJoints",nullptr,LUA_GETGROUPJOINTS_CALLBACK);
    simRegisterScriptCallbackFunction("addElement",nullptr,LUA_ADDIKELEMENT_CALLBACK);
    simRegisterScriptCallbackFunction("getElementFlags",nullptr,LUA_GETIKELEMENTFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("setElementFlags",nullptr,LUA_SETIKELEMENTFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("getElementBase",nullptr,LUA_GETIKELEMENTBASE_CALLBACK);
    simRegisterScriptCallbackFunction("setElementBase",nullptr,LUA_SETIKELEMENTBASE_CALLBACK);
    simRegisterScriptCallbackFunction("getElementConstraints",nullptr,LUA_GETIKELEMENTCONSTRAINTS_CALLBACK);
    simRegisterScriptCallbackFunction("setElementConstraints",nullptr,LUA_SETIKELEMENTCONSTRAINTS_CALLBACK);
    simRegisterScriptCallbackFunction("getElementPrecision",nullptr,LUA_GETIKELEMENTPRECISION_CALLBACK);
    simRegisterScriptCallbackFunction("setElementPrecision",nullptr,LUA_SETIKELEMENTPRECISION_CALLBACK);
    simRegisterScriptCallbackFunction("getElementWeights",nullptr,LUA_GETIKELEMENTWEIGHTS_CALLBACK);
    simRegisterScriptCallbackFunction("setElementWeights",nullptr,LUA_SETIKELEMENTWEIGHTS_CALLBACK);
    simRegisterScriptCallbackFunction("_handleGroups",nullptr,LUA_HANDLEIKGROUPS_CALLBACK);
    simRegisterScriptCallbackFunction("_findConfig",nullptr,LUA_FINDCONFIG_CALLBACK);
    simRegisterScriptCallbackFunction("getObjectTransformation",nullptr,LUA_GETOBJECTTRANSFORMATION_CALLBACK);
    simRegisterScriptCallbackFunction("setObjectTransformation",nullptr,LUA_SETOBJECTTRANSFORMATION_CALLBACK);
    simRegisterScriptCallbackFunction("getObjectMatrix",nullptr,LUA_GETOBJECTMATRIX_CALLBACK);
    simRegisterScriptCallbackFunction("setObjectMatrix",nullptr,LUA_SETOBJECTMATRIX_CALLBACK);
    simRegisterScriptCallbackFunction("computeJacobian",nullptr,LUA_COMPUTEJACOBIAN_CALLBACK);
    simRegisterScriptCallbackFunction("computeGroupJacobian",nullptr,LUA_COMPUTEGROUPJACOBIAN_CALLBACK);

    simRegisterScriptVariable("handleflag_tipdummy",std::to_string(ik_handleflag_tipdummy).c_str(),0);
    simRegisterScriptVariable("objecttype_joint",std::to_string(ik_objecttype_joint).c_str(),0);
    simRegisterScriptVariable("objecttype_dummy",std::to_string(ik_objecttype_dummy).c_str(),0);
    simRegisterScriptVariable("jointmode_passive",std::to_string(ik_jointmode_passive).c_str(),0);
    simRegisterScriptVariable("jointmode_ik",std::to_string(ik_jointmode_ik).c_str(),0);
    simRegisterScriptVariable("jointtype_revolute",std::to_string(ik_jointtype_revolute).c_str(),0);
    simRegisterScriptVariable("jointtype_prismatic",std::to_string(ik_jointtype_prismatic).c_str(),0);
    simRegisterScriptVariable("jointtype_spherical",std::to_string(ik_jointtype_spherical).c_str(),0);
    simRegisterScriptVariable("handle_all",std::to_string(ik_handle_all).c_str(),0);
    simRegisterScriptVariable("handle_parent",std::to_string(ik_handle_parent).c_str(),0);
    simRegisterScriptVariable("handle_world",std::to_string(ik_handle_world).c_str(),0);
    simRegisterScriptVariable("constraint_x",std::to_string(ik_constraint_x).c_str(),0);
    simRegisterScriptVariable("constraint_y",std::to_string(ik_constraint_y).c_str(),0);
    simRegisterScriptVariable("constraint_z",std::to_string(ik_constraint_z).c_str(),0);
    simRegisterScriptVariable("constraint_alpha_beta",std::to_string(ik_constraint_alpha_beta).c_str(),0);
    simRegisterScriptVariable("constraint_gamma",std::to_string(ik_constraint_gamma).c_str(),0);
    simRegisterScriptVariable("constraint_position",std::to_string(ik_constraint_position).c_str(),0);
    simRegisterScriptVariable("constraint_orientation",std::to_string(ik_constraint_orientation).c_str(),0);
    simRegisterScriptVariable("constraint_pose",std::to_string(ik_constraint_pose).c_str(),0);
    simRegisterScriptVariable("method_pseudo_inverse",std::to_string(ik_method_pseudo_inverse).c_str(),0);
    simRegisterScriptVariable("method_damped_least_squares",std::to_string(ik_method_damped_least_squares).c_str(),0);
    simRegisterScriptVariable("method_jacobian_transpose",std::to_string(ik_method_jacobian_transpose).c_str(),0);
    simRegisterScriptVariable("method_undamped_pseudo_inverse",std::to_string(ik_method_undamped_pseudo_inverse).c_str(),0);

    simRegisterScriptVariable("result_not_performed",std::to_string(ik_result_not_performed).c_str(),0);
    simRegisterScriptVariable("result_success",std::to_string(ik_result_success).c_str(),0);
    simRegisterScriptVariable("result_fail",std::to_string(ik_result_fail).c_str(),0);

    simRegisterScriptVariable("calc_notperformed",std::to_string(ik_calc_notperformed).c_str(),0);
    simRegisterScriptVariable("calc_cannotinvert",std::to_string(ik_calc_cannotinvert).c_str(),0);
    simRegisterScriptVariable("calc_notwithintolerance",std::to_string(ik_calc_notwithintolerance).c_str(),0);
    simRegisterScriptVariable("calc_stepstoobig",std::to_string(ik_calc_stepstoobig).c_str(),0);
    simRegisterScriptVariable("calc_limithit",std::to_string(ik_calc_limithit).c_str(),0);
    simRegisterScriptVariable("calc_invalidcallbackdata",std::to_string(ik_calc_invalidcallbackdata).c_str(),0);

    simRegisterScriptVariable("group_enabled",std::to_string(ik_group_enabled).c_str(),0);
    simRegisterScriptVariable("group_ignoremaxsteps",std::to_string(ik_group_ignoremaxsteps).c_str(),0);
    simRegisterScriptVariable("group_restoreonbadlintol",std::to_string(ik_group_restoreonbadlintol).c_str(),0);
    simRegisterScriptVariable("group_restoreonbadangtol",std::to_string(ik_group_restoreonbadangtol).c_str(),0);
    simRegisterScriptVariable("group_stoponlimithit",std::to_string(ik_group_stoponlimithit).c_str(),0);
    simRegisterScriptVariable("group_avoidlimits",std::to_string(ik_group_avoidlimits).c_str(),0);

    // deprecated:
    simRegisterScriptCallbackFunction("_getConfigForTipPose",nullptr,LUA_GETCONFIGFORTIPPOSE_CALLBACK);
    simRegisterScriptCallbackFunction("getJointScrewPitch",nullptr,LUA_GETJOINTSCREWPITCH_CALLBACK);
    simRegisterScriptCallbackFunction("setJointScrewPitch",nullptr,LUA_SETJOINTSCREWPITCH_CALLBACK);
    simRegisterScriptCallbackFunction("getLinkedDummy",nullptr,LUA_GETLINKEDDUMMY_CALLBACK);
    simRegisterScriptCallbackFunction("setLinkedDummy",nullptr,LUA_SETLINKEDDUMMY_CALLBACK);
    simRegisterScriptCallbackFunction("getJacobian",nullptr,LUA_GETJACOBIAN_CALLBACK);
    simRegisterScriptCallbackFunction("getManipulability",nullptr,LUA_GETMANIPULABILITY_CALLBACK);
    simRegisterScriptCallbackFunction("getJointIkWeight",nullptr,LUA_GETJOINTIKWEIGHT_CALLBACK);
    simRegisterScriptCallbackFunction("setJointIkWeight",nullptr,LUA_SETJOINTIKWEIGHT_CALLBACK);
    simRegisterScriptCallbackFunction("getIkGroupHandle",nullptr,LUA_GETIKGROUPHANDLE_CALLBACK);
    simRegisterScriptCallbackFunction("doesIkGroupExist",nullptr,LUA_DOESIKGROUPEXIST_CALLBACK);
    simRegisterScriptCallbackFunction("createIkGroup",nullptr,LUA_CREATEIKGROUP_CALLBACK);
    simRegisterScriptCallbackFunction("getIkGroupFlags",nullptr,LUA_GETIKGROUPFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("setIkGroupFlags",nullptr,LUA_SETIKGROUPFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("getIkGroupCalculation",nullptr,LUA_GETIKGROUPCALCULATION_CALLBACK);
    simRegisterScriptCallbackFunction("setIkGroupCalculation",nullptr,LUA_SETIKGROUPCALCULATION_CALLBACK);
    simRegisterScriptCallbackFunction("getIkGroupJointLimitHits",nullptr,LUA_GETIKGROUPJOINTLIMITHITS_CALLBACK);
    simRegisterScriptCallbackFunction("addIkElement",nullptr,LUA_ADDIKELEMENT_CALLBACK);
    simRegisterScriptCallbackFunction("getIkElementFlags",nullptr,LUA_GETIKELEMENTFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("setIkElementFlags",nullptr,LUA_SETIKELEMENTFLAGS_CALLBACK);
    simRegisterScriptCallbackFunction("getIkElementBase",nullptr,LUA_GETIKELEMENTBASE_CALLBACK);
    simRegisterScriptCallbackFunction("setIkElementBase",nullptr,LUA_SETIKELEMENTBASE_CALLBACK);
    simRegisterScriptCallbackFunction("getIkElementConstraints",nullptr,LUA_GETIKELEMENTCONSTRAINTS_CALLBACK);
    simRegisterScriptCallbackFunction("setIkElementConstraints",nullptr,LUA_SETIKELEMENTCONSTRAINTS_CALLBACK);
    simRegisterScriptCallbackFunction("getIkElementPrecision",nullptr,LUA_GETIKELEMENTPRECISION_CALLBACK);
    simRegisterScriptCallbackFunction("setIkElementPrecision",nullptr,LUA_SETIKELEMENTPRECISION_CALLBACK);
    simRegisterScriptCallbackFunction("getIkElementWeights",nullptr,LUA_GETIKELEMENTWEIGHTS_CALLBACK);
    simRegisterScriptCallbackFunction("setIkElementWeights",nullptr,LUA_SETIKELEMENTWEIGHTS_CALLBACK);

    ikSetLogCallback(_logCallback);

    _allEnvironments=new CEnvCont();

    return(4); // 3 since V4.6.0
}

SIM_DLLEXPORT void simCleanup()
{
    delete _allEnvironments;
}

SIM_DLLEXPORT void simMsg(SSimMsg* info)
{
    if (info->msgId==sim_message_eventcallback_scriptstatedestroyed)
    {
        int env=_allEnvironments->removeOneFromScriptHandle(info->auxData[0]);
        while (env>=0)
        {
            if (ikSwitchEnvironment(env))
                ikEraseEnvironment();
            env=_allEnvironments->removeOneFromScriptHandle(info->auxData[0]);

            for (int i=0;i<int(jointDependInfo.size());i++)
            {
                if (jointDependInfo[i].scriptHandle==info->auxData[0])
                {
                    jointDependInfo.erase(jointDependInfo.begin()+i);
                    i--;
                }
            }
        }
    }

    if (info->msgId==sim_message_eventcallback_instancepass)
    {
        int consoleV=sim_verbosity_none;
        int statusbarV=sim_verbosity_none;
        simGetModuleInfo(nullptr,sim_moduleinfo_verbosity,nullptr,&consoleV);
        simGetModuleInfo(nullptr,sim_moduleinfo_statusbarverbosity,nullptr,&statusbarV);
        if ( (consoleV>sim_verbosity_none)||(statusbarV>sim_verbosity_none) )
        {
            if ( (consoleV>=sim_verbosity_trace)||(statusbarV>=sim_verbosity_trace) )
                ikSetVerbosity(5);
            else
            {
                if ( (consoleV>=sim_verbosity_debug)||(statusbarV>=sim_verbosity_debug) )
                    ikSetVerbosity(4);
                else
                    ikSetVerbosity(1);
            }
        }
        else
            ikSetVerbosity(0);
    }
}

