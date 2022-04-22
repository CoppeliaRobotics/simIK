#include "envCont.h"
#include <stddef.h>

CEnvCont::CEnvCont()
{
}

CEnvCont::~CEnvCont()
{
}

void CEnvCont::add(int env,int script)
{
    _allObjects.push_back(env);
    _allObjects.push_back(script);
}

void CEnvCont::removeFromEnvHandle(int h)
{
    for (size_t i=0;i<_allObjects.size()/2;i++)
    {
        if (_allObjects[2*i+0]==h)
        {
            _allObjects.erase(_allObjects.begin()+2*i,_allObjects.begin()+2*i+2);
            return;
        }
    }
}

int CEnvCont::removeOneFromScriptHandle(int h)
{
    for (size_t i=0;i<_allObjects.size()/2;i++)
    {
        if (_allObjects[2*i+1]==h)
        {
            int retVal=_allObjects[2*i+0];
            _allObjects.erase(_allObjects.begin()+2*i,_allObjects.begin()+2*i+2);
            return(retVal);
        }
    }
    return(-1);
}
