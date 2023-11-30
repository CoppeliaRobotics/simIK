#pragma once

#include <vector>

class CEnvCont
{
public:
    CEnvCont();
    virtual ~CEnvCont();

    void add(int env,int script);
    void removeFromEnvHandle(int h);
    int removeOneFromScriptHandle(int h);

private:
    std::vector<int> _allObjects; // env-script pairs
};
