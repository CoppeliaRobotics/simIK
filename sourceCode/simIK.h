#pragma once

#include <simLib-2/simTypes.h>
#include <simLib-2/simExp.h>

SIM_DLLEXPORT int simInit(SSimInit*);
SIM_DLLEXPORT void simCleanup();
SIM_DLLEXPORT void simMsg(SSimMsg*);
