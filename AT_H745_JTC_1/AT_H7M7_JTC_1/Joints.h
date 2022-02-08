#ifndef	_JOINTS
#define _JOINTS
#include "Control.h"
void Joints_FindMinMaxVelTempInFrictionTabeIdx(void);
void Joints_SetDefaultPidParam(void);
void Joints_SetDefaultVariables(void);
void Joints_CalcFrictionCompensate(void);
void Joints_CalcPIDs(void);
void Joints_ClearCanValues(uint8_t num);
void Joints_SetDefaultFriction(void);


#endif
