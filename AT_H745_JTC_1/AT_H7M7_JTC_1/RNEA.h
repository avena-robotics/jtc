#ifndef _RNEA
#define _RNEA

#include "Control.h"

void RNEA_Conf(void);
void RNEA_CalcTorques(void);
void Kin_Conf(void);
sRobPos Kin_FKCalc(sRobPos pointIn);
sRobPos Kin_IKCalc(sRobPos pointIn);
void Kin_RobPosAct(void);

#endif
