#ifndef _RNEA
#define _RNEA

#include "Control.h"

sVector3 Vec3Zeros(void);
sVector4 Vec4Zeros(void);
sVector6 Vec6Zeros(void);
sMatrix3 Mat3Ones(void);
sMatrix4 Mat4Ones(void);
void RNEA_Conf(void);
void RNEA_CalcTorques(void);
void Kin_Conf(void);
sRobPos Kin_FKCalc(sRobPos pointIn);
sRobPos Kin_IKCalc(sRobPos pointIn);
void Kin_RobPosAct(void);

#endif
