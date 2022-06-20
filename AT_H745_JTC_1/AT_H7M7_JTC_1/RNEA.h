#ifndef _RNEA
#define _RNEA

#include "Control.h"

sVector3 Vec3Zeros(void);
sVector4 Vec4Zeros(void);
sVector6 Vec6Zeros(void);
sMatrix3 Mat3Ones(void);
sMatrix4 Mat4Ones(void);
sMatrix4 HT(double x, double y, double z);
sMatrix4 HRX(double alpha);
sMatrix4 HRY(double betha);
sMatrix4 HRZ(double gamma);
sMatrix3 Mat4ToMat3(sMatrix4 m);
sMatrix4 Mat3ToMat4(sMatrix3 m3, sMatrix4 m4);
sMatrix3 Mat3xMat3(sMatrix3 m1, sMatrix3 m2);
sMatrix4 Mat4xMat4(sMatrix4 m1, sMatrix4 m2);
sVector6 Vec6SetValues(double a0, double a1, double a2, double a3, double a4, double a5);
void RNEA_Conf(void);
void RNEA_CalcTorques(void);
void Kin_Conf(void);
sRobPos Kin_FKCalc(sRobPos pointIn);
sRobPos Kin_IKCalcFromQuat(sRobPos pointIn);
sRobPos Kin_IKCalcFromRotMat(sRobPos pointIn);
void Kin_RobPosAct(void);
sMatrix4 InvMat4(sMatrix4 m);
sRobPos Kin_FindNearestSolution(sRobPos targetPos);

#endif
