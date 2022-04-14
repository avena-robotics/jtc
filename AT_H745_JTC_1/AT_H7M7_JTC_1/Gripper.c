#include "Gripper.h"
extern sControl* pC;
void Gripper_SetStartValuesVariables(void)
{
	pC->Gripper.targetFsm = Joint_FSM_Init;
	pC->Gripper.currentFsm = Joint_FSM_Start;
	pC->Gripper.confFun = 0x04; //[0x01 - wlaczenie ograniczenia zakresu pracy, 0x02 - wlaczenie MA730, 0x04 - obsluga safety]
	pC->Gripper.currentError = 0x00;
	pC->Gripper.currentWarning = 0x00;
	pC->Gripper.internallErrors = 0x00;
	pC->Gripper.internallOccuredErrors = 0x00;
	pC->Gripper.reqCanReset = false;
	pC->Gripper.flagFirstPosRead = false;
	pC->Gripper.flagConfirmChangeConf = false;
	pC->Gripper.flagCanError = false;
	pC->Gripper.flagJtcError = false;
	pC->Gripper.targetPumpState = 0x00;
	pC->Gripper.currentPumpState = 0x00;
	pC->Gripper.pressure1 = 0x00;
	pC->Gripper.pressure2 = 0x00;
}

void Gripper_SetResetValuesVariables(void)
{
	Gripper_SetStartValuesVariables();
}
