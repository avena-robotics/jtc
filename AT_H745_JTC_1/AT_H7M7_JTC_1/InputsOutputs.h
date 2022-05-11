#ifndef _INPUTSOUTPUTS
#define _INPUTSOUTPUTS
#include "Control.h"
void IO_Conf(void);
void IO_OutputsAct(void);
void IO_InputsAct(void);
void IO_ParkBrakeUnlock(void);
void IO_ParkBrakeLock(void);
bool IO_ParkBrakeInRead(void);

#endif
