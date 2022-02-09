#include "Control.h"
sControl Control;
sTrajectory Traj __attribute__((section (".ARM.__at_0x24000000"))); // Trajektoria, w pamieci AXI_RAM, max 512kB
sHost_Com Com  __attribute__((section (".ARM.__at_0x30000000"))); // Bufory komunikacji z hostem, w pamieci SRAM1 do SRAM3, max 288kB
sControl* pC = &Control;

double time[10];
double totaltime[10];
// *********************** General functions ***************************************
static void Control_SetTable(double* t, double a0, double a1, double a2, double a3, double a4, double a5)
{
	t[0] = a0; t[1] = a1; t[2] = a2; t[3] = a3; t[4] = a4; t[5] = a5;
}
void Control_SetDefualtArmModel(void)
{
	Control_SetTable(pC->Arm.Joints[0].origin, 0, 0, 0.063, 0, 0, -0.1963);									//joint 0 coordinate system
	Control_SetTable(pC->Arm.Joints[1].origin, 0.064, 0, 0.0495, 0, 1.570796, 0);						//joint 1 coordinate system
	Control_SetTable(pC->Arm.Joints[2].origin, -0.452, 0, 0, 0, 3.141592, 0);								//joint 2 coordinate system
	Control_SetTable(pC->Arm.Joints[3].origin, 0.452, 0, 0.0226, 3.141592, 0, 0);						//joint 3 coordinate system
	Control_SetTable(pC->Arm.Joints[4].origin, 0.0495, 0, 0.064, 0, -1.570796, 3.141592);		//joint 4 coordinate system
	Control_SetTable(pC->Arm.Joints[5].origin, 0.0495, 0, 0.064, 0, 1.570796, 0);						//joint 5 coordinate system
	Control_SetTable(pC->Arm.Joints[6].origin, 0, 0, 0.0181, 0, 0, 0);											//joint 6 coordinate system
	
	Control_SetTable(pC->Arm.Links[0].origin, 0, -0.0129, 0.00034, 0, 0, 0);								//center of mass link 0 coordinate system
	Control_SetTable(pC->Arm.Links[1].origin, 0.019, 0, 0.0445, 0, 0, 1.570796);						//center of mass link 1 coordinate system
	Control_SetTable(pC->Arm.Links[2].origin, -0.226, 0, 0.0334, 0, 0, 1.570796);						//center of mass link 2 coordinate system
	Control_SetTable(pC->Arm.Links[3].origin, 0.3046, 0, 0.0588, 0, 0, 1.570796);						//center of mass link 3 coordinate system
	Control_SetTable(pC->Arm.Links[4].origin, 0.019, 0, 0.059, 0, 0, 1.570796);							//center of mass link 4 coordinate system
	Control_SetTable(pC->Arm.Links[5].origin, 0.019, 0, 0.059, 0, 0, 1.570796);							//center of mass link 5 coordinate system
	Control_SetTable(pC->Arm.Links[6].origin, 0, 0, 0, 0, 0, 0);														//center of mass link 6 coordinate system
	
	Control_SetTable(pC->Arm.Links[0].innertia, 1, 0, 0, 1, 0, 1);													//link 0 innertial values
	Control_SetTable(pC->Arm.Links[1].innertia, 0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 1 innertial values
	Control_SetTable(pC->Arm.Links[2].innertia, 0.1941, 0, 0, 0.005522, 0, 0.1941);					//link 2 innertial values
	Control_SetTable(pC->Arm.Links[3].innertia, 0.1149, 0, 0, 0.004518, 0, 0.1148);					//link 3 innertial values
	Control_SetTable(pC->Arm.Links[4].innertia, 0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 4 innertial values
	Control_SetTable(pC->Arm.Links[5].innertia, 0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 5 innertial values
	Control_SetTable(pC->Arm.Links[6].innertia, 0.000041, 0, 0, 0.000042, 0, 0.00008);			//link 6 innertial values
	
	pC->Arm.Links[0].mass = 0.750;		//mass of link 0
	pC->Arm.Links[1].mass = 1.960;		//mass of link 1
	pC->Arm.Links[2].mass = 4.660;		//mass of link 2
	pC->Arm.Links[3].mass = 3.375;		//mass of link 3
	pC->Arm.Links[4].mass = 1.960;		//mass of link 4
	pC->Arm.Links[5].mass = 1.960;		//mass of link 5
	pC->Arm.Links[6].mass = 0.082;		//mass of link 6
}
static void Control_JtcVariableConf(void)
{
	pC->Jtc.currentFsm = JTC_FSM_Start;
	pC->Jtc.fricType = JTC_FT_Polynomial;
	pC->Jtc.flagInitGetArmModel = true;
	pC->Jtc.flagInitGetFrictionTable = true;
	pC->Jtc.flagInitGetPidParam = true;
	for(int i=0;i<JOINTS_MAX;i++)
		pC->Jtc.flagInitJointsTab[i] = true;
	
	if(pC->Jtc.flagInitGetFrictionTable == true)
		pC->Jtc.jtcInitStatus |= (1 << 0);
	if(pC->Jtc.flagInitGetPidParam == true)
		pC->Jtc.jtcInitStatus |= (1 << 1);
	if(pC->Jtc.flagInitGetArmModel == true)
		pC->Jtc.jtcInitStatus |= (1 << 2);
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Jtc.flagInitJointsTab[num] == true)
			pC->Jtc.jointsInitStatus |= (1 << num);
		
	pC->Jtc.internalError = false;
	pC->Jtc.externalError = false;
	pC->Jtc.externaljointsError = false;
	pC->Jtc.internalJointsError = false;
	pC->Jtc.internalCanError = false;
	pC->Jtc.internalComError = false;
}
static void Control_TrajClearPointDouble(sTrajPointDouble * p)
{
	for(uint32_t num=0;num<JOINTS_MAX;num++)
	{
		p->pos[num] = 0.;
		p->vel[num] = 0.;
		p->acc[num] = 0.;
	}
}
static void Control_TrajVariableConf(void)
{
	Control_TrajClear();
}
static void Control_VariablesConf(void)
{
	Control_JtcVariableConf();
	Control_TrajVariableConf();
	Joints_SetDefaultVariables();
	Joints_SetDefaultFriction();
	Joints_SetDefaultPidParam();
	Control_SetDefualtArmModel();
}
static void Control_ClockConf(void)
{
	//System Clock 480MHz
	uint32_t DIVM1=32, DIVN1=480-1, DIVP1=2-1, DIVQ1=2-1, DIVR1=2-1;
	//FDCAN Clock 85MHz
	uint32_t DIVM2=32, DIVN2=170-1, DIVP2=2-1, DIVQ2=4-1, DIVR2=2-1;
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY));
	
	RCC->PLLCKSELR = (DIVM1 << 4) | (DIVM2 << 12);
	RCC->PLL1DIVR = (DIVR1 << 24) | (DIVQ1 << 16) | (DIVP1 << 9) | (DIVN1 << 0);
	RCC->PLL2DIVR = (DIVR2 << 24) | (DIVQ2 << 16) | (DIVP2 << 9) | (DIVN2 << 0);
	RCC->D1CFGR = RCC_D1CFGR_D1CPRE_DIV1 | RCC_D1CFGR_HPRE_DIV2 | RCC_D1CFGR_D1PPRE_DIV2;
	RCC->D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2;
	RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;
	
	FLASH->ACR |= FLASH_ACR_WRHIGHFREQ_1 | FLASH_ACR_WRHIGHFREQ_0 | FLASH_ACR_LATENCY_7WS;
	RCC->CR |= RCC_CR_PLL1ON;
	while(!(RCC->CR & RCC_CR_PLL1RDY));
	RCC->CR |= RCC_CR_PLL2ON;
	while(!(RCC->CR & RCC_CR_PLL2RDY));
	RCC->CFGR |= RCC_CFGR_SW_PLL1;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1);
}
static void Control_RccConf(void)
{
	RCC->AHB2ENR |= (RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN | RCC_AHB2ENR_SRAM3EN);
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN;
	RCC->APB1LENR |= RCC_APB1LENR_USART2EN | RCC_APB1LENR_USART3EN | RCC_APB1LENR_TIM6EN | RCC_APB1LENR_TIM7EN | RCC_APB1LENR_TIM13EN | RCC_APB1LENR_TIM5EN;
	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
	RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
	RCC->D2CCIP1R |= RCC_D2CCIP1R_FDCANSEL_1;
	RCC->AHB3ENR |= RCC_AHB3ENR_AXISRAMEN;
}
static void Control_LedConf(void)
{
	GPIOB->MODER &= ~GPIO_MODER_MODE0;
	GPIOB->MODER 	|= GPIO_MODER_MODE0_0;
	GPIOE->MODER &= ~GPIO_MODER_MODE1;
	GPIOE->MODER 	|= GPIO_MODER_MODE1_0;
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER 	|= GPIO_MODER_MODE14_0;
}
static void Control_SafetyOutOn(void)
{
	GPIOD->ODR &= ~GPIO_ODR_OD3;
	pC->Jtc.emergencyOutput = true;
}
static void Control_SafetyOutOff(void)
{
	GPIOD->ODR |= GPIO_ODR_OD3;
	pC->Jtc.emergencyOutput = false;
}
static bool Control_SafetyInRead(void)
{
	bool in = false;
	if((GPIOD->IDR & GPIO_IDR_ID4) == RESET)
		in = true;
	return in;
}
static void Cotrol_SafetyConf(void)
{
	//Safety out conf
	GPIOD->MODER &= ~GPIO_MODER_MODE3;
	GPIOD->MODER |= GPIO_MODER_MODE3_0;
	
	//Safety in conf
	GPIOD->MODER &= ~GPIO_MODER_MODE4;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD4_0;
	
	Control_SafetyOutOff();
}
static void Control_TimerConf(void)
{
	TIM7->CR1 &= ~TIM_CR1_CEN;
	TIM7->CNT = 0;
	TIM7->PSC = 240-1;
	TIM7->ARR = 1000-1;	//Przerwanie co 1 ms
	TIM7->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);
	TIM7->CR1 |= TIM_CR1_CEN;
	
	TIM5->CR1 &= ~TIM_CR1_CEN;
	TIM5->CNT = 0;
	TIM5->PSC = 0;
	TIM5->ARR = 0xffffffff-1;
	TIM5->CR1 |= TIM_CR1_CEN;
}
void Control_SystemConf(void)
{
	Control_ClockConf();
	SysTick_Config(480000000 / 1000);
	Control_RccConf();
	Control_LedConf();
	Control_VariablesConf();
	Host_ComConf();
	Can_Conf();
	RNEA_Conf();
	Cotrol_SafetyConf();
	Control_TimerConf();
	pC->Jtc.currentFsm = JTC_FSM_Init;
}
void Control_Delay(uint32_t ms)
{
	pC->tick = 0;
	while(pC->tick < ms);
}
// *********************** Joints functions ***************************************
static void Control_JtcSetJointToInit(uint8_t num)
{
	pC->Joints[num].targetFsm = Joint_FSM_Init;
	pC->Joints[num].setTorque = 0.0;
}
static void Control_JtcSetJointToReadyToOperate(uint8_t num)
{
	pC->Joints[num].targetFsm = Joint_FSM_ReadyToOperate;
	pC->Joints[num].setTorque = 0.0;
}
static void Control_JtcSetJointToEnable(uint8_t num)
{
	pC->Joints[num].targetFsm = Joint_FSM_OperationEnable;
}
static void Control_CheckLimits(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		// Check torque limits
		if(pC->Joints[num].setTorqueTemp < pC->Joints[num].limitTorqueMin || pC->Joints[num].setTorqueTemp > pC->Joints[num].limitTorqueMax)
			pC->Joints[num].flagSetTorqueOverlimit = true;
		
		// Check pos limits
		if(pC->Joints[num].setPosTemp < pC->Joints[num].limitPosMin || pC->Joints[num].setPosTemp > pC->Joints[num].limitPosMax)
			pC->Joints[num].flagSetPosOverlimit = true;
		
		// Check vel limits
		if(pC->Joints[num].setVelTemp < pC->Joints[num].limitVelMin || pC->Joints[num].setVelTemp > pC->Joints[num].limitVelMax)
			pC->Joints[num].flagSetVelOverlimit = true;
		
		// Check acc limits
		if(pC->Joints[num].setAccTemp < pC->Joints[num].limitAccMin || pC->Joints[num].setAccTemp > pC->Joints[num].limitAccMax)
			pC->Joints[num].flagSetAccOverlimit = true;
		
		// Check pos error limits
		if(pC->Joints[num].pidErrorCurrent < pC->Joints[num].limitPosErrorMin || pC->Joints[num].pidErrorCurrent > pC->Joints[num].limitPosErrorMax)
			pC->Joints[num].flagPosErrorOverlimit = true;
	}
}
static void Control_CheckErrorFlags(void)
{
	pC->Jtc.internalError = false;
	pC->Jtc.externalError = false;
	pC->Jtc.externaljointsError = false;
	pC->Jtc.internalJointsError = false;
	pC->Jtc.internalCanError = false;
	pC->Jtc.internalComError = false;
	
	// Check Host comuniaction (PC by RS422)
	Com_HostCheckGeneralTimeout();
	
	// Check Joints internall and external errors
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].flagCanError = false;
		pC->Joints[num].flagJtcError = false;
		
		if(pC->Joints[num].currentError != 0x00)										pC->Joints[num].flagCanError = true;
		if(pC->Joints[num].mcCurrentError != 0x00)									pC->Joints[num].flagCanError = true;
		
		if(pC->Joints[num].flagSetTorqueOverlimit == true)					pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagFricTableValueOverlimit == true)			pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagSetPosOverlimit == true)							pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagSetVelOverlimit == true)							pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagSetAccOverlimit == true)							pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagPosErrorOverlimit == true)						pC->Joints[num].flagJtcError = true;
		
		if(pC->Joints[num].flagCanError == true)										pC->Jtc.externaljointsError = true;
		if(pC->Joints[num].flagJtcError == true)										pC->Jtc.internalJointsError = true;
	}
	
	// Check CAN comunication errors
	for(int num=0;num<CAN_RXBUF_MAX;num++)
	{
		if(pC->Can.RxMsgs[num].flagTimeout == true)									pC->Jtc.internalCanError = true;
	}
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		if(pC->Can.TxMsgs[num].flagTimeout == true)									pC->Jtc.internalCanError = true;
	}
	
	
	// Check hardware emergency line
	pC->Jtc.emergencyInput = Control_SafetyInRead();
	
	
	#ifdef TESTMODE
	pC->Jtc.internalJointsError = false;
	pC->Jtc.internalCanError = false;
	#endif
	
	//Check global internall error
	if(pC->Jtc.internalJointsError == true || pC->Jtc.internalCanError == true || pC->Jtc.internalComError == true)
	{
		pC->Jtc.internalError = true;
		Control_SafetyOutOn();
	}
	else
	{
		Control_SafetyOutOff();
	}
	//Check global external error
	if(pC->Jtc.externaljointsError == true || pC->Jtc.emergencyInput == true)
		pC->Jtc.externalError = true;
	
	// Przygotowywanie flag bled�w JTC do wyslania
	pC->Jtc.errors = 0x0000;
	pC->Jtc.errors |= pC->Jtc.emergencyInput << 0; 				// bit 0
	pC->Jtc.errors |= pC->Jtc.emergencyOutput << 1; 			// bit 1
	pC->Jtc.errors |= pC->Jtc.internalError << 2; 				// bit 2
	pC->Jtc.errors |= pC->Jtc.externalError << 3; 				// bit 3
	pC->Jtc.errors |= pC->Jtc.internalJointsError << 4; 	// bit 4
	pC->Jtc.errors |= pC->Jtc.internalCanError << 5; 			// bit 5
	pC->Jtc.errors |= pC->Jtc.internalComError << 6; 			// bit 6
	pC->Jtc.errors |= pC->Jtc.externaljointsError << 7; 	// bit 7
	
	pC->Jtc.occuredErrors |= pC->Jtc.errors;
	
	// Przygotowywanie flag bled�w wewnetrznych jointow do wyslania
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].internallErrors = 0x0000;
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetPosOverlimit << 0;							// bit 0
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetVelOverlimit << 1;							// bit 1
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetAccOverlimit << 2;							// bit 2
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetTorqueOverlimit << 3;						// bit 3
		pC->Joints[num].internallErrors |= pC->Joints[num].flagPosErrorOverlimit << 4;						// bit 4
		pC->Joints[num].internallErrors |= pC->Joints[num].flagFricTableValueOverlimit << 5;			// bit 5
		
		pC->Joints[num].internallOccuredErrors |= pC->Joints[num].internallErrors;
	}
}
static void Control_SetNewTorqueValues(void)
{
	if(pC->Jtc.externalError == false && pC->Jtc.internalError == false)
	{
		for(uint8_t num=0;num<JOINTS_MAX;num++)
			pC->Joints[num].setTorque = pC->Joints[num].setTorqueTemp;
	}
	else
	{
		for(uint8_t num=0;num<JOINTS_MAX;num++)
			pC->Joints[num].setTorque = 0.0;
	}
}
static void Control_SendDataToJoints(void)
{
	Can_SendDataToJoints();
}
// ***************** Trajectory functions ************************************
void Control_TrajClear(void)
{
	if(Traj.currentTES == TES_Null)
		return;
	
	Traj.comStatus = TCS_Null;
	Traj.targetTES = TES_Null;
	Traj.currentTES = TES_Null;
	Traj.numTraj = 0;
	Traj.numSeg = 0;
	Traj.maxSeg = 0;
	Traj.maxPoints = 0;
	Traj.stepTime = 0;
	Traj.numRecPoints = 0;
	
	for(uint32_t i=0;i<TRAJ_SEGSSMAX;i++)
	{
		Traj.flagReadSeg[i] = false;
		Traj.numPointsSeg[i] = 0;
	}
	
	Traj.maxInterPoints = 0;
	Traj.numInterPoint = 0;
	
	Control_TrajClearPointDouble(&Traj.startPoint);
	Control_TrajClearPointDouble(&Traj.interpolatePoint);
	Control_TrajClearPointDouble(&Traj.endPoint);
}
static void Control_TrajInterpolate(void)
{
	if(Traj.stepTime == 0)
		return;
	if((Traj.numInterPoint + 1) >= Traj.maxInterPoints)
		return;

	uint16_t n = Traj.numInterPoint / Traj.stepTime; // numer punktu z przeslanej trajektorii (zakres od 0 do Traj.maxPoints-1)
	uint16_t m = Traj.numInterPoint % Traj.stepTime; // numer kroku w interpolaji pomiedzy punktem poczatkowym a koncowym (zakres od 0 do Traj.stepTime-1)
	
	sTrajPoint start = Traj.points[n];
	sTrajPoint end = Traj.points[n + 1];
	
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		Traj.startPoint.pos[num] = (double)start.pos[num] * pC->Joints[num].limitPosMax / MAXINT16;
		Traj.startPoint.vel[num] = (double)start.vel[num] * pC->Joints[num].limitVelMax / MAXINT16;
		Traj.startPoint.acc[num] = (double)start.acc[num] * pC->Joints[num].limitVelMax / MAXINT16;
		
		Traj.endPoint.pos[num] = (double)end.pos[num] * pC->Joints[num].limitPosMax / MAXINT16;
		Traj.endPoint.vel[num] = (double)end.vel[num] * pC->Joints[num].limitVelMax / MAXINT16;
		Traj.endPoint.acc[num] = (double)end.acc[num] * pC->Joints[num].limitVelMax / MAXINT16;
	}
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		double dPos = Traj.endPoint.pos[num] - Traj.startPoint.pos[num];
		double offsetPos = (double)m * (dPos / (double)Traj.stepTime);
		Traj.interpolatePoint.pos[num] = Traj.startPoint.pos[num] + offsetPos;
		
		double dVel = Traj.endPoint.pos[num] - Traj.startPoint.vel[num];
		double offsetVel = (double)m * (dVel / (double)Traj.stepTime);
		Traj.interpolatePoint.vel[num] = Traj.startPoint.vel[num] + offsetVel;
		
		double dAcc = Traj.endPoint.acc[num] - Traj.startPoint.acc[num];
		double offsetAcc = (double)m * (dAcc / (double)Traj.stepTime);
		Traj.interpolatePoint.acc[num] = Traj.startPoint.acc[num] + offsetAcc;
	}
}
static void Control_TrajCheckState(void)
{
	if(Traj.currentTES == TES_Null && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_TransNullToStop; 				// Przejscie przez stan posredni
	else if(Traj.currentTES == TES_Stop && Traj.targetTES == TES_Execute)
		Traj.currentTES = TES_Execute; 								// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Pause && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_Stop; 									// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Pause && Traj.targetTES == TES_Execute)
		Traj.currentTES = TES_Execute; 								// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Execute && Traj.targetTES == TES_Pause)
		Traj.currentTES = TES_Pause; 									// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Execute && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_Stop; 									// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Execute && Traj.targetTES == TES_Finish)
		Traj.currentTES = TES_Finish; 								// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Finish && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_Stop; 									// Przejscie natychmiastowe
	
	
	// Check JTC status: HoldPos
	if(Traj.currentTES == TES_Null || Traj.currentTES == TES_Stop || Traj.currentTES == TES_Pause || Traj.currentTES == TES_Finish ||  Traj.currentTES == TES_TransNullToStop)
		pC->Jtc.holdposModeReq = true;
	else
		pC->Jtc.holdposModeReq = false;
	// Check JTC status: Operate
	if(Traj.currentTES == TES_Execute)
		pC->Jtc.operateModeReq = true;
	else
		pC->Jtc.operateModeReq = false;
}
static void Control_TrajStop(void)
{
	Traj.maxPoints = Traj.numRecPoints;
	Traj.maxInterPoints = Traj.maxPoints * Traj.stepTime;
	Traj.numInterPoint = 0;
	
	Control_TrajClearPointDouble(&Traj.startPoint);
	Control_TrajClearPointDouble(&Traj.interpolatePoint);
	Control_TrajClearPointDouble(&Traj.endPoint);
}
static void Control_TrajTransNullToStop(void)
{
	if(Traj.comStatus == TCS_WasRead)
	{
		Traj.currentTES = TES_Stop;
	}
	else
	{
		Traj.targetTES = TES_Null;
		Traj.currentTES = TES_Null;
	}
}
// ***************** Joint Trajectory Controller functions *********************
static void Control_JtcPrepareSetedValuesForTeaching(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;		// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
	}
	
	pC->Joints[0].idSetPos = -0.103;
	pC->Joints[1].idSetPos = -1.676;
	pC->Joints[2].idSetPos = -0.042;
	pC->Joints[3].idSetPos = 0.368;
	pC->Joints[4].idSetPos = 0.001;
	pC->Joints[5].idSetPos = -0.010;
}
static void Control_JtcPrepareSetedValuesForHoldPos(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;		// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
	}
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].setPosTemp = pC->Joints[num].setPosTemp; 	//utrzymywanie zadanej pozycji
		pC->Joints[num].setVelTemp = 0.0;
		pC->Joints[num].setAccTemp = 0.0;
	}
}
static void Control_JtcPrepareSetedValuesForOperate(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;					// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = Traj.interpolatePoint.vel[num];			// przewidywana zadana predkosc katowa tymczasowa
		pC->Joints[num].idSetAcc = Traj.interpolatePoint.acc[num];			// przewidywane zadane przyspieszenie katowe tymczasowa
	}
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].setPosTemp = Traj.interpolatePoint.pos[num];
		pC->Joints[num].setVelTemp = Traj.interpolatePoint.vel[num];
		pC->Joints[num].setAccTemp = Traj.interpolatePoint.acc[num];
	}
}
static void Control_ClearErrorSourceVariables(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].currentError = 0x00;
		pC->Joints[num].mcCurrentError = 0x00;
		
		pC->Joints[num].pidErrorCurrent = 0.0;
	}
}
static void Control_ClearErrorFlags(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].flagCanError = false;
		pC->Joints[num].flagJtcError = false;
		
		pC->Joints[num].flagSetTorqueOverlimit = false;
		pC->Joints[num].flagFricTableValueOverlimit = false;
		pC->Joints[num].flagSetPosOverlimit = false;
		pC->Joints[num].flagSetVelOverlimit = false;
		pC->Joints[num].flagSetAccOverlimit = false;
		pC->Joints[num].flagPosErrorOverlimit = false;
	}
	
	for(int num=0;num<CAN_RXBUF_MAX;num++)
	{
		pC->Can.RxMsgs[num].flagTimeout = false;
	}
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		pC->Can.TxMsgs[num].flagTimeout = false;
	}
}
void Control_ClearCurrentErrors(void)
{
	Control_ClearErrorSourceVariables();
	Control_ClearErrorFlags();
	Control_CheckErrorFlags();
}
void Control_ClearOccuredErrors(void)
{
	Control_ClearErrorSourceVariables();
	Control_ClearErrorFlags();
	pC->Jtc.occuredErrors = 0x0000;
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].internallOccuredErrors = 0x0000;
	}
	Control_CheckErrorFlags();
}
static void Control_JtcCheckStateError(void)
{
	// Error check
	if(pC->Jtc.internalError == true || pC->Jtc.externalError == true)
		pC->Jtc.errorModeReq = true;
	else
		pC->Jtc.errorModeReq = false;
}
static void Control_JtcCheckStateInit(void)
{
	// Check JTC init status flags
	if(pC->Jtc.flagInitGetFrictionTable == false)
		pC->Jtc.jtcInitStatus &= ~(1 << 0);
	if(pC->Jtc.flagInitGetPidParam == false)
		pC->Jtc.jtcInitStatus &= ~(1 << 1);
	if(pC->Jtc.flagInitGetArmModel == false)
		pC->Jtc.jtcInitStatus &= ~(1 << 2);
	
	#ifdef TESTMODE
	pC->Joints[0].currentFsm = Joint_FSM_ReadyToOperate;
	pC->Joints[1].currentFsm = Joint_FSM_ReadyToOperate;
	pC->Joints[2].currentFsm = Joint_FSM_ReadyToOperate;
	pC->Joints[3].currentFsm = Joint_FSM_ReadyToOperate;
	pC->Joints[5].currentFsm = Joint_FSM_ReadyToOperate;
	#endif
	
	
	// Check Joints init status flags
	for(uint8_t num=0;num<JOINTS_MAX;num++)
	{
		if(pC->Joints[num].currentFsm == Joint_FSM_ReadyToOperate)
			pC->Jtc.jointsInitStatus &= ~(1 << num);
		else if(pC->Joints[num].currentFsm == Joint_FSM_Start || pC->Joints[num].currentFsm == Joint_FSM_Init)
			pC->Jtc.jointsInitStatus |= (1 << num);
	}
	
	// Check JTC init status and Joints init status
	if(pC->Jtc.jtcInitStatus != 0x00 || pC->Jtc.jointsInitStatus != 0x00)
		pC->Jtc.initModeReq = true;			// Continue of Init state
	else
		pC->Jtc.initModeReq = false;		// Finish of Init state
}
static void Control_JtcCheckState(void)
{
	Control_JtcCheckStateError();
	Control_JtcCheckStateInit();
	Control_TrajCheckState();
	
	if(pC->Jtc.errorModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Error;
	}
	else if(pC->Jtc.initModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Init;
	}
	else if(pC->Jtc.teachingModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Teaching;
	}
	else if(pC->Jtc.holdposModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_HoldPos;
	}
	else if(pC->Jtc.operateModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Operate;
	}
	

	if(pC->Jtc.targetFsm == JTC_FSM_Error)
	{
		pC->Jtc.currentFsm = JTC_FSM_Error;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_Init)
	{
		pC->Jtc.currentFsm = JTC_FSM_Init;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_Teaching)
	{
		if(pC->Jtc.currentFsm != JTC_FSM_Teaching)
		{
			Joints_SetDefaultVariables();
			for(int num=0;num<JOINTS_MAX;num++)
			{
				pC->Joints[num].setPosTemp = 0.0;
				pC->Joints[num].setVelTemp = 0.0;
				pC->Joints[num].setAccTemp = 0.0;
			}
		}
		pC->Jtc.currentFsm = JTC_FSM_Teaching;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_HoldPos)
	{
		if(pC->Jtc.currentFsm != JTC_FSM_HoldPos)
		{
			Joints_SetDefaultVariables();
			for(int num=0;num<JOINTS_MAX;num++)
			{
				pC->Joints[num].setPosTemp = pC->Joints[num].currentPos; 	// aktualna pozycja jako pozycja zadana
				pC->Joints[num].setVelTemp = 0.0;
				pC->Joints[num].setAccTemp = 0.0;
			}
		}
		pC->Jtc.currentFsm = JTC_FSM_HoldPos;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_Operate)
	{
		if(pC->Jtc.currentFsm != JTC_FSM_Operate)
		{
			Joints_SetDefaultVariables();
		}
		pC->Jtc.currentFsm = JTC_FSM_Operate;
	}
}
static void Control_JtcError(void)
{
	Control_TrajClear();
	Joints_SetDefaultVariables();
	pC->Jtc.teachingModeReq = false;
	
	// Reaction for internall error
	if(pC->Jtc.internalError == true)
	{
		for(uint8_t num=0;num<JOINTS_MAX;num++)
			Control_JtcSetJointToReadyToOperate(num);
		for(uint8_t num=0;num<JOINTS_MAX;num++)
			if(pC->Can.RxMsgs[num].flagTimeout == true)
				Joints_ClearCanValues(num);
	}
	
	// Reaction for externall error
	if(pC->Jtc.externalError == true)
	{
		for(uint8_t num=0;num<JOINTS_MAX;num++)
		{
			if(pC->Joints[num].flagCanError == true)
				Control_JtcSetJointToInit(num);
			else
				Control_JtcSetJointToReadyToOperate(num);
		}
	}
}
static void Control_JtcInit(void)
{
	Control_TrajClear();
	Joints_SetDefaultVariables();
	pC->Jtc.teachingModeReq = false;
	
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		Control_JtcSetJointToReadyToOperate(num);
	
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = 0.0;
}
static void Control_JtcTeaching(void)
{
	Control_TrajClear();
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		Control_JtcSetJointToEnable(num);
	
	Control_JtcPrepareSetedValuesForTeaching();
	RNEA_CalcTorques();
	
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = pC->Joints[num].idTorque;
}
static void Control_JtcHoldPos(void)
{
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		Control_JtcSetJointToEnable(num);
	
	if(Traj.currentTES == TES_Stop)
		Control_TrajStop();
	else if(Traj.currentTES == TES_TransNullToStop)
		Control_TrajTransNullToStop();
	
	Control_JtcPrepareSetedValuesForHoldPos();
	RNEA_CalcTorques();
	Joints_CalcPIDs();
	
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = pC->Joints[num].pidTorque + pC->Joints[num].idTorque;
}
static void Control_JtcOperate(void)
{
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		Control_JtcSetJointToEnable(num);
	
	Traj.numInterPoint++;
	if(Traj.numInterPoint >= Traj.maxInterPoints)
	{
		Traj.targetTES = TES_Finish; // Koniec trajektorii
		return;
	}
	
	Control_TrajInterpolate();
	Control_JtcPrepareSetedValuesForOperate();
	RNEA_CalcTorques();
	Joints_CalcPIDs();
	Joints_CalcFrictionCompensate();
	
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = pC->Joints[num].pidTorque + pC->Joints[num].idTorque + pC->Joints[num].fricTorque;
}
static void Control_JtcAct(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	
	Control_CheckErrorFlags();
	Control_JtcCheckState();
	
	if(pC->Jtc.currentFsm == JTC_FSM_Error)
	{
		LED3_ON;
		Control_JtcError();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_Init)
	{
		LED1_ON;
		Control_JtcInit();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_Teaching)
	{
		LED1_ON;
		LED2_ON;
		Control_JtcTeaching();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_HoldPos)
	{
		LED2_ON;
		Control_JtcHoldPos();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_Operate)
	{
		LED2_ON;
		Control_JtcOperate();
	}
	
	Control_CheckLimits();
	Control_CheckErrorFlags();
	Control_SetNewTorqueValues();
	Control_SendDataToJoints();
}
// ********************** Interrupts functions ***********************************
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		Control_JtcAct();
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
void SysTick_Handler(void)
{
	pC->tick++;
}
