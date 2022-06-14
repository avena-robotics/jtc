#include "MB_RTU_Slave.h"
#ifdef MODBUS
extern sControl* pC;
extern sTrajectory Traj;
extern sMB_RTUSlave	Mbs;
union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};
union conv64
{
    uint64_t u64; // here_write_bits
    double   d64; // here_read_double
};
static void MBS_ClrStr(uint8_t* str, uint32_t n)
{
	for(uint32_t i=0;i<n;i++)
		str[i] = 0x00;
}
static uint16_t MBS_Crc16(uint8_t* buf, uint32_t len)
{
  uint16_t crc = 0xffff;
  for (uint32_t pos = 0; pos < len; pos++) 
	{
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 8; i != 0; i--)
		{
      if ((crc & 0x0001) != 0) 
			{
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
			{
        crc >>= 1;
			}
    }
  }
  return crc;  
}
static void MBS_StructConf(void)
{
	MBS_ClrStr(Mbs.bufread, MBS_BUFMAX);
	MBS_ClrStr(Mbs.bufwrite, MBS_BUFMAX);
	Mbs.baud = MBS_COMBAUDRATE;
	Mbs.unittime = 1000000/Mbs.baud;
	Mbs.fun = MF_I;
	Mbs.address = 1;
	Mbs.coils = 0;
	for(uint16_t j=0;j<MBS_REGMAX;j++)
		Mbs.hregs[j] = 0;
}
static void MBS_ComUart2Conf(void)
{
	GPIOD->MODER &= ~GPIO_MODER_MODE5 & ~GPIO_MODER_MODE6;
	GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0;
	GPIOD->AFR[0] |= 0x07700000;
	
	DMA1_Stream0->CR = 0x00;
	DMA1_Stream0->PAR 	= (uint32_t)&USART2->RDR;
	DMA1_Stream0->M0AR 	= (uint32_t)Mbs.bufread;
	DMA1_Stream0->NDTR 	= (uint16_t)MBS_BUFMAX;
	DMAMUX1_Channel0->CCR = (43 << 0); //USART2 RX
	DMA1_Stream0->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	
	DMA1_Stream1->CR = 0x00;
	DMA1_Stream1->PAR 	= (uint32_t)&USART2->TDR;
	DMA1_Stream1->M0AR 	= (uint32_t)Mbs.bufwrite;
	DMA1_Stream1->NDTR 	= (uint16_t)MBS_BUFMAX;
	DMAMUX1_Channel1->CCR = (44 << 0); //USART2 TX
	DMA1_Stream1->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	
	USART2->BRR = 120000000/Mbs.baud;
	USART2->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART2_IRQn);
}
static void MBS_TimConf(void)
{
	TIM5->CR1 &= ~TIM_CR1_CEN;
	TIM5->CNT = 0;
	TIM5->PSC = 240-1;
	TIM5->ARR = 1000; //1 ms delay before response
	TIM5->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM5_IRQn);
}
void MBS_Conf(void)
{
	MBS_StructConf();
	MBS_ComUart2Conf();
	MBS_TimConf();
}
static void MBS_ReinitDmaReadStream(void)
{
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
	DMA1_Stream0->NDTR 	= (uint16_t)MBS_BUFMAX;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
}
static void MBS_ReinitDmaWriteStream(uint16_t len)
{
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
	DMA1_Stream1->NDTR 	= (uint16_t)len;
	DMA1_Stream1->CR |= DMA_SxCR_EN;
}
static void MBS_UseNewPidParam(void)
{
	uint16_t idx = MRN_PidStart;
	union conv32 x;
	for(int i=0;i<JOINTS_MAX;i++)
	{
		x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
		x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
		pC->Joints[i].pidKp = x.f32;
		
		x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
		x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
		pC->Joints[i].pidKi = x.f32;
		
		x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
		x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
		pC->Joints[i].pidKd = x.f32;
		
		x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
		x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
		pC->Joints[i].pidErrorIntMin = x.f32;
		
		x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
		x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
		pC->Joints[i].pidErrorIntMax = x.f32;
	}
	
	pC->Jtc.flagInitGetPidParam = false;
}
static void MBS_UseDefaultPidParam(void)
{
	Joints_SetDefaultPidParam();
	pC->Jtc.flagInitGetPidParam = false;
}
static void MBS_UseNewArmModel(void)
{
	uint16_t idx = MRN_ArmStart;
	union conv32 x;
	
	for(int i=0;i<ARMMODEL_DOF+1;i++)
	{
		for(int j=0;j<6;j++)
		{
			x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
			x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
			pC->Arm.Joints[i].origin[j] = x.f32;
		}
	}
	for(int i=0;i<ARMMODEL_DOF+1;i++)
	{
		for(int j=0;j<6;j++)
		{
			x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
			x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
			pC->Arm.Links[i].origin[j] = x.f32;
		}
		for(int j=0;j<6;j++)
		{
			x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
			x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
			pC->Arm.Links[i].innertia[j] = x.f32;
		}
		x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
		x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
		pC->Arm.Links[i].mass = x.f32;
	}
	
	pC->Jtc.flagInitGetArmModel = false;
	RNEA_Conf();
}
static void MBS_UseDefaultArmModel(void)
{
	Control_SetDefualtArmModel();
	RNEA_Conf();
	pC->Jtc.flagInitGetArmModel = false;
}
static void MBS_UseNewFricPolynomialParam(void)
{
	uint16_t idx = MRN_FricStart;
	union conv32 x;
	for(int num=0;num<JOINTS_MAX;num++)
	{
		for(int i=0;i<JOINTS_FRICCOEFFMAX;i++)
		{
			x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
			x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
			pC->Joints[num].fricCoeff[i] = x.f32;
		}
	}
	pC->Jtc.fricType = JTC_FT_Polynomial;
	pC->Jtc.flagInitGetFrictionTable = false;
}
static void MBS_UseDefaultFricPolynomialParam(void)
{
	pC->Jtc.fricType = JTC_FT_Polynomial;
	Joints_SetDefaultFriction();
	pC->Jtc.flagInitGetFrictionTable = false;
}
static void MBS_UseDefaultFricTableParam(void)
{
	pC->Jtc.fricType = JTC_FT_Table;
	Joints_SetDefaultFriction();
	pC->Jtc.flagInitGetFrictionTable = false;
}
static void MBS_ActTelemetry(void)
{
	uint16_t idx = MRN_TStart;
	
	//JTC status
	Mbs.hregs[idx++] = (uint16_t)(pC->Jtc.currentFsm >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Jtc.errors >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Jtc.occuredErrors >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Jtc.jtcInitStatus >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Jtc.jointsInitStatus >> 0);
	Mbs.hregs[idx++] = (uint16_t)(Traj.currentTES >> 0);
	Mbs.hregs[idx++] = (uint16_t)(Traj.Tgen.trajPrepStatus >> 0);
	Mbs.hregs[idx++] = (uint16_t)(Traj.numInterPoint >> 16);
	Mbs.hregs[idx++] = (uint16_t)(Traj.numInterPoint >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Jtc.fricType >> 0);
	
	//CAN Status
	Mbs.hregs[idx++] = pC->Can.statusId;
	Mbs.hregs[idx++] = (uint16_t)(pC->Can.statusFlags >> 16);
	Mbs.hregs[idx++] = (uint16_t)(pC->Can.statusFlags >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Can.statusOccurredFlags >> 16);
	Mbs.hregs[idx++] = (uint16_t)(pC->Can.statusOccurredFlags >> 0);
	
	//Joints Status and values
	union conv32 x;
	for(int num=0;num<JOINTS_MAX;num++)
	{
		Mbs.hregs[idx++] = (uint16_t)(pC->Joints[num].currentFsm >> 0);
		Mbs.hregs[idx++] = (uint16_t)(pC->Joints[num].mcCurrentError >> 0);
		Mbs.hregs[idx++] = (uint16_t)(pC->Joints[num].mcOccuredError >> 0);
		Mbs.hregs[idx++] = (uint16_t)(pC->Joints[num].currentError >> 0);
		Mbs.hregs[idx++] = (uint16_t)(pC->Joints[num].currentWarning >> 0);
		Mbs.hregs[idx++] = (uint16_t)(pC->Joints[num].internallErrors >> 0);
		Mbs.hregs[idx++] = (uint16_t)(pC->Joints[num].internallOccuredErrors >> 0);
		
		x.f32 = pC->Joints[num].currentPos;
		Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
		Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
		
		x.f32 = pC->Joints[num].currentVel;
		Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
		Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
		
		x.f32 = pC->Joints[num].currentTorque;
		Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
		Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
		
		Mbs.hregs[idx++] = (uint16_t)pC->Joints[num].currentBearingTemp;
	}
	//Gripper status
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.currentFsm);
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.currentPumpState);
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.pressure1);
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.pressure2);
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.currentError >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.currentWarning >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.internallErrors >> 0);
	Mbs.hregs[idx++] = (uint16_t)(pC->Gripper.internallOccuredErrors >> 0);
	Mbs.hregs[idx++] = 0x0000;	//Rest of joint6 (gripper) data. Reserved for future use
	Mbs.hregs[idx++] = 0x0000;	//Rest of joint6 (gripper) data. Reserved for future use
	Mbs.hregs[idx++] = 0x0000;	//Rest of joint6 (gripper) data. Reserved for future use
	Mbs.hregs[idx++] = 0x0000;	//Rest of joint6 (gripper) data. Reserved for future use
	Mbs.hregs[idx++] = 0x0000;	//Rest of joint6 (gripper) data. Reserved for future use
	Mbs.hregs[idx++] = 0x0000;	//Rest of joint6 (gripper) data. Reserved for future use

	//DI status
	Mbs.hregs[idx++] = pC->IO.DIReg;
	//DQ status
	Mbs.hregs[idx++] = pC->IO.DQReg;
	//AI status
	Mbs.hregs[idx++] = pC->IO.AIRegs[0];
	Mbs.hregs[idx++] = pC->IO.AIRegs[1];
	Mbs.hregs[idx++] = pC->IO.AIRegs[2];
	Mbs.hregs[idx++] = pC->IO.AIRegs[3];
	//AQ status
	Mbs.hregs[idx++] = pC->IO.AQRegs[0];
	Mbs.hregs[idx++] = pC->IO.AQRegs[1];
	Mbs.hregs[idx++] = pC->IO.AQRegs[2];
	Mbs.hregs[idx++] = pC->IO.AQRegs[3];
	
	// Pozycja efektora w ukladzie kartezjanskim - numer ukladu odniesienia
	Mbs.hregs[idx++] = 0;
	// Pozycja efektora w ukladzie kartezjanskim
	x.f32 = pC->Jtc.robPos.mat.v[0][3];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[1][3];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[2][3];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	// Pozycja efektora w ukladzie kartezjanskim w kwaternionach
	x.f32 = pC->Jtc.robPos.quat.v[0];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.quat.v[1];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.quat.v[2];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.quat.v[3];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	// Pozycja efektora w ukladzie kartezjanskim jako macierz rotacji
	x.f32 = pC->Jtc.robPos.mat.v[0][0];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[0][1];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[0][2];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[1][0];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[1][1];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[1][2];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[2][0];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[2][1];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	x.f32 = pC->Jtc.robPos.mat.v[2][2];
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 16);
	Mbs.hregs[idx++] = (uint16_t)(x.u32 >> 0);
	
	Mbs.hregs[idx++] = pC->Jtc.robToolNum;
}
static void MBS_ActConfigWords(void)
{
	uint16_t idx = MRN_CfStart + 12;
	pC->Joints[0].reqIgnore = (bool)Mbs.hregs[idx++];
	pC->Joints[1].reqIgnore = (bool)Mbs.hregs[idx++];
	pC->Joints[2].reqIgnore = (bool)Mbs.hregs[idx++];
	pC->Joints[3].reqIgnore = (bool)Mbs.hregs[idx++];
	pC->Joints[4].reqIgnore = (bool)Mbs.hregs[idx++];
	pC->Joints[5].reqIgnore = (bool)Mbs.hregs[idx++];
	pC->Gripper.reqIgnore = (bool)Mbs.hregs[idx++];
	pC->Jtc.robToolNum = Mbs.hregs[idx++];
	if(pC->Jtc.robToolNum >= ROBTOOLMAX)
		pC->Jtc.robToolNum = 0;
}
static void MBS_ActControlWords(void)
{
	uint16_t idx = MRN_CtrlStart;
	
	if(Mbs.hregs[idx++] == 0x01)		Traj.targetTES = TES_Stop;
	if(Mbs.hregs[idx++] == 0x01)		Traj.targetTES = TES_Pause;
	if(Mbs.hregs[idx++] == 0x01)		Traj.targetTES = TES_Execute;
	if(Mbs.hregs[idx++] == 0x01)		Traj.Tgen.reqTrajPrepare = true;
	Traj.numTraj = Mbs.hregs[idx++];
	if(Mbs.hregs[idx++] == 0x01)		pC->Jtc.teachingModeReq = true;
	if(Mbs.hregs[idx++] == 0x01)		pC->Jtc.teachingModeReq = false;
	if(Mbs.hregs[idx++] == 0x01)		MBS_UseDefaultPidParam();
	if(Mbs.hregs[idx++] == 0x01)		MBS_UseNewPidParam();
	if(Mbs.hregs[idx++] == 0x01)		MBS_UseDefaultArmModel();
	if(Mbs.hregs[idx++] == 0x01)		MBS_UseNewArmModel();
	if(Mbs.hregs[idx++] == 0x01)		MBS_UseDefaultFricPolynomialParam();
	if(Mbs.hregs[idx++] == 0x01)		MBS_UseNewFricPolynomialParam();
	if(Mbs.hregs[idx++] == 0x01)		MBS_UseDefaultFricTableParam();
	if(Mbs.hregs[idx++] == 0x01)		pC->Gripper.targetPumpState = 0x01;
	if(Mbs.hregs[idx++] == 0x01)		pC->Gripper.targetPumpState = 0x00;
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearInternallErrorsInJtc();
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearExternallErrorsViaCan(0x01);
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearExternallErrorsViaCan(0x02);
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearExternallErrorsViaCan(0x04);
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearExternallErrorsViaCan(0x08);
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearExternallErrorsViaCan(0x10);
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearExternallErrorsViaCan(0x20);
	if(Mbs.hregs[idx++] == 0x01)		Control_ClearExternallErrorsViaCan(0x40);
	if(Mbs.hregs[idx++] == 0x01)		Control_ResetDevicesViaCan(0x01);
	if(Mbs.hregs[idx++] == 0x01)		Control_ResetDevicesViaCan(0x02);
	if(Mbs.hregs[idx++] == 0x01)		Control_ResetDevicesViaCan(0x04);
	if(Mbs.hregs[idx++] == 0x01)		Control_ResetDevicesViaCan(0x08);
	if(Mbs.hregs[idx++] == 0x01)		Control_ResetDevicesViaCan(0x10);
	if(Mbs.hregs[idx++] == 0x01)		Control_ResetDevicesViaCan(0x20);
	if(Mbs.hregs[idx++] == 0x01)		Control_ResetDevicesViaCan(0x40);
	
	
	for(uint16_t i=MRN_CtrlStart;i<=MRN_CtrlFinish;i++)
		Mbs.hregs[i] = 0x00;
}
static void MBS_ActJogWords(void)
{
	uint16_t idx = MRN_JogStart;
	
	pC->Jtc.robJog.refSystem = (eJogRefSystem)Mbs.hregs[idx++];
	union conv32 x;
	for(int i=0;i<JOINTS_MAX;i++)
	{
		x.u32 = ((uint32_t)Mbs.hregs[idx++]<<16);
		x.u32 += ((uint32_t)Mbs.hregs[idx++]<<0);
		pC->Jtc.robJog.percentVel.v[i] = x.f32;
	}
}
static void MBS_ResponseError_IDR(void)
{
	Mbs.error = MFE_IDR;
	uint8_t* bufw = Mbs.bufwrite;
	MBS_ClrStr(bufw, MBS_BUFMAX);
	uint16_t idx = 0;
	bufw[idx++] = Mbs.address;
	bufw[idx++] = 0x80 + Mbs.fun;
	bufw[idx++] = Mbs.error;
	uint16_t crc = MBS_Crc16(bufw, idx);
	bufw[idx++] = crc;
	bufw[idx++] = crc >> 8;

	MBS_ReinitDmaWriteStream(idx);
}
static void MBS_Response_RnDQ(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint8_t* bufw = Mbs.bufwrite;
	MBS_ClrStr(bufw, MBS_BUFMAX);
	uint16_t coilstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
	uint16_t coilnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
	uint8_t numbytes = (coilnum + 7) / 8; // wyznaczenie liczby bajtów. Zaokraglenie w góre
	
	uint16_t idx = 0;
	bufw[idx++] = Mbs.address;
	bufw[idx++] = Mbs.fun;
	bufw[idx++] = numbytes;
	
	if(numbytes == 1)
		bufw[idx++] = (uint8_t)(Mbs.coils >> coilstart);
	if(numbytes == 2)
	{
		bufw[idx++] = (uint8_t)(Mbs.coils >> coilstart);
		bufw[idx++] = (uint8_t)(Mbs.coils >> (coilstart+8));
	}
	
	uint16_t crc = MBS_Crc16(bufw, idx);
	bufw[idx++] = crc;
	bufw[idx++] = crc >> 8;
	
	MBS_ReinitDmaWriteStream(idx);
}
static void MBS_Response_WnDQ(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint8_t* bufw = Mbs.bufwrite;
	MBS_ClrStr(bufw, MBS_BUFMAX);
	uint8_t numbytes = bufr[6];
	uint16_t coilstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
	uint16_t coilnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
	uint16_t coilend = coilstart + coilnum - 1;
	
	uint16_t temp1 = Mbs.coils;
	uint16_t temp2 = 0;
	if(numbytes == 1)
		temp2 = (uint16_t)bufr[7] << coilstart;
	if(numbytes == 2)
		temp2 = (((uint16_t)bufr[8]<<8) + ((uint16_t)bufr[7]<<0)) << coilstart;
	
	uint16_t mask = 0;
	for(int i=0;i<16;i++)
		if(i>=coilstart && i<= coilend)
			mask += (1<<i);
	
	Mbs.coils = ((temp1 & (~mask)) | (temp2 & mask));
		
	for(int i=0;i<6;i++)
		bufw[i] = bufr[i];
	
	uint16_t crc = MBS_Crc16(bufw, 6);
	bufw[6] = crc;
	bufw[7] = crc >> 8;
	
	MBS_ReinitDmaWriteStream(8);
}
static void MBS_Response_RnHR(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint8_t* bufw = Mbs.bufwrite;
	MBS_ClrStr(bufw, MBS_BUFMAX);
	uint16_t regstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
	uint16_t regnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
	uint16_t regend = regstart + regnum - 1;
	uint16_t numbytes = 2 * regnum;
	
	uint16_t idx = 0;
	bufw[idx++] = Mbs.address;
	bufw[idx++] = Mbs.fun;
	bufw[idx++] = numbytes;
	for(int i=regstart;i<=regend;i++)
	{
		bufw[idx++] = Mbs.hregs[i] >> 8;
		bufw[idx++] = Mbs.hregs[i];
	}
	uint16_t crc = MBS_Crc16(bufw, idx);
	bufw[idx++] = crc;
	bufw[idx++] = crc >> 8;
	
	MBS_ReinitDmaWriteStream(idx);
}
static void MBS_Response_W1HR(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint8_t* bufw = Mbs.bufwrite;
	MBS_ClrStr(bufw, MBS_BUFMAX);
	uint16_t regstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
	Mbs.hregs[regstart] = ((uint16_t)bufr[4]<<8);
	Mbs.hregs[regstart] += ((uint16_t)bufr[5]<<0);
	
	for(int i=0;i<6;i++)
		bufw[i] = bufr[i];
	uint16_t crc = MBS_Crc16(bufw, 6);
	bufw[6] = crc;
	bufw[7] = crc >> 8;
	
	MBS_ReinitDmaWriteStream(8);
}
static void MBS_Response_WnHR(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint8_t* bufw = Mbs.bufwrite;
	MBS_ClrStr(bufw, MBS_BUFMAX);
	uint16_t regstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
	uint16_t regnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
	uint16_t regend = regstart + regnum - 1;
	uint16_t index = 7;
	for(int i=regstart;i<=regend;i++)
	{
		Mbs.hregs[i] = ((uint16_t)bufr[index++]<<8);
		Mbs.hregs[i] += ((uint16_t)bufr[index++]<<0);
	}
	
	for(int i=0;i<6;i++)
		bufw[i] = bufr[i];
	uint16_t crc = MBS_Crc16(bufw, 6);
	bufw[6] = crc;
	bufw[7] = crc >> 8;
	
	MBS_ReinitDmaWriteStream(8);
}
static void MBS_ReadRequest_RnDQ(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint16_t crc1 = MBS_Crc16(bufr, 6);
	uint16_t crc2 = ((uint16_t)bufr[6]<<0) + ((uint16_t)bufr[7]<<8);
	if(crc1 == crc2)
	{
		Mbs.fun = MF_RnDQ;
		uint16_t coilstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
		uint16_t coilnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
		uint16_t coilend = coilstart + coilnum - 1;
		if(coilend >= MBS_COILMAX)
		{
			MBS_ResponseError_IDR();
		}
		else
		{
			MBS_Response_RnDQ();
		}
	}
}
static void MBS_ReadRequest_WnDQ(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint8_t numbytes = 7 + bufr[6];
	uint16_t crc1 = MBS_Crc16(bufr, numbytes);
	uint16_t crc2 = ((uint16_t)bufr[numbytes]<<0) + ((uint16_t)bufr[numbytes+1]<<8);
	if(crc1 == crc2)
	{
		Mbs.fun = MF_WnDQ;
		uint16_t coilstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
		uint16_t coilnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
		uint16_t coilend = coilstart + coilnum - 1;
		if(coilend >= MBS_COILMAX)
		{
			MBS_ResponseError_IDR();
		}
		else
		{
			MBS_Response_WnDQ();
		}
	}
}
static void MBS_ReadRequest_RnHR(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint16_t crc1 = MBS_Crc16(bufr, 6);
	uint16_t crc2 = ((uint16_t)bufr[6]<<0) + ((uint16_t)bufr[7]<<8);
	if(crc1 == crc2)
	{
		Mbs.fun = MF_RnHR;
		uint16_t regstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
		uint16_t regnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
		uint16_t regend = regstart + regnum - 1;
		if(regend >= MBS_REGMAX)
		{
			MBS_ResponseError_IDR();
		}
		else
		{
			MBS_Response_RnHR();
		}
	}
}
static void MBS_ReadRequest_W1HR(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint16_t crc1 = MBS_Crc16(bufr, 6);
	uint16_t crc2 = ((uint16_t)bufr[6]<<0) + ((uint16_t)bufr[7]<<8);
	if(crc1 == crc2)
	{
		Mbs.fun = MF_W1HR;
		uint16_t regstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
		uint16_t regnum = 1;
		uint16_t regend = regstart + regnum - 1;
		if(regend >= MBS_REGMAX)
		{
			MBS_ResponseError_IDR();
		}
		else
		{
			MBS_Response_W1HR();
		}
	}
}
static void MBS_ReadRequest_WnHR(void)
{
	uint8_t* bufr = Mbs.bufread;
	uint8_t numbytes = 7 + bufr[6];
	uint16_t crc1 = MBS_Crc16(bufr, numbytes);
	uint16_t crc2 = ((uint16_t)bufr[numbytes]<<0) + ((uint16_t)bufr[numbytes+1]<<8);
	if(crc1 == crc2)
	{
		Mbs.fun = MF_WnHR;
		uint16_t regstart = ((uint16_t)bufr[2]<<8) + ((uint16_t)bufr[3]<<0);
		uint16_t regnum = ((uint16_t)bufr[4]<<8) + ((uint16_t)bufr[5]<<0);
		uint16_t regend = regstart + regnum - 1;
		if(regend >= MBS_REGMAX)
		{
			MBS_ResponseError_IDR();
		}
		else
		{
			MBS_Response_WnHR();
		}
	}
}
static void MBS_ReadRequest(void)
{
	uint8_t* buf = Mbs.bufread;
	if(buf[0] == Mbs.address)
	{
		switch(buf[1])
		{
			case MF_RnDQ:		MBS_ReadRequest_RnDQ();	break;
			case MF_WnDQ:		MBS_ReadRequest_WnDQ(); break;
			case MF_RnHR:		MBS_ReadRequest_RnHR();	break;
			case MF_W1HR:		MBS_ReadRequest_W1HR();	break;
			case MF_WnHR:		MBS_ReadRequest_WnHR();	break;
			default:																break;
		}
	}
	MBS_ReinitDmaReadStream();
}
static void MBS_CheckAddress(void)
{
	uint8_t* buf = Mbs.bufread;
	if(buf[0] == Mbs.address)
	{
		TIM5->CR1 |= TIM_CR1_CEN;
	}
}
void MBS_Act(void)
{
	MBS_ActConfigWords();
	MBS_ActControlWords();
	MBS_ActTelemetry();
	MBS_ActJogWords();
}
void USART2_IRQHandler(void)
{
	if((USART2->ISR & USART_ISR_IDLE) != RESET)
	{
		char c = USART3->RDR;
		MBS_CheckAddress();
		USART2->ICR |= USART_ICR_IDLECF;
	}
}
// Przerwanie od strumienia odbiorczego
void DMA1_Stream0_IRQHandler(void)
{
	if((DMA1->LISR & DMA_LISR_TCIF0) != RESET)
	{
		MBS_ReinitDmaReadStream();
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
	}
}
// Przerwanie od strumienia wysylajacego
void DMA1_Stream1_IRQHandler(void)
{
	if((DMA1->LISR & DMA_LISR_TCIF1) != RESET)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
	}
}
void TIM5_IRQHandler(void)
{
	if((TIM5->SR & TIM_SR_UIF) != RESET)
	{
		TIM5->CR1 &= ~TIM_CR1_CEN;
		MBS_ReadRequest();
		TIM5->SR &= ~TIM_SR_UIF;
	}
}
#endif
