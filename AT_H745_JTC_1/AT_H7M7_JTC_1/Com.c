#include "Control.h"
extern sControl* pC;
extern sTrajectory Traj;
extern sHost_Com Com;
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
void Com_HostCheckGeneralTimeout(void)
{
	Com.timeout++;
	if(Com.timeout > HOST_COMTIMEOUTMAX)
	{
		Com.timeout = HOST_COMTIMEOUTMAX;
		Com.flagTimeout = true;
	}
	else
	{
		Com.flagTimeout = false;
	}
}
static void Com_ClearReadFrame(void)
{
	Com.rxFrame.isReading = false;
	Com.rxFrame.consist = Host_RxFC_WasReceived;
	Com.rxFrame.status = Host_RxFS_Idle;
	Com.rxFrame.dataStatus = Host_RxDS_Idle;
	Com.rxFrame.frameType = Host_FT_null;
	Com.rxFrame.receivedLength = 0;
	Com.rxFrame.expectedLength = 1;
}
static void Com_ReinitDmaReadStream(void)
{
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
	DMA1_Stream0->NDTR 	= (uint16_t)HOST_COMBUFREADSIZE;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
}
static uint16_t Com_Crc16(uint8_t* packet, uint32_t nBytes)
{
	uint16_t crc = 0;
	for(uint32_t byte = 0; byte < nBytes; byte++)
	{
		crc = crc ^ ((uint16_t)packet[byte] << 8);
		for (uint8_t bit = 0; bit < 8; bit++)
			if(crc & 0x8000) 	crc = (crc << 1) ^ 0x1021;
			else							crc = crc << 1;
	}
	return crc;
}
static void Com_ClearStr(uint8_t* str, uint32_t l)
{
	for(uint32_t i=0;i<l;i++)
		str[i] = 0x00;
}
static void Host_ComPrepareFrameJtcStatus(void)
{
	uint8_t *buf = Com.txFrames[Host_TxFN_JtcStatus].frame;
	uint8_t idx = 0;
	buf[idx++] = (uint8_t)Host_FT_Header;
	buf[idx++] = (uint8_t)Host_FT_JtcStatus;
	buf[idx++] = (uint8_t)(0 >> 8); // Miejsce na liczbe bajtow w ramce
	buf[idx++] = (uint8_t)(0 >> 0); // Miejsce na liczbe bajtow w ramce
	
	//Response
	buf[idx++] = (uint8_t)Com.rxFrame.frameType;
	buf[idx++] = (uint8_t)Com.rxFrame.status;
	buf[idx++] = (uint8_t)Com.rxFrame.dataStatus;
	buf[idx++] = (uint8_t)(Com.rxFrame.receivedLength>>8);
	buf[idx++] = (uint8_t)(Com.rxFrame.receivedLength>>0);
	
	//JTC status
	buf[idx++] = (uint8_t)pC->Jtc.currentFsm;
	buf[idx++] = (uint8_t)(pC->Jtc.errors >> 8);
	buf[idx++] = (uint8_t)(pC->Jtc.errors >> 0);
	buf[idx++] = (uint8_t)(pC->Jtc.occuredErrors >> 8);
	buf[idx++] = (uint8_t)(pC->Jtc.occuredErrors >> 0);
	buf[idx++] = (uint8_t)pC->Jtc.jtcInitStatus;
	buf[idx++] = (uint8_t)pC->Jtc.jointsInitStatus;
	buf[idx++] = (uint8_t)Traj.currentTES;
	buf[idx++] = (uint8_t)(Traj.numInterPoint >> 24);
	buf[idx++] = (uint8_t)(Traj.numInterPoint >> 16);
	buf[idx++] = (uint8_t)(Traj.numInterPoint >> 8);
	buf[idx++] = (uint8_t)(Traj.numInterPoint >> 0);
	buf[idx++] = (uint8_t)pC->Jtc.fricType;
	
	//CAN Status
	buf[idx++] = (uint8_t)pC->Can.statusId;
	buf[idx++] = (uint8_t)(pC->Can.statusFlags >> 24);
	buf[idx++] = (uint8_t)(pC->Can.statusFlags >> 16);
	buf[idx++] = (uint8_t)(pC->Can.statusFlags >> 8);
	buf[idx++] = (uint8_t)(pC->Can.statusFlags >> 0);
	buf[idx++] = (uint8_t)(pC->Can.statusOccurredFlags >> 24);
	buf[idx++] = (uint8_t)(pC->Can.statusOccurredFlags >> 16);
	buf[idx++] = (uint8_t)(pC->Can.statusOccurredFlags >> 8);
	buf[idx++] = (uint8_t)(pC->Can.statusOccurredFlags >> 0);
	
	//Joints Status and values
	union conv32 x;
	for(int num=0;num<JOINTS_MAX;num++)
	{
		buf[idx++] = (uint8_t)(pC->Joints[num].currentFsm >> 0);
		buf[idx++] = (uint8_t)(pC->Joints[num].mcCurrentError >> 0);
		buf[idx++] = (uint8_t)(pC->Joints[num].mcOccuredError >> 0);
		buf[idx++] = (uint8_t)(pC->Joints[num].currentError >> 0);
		buf[idx++] = (uint8_t)(pC->Joints[num].currentWarning >> 0);
		buf[idx++] = (uint8_t)(pC->Joints[num].internallErrors >> 8);
		buf[idx++] = (uint8_t)(pC->Joints[num].internallErrors >> 0);
		buf[idx++] = (uint8_t)(pC->Joints[num].internallOccuredErrors >> 8);
		buf[idx++] = (uint8_t)(pC->Joints[num].internallOccuredErrors >> 0);
		
		x.f32 = pC->Joints[num].currentPos;
		buf[idx++] = (uint8_t)(x.u32 >> 24);
		buf[idx++] = (uint8_t)(x.u32 >> 16);
		buf[idx++] = (uint8_t)(x.u32 >> 8);
		buf[idx++] = (uint8_t)(x.u32 >> 0);
		
		x.f32 = pC->Joints[num].currentVel;
		buf[idx++] = (uint8_t)(x.u32 >> 24);
		buf[idx++] = (uint8_t)(x.u32 >> 16);
		buf[idx++] = (uint8_t)(x.u32 >> 8);
		buf[idx++] = (uint8_t)(x.u32 >> 0);
		
		x.f32 = pC->Joints[num].currentTorque;
		buf[idx++] = (uint8_t)(x.u32 >> 24);
		buf[idx++] = (uint8_t)(x.u32 >> 16);
		buf[idx++] = (uint8_t)(x.u32 >> 8);
		buf[idx++] = (uint8_t)(x.u32 >> 0);
		
		buf[idx++] = (uint8_t)pC->Joints[num].currentTemp;
	}
	
	// Number of bytes in frame and CRC
	buf[2] = (uint8_t)((idx + 2) >> 8); // Liczba bajtow w ramce
	buf[3] = (uint8_t)((idx + 2) >> 0); // Liczba bajtow w ramce
	uint16_t crc = Com_Crc16(buf, idx);
	buf[idx++] = (uint8_t)(crc >> 8); 	// CRC
	buf[idx++] = (uint8_t)(crc >> 0); 	// CRC
	Com.txFrames[Host_TxFN_JtcStatus].status = Host_TxFS_ReadyToSend;
	Com.txFrames[Host_TxFN_JtcStatus].len = idx;
	Com.txFrames[Host_TxFN_JtcStatus].active = false;
}
static void Host_ComStructConf(void)
{
	Com.txFrames[Host_TxFN_JtcStatus].active = false;
	
	Com.txFrames[Host_TxFN_JtcStatus].funPrepareFrame = Host_ComPrepareFrameJtcStatus;
}
static void Host_ComTimCof(void)
{
	// timer do wysylania
	TIM6->CR1 &= ~TIM_CR1_CEN;
	TIM6->CNT = 0;
	TIM6->PSC = 240-1;
	TIM6->ARR = (1000 * HOST_COMTIMSEND)-1;
	TIM6->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->CR1 |= TIM_CR1_CEN;
	
	// timer do timeout od spójnosci odbieranych ramek, przerwanie po 5ms
	TIM13->PSC = 240-1;
	TIM13->ARR = 5000-1;
	TIM13->CR1 = TIM_CR1_CEN;
	TIM13->SR &= ~TIM_SR_UIF;
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}
static void Host_ComUart2Conf(void)
{
	GPIOD->MODER &= ~GPIO_MODER_MODE5 & ~GPIO_MODER_MODE6;
	GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0;
	GPIOD->AFR[0] |= 0x07700000;
	
	DMA1_Stream0->CR = 0x00;
	DMA1_Stream0->PAR 	= (uint32_t)&USART2->RDR;
	DMA1_Stream0->M0AR 	= (uint32_t)Com.bufread0;
	DMA1_Stream0->NDTR 	= (uint16_t)HOST_COMBUFREADSIZE;
	DMAMUX1_Channel0->CCR = (43 << 0); //USART2 RX
	DMA1_Stream0->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	
	DMA1_Stream1->CR = 0x00;
	DMA1_Stream1->PAR 	= (uint32_t)&USART2->TDR;
	DMA1_Stream1->M0AR 	= (uint32_t)Com.bufwrite;
	DMA1_Stream1->NDTR 	= (uint16_t)HOST_COMBUFWRITESIZE;
	DMAMUX1_Channel1->CCR = (44 << 0); //USART2 TX
	DMA1_Stream1->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	
	USART2->BRR = 120000000/HOST_COMBAUDRATE;
	USART2->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART2_IRQn);
}
static void Host_ComUart3Conf(void)
{
	GPIOD->MODER &= ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE9;
	GPIOD->MODER |= GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0;
	GPIOD->AFR[1] |= 0x00000077;
	
	DMA1_Stream0->CR = 0x00;
	DMA1_Stream0->PAR 	= (uint32_t)&USART3->RDR;
	DMA1_Stream0->M0AR 	= (uint32_t)Com.bufread0;
	DMA1_Stream0->NDTR 	= (uint16_t)HOST_COMBUFREADSIZE;
	DMAMUX1_Channel0->CCR = (45 << 0); //USART3 RX
	DMA1_Stream0->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	
	DMA1_Stream1->CR = 0x00;
	DMA1_Stream1->PAR 	= (uint32_t)&USART3->TDR;
	DMA1_Stream1->M0AR 	= (uint32_t)Com.bufwrite;
	DMA1_Stream1->NDTR 	= (uint16_t)HOST_COMBUFWRITESIZE;
	DMAMUX1_Channel1->CCR = (46 << 0); //USART3 TX
	DMA1_Stream1->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	
	USART3->BRR = 120000000/HOST_COMBAUDRATE;
	USART3->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART3_IRQn);
}
void Host_ComConf(void)
{
	Host_ComStructConf();
	
	#ifdef RS422
		Host_ComUart2Conf();
	#endif
	
	#ifdef UARTUSB
		Host_ComUart3Conf();
	#endif
	
	Host_ComTimCof();
}
static void Host_ComStartSeding(sHost_TxFrame* f)
{
	Com_ClearStr(Com.bufwrite, f->len);
	for(uint8_t i=0;i<f->len;i++)
		Com.bufwrite[i] = f->frame[i];
	
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	DMA1_Stream1->NDTR 	= (uint16_t)f->len;
	DMA1_Stream1->CR |= DMA_SxCR_EN;
	
	f->status = Host_TxFS_Sending;
}
static void Host_ComSendResponseToHost(void)
{
	if(Com.status == Host_CS_Idle)
	{
		sHost_TxFrame* f;
		for(uint8_t i=0;i<HOST_COMFRAMESSIZE;i++)
		{
			Com.framenum++;
			if(Com.framenum >= HOST_COMFRAMESSIZE)
				Com.framenum = 0;
			f = &Com.txFrames[Com.framenum];
			if(f->active == true)
			{
				f->funPrepareFrame();
				Host_ComStartSeding(f);
				Com_ClearReadFrame();
			}
			else
			{
				continue;
			}
		}
	}
}
static void Com_CheckConsistencyReceivedFrame(uint8_t* buf)
{
	Com.rxFrame.receivedLength = HOST_COMBUFREADSIZE - DMA1_Stream0->NDTR;
	Com.rxFrame.expectedLength = ((uint16_t)buf[2]<<8) + ((uint16_t)buf[3]<<0);
	
	if(Com.rxFrame.receivedLength >= Com.rxFrame.expectedLength)
	{
		Com.rxFrame.consist = Host_RxFC_WasReceived;
		TIM13->DIER &= ~TIM_DIER_UIE;
	}
	else
	{
		Com.rxFrame.consist = Host_RxFC_IsReceived;
		TIM13->CNT = 0;
		TIM13->SR &= ~TIM_SR_UIF;
		TIM13->DIER = TIM_DIER_UIE;
	}
}
static void Host_ComReadFrameClearCurrentErrors(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		Control_ClearCurrentErrors();
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameClearOccuredErrors(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		Control_ClearOccuredErrors();
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameTrajectory(uint8_t* buf)
{
	uint16_t idx = 4, np;
	
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		uint16_t trajNum = ((uint16_t)buf[idx++]<<8);
		trajNum += ((uint16_t)buf[idx++]<<0);
		uint16_t segNum = ((uint16_t)buf[idx++]<<8);
		segNum += ((uint16_t)buf[idx++]<<0);
		uint16_t segMax = ((uint16_t)buf[idx++]<<8);
		segMax += ((uint16_t)buf[idx++]<<0);
		uint16_t stepTime = ((uint16_t)buf[idx++]<<8);
		stepTime += ((uint16_t)buf[idx++]<<0);
		
		// liczba punktow w odebranym segmencie
		np = (nd - 14) / (3 * JOINTS_MAX * 2);

		// segment o numerze 0 oznacza nowa trajektorie, poprzednia jest czyszczona
		if(segNum == 0)
			Control_TrajClear();
		
		// odebrano juz zbyt wiele punktów w tej trajektorii
		if((Traj.numRecPoints + np) > TRAJ_POINTSMAX)
		{
			Com.rxFrame.dataStatus = Host_RxDS_TrajTooManyPoints;
			return;
		}
		// odebrano juz zbyt wiele segmentow w tej trajektorii
		if(segNum > TRAJ_SEGSSMAX || segMax > TRAJ_SEGSSMAX)
		{
			Com.rxFrame.dataStatus = Host_RxDS_TrajTooManySegs;
			return;
		}
		// odebrano segment o zlym numerze (np: zla kolejnosc transmitowanych segmentow)
		if(segNum != 0 && (((int16_t)(segNum - Traj.numSeg) != 1) || (segNum >= segMax)))
		{
			Com.rxFrame.dataStatus = Host_RxDS_TrajIncorrectSegOrder;
			return;
		}
		// odebrano niepoprawny krok czasowy (np: stepTime = 0)
		if(stepTime == 0)
		{
			Com.rxFrame.dataStatus = Host_RxDS_TrajIncorrectStepTime;
			return;
		}
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
				
		LED3_TOG;
		// sprawdzenie czy odebrany segment jest ostatnim w trajektorii
		if(segNum == (segMax-1))
		{
			Traj.comStatus = TCS_WasRead;	// odebrany segment jest ostatnim segmentem w trajektorii
			Traj.targetTES = TES_Stop;
		}
		else
			Traj.comStatus = TCS_IsRead;		// odebrany segment nie jest ostatnim segmentem w trajektorii
		
		Traj.numTraj = trajNum;
		Traj.numSeg = segNum;
		Traj.maxSeg = segMax;
		Traj.stepTime = stepTime;
		Traj.flagReadSeg[Traj.numSeg] = true;
		Traj.numPointsSeg[Traj.numSeg] = np;
		
		for(uint16_t i=Traj.numRecPoints;i<Traj.numRecPoints+np;i++)
		{
			for(uint16_t j=0;j<JOINTS_MAX;j++)
			{
				Traj.points[i].pos[j] = ((uint16_t)buf[idx++]<<8);
				Traj.points[i].pos[j] += ((uint16_t)buf[idx++]<<0);
			}
			for(uint16_t j=0;j<JOINTS_MAX;j++)
			{
				Traj.points[i].vel[j] = ((uint16_t)buf[idx++]<<8);
				Traj.points[i].vel[j] += ((uint16_t)buf[idx++]<<0);
			}
			for(uint16_t j=0;j<JOINTS_MAX;j++)
			{
				Traj.points[i].acc[j] = ((uint16_t)buf[idx++]<<8);
				Traj.points[i].acc[j] += ((uint16_t)buf[idx++]<<0);
			}
		}
		Traj.numRecPoints += np;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameFricionTable(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	uint16_t idx = 4;
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		union conv32 x;
		for(int num=0;num<JOINTS_MAX;num++)
		{
			for(int i=0;i<JOINTS_FRICTABVELSIZE;i++)
			{
				x.u32 = ((uint32_t)buf[idx++]<<24);
				x.u32 += ((uint32_t)buf[idx++]<<16);
				x.u32 += ((uint32_t)buf[idx++]<<8);
				x.u32 += ((uint32_t)buf[idx++]<<0);
				pC->Joints[num].fricTableVelIdx[i] = x.f32;
			}
			for(int i=0;i<JOINTS_FRICTABTEMPSIZE;i++)
			{
				x.u32 = ((uint32_t)buf[idx++]<<24);
				x.u32 += ((uint32_t)buf[idx++]<<16);
				x.u32 += ((uint32_t)buf[idx++]<<8);
				x.u32 += ((uint32_t)buf[idx++]<<0);
				pC->Joints[num].fricTableTempIdx[i] = x.f32;
			}
			for(int i=0;i<JOINTS_FRICTABVELSIZE;i++)
			{
				for(int j=0;j<JOINTS_FRICTABTEMPSIZE;j++)
				{
					x.u32 = ((uint32_t)buf[idx++]<<24);
					x.u32 += ((uint32_t)buf[idx++]<<16);
					x.u32 += ((uint32_t)buf[idx++]<<8);
					x.u32 += ((uint32_t)buf[idx++]<<0);
					pC->Joints[num].fricTable[i][j] = x.f32;
				}
			}
		}
		pC->Jtc.fricType = JTC_FT_Table;
		Joints_FindMinMaxVelTempInFrictionTabeIdx();
		pC->Jtc.flagInitGetFrictionTable = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameFricionTableUseDefault(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		pC->Jtc.fricType = JTC_FT_Table;
		Joints_SetDefaultFriction();
		pC->Jtc.flagInitGetFrictionTable = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameFricionPolynomial(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	uint16_t idx = 4;
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		union conv32 x;
		for(int num=0;num<JOINTS_MAX;num++)
		{
			for(int i=JOINTS_FRICCOEFFMAX-1;i>=0;i--)
			{
				x.u32 = ((uint32_t)buf[idx++]<<24);
				x.u32 += ((uint32_t)buf[idx++]<<16);
				x.u32 += ((uint32_t)buf[idx++]<<8);
				x.u32 += ((uint32_t)buf[idx++]<<0);
				pC->Joints[num].fricCoeff[i] = x.f32;
			}
		}
		pC->Jtc.fricType = JTC_FT_Polynomial;
		pC->Jtc.flagInitGetFrictionTable = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameFricionPolynomialUseDefault(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		pC->Jtc.fricType = JTC_FT_Polynomial;
		Joints_SetDefaultFriction();
		pC->Jtc.flagInitGetFrictionTable = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFramePidParam(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength; //JOINTS_MAX * 7 * 4 + 4;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	uint16_t idx = 4;
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		union conv32 x;
		for(int i=0;i<JOINTS_MAX;i++)
		{
			x.u32 = ((uint32_t)buf[idx++]<<24);
			x.u32 += ((uint32_t)buf[idx++]<<16);
			x.u32 += ((uint32_t)buf[idx++]<<8);
			x.u32 += ((uint32_t)buf[idx++]<<0);
			pC->Joints[i].pidKp = x.f32;
			
			x.u32 = ((uint32_t)buf[idx++]<<24);
			x.u32 += ((uint32_t)buf[idx++]<<16);
			x.u32 += ((uint32_t)buf[idx++]<<8);
			x.u32 += ((uint32_t)buf[idx++]<<0);
			pC->Joints[i].pidKi = x.f32;
			
			x.u32 = ((uint32_t)buf[idx++]<<24);
			x.u32 += ((uint32_t)buf[idx++]<<16);
			x.u32 += ((uint32_t)buf[idx++]<<8);
			x.u32 += ((uint32_t)buf[idx++]<<0);
			pC->Joints[i].pidKd = x.f32;
			
			x.u32 = ((uint32_t)buf[idx++]<<24);
			x.u32 += ((uint32_t)buf[idx++]<<16);
			x.u32 += ((uint32_t)buf[idx++]<<8);
			x.u32 += ((uint32_t)buf[idx++]<<0);
			pC->Joints[i].pidErrorIntMin = x.f32;
			
			x.u32 = ((uint32_t)buf[idx++]<<24);
			x.u32 += ((uint32_t)buf[idx++]<<16);
			x.u32 += ((uint32_t)buf[idx++]<<8);
			x.u32 += ((uint32_t)buf[idx++]<<0);
			pC->Joints[i].pidErrorIntMax = x.f32;
		}
		
		pC->Jtc.flagInitGetPidParam = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFramePidParamUseDefault(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		Joints_SetDefaultPidParam();
		pC->Jtc.flagInitGetPidParam = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameArmModel(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	uint16_t idx = 4;
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		union conv32 x;
		for(int i=0;i<ARMMODEL_DOF+1;i++)
		{
			for(int j=0;j<6;j++)
			{
				x.u32 = ((uint32_t)buf[idx++]<<24);
				x.u32 += ((uint32_t)buf[idx++]<<16);
				x.u32 += ((uint32_t)buf[idx++]<<8);
				x.u32 += ((uint32_t)buf[idx++]<<0);
				pC->Arm.Joints[i].origin[j] = x.f32;
			}
		}
		for(int i=0;i<ARMMODEL_DOF+1;i++)
		{
			for(int j=0;j<6;j++)
			{
				x.u32 = ((uint32_t)buf[idx++]<<24);
				x.u32 += ((uint32_t)buf[idx++]<<16);
				x.u32 += ((uint32_t)buf[idx++]<<8);
				x.u32 += ((uint32_t)buf[idx++]<<0);
				pC->Arm.Links[i].origin[j] = x.f32;
			}
			for(int j=0;j<6;j++)
			{
				x.u32 = ((uint32_t)buf[idx++]<<24);
				x.u32 += ((uint32_t)buf[idx++]<<16);
				x.u32 += ((uint32_t)buf[idx++]<<8);
				x.u32 += ((uint32_t)buf[idx++]<<0);
				pC->Arm.Links[i].innertia[j] = x.f32;
			}
			x.u32 = ((uint32_t)buf[idx++]<<24);
			x.u32 += ((uint32_t)buf[idx++]<<16);
			x.u32 += ((uint32_t)buf[idx++]<<8);
			x.u32 += ((uint32_t)buf[idx++]<<0);
			pC->Arm.Links[i].mass = x.f32;
		}
		
		pC->Jtc.flagInitGetArmModel = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameArmModelUseDefault(uint8_t* buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		Control_SetDefualtArmModel();
		
		pC->Jtc.flagInitGetArmModel = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameTrajSetExecStatus(uint8_t *buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		Traj.targetTES = (eTrajExecStatus)buf[4];
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameTeachingModeEnable(uint8_t *buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		pC->Jtc.teachingModeReq = true;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFrameTeachingModeDisable(uint8_t *buf)
{
	uint16_t nd = Com.rxFrame.expectedLength;
	
	uint16_t crc1 = Com_Crc16(buf, nd-2);
	uint16_t crc2 = ((uint16_t)buf[nd-2]<<8) + ((uint16_t)buf[nd-1]<<0);
	
	if(crc1 == crc2)
	{
		Com.timeout = 0;
		
		// odebrane dane sa poprawne
		Com.rxFrame.dataStatus = Host_RxDS_NoError;
		
		pC->Jtc.teachingModeReq = false;
	}
	else
	{
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectCrc;
	}
}
static void Host_ComReadFromHost(void)
{
	if(Com.firstRun == false)
	{
		Com.firstRun = true;
		return;
	}
	
	uint8_t* buf = Com.bufread0;
	
	if(buf[0] == Host_FT_Header)
	{
		if(buf[1] == Host_FT_ClearCurrentErrors || buf[1] ==  Host_FT_ClearOccuredErrors || buf[1] == Host_FT_Trajectory || buf[1] == Host_FT_FrictionTable || buf[1] == Host_FT_FrictionTableUseDefault || buf[1] == Host_FT_PidParam || 
			 buf[1] == Host_FT_PidParamUseDefault || buf[1] == Host_FT_ArmModel || buf[1] == Host_FT_ArmModelUseDefault || buf[1] == Host_FT_TrajSetExecStatus || 
			 buf[1] == Host_FT_TeachingModeEnable || buf[1] == Host_FT_TeachingModeDisable || buf[1] == Host_FT_FrictionPolynomial || buf[1] == Host_FT_FrictionPolynomialUseDefault)
		{
			Com_CheckConsistencyReceivedFrame(buf);
			if(Com.rxFrame.consist == Host_RxFC_IsReceived)
				return;
		}
	}
	if(buf[0] == Host_FT_Header)
	{
		Com.rxFrame.status = Host_RxFS_NoError;
		Com.rxFrame.frameType = (eHost_FrameType)buf[1];
		if(buf[1] == Host_FT_ClearCurrentErrors)
		{
			Host_ComReadFrameClearCurrentErrors(buf);
		}
		else if(buf[1] == Host_FT_ClearOccuredErrors)
		{
			Host_ComReadFrameClearOccuredErrors(buf);
		}
		else if(buf[1] == Host_FT_Trajectory)
		{
			Host_ComReadFrameTrajectory(buf);
		}
		else if(buf[1] == Host_FT_FrictionTable)
		{
			Host_ComReadFrameFricionTable(buf);
		}
		else if(buf[1] == Host_FT_FrictionTableUseDefault)
		{
			Host_ComReadFrameFricionTableUseDefault(buf);
		}
		else if(buf[1] == Host_FT_PidParam)
		{
			Host_ComReadFramePidParam(buf);
		}
		else if(buf[1] == Host_FT_PidParamUseDefault)
		{
			Host_ComReadFramePidParamUseDefault(buf);
		}
		else if(buf[1] == Host_FT_ArmModel)
		{
			Host_ComReadFrameArmModel(buf);
		}
		else if(buf[1] == Host_FT_ArmModelUseDefault)
		{
			Host_ComReadFrameArmModelUseDefault(buf);
		}
		else if(buf[1] == Host_FT_TrajSetExecStatus)
		{
			Host_ComReadFrameTrajSetExecStatus(buf);
		}
		else if(buf[1] == Host_FT_TeachingModeEnable)
		{
			Host_ComReadFrameTeachingModeEnable(buf);
		}
		else if(buf[1] == Host_FT_TeachingModeDisable)
		{
			Host_ComReadFrameTeachingModeDisable(buf);
		}
		else if(buf[1] == Host_FT_FrictionPolynomial)
		{
			Host_ComReadFrameFricionPolynomial(buf);
		}
		else if(buf[1] == Host_FT_FrictionPolynomialUseDefault)
		{
			Host_ComReadFrameFricionPolynomialUseDefault(buf);
		}
		else
		{
			Com.rxFrame.frameType = Host_FT_null;
			Com.rxFrame.status = Host_RxFS_ErrorIncorrectFrameType;
		}
	}
	else
	{
		Com.rxFrame.frameType = Host_FT_null;
		Com.rxFrame.status = Host_RxFS_ErrorIncorrectHeader;
	}
	Com_ClearStr(buf, HOST_COMBUFREADSIZE);
	Com_ReinitDmaReadStream();
}
static void Com_ReadDiscontinuousFrameTimeout(void)
{
	TIM13->DIER &= ~TIM_DIER_UIE;
	Com.rxFrame.frameType = Host_FT_null;
	Com.rxFrame.status = Host_RxFS_ErrorDiscontinuousFrame;
	Com_ReinitDmaReadStream();
	Com.rxFrame.status = Host_RxFS_Idle;
}
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_SR_UIF) != RESET)
	{
		Com.txFrames[Host_TxFN_JtcStatus].active = true;
		Host_ComSendResponseToHost();
		TIM6->SR &= ~TIM_SR_UIF;
	}
}
void TIM8_UP_TIM13_IRQHandler(void)
{
	if((TIM13->SR & TIM_SR_UIF) != RESET)
	{
		Com_ReadDiscontinuousFrameTimeout();
		TIM13->SR &= ~TIM_SR_UIF;
	}
}
void USART2_IRQHandler(void)
{
	if((USART2->ISR & USART_ISR_IDLE) != RESET)
	{
		char c = USART3->RDR;
		Host_ComReadFromHost();
		USART2->ICR |= USART_ICR_IDLECF;
	}
}
void USART3_IRQHandler(void)
{
	if((USART3->ISR & USART_ISR_IDLE) != RESET)
	{
		char c = USART3->RDR;
		Host_ComReadFromHost();
		USART3->ICR |= USART_ICR_IDLECF;
	}
}
// Przerwanie od strumienia odbiorczego
void DMA1_Stream0_IRQHandler(void)
{
	if((DMA1->LISR & DMA_LISR_TCIF0) != RESET)
	{
		Com_ReinitDmaReadStream();
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
	}
}
// Przerwanie od strumienia wysylajacego
void DMA1_Stream1_IRQHandler(void)
{
	if((DMA1->LISR & DMA_LISR_TCIF1) != RESET)
	{
		Com.txFrames[Com.framenum].status = Host_TxFS_Idle;
		Com.status = Host_CS_Idle;
		DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
	}
}
