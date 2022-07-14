#include "Debug.h"
#ifdef DEBUG
extern sControl* pC;
extern sTrajectory Traj;
extern sDebug Debug;
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
static uint16_t Debug_Crc16(uint8_t* packet, uint32_t nBytes)
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
static void Debug_ReinitDmaReadStream(void)
{
	DMA1_Stream2->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF2;
	DMA1_Stream2->NDTR 	= (uint16_t)DEBUG_COMBUFREADSIZE;
	DMA1_Stream2->CR |= DMA_SxCR_EN;
}
static void Debug_StructConf(void)
{
	Debug.enableSending = false;	//Domyslnie wylaczone wysylanie ramek
	Debug.sampleTime = 2; //co ile ms ma byc zbierana ramka
	Debug.numFrames = 100; //ile ramek w jednym pakiecie
	Debug.bufNumber = 0;
	Debug.frameCnt = 0;
	Debug.frameLen = 166; //dlugosc pojedynczej ramki
}
static void Debug_ComUart3Conf(void)
{
	GPIOD->MODER &= ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE9;
	GPIOD->MODER |= GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0;
	GPIOD->AFR[1] |= 0x00000077;
	
	DMA1_Stream2->CR = 0x00;
	DMA1_Stream2->PAR 	= (uint32_t)&USART3->RDR;
	DMA1_Stream2->M0AR 	= (uint32_t)Debug.bufread;
	DMA1_Stream2->NDTR 	= (uint16_t)DEBUG_COMBUFREADSIZE;
	DMAMUX1_Channel2->CCR = (45 << 0); //USART3 RX
	DMA1_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	
	DMA1_Stream3->CR = 0x00;
	DMA1_Stream3->PAR 	= (uint32_t)&USART3->TDR;
	DMA1_Stream3->M0AR 	= (uint32_t)Debug.bufwrite0;
	DMA1_Stream3->NDTR 	= (uint16_t)DEBUG_COMBUFWRITESIZE;
	DMAMUX1_Channel3->CCR = (46 << 0); //USART3 TX
	DMA1_Stream3->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	
	USART3->BRR = 120000000/DEBUG_COMBAUDRATE;
	USART3->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART3_IRQn);
}
void Debug_Conf(void)
{
	Debug_StructConf();
	Debug_ComUart3Conf();
}
static void Debug_SendFrame(void)
{
	uint8_t* buf;
	if(Debug.bufNumber == 0)
		buf = Debug.bufwrite0;
	else if(Debug.bufNumber == 1)
		buf = Debug.bufwrite1;
	
	uint16_t idx = 0;
	uint16_t val;
	
	//Header - 4 bajty: 155, 156, 157, 158
	buf[idx++] = 155;
	buf[idx++] = 156;
	buf[idx++] = 157;
	buf[idx++] = 158;
	
	val = Debug.numFrames;
	buf[idx++] = val >> 8;
	buf[idx++] = val >> 0;
	
	Debug.packetLen = Debug.frameLen * Debug.frameCnt + 10;
	val = Debug.packetLen;
	buf[idx++] = val >> 8;
	buf[idx++] = val >> 0;
	
	idx = Debug.frameLen * Debug.frameCnt + 8;
//	uint16_t crc = Debug_Crc16(buf, idx);
	uint16_t crc = 0;
	buf[idx++] = crc >> 8;
	buf[idx++] = crc >> 0;
	
	
	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
	if(Debug.bufNumber == 0)
		DMA1_Stream3->M0AR 	= (uint32_t)buf;
	else if(Debug.bufNumber == 1)
		DMA1_Stream3->M0AR 	= (uint32_t)buf;
	DMA1_Stream3->NDTR 	= (uint16_t)idx;
	DMA1_Stream3->CR |= DMA_SxCR_EN;
	
	Debug.bufNumber++;
	Debug.bufNumber %= 2;
	Debug.frameCnt = 0;
}
void Debug_PrepareFrame(void)
{
	if(Debug.enableSending == false)
		return;
	
	Debug.tick++;
	if((Debug.tick % Debug.sampleTime) != 0)
		return;

	uint8_t* buf;
	if(Debug.bufNumber == 0)
		buf = Debug.bufwrite0;
	else if(Debug.bufNumber == 1)
		buf = Debug.bufwrite1;
	
	int16_t val = 0;
	union conv32 x;
	uint16_t idx = Debug.frameLen * Debug.frameCnt + 8;
	
	//znacznik czasu
	Debug.timeStamp += (double)Debug.sampleTime / 1000.0;
	x.f32 = Debug.timeStamp;
	buf[idx++] = x.u32 >> 24;
	buf[idx++] = x.u32 >> 16;
	buf[idx++] = x.u32 >> 8;
	buf[idx++] = x.u32 >> 0;

	for(int num=0;num<JOINTS_MAX;num++)
	{
		//Wartosci z trajektorii
		val = Traj.interpolatePoint.pos[num] / pC->Joints[num].maxPosCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = Traj.interpolatePoint.vel[num] / pC->Joints[num].maxVelCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = Traj.interpolatePoint.acc[num] / pC->Joints[num].maxAccCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		
		//Wartosci z jointów
		val = pC->Joints[num].currentPos / pC->Joints[num].maxPosCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].currentVel / pC->Joints[num].maxVelCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].currentTorque / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].currentBearingTemp;
		buf[idx++] = val >> 0;
		
		//Wartosci zadana momentu
		val = pC->Joints[num].setTorque / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		
		//Wartosci skladowe momentu
		val = pC->Joints[num].pidTorqueP / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].pidTorqueI / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].pidTorqueD / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].pidTorque / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].fricTorque / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
		val = pC->Joints[num].idTorque / pC->Joints[num].maxTorqueCom * MAXINT16;
		buf[idx++] = val >> 8;
		buf[idx++] = val >> 0;
	}
	Debug.frameCnt++;
	if(Debug.frameCnt >= Debug.numFrames)
		Debug_SendFrame();
}
static void Debug_ReadFrame(void)
{
	uint8_t* buf = Debug.bufread;
	//uint16_t crc1 = Debug_Crc16(buf, 2);
	//uint16_t crc2 = ((uint16_t)buf[2]<<8) + ((uint16_t)buf[3]<<0);
	uint16_t crc1 = 0;
	uint16_t crc2 = 0;
	if(crc1 == crc2)
	{
		if(buf[0] == Debug_FT_Header)
		{
			if(buf[1] == Debug_FT_EnableSending)
			{
				//155 1 212 48 - polecenie do wlaczenia
				Debug.enableSending = true;
			}
			if(buf[1] == Debug_FT_DisableSending)
			{
				//155 2 228 83 - polecenie do wylaczenia
				Debug.enableSending = false;
			}
		}
	}
	Debug_ReinitDmaReadStream();
}
void USART3_IRQHandler(void)
{
	if((USART3->ISR & USART_ISR_IDLE) != RESET)
	{
		char c = USART3->RDR;
		Debug_ReadFrame();
		USART3->ICR |= USART_ICR_IDLECF;
	}
}
// Przerwanie od strumienia wysylajacego
void DMA1_Stream3_IRQHandler(void)
{
	if((DMA1->LISR & DMA_LISR_TCIF3) != RESET)
	{
//		Com.txFrames[Com.framenum].status = Host_TxFS_Idle;
//		Com.status = Host_CS_Idle;
		DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
	}
}
#endif
