#include "MatlabSim.h"
#ifdef MATLABSIM
extern sControl* pC;
extern sMatSim MatSim;
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
static uint16_t MatlabSim_Crc16(uint8_t* packet, uint32_t nBytes)
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
static void MatlabSim_ReinitDmaReadStream(void)
{
	DMA1_Stream2->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF2;
	DMA1_Stream2->NDTR 	= (uint16_t)MATLABSIM_COMBUFREADSIZE;
	DMA1_Stream2->CR |= DMA_SxCR_EN;
}
static void MatlabSim_ComUart3Conf(void)
{
	GPIOD->MODER &= ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE9;
	GPIOD->MODER |= GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0;
	GPIOD->AFR[1] |= 0x00000077;
	
	DMA1_Stream2->CR = 0x00;
	DMA1_Stream2->PAR 	= (uint32_t)&USART3->RDR;
	DMA1_Stream2->M0AR 	= (uint32_t)MatSim.bufread;
	DMA1_Stream2->NDTR 	= (uint16_t)MATLABSIM_COMBUFREADSIZE;
	DMAMUX1_Channel2->CCR = (45 << 0); //USART3 RX
	DMA1_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	
	DMA1_Stream3->CR = 0x00;
	DMA1_Stream3->PAR 	= (uint32_t)&USART3->TDR;
	DMA1_Stream3->M0AR 	= (uint32_t)MatSim.bufwrite;
	DMA1_Stream3->NDTR 	= (uint16_t)MATLABSIM_COMBUFWRITESIZE;
	DMAMUX1_Channel3->CCR = (46 << 0); //USART3 TX
	DMA1_Stream3->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	
	USART3->BRR = 120000000/MATLABSIM_COMBAUDRATE;
	USART3->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART3_IRQn);
}
void MatlabSim_Conf(void)
{
	MatlabSim_ComUart3Conf();
}
void MatlabSim_SendFrame(void)
{
	MatSim.tick++;
	if((MatSim.tick % 50) != 0)
		return;
	
	uint16_t idx = 0;
	int16_t val;
	
//	//Header - 4 bajty: A, B, C, D
//	MatSim.bufwrite[idx++] = 'A';
//	MatSim.bufwrite[idx++] = 'B';
//	MatSim.bufwrite[idx++] = 'C';
//	MatSim.bufwrite[idx++] = 'D';
//	for(int num=0;num<JOINTS_MAX;num++)
//	{
//		//Wartosci z jointów
//		val = pC->Joints[num].setPosTemp / pC->Joints[num].maxPosCom * MAXINT16;
//		MatSim.bufwrite[idx++] = val >> 0;
//		MatSim.bufwrite[idx++] = val >> 8;
//	}
	
	idx = sprintf(MatSim.bufwrite, "ABCD%f,%f,%f,%f,%f,%f", pC->Joints[0].setPosTemp, pC->Joints[1].setPosTemp,pC->Joints[2].setPosTemp,pC->Joints[3].setPosTemp,pC->Joints[4].setPosTemp,pC->Joints[5].setPosTemp);

	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
	DMA1_Stream3->NDTR 	= (uint16_t)idx;
	DMA1_Stream3->CR |= DMA_SxCR_EN;
}
void USART3_IRQHandler(void)
{
	if((USART3->ISR & USART_ISR_IDLE) != RESET)
	{
		char c = USART3->RDR;
		MatlabSim_ReinitDmaReadStream();
		USART3->ICR |= USART_ICR_IDLECF;
	}
}
void DMA1_Stream3_IRQHandler(void)
{
	if((DMA1->LISR & DMA_LISR_TCIF3) != RESET)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
	}
}
#endif
