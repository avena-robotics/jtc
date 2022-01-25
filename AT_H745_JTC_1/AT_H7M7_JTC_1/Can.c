#include "Can.h"
extern  sControl* pC;
static void Can_StructConf(void)
{
	pC->Can.filterAddrOffset = 0x0000;
	pC->Can.txBufAddrOffset = pC->Can.filterAddrOffset + 4 * CAN_FILTERS_MAX;
	pC->Can.rxBufAddrOffset = pC->Can.txBufAddrOffset + (CAN_TXDATA_LEN + 8) * CAN_TXBUF_MAX;
	pC->Can.rxFifo0AddrOffset = pC->Can.rxBufAddrOffset + (CAN_RXDATA_LEN + 8) * CAN_RXBUF_MAX;
	
	pC->Can.filterAddr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.filterAddrOffset);
	pC->Can.txBufAddr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.txBufAddrOffset);
	pC->Can.rxBufAddr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.rxBufAddrOffset);
	pC->Can.rxFifo0Addr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.rxFifo0AddrOffset);
}
static void Can_FiltersConf(void)
{
	// filtr dla Joint0
	pC->Can.Filters[0].sfid1 = 0xA0;
	
	// filtr dla Joint1
	pC->Can.Filters[1].sfid1 = 0xB0;
	
	// filtr dla Joint2
	pC->Can.Filters[2].sfid1 = 0xC0;
	
	// filtr dla Joint3
	pC->Can.Filters[3].sfid1 = 0xD0;
	
	// filtr dla Joint4
	pC->Can.Filters[4].sfid1 = 0xE0;
	
	// filtr dla Joint5
	pC->Can.Filters[5].sfid1 = 0xF0;
	
	for(int i=0;i<CAN_FILTERS_MAX;i++)
	{
		pC->Can.Filters[i].sft = 0x01;
		// ramki zgodne z filtrm trafiaja do buforow odbiorczych
		pC->Can.Filters[i].sfec = 0x07;
		pC->Can.Filters[i].sfid2 = i;
		pC->Can.Filters[i].r0 = (pC->Can.Filters[i].sft << 30) | (pC->Can.Filters[i].sfec << 27) | (pC->Can.Filters[i].sfid1 << 16) | (pC->Can.Filters[i].sfid2 << 0);
		*(pC->Can.filterAddr + i) = pC->Can.Filters[i].r0;
	}
}
static void Can_FdcanConf(void)
{
	// konfiguracja wyprowadzen
	GPIOD->MODER &= ~GPIO_MODER_MODE0 & ~GPIO_MODER_MODE1;
	GPIOD->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;
	GPIOD->AFR[0] |= 0x00000099;
	
	// rozpoczecie konfiguracji FDCAN
	FDCAN1->CCCR |= FDCAN_CCCR_INIT | FDCAN_CCCR_CCE;
	// Ramka w formacie CANFD ze zmienna predkoscia (bit BRSE)
	FDCAN1->CCCR |= FDCAN_CCCR_BRSE | FDCAN_CCCR_FDOE | FDCAN_CCCR_TXP;
	
	// predkosc transmisji w trybie nominalnym
	FDCAN1->NBTP = (0x01 << FDCAN_NBTP_NSJW_Pos) | (0x04 << FDCAN_NBTP_NBRP_Pos) | (0x0B << FDCAN_NBTP_NTSEG1_Pos) | (0x03 << FDCAN_NBTP_NTSEG2_Pos);
	// predkosc transmisji w trybie danych
	FDCAN1->DBTP = (0x01 << FDCAN_DBTP_DSJW_Pos) | (0x00 << FDCAN_DBTP_DBRP_Pos) | (0x0B << FDCAN_DBTP_DTSEG1_Pos) | (0x03 << FDCAN_DBTP_DTSEG2_Pos);
	
	// wszystkie ramki zdalne i ramki ktore nie przeszly filtracji sa odrzucane
	FDCAN1->GFC = (0x03 << FDCAN_GFC_ANFS_Pos) | (0x03 << FDCAN_GFC_ANFE_Pos) | FDCAN_GFC_RRFS | FDCAN_GFC_RRFE;
	// CAN_FILTERS_MAX filtrow standardowych i adres filtrow
	FDCAN1->SIDFC = (CAN_FILTERS_MAX << FDCAN_SIDFC_LSS_Pos) | (pC->Can.filterAddrOffset << 0); 
	
	//CAN_TXBUF_MAX buforow nadawczych i adres pierwszego bufora nadawczego
	FDCAN1->TXBC = (CAN_TXBUF_MAX << FDCAN_TXBC_NDTB_Pos) | (pC->Can.txBufAddrOffset << 0);
	//bufory nadawcze o rozmiarze 20 bajtow
	FDCAN1->TXESC = (CAN_TXBUFSIZE_CODE << FDCAN_TXESC_TBDS_Pos);
	
	// bufory odbiorcze o rozmiarze 12 bajtow, elementy RXFIFO1 i RXFIFO0 o rozmiarze 12 bajtow
	FDCAN1->RXESC = (CAN_RXBUFSIZE_CODE << FDCAN_RXESC_RBDS_Pos) | (CAN_RXBUFSIZE_CODE << FDCAN_RXESC_F1DS_Pos) | (CAN_RXBUFSIZE_CODE << FDCAN_RXESC_F0DS_Pos);
	// offset adresu pierwszego bufora odbiorczego
	FDCAN1->RXBC = (pC->Can.rxBufAddrOffset << 0); 
	// CAN_RXBUFF_MAX buforow odbiorczych, CAN_RXFIFO0_MAX elementow fifo0 i adres pierwszego elementu fifo0
	FDCAN1->RXF0C = (CAN_RXFIFO0_MAX << FDCAN_RXF0C_F0S_Pos) | (pC->Can.rxFifo0AddrOffset << 0); 

	// Przrwanie od odbioru do bufora, te przerwania kierowane sa do EINT0
	FDCAN1->IE = FDCAN_IE_TCE | FDCAN_IE_DRXE;
	// Wlaczenie przerwan od transfer complete indywidualnie dla kazdego bufora nadawczego
	for(int i=0;i<CAN_TXBUF_MAX;i++)
		FDCAN1->TXBTIE |= (1 << i);
	// Przrwanie od obslugi bledow, te przerwania kierowane sa do EINT1
	FDCAN1->IE |= FDCAN_IE_ARAE | FDCAN_IE_PEDE | FDCAN_IE_PEAE | FDCAN_IE_WDIE | FDCAN_IE_BOE | FDCAN_IE_EWE | FDCAN_IE_EPE | FDCAN_IE_ELOE;
	FDCAN1->ILS = FDCAN_ILS_ARAE | FDCAN_ILS_PEDE | FDCAN_ILS_PEAE | FDCAN_ILS_WDIE | FDCAN_ILS_BOE | FDCAN_ILS_EWE | FDCAN_ILS_EPE | FDCAN_ILS_ELOE;
	// wlaczenie linii przerwania
	FDCAN1->ILE = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;
	NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}
static void Can_Start(void)
{
	// koniec konfiguracji i uruchomienie FDCAN
	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT; 
}
void Can_Conf(void)
{
	Can_StructConf();
	Can_FiltersConf();
	Can_FdcanConf();
	Can_Start();
}
static void Can_SendFrameOccurred(void)
{
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		if(((FDCAN1->TXBTO >> num) & 0x01) != RESET)
		{
			pC->Can.TxMsgs[num].status = Can_TxS_Sent;
			pC->Can.TxMsgs[num].timeoutCnt = 0;
			pC->Can.TxMsgs[num].frameTotalCnt++;
		}
	}
}
static void Can_SendFrameToJoints(void)
{
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
		{
			pC->Can.TxMsgs[num].esi = 0x00;	//passive error
			pC->Can.TxMsgs[num].xtd = 0x00;	//standardowe identyfikatory 11bit
			pC->Can.TxMsgs[num].rtr = 0x00;	//ramka z danymi
			pC->Can.TxMsgs[num].id = 0xAA;		//identyfikator 11 bitowy
			pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
			pC->Can.TxMsgs[num].efc = 0x00;	//bez geneorwanie eventow
			pC->Can.TxMsgs[num].fdf = 0x01;	//ramka w formacie CANFD
			pC->Can.TxMsgs[num].brs = 0x01;	//zmienna predkosc - BRS = On
			pC->Can.TxMsgs[num].dlc = CAN_TXDATA_DLCCODE;	//20 bajtow w ramce
			pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
			pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);
			
			uint16_t idx = 0;
			for(int i=0;i<JOINTS_MAX;i++)
			{
				pC->Can.TxMsgs[num].bytes[idx++] = (int16_t)(pC->Joints[i].setTorque / pC->Joints[i].maxTorqueCom * MAXINT16) >> 8;
				pC->Can.TxMsgs[num].bytes[idx++] = (int16_t)(pC->Joints[i].setTorque / pC->Joints[i].maxTorqueCom * MAXINT16) >> 0;
				pC->Can.TxMsgs[num].bytes[idx++] = pC->Joints[i].targetFsm;
			}
			
			for(int i=0;i<4;i++)
			{
				pC->Can.TxMsgs[num].data[i] = 0x00;
				pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
				pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
				pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
				pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
			}
			
			*(pC->Can.txBufAddr + 0) = pC->Can.TxMsgs[num].r0;
			*(pC->Can.txBufAddr + 1) = pC->Can.TxMsgs[num].r1;
			*(pC->Can.txBufAddr + 2) = pC->Can.TxMsgs[num].data[num];
			*(pC->Can.txBufAddr + 3) = pC->Can.TxMsgs[num].data[1];
			*(pC->Can.txBufAddr + 4) = pC->Can.TxMsgs[num].data[2];
			*(pC->Can.txBufAddr + 5) = pC->Can.TxMsgs[num].data[3];
			*(pC->Can.txBufAddr + 6) = pC->Can.TxMsgs[num].data[4];
			
			pC->Can.TxMsgs[num].status = Can_TxS_Sending;
			
			FDCAN1->TXBAR |= (1 << num);
		}
	}
}
static void Can_ReadFrameFromBuffer(void)
{
	for(int num=0;num<CAN_RXBUF_MAX;num++)
	{
		if(((FDCAN1->NDAT1 >> num) & 0x01) != RESET)
		{
			uint16_t idx = 5 * num;
			pC->Can.RxMsgs[num].r0 = *(pC->Can.rxBufAddr + idx + 0);
			pC->Can.RxMsgs[num].r1 = *(pC->Can.rxBufAddr + idx + 1);
			pC->Can.RxMsgs[num].data[0] = *(pC->Can.rxBufAddr + idx + 2);
			pC->Can.RxMsgs[num].data[1] = *(pC->Can.rxBufAddr + idx + 3);
			pC->Can.RxMsgs[num].data[2] = *(pC->Can.rxBufAddr + idx + 4);
			
			pC->Can.RxMsgs[num].esi = (pC->Can.RxMsgs[num].r0 >> 31) & 0x01;
			pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 30) & 0x01;
			pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 29) & 0x01;
			pC->Can.RxMsgs[num].id = (pC->Can.RxMsgs[num].r0 >> 18) & 0x07FF;
			
			pC->Can.RxMsgs[num].anmf = (pC->Can.RxMsgs[num].r1 >> 31) & 0x01;
			pC->Can.RxMsgs[num].fidx = (pC->Can.RxMsgs[num].r1 >> 24) & 0x3F;
			pC->Can.RxMsgs[num].fdf = (pC->Can.RxMsgs[num].r1 >> 21) & 0x01;
			pC->Can.RxMsgs[num].brs = (pC->Can.RxMsgs[num].r1 >> 20) & 0x01;
			pC->Can.RxMsgs[num].dlc = (pC->Can.RxMsgs[num].r1 >> 16) & 0x0F;
			pC->Can.RxMsgs[num].rxts = (pC->Can.RxMsgs[num].r1 >> 16) & 0xFFFF;
			
			for(int i=0;i<3;i++)
			{
				pC->Can.RxMsgs[num].bytes[4*i+0] = pC->Can.RxMsgs[num].data[i] >> 0;
				pC->Can.RxMsgs[num].bytes[4*i+1] = pC->Can.RxMsgs[num].data[i] >> 8;
				pC->Can.RxMsgs[num].bytes[4*i+2] = pC->Can.RxMsgs[num].data[i] >> 16;
				pC->Can.RxMsgs[num].bytes[4*i+3] = pC->Can.RxMsgs[num].data[i] >> 24;
			}
			
			// pozycja jest w rad
			pC->Joints[num].currentPos = (double)((int16_t)(((uint16_t)pC->Can.RxMsgs[num].bytes[0] << 8) + ((uint16_t)pC->Can.RxMsgs[num].bytes[1] << 0))) * pC->Joints[num].maxPosCom / MAXINT16;
			// predkosc jest w rad/s
			pC->Joints[num].currentVel = (double)((int16_t)(((uint16_t)pC->Can.RxMsgs[num].bytes[2] << 8) + ((uint16_t)pC->Can.RxMsgs[num].bytes[3] << 0))) * pC->Joints[num].maxVelCom / MAXINT16;
			// moment obrotowy jest w Nm
			pC->Joints[num].currentTorque = (double)((int16_t)(((uint16_t)pC->Can.RxMsgs[num].bytes[4] << 8) + ((uint16_t)pC->Can.RxMsgs[num].bytes[5] << 0))) * pC->Joints[num].maxTorqueCom / MAXINT16;
			pC->Joints[num].currentTemp = (double)pC->Can.RxMsgs[num].bytes[6];
			pC->Joints[num].currentFsm = (eJoint_FSM)pC->Can.RxMsgs[num].bytes[7];
			pC->Joints[num].mcCurrentError = pC->Can.RxMsgs[num].bytes[8];
			pC->Joints[num].mcOccuredError = pC->Can.RxMsgs[num].bytes[9];
			pC->Joints[num].currentError = pC->Can.RxMsgs[num].bytes[10];
			pC->Joints[num].currentWarning = pC->Can.RxMsgs[num].bytes[11];
			
			pC->Can.RxMsgs[num].timeoutCnt = 0;
			pC->Can.RxMsgs[num].frameTotalCnt++;
			pC->Can.RxMsgs[num].status = Can_RxS_Idle;
			
			FDCAN1->NDAT1 = (1 << num);
		}
	}
}
static void Can_TimeoutCntInc(void)
{
	for(int num=0;num<CAN_TXBUF_MAX;num++)
		pC->Can.TxMsgs[num].timeoutCnt++;
		
	for(int num=0;num<CAN_RXBUF_MAX;num++)
		pC->Can.RxMsgs[num].timeoutCnt++;
}
static void Can_CheckTxStatus(void)
{
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		if(pC->Can.TxMsgs[num].timeoutCnt >= CAN_TIMEOUTMAX)
		{
			pC->Can.TxMsgs[num].timeoutCnt = CAN_TIMEOUTMAX;
			pC->Can.TxMsgs[num].flagTimeout = true;
		}
		else
			pC->Can.TxMsgs[num].flagTimeout = false;
	}
}
static void Can_CheckRxStatus(void)
{
	for(int num=0;num<CAN_RXBUF_MAX;num++)
	{
		if(pC->Can.RxMsgs[num].timeoutCnt >= CAN_TIMEOUTMAX)
		{
			pC->Can.RxMsgs[num].timeoutCnt = CAN_TIMEOUTMAX;
			pC->Can.RxMsgs[num].flagTimeout = true;
		}
		else
			pC->Can.RxMsgs[num].flagTimeout = false;
	}
}
static void Can_CheckCanStatus(void)
{
	pC->Can.statusFlags = 0x00;
	// Bajt 0 dla Tx Timeout
	if(pC->Can.TxMsgs[0].flagTimeout) 	pC->Can.statusFlags |= (1 << Can_SFP_Tx0Timeout);

	// Bajt 1 dla Rx Timeout
	if(pC->Can.RxMsgs[0].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx0Timeout);
	if(pC->Can.RxMsgs[1].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx1Timeout);
	if(pC->Can.RxMsgs[2].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx2Timeout);
	if(pC->Can.RxMsgs[3].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx3Timeout);
	if(pC->Can.RxMsgs[4].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx4Timeout);
	if(pC->Can.RxMsgs[5].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx5Timeout);
		
	if(pC->Can.statusFlags != 0x00)
		pC->Can.statusId = Can_SId_NoError;
	else
		pC->Can.statusId = Can_SId_Error;
	
	pC->Can.statusOccurredFlags |= pC->Can.statusFlags;
}
static void Can_CheckStatus(void)
{
	Can_CheckTxStatus();
	Can_CheckRxStatus();
	Can_CheckCanStatus();	
}
void Can_SendDataToJoints(void)
{
	Can_SendFrameToJoints();
	Can_CheckStatus();
	Can_TimeoutCntInc();
}
void FDCAN1_IT0_IRQHandler(void)
{
	if((FDCAN1->IR & FDCAN_IR_TC) != RESET)
	{
		Can_SendFrameOccurred();
		FDCAN1->IR = FDCAN_IR_TC;
	}
	if((FDCAN1->IR & FDCAN_IR_DRX) != RESET)
	{
		Can_ReadFrameFromBuffer();
		FDCAN1->IR = FDCAN_IR_DRX;
	}
}
void FDCAN1_IT1_IRQHandler(void)
{
	if((FDCAN1->IR & FDCAN_IR_ARA) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_ARA;
	}
	if((FDCAN1->IR & FDCAN_IR_PED) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_PED;
	}
	if((FDCAN1->IR & FDCAN_IR_PEA) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_PEA;
	}
	if((FDCAN1->IR & FDCAN_IR_WDI) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_WDI;
	}
	if((FDCAN1->IR & FDCAN_IR_BO) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_BO;
	}
	if((FDCAN1->IR & FDCAN_IR_EW) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_EW;
	}
	if((FDCAN1->IR & FDCAN_IR_EP) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_EP;
	}
	if((FDCAN1->IR & FDCAN_IR_ELO) != RESET)
	{
		uint32_t reg = FDCAN1->ECR;
		FDCAN1->IR = FDCAN_IR_ELO;
	}
}
