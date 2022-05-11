#include "InputsOutputs.h"
extern sControl* pC;
static void IO_DIConf(void)
{
	//DI0 - hamulec parkowania - potwierdzenie
	GPIOC->MODER &= ~GPIO_MODER_MODE0;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD0_0;
}
static void IO_DQConf(void)
{
	//DQ0 - hamulec parkowania - sterowanie
	GPIOA->MODER &= ~GPIO_MODER_MODE3;
	GPIOA->MODER |= GPIO_MODER_MODE3_0;
}
static void IO_AIConf(void)
{
}
static void IO_AQConf(void)
{
}
void IO_Conf(void)
{
	IO_DIConf();
	IO_DQConf();
	IO_AIConf();
	IO_AQConf();
}
void IO_OutputsAct(void)
{
	//DQ Act
	pC->IO.DQReg = 0x0000;
	for(int i=0;i<DQ_MAX;i++)
		pC->IO.DQReg |= (pC->IO.DQ[i] << i);
	
	if(pC->IO.DQ[0] == true)		GPIOA->ODR |= GPIO_ODR_OD3;
	else												GPIOA->ODR &= ~GPIO_ODR_OD3;
	
	//AQ Act - currently not used
}
void IO_InputsAct(void)
{
	//DI Act
	if((GPIOC->IDR & GPIO_IDR_ID0) == RESET)			pC->IO.DI[0] = false;
	else																					pC->IO.DI[0] = true;
	
	for(int i=0;i<DI_MAX;i++)
		pC->IO.DIReg |= (pC->IO.DI[i] << i);
	
	//AI Act - currently not used
}
void IO_ParkBrakeUnlock(void)
{
	pC->IO.DQ[DQNum_ParkBrake] = true; //Stan wysoki na wyjsciu (3.3V) - hamulec odblokowany
}
void IO_ParkBrakeLock(void)
{
	pC->IO.DQ[DQNum_ParkBrake] = false;	//Stan niski na wyjsciu (0.0V) - hamulec zablokowany
}
bool IO_ParkBrakeInRead(void)
{
	//Stan wysoki na wyjsciu (3.3V) - hamulec zablokowany
	//Stan niski na wyjsciu (0.0V) - hamulec odblokowany
	return pC->IO.DI[DINum_ParkBrake]; 
}
