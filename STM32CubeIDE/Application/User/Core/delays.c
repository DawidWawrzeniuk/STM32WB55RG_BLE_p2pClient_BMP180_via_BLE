#include "main.h"


#include "delays.h"
extern TIM_HandleTypeDef htim16;

void Delay_us(uint16_t us)
{
	htim16.Instance->CNT = 0;
	while(htim16.Instance->CNT <= us);
}
