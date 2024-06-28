#include "Water_tank_liquid.h"
#include "gpio.h"
#include "key.h"
#include "delay.h"

u8 Water_tank_liquid_low(u8 mode)
{
	
	if(Water_tank_Low == GPIO_PIN_RESET)//检测到低水位触发。
	{
		HAL_Delay(50);
		if(Water_tank_Low == GPIO_PIN_RESET)
		{
			return 1;//触发低水位
		}
	}
		
	return 0;//没有触发水位

}


u8 Water_tank_liquid_high(u8 mode)
{
	
	if(Water_tank_High == GPIO_PIN_RESET)//检测到高水位触发。
	{
		HAL_Delay(50);
		if(Water_tank_High == GPIO_PIN_RESET)
		{
			return 2;//触发高水位
		}
	}
		
	return 0;//没有触发水位
}












