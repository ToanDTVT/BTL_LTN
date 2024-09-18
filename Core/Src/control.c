#include "control.h"

 char gas (uint32_t value)
{
	 if (value < 100)
		return 10 ;
	 if ((value > 400) & (value < 500))
		return 20 ;
	 if ((value > 700) & (value < 800) )
		return 30 ;
	 if ((value > 900) & (value < 1000) )
		return 40 ;
	 if ((value > 1100) & (value < 1200) )
		return 50 ;
	 if ((value > 1300) & (value < 1400) )
		return 60 ;
	 if ((value > 1500) & (value < 1600) )
		return 70 ;
	 if ((value > 1700) & (value < 1800) )
		return 80 ;
	 if ((value > 1900) & (value < 2000) )
		return 90 ;
	 if (value > 2100)
		return 100 ;
	 return 0;
}



void toggle(void)
{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
}








