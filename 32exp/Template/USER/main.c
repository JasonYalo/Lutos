#include"sys.h"
#include"delay.h"
#include"usart.h"

void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{

while(nCount--){}

}

int main(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
  Cache_Enable();   //打开L1-Cache
	HAL_Init();
	Stm32_Clock_Init(432,25,2,9);  //设置时钟213Mhz
  __HAL_RCC_GPIOB_CLK_ENABLE();   //开启GPIOB时钟
	
	GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1;
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;    //推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;  //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;
	
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	while(1)
	{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);         	
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);    //置1
		Delay(0x7FFFFF);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);         	
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);    //置0
		
		Delay(0x7FFFFF);
	
	}
	

}
