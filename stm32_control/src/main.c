
#include <stdio.h>
#include "stm32f407xx.h"
#include "support.hpp"
////////////////////////////////////////////////////
#define PLL_M 4
#define PLL_N 168
#define PLL_P 0
///////////////////////////////////////////////////
void SysClock_Config(void);
void GPIOA_Config(int);
//1. Enable clock access to the PORT of the PIN
//2. Set the PIN's mode
void TIM6_Config(void);
void delay_us(uint16_t time);
void delay_ms(uint16_t time);
void setup_can();
///////////////////////////////////////////////////
inline void initialize_func()
{
	SysClock_Config();
	GPIOA_Config(6);
	GPIOA_Config(7);
	TIM6_Config();
}
///////////////////////////////////////////////////
int main(void){

	initialize_func();
	
    while(1){
    	printf("Hello");
    	GPIOA->ODR |= GPIO_En(6);
    	GPIOA->BSRR |= GPIO_Re(7);
    	delay_ms(1000);
    	
    	GPIOA->ODR &= ~GPIO_En(6);
    	GPIOA->BSRR |= GPIO_En(7);
    	delay_ms(1000);
    
    }
}
///////////////////////////////////////////////////
void SysClock_Config(void){
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	RCC->PLLCFGR =  (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE;
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
/**********************************************************/
void GPIOA_Config(int PIN_NO){
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER |= (1<<(2*PIN_NO));
	GPIOA->OTYPER &= ~(1<<(PIN_NO));
	GPIOA->OSPEEDR |= (1<<(2*PIN_NO+1));
	GPIOA->PUPDR &= ~(1<<(2*PIN_NO) | 1<<(2*PIN_NO+1));
}
/**********************************************************/
void TIM6_Config(void){
	//Enable and Configure TIMER6
	RCC->APB1ENR |= (1<<4);
	TIM6->PSC = 83;
	TIM6->ARR = 0xFFFF;
	TIM6->CR1 |= (1<<0);
	while(!(TIM6->SR & (1<<0)));
}
/**********************************************************/
void delay_us(uint16_t time){
	TIM6->CNT = 0;
	while (TIM6->CNT < time);
}
/**********************************************************/
void delay_ms(uint16_t time){

	for(uint16_t i=0; i<time; i++){
		delay_us(1000);
	}
}
/***********************************************************/


