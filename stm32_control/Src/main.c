/*
 * PWM on PA3 and PA5 - Channel 4 and Channel 1 Respectively
 *
 *
 */
///////////////////////////////////////////////////

#include <stdio.h>
#include <stm32f407xx.h>
#include <math.h>
#define GPIO_En(x) (1<<(x))

////////////////////////////////////////////////////

#define PLL_M 4
#define PLL_N 168
#define PLL_P 0

///////////////////////////////////////////////////

#define set_point 10000
#define k_p 80
#define k_d 3
#define k_i 10
#define integral_threshold 320

///////////////////////////////////////////////////

volatile uint32_t counter;

///////////////////////////////////////////////////

void SysClock_Config(void);
void GPIOA_Config(int);
void TIM2_PWM_Config();

////////////////////////OLD Interrupt Based Approach/////////////
//void EXTI1_IRQHandler();
//void EXTI0_IRQHandler();
//void GPIOA_ExtInt_Config();
/////////////////////////////////////////////////////////////////

///////////////////////NEW Encoder Mode Based Approach///////////

void enable_encoderMode();

///////////////////Debugging///////////////////////

void UART3Config(void);
void UART3SendCh(uint8_t ch);
void UART3SendString (char *string);
uint8_t UART3GetCh(void);
void itoa(uint16_t number,char* data, int length);

void delay_us(uint16_t time);
void delay_ms(uint16_t time);
void TIM6Config(void);

///////////////////////////////////////////////////
void initialize_func(){

	SysClock_Config();
	GPIOA_Config(6);
	GPIOA_Config(7);
	//GPIOA_ExtInt_Config();
	TIM2_PWM_Config();
	UART3Config();
	TIM6Config();
	enable_encoderMode();

}
///////////////////////////////////////////////////
int main(void){

	initialize_func();
	char data[20];

	int32_t current_cnt;
	double error, derivative,integral, error_difference,pid;
	double previous_error = 0;


    while(1){
    	current_cnt = TIM4->CNT-0xFFFF/2;
    	error = (double)(current_cnt - set_point);
    	error_difference = error - previous_error;
    	derivative = error_difference;

		if(fabs(error) < integral_threshold)
			integral +=  error;
		else
			integral = 0;


		pid = k_p * error +  k_d * derivative + k_i * integral;


		if(fabs(pid) >26667)
			pid = (pid/fabs(pid))*26667;



		if(error > 0){
			TIM2->CCR1 = fabs(pid); /* pulse width 1/3 of the period */
			TIM2->CCR4 = 1 - 1;
		}
		else if(error < 0){
			TIM2->CCR1 = 1 - 1; /* pulse width 1/3 of the period */
			TIM2->CCR4 = fabs(pid);
		}
		else{
			TIM2->CCR1 = 1 - 1; /* pulse width 1/3 of the period */
			TIM2->CCR4 = 1 - 1;
		}
		previous_error = error;

		itoa(current_cnt,data,20);
		UART3SendString(data);
		UART3SendCh('\n');

		delay_ms(10);

    }
}
///////////////////////////////////////////////////
void SysClock_Config(void){
	RCC->CR  |= RCC_CR_HSEON;
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
/*void GPIOA_ExtInt_Config(){
	RCC->APB2ENR |= GPIO_En(14);  // Enable SYSCFG

	SYSCFG->EXTICR[0] &= ~(0xFF);  // Configure EXTI1 and EXTI0 line for PA1 and PA0

	EXTI->IMR |= (GPIO_En(1)|GPIO_En(0));  // Bit[1] = 1  --> Disable the Mask on EXTI 1

	EXTI->RTSR |= (GPIO_En(1)|GPIO_En(0));  // Enable Rising Edge Trigger for PA1
	EXTI->FTSR |= (GPIO_En(1)|GPIO_En(0));  // Enable Falling Edge Trigger for PA1
	NVIC_SetPriority (EXTI1_IRQn, 0);  // Set Priority
	NVIC_EnableIRQ (EXTI1_IRQn);  // Enable Interrupt
	NVIC_EnableIRQ (EXTI0_IRQn);  // Enable Interrupt
}*/
/**********************************************************/
/*void EXTI1_IRQHandler(void){


   if(GPIOA->IDR & 0x00)
     counter++;
   else
	 counter--;

   EXTI->PR |= 1<<1;
}*/
/**********************************************************/
/*void EXTI0_IRQHandler(void){


	if(GPIOA->IDR & 0x01)
	     counter--;
	else
		 counter++;
	EXTI->PR |= 1<<0;
}
*/
/**********************************************************/
void GPIOA_Config(int PIN_NO){
	RCC->AHB1ENR |= GPIO_En(0);;
	GPIOA->MODER |= GPIO_En(2*PIN_NO);
	GPIOA->OTYPER &= ~(GPIO_En(PIN_NO));
	GPIOA->OSPEEDR |= GPIO_En(2*PIN_NO+1);
	GPIOA->PUPDR &= ~(GPIO_En(2*PIN_NO) | GPIO_En(2*PIN_NO+1));
}
/**********************************************************/
void TIM2_PWM_Config(){
	//Enable and Configure TIMER2
	GPIOA->AFR[0] |= 0x00101000; /* PA5 pin for tim2 */
	GPIOA->MODER &= ~0x00000CC0;
	GPIOA->MODER |= 0x00000880;
	/* setup TIM2 */

	RCC->APB1ENR |= (1<<0);

	TIM2->ARR = 26667 - 1; /* divided by 26667 */
	TIM2->CNT = 0;

	TIM2->CCMR1 = 0x0060; /* PWM mode */
	TIM2->CCMR2 = 0x6000;

	TIM2->CCER = 1 | GPIO_En(12); /* enable PWM Ch1 & Ch 4*/

	TIM2->CCR1 = 1 - 1; /* pulse width 1/3 of the period */
	TIM2->CCR4 = 1 - 1;

	TIM2->CR1 = 1; /*enable timer */
}
/**********************************************************/
void UART3Config(void){
	RCC->APB1ENR |= GPIO_En(18);
	RCC->AHB1ENR |= GPIO_En(1);

	GPIOB->MODER |=(2<<20)|(2<<22);
	GPIOB->OSPEEDR |= ((3<<4)|(3<<6));
	GPIOB->AFR[1] |= (7<<8) | (7<<12);

	USART3->CR1 = 0x00;
	USART3->CR1 |= (1<<13);
	USART3->BRR = (13<<0) | (22<<4);

	USART3->CR1 |= (1<<2);
	USART3->CR1 |= (1<<3);
}
/***************************************************************/
void UART3SendCh(uint8_t ch){
	USART3->DR = ch;
	while(!(USART3->SR & (1<<6)));
}
/***************************************************************/
void UART3SendString (char *string){
	while (*string) UART3SendCh(*string++);
}
/***************************************************************/
uint8_t UART3GetCh(void){
	uint8_t Temp;
	while (!(USART3->SR & (1<<5)));
	Temp = USART3->DR;
	return Temp;
}
/**********************************************************/
void itoa(uint16_t number,char* data, int length){
	uint16_t temp,i=0;

	for(int j=0; j< length;j++)
		*(data+j)='0';

	while(number != 0){
		temp = number - (number/10)*10;
		*(data+length-i) = temp + '0';
		number = number/10;
		i++;
	}
}
/**********************************************************/
void TIM6Config(void){
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
/************************************************************/
void enable_encoderMode(){
	RCC->AHB1ENR = (1<<1);
	RCC->APB1ENR |= (1<<2);
	GPIOB->MODER |= (2<<12)|(2<<14);
	GPIOB->AFR[0] = 0x22000000;

	TIM4->ARR  = 0xFFFF;
	TIM4->CCER &= ~((1<<1) | (1<<5) |(1<<3) |(1<<7));
	TIM4->CCMR1 |= ((1<<0) | (1<<8));

	TIM4->SMCR |= (1<<0)|(1<<1);
	TIM4->CNT = 0xFFFF/2;
	TIM4->CR1 |= (1<<0);
}
