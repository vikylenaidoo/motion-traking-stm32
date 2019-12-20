/*motion tracking
 * this code for controlling the stepper motor
 *
 * */
// --------------------IMPORTS--------------------------------------------------------

#include "stm32f0xx.h"
#include "motion_tracking.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "stm32f0xx_misc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_dma.h"



//----------------------GLOBAL VARS && CONSTANTS------------------------------------------------------------
volatile bool status = 0; //used to keep step status
bool isCalibrated = 0;

bool dir = 0;
unsigned int steps = 0;
//float target_angle = 0;
unsigned int target_steps = 300; //max: 1200 steps == 180*
uint16_t x = 0;






// ------------------------MAIN--------------------------------------------


int main(){
//setup

	init_TIM14();
	init_GPIOA();
	init_GPIOB();
	init_UART1();

	dir = 1;
	GPIOB->ODR |= GPIO_ODR_1; //SET DIRECTION 1
	calibrate();

//main loop
	while(1){
		//float current_angle = steps*0.15;
		unsigned int current_steps = steps;

	/*receive x_center from jetson here*/
		//code

		update_motor_control(current_steps);



	}
}




//-----------------------------INITIALISING FUNCTIONS-------------------------------------

void init_GPIOB(void) //ouputs port
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable port B
//	GPIOB->MODER  |= 0x00505555;
	//set PB0 and PB1 to output mode
	GPIOB->MODER |= (GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0);

	GPIOB->ODR     = 0b0000000000000000; //set all output pins low

}


void init_GPIOA(void){ //inputs port
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER0;  // set sw0 to input
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; //pullup for sw0

	//configure interrupts
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; //enable clock for syscfg
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // map pa0 to exti0

	//SW0
	EXTI->IMR |= EXTI_IMR_MR0; //UNMASK EXTI0
	EXTI->RTSR |= EXTI_RTSR_TR0; // trigger on rising edge
	EXTI->FTSR |= EXTI_FTSR_TR0; // trigger on falling edge


	NVIC_EnableIRQ(EXTI0_1_IRQn);


}


void init_TIM14(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	//freq = 48000000/psc/arr hz
	TIM14->PSC = 999; //scale frequency
	TIM14->ARR = 23999; //set value to count up to

	TIM14->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM14->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_EnableIRQ(TIM14_IRQn);

}

void init_UART1(void){ //initilaise UART1
	/*http://www.sasabremec.com/?page_id=306
	 *https://community.st.com/s/question/0D50X00009XkZ55SAF/uart-example-code-for-stm32f0
	 */
// Configure PA9 (Tx) and PA10 (Rx)
	GPIOA->AFR[1]	|=  0b0001 << (1 * 4);      //AFRH, PA9, UART1
	GPIOA->AFR[1]	|=  0b0001 << (2 * 4);      //AFRH, PA10, UART1
	GPIOA->MODER	|=  GPIO_MODER_MODER9_1;  		// PA9 to AF mode
	GPIOA->MODER	|=  GPIO_MODER_MODER10_1;  		// PA10 to AF mode
	GPIOA->OTYPER  	&= ~GPIO_OTYPER_OT_9;       //Output Push/Pull
	GPIOA->OTYPER  	&= ~GPIO_OTYPER_OT_10;       //Output Push/Pull
	GPIOA->OSPEEDR 	|=  GPIO_OSPEEDR_OSPEEDR9;  //PA9 high speed
	GPIOA->OSPEEDR 	|=  GPIO_OSPEEDR_OSPEEDR10;  //PA10 high speed
	GPIOA->PUPDR   	|=  GPIO_PUPDR_PUPDR9_0;         //PA9 Pull up
	GPIOA->PUPDR   	|=  GPIO_PUPDR_PUPDR10_0;         //PA10 Pull up
//configure USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	//enable usart1
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


//configure interrupt on uart
	NVIC_EnableIRQ(USART1_IRQn);


//enable uart
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //interrupt for transmit empty
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//interrupt for receive not empty

}





//-----------------------------------------AUX FUNCTIONS -----------------------------------------------
void calibrate(void){
	//set the freq slow in one direction till the button interrupt
	while(!isCalibrated){

	}
}

void update_motor_control(unsigned int current_steps){
/*change freq and direction according to current position away from target_angle*/

	//set frequency
	//note: freq = 48000000/psc/arr psc=999+1
	unsigned int error = abs(target_steps-current_steps);
	unsigned int arr;
	if(error<1){
		arr = 959; //50 Hz
	}
	else{
		if(error>134){ //20 degrees: fastest
			arr = 479; //100 hz
		}
		else{
			arr = (int)(80000/error) - 1;
		}
	}
	TIM14->ARR = arr; //set value to count up to

	//set direction
	if(current_steps<target_steps){
		dir = 1;
		GPIOB->ODR |= GPIO_ODR_1;
	}
	else{
		dir = 0;
		GPIOB->ODR &= ~GPIO_ODR_1;
	}

}

uint16_t read_xcenter_from_serial(){ //read 2 bytes of data
	union d{
		char b[2];
		int xcenter;
	} data;

	data.b[0] = USART_ReceiveData(USART1);
	data.b[1] = USART_ReceiveData(USART1);

	return data.xcenter;
}



//----------------------------------------INTERRUPT HANDLERS--------------------------------------
void TIM14_IRQHandler(void){

	status = !status;

	if(status){
		if(dir){
			steps--;
		}
		else{
			steps++;
		}
		GPIOB->ODR |= GPIO_ODR_0; //set PB0 high
	}
	else{
		GPIOB->ODR &= ~GPIO_ODR_0; //set PB0 low

	}
	//clear the interrupt flag
	TIM14->SR &= ~TIM_SR_UIF;

}

void EXTI0_1_IRQHandler(void){
	if(!isCalibrated){
		isCalibrated = 1;
		steps = 0;
		//also set start time now
	}

	EXTI->PR |= EXTI_PR_PR0; //clear the interrupt

}


void USART1_IRQHandler(void){
#include "stm32f0-stdperiph/stm32f0xx_usart.h"
//receiving
	//(USART1, USART_IT_RXNE);
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){ //receive buffer not empty
		//read_xcenter_from_serial();
		USART_ReceiveData(USART1);
		x = x+1;
		//USART_ClearITPendingBit(USART1, USART_IT_RXNE);//clear the interrupt
	}

//sending
	/*if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET){ // Transmit the string in a loop
		USART_SendData(USART1, StringLoop[tx_index++]);
		if (tx_index >= (sizeof(StringLoop) - 1)){
			tx_index = 0;
		}
	}*/

}
// ---------------------------END-------------------------------------------------
