/*motion tracking

 * this code for controlling the stepper motor
 * @TODO: JETSON will send position asynchronously to be read on uart via polling
 * stepper motor controlled by tim14 and freq is changed according to position
 * @TODD: aNOTHER timer for keeping microsecond time
 * @TODO: TIM15 for 90HZ to send the current angle(current_steps) and timestamp(microseconds) to the dma to go to jetson to be logged
 *
 * */
// --------------------IMPORTS--------------------------------------------------------
#include <stdbool.h>
#include <stdlib.h>
//#include <math.h>

#include "stm32f0xx.h"
#include "diag/Trace.h"
#include "motion_tracking.h"

#include "stm32f0xx_usart.h"
#include "stm32f0xx_dma.h"

//----------------------GLOBAL VARS && CONSTANTS------------------------------------------------------------
#define width 320

volatile uint8_t DMA_BUFFER[6]; //6bytes of data to be sent

volatile uint32_t microseconds;

volatile bool status = 0; //used to keep step status
bool isCalibrated = 0;
bool dir = 0;

volatile unsigned int current_steps = 0;
volatile unsigned int target_steps = 300; //max: 1200 current_steps == 180*

//as received from the uart
volatile uint16_t x_center;

//will be used to keep the incoming uart data
#define INPUT_BUFFER_MAX_LENGTH 2
uint8_t INPUT_BUFFER[INPUT_BUFFER_MAX_LENGTH]; //max 2 bytes
uint8_t input_buffer_index = 0;




// ------------------------MAIN--------------------------------------------


int main(){
//setup

	init_TIM14(); //used for motor stepping
	init_GPIOA();
	init_GPIOB();
	init_UART1();

	dir = 1;
	GPIOB->ODR |= GPIO_ODR_1; //SET DIRECTION 1
	calibrate();

//main loop
	while(1){
		//float current_angle = current_steps*0.15;
		unsigned int steps = current_steps;

	/*receive x_center from jetson here*/
		update_target_steps(x_center);
		update_motor_control(steps);



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


void init_TIM14(void){ /*this timer used for motor control*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	//freq = 48000000/psc/arr hz
	TIM14->PSC = 999; //scale frequency
	TIM14->ARR = 499; //set value to count up to

	TIM14->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM14->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_SetPriority(TIM15_IRQn, 2);
	NVIC_EnableIRQ(TIM14_IRQn);

}

void init_TIM15(void){ /*this timer used at 90Hz to send angle & timestamp data back to the jetson*/
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

	//freq = 48000000/psc/arr = 90hz
	TIM15->PSC = 9;
	TIM15->ARR = 53332;

	TIM15->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM15->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_SetPriority(TIM15_IRQn, 0); //set to highest priority
	NVIC_EnableIRQ(TIM15_IRQn);


}

void init_TIM16(){ //used for keeping time
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	//freq = 48000000/psc/arr = 1MHz (1 usecond period)
	TIM16->PSC = 0;
	TIM16->ARR = 47;

	TIM16->DIER |= TIM_DIER_UIE; // enable update interrupt event
	TIM16->CR1 |= TIM_CR1_CEN; //start timer

	NVIC_EnableIRQ(TIM16_IRQn);


}

void init_UART1(void){
	/*initilaise UART1
	 *
	 **/

	/*
	 *https://community.st.com/s/question/0D50X00009XkZ55SAF/uart-example-code-for-stm32f0
	 *https://www.st.com/content/ccc/resource/technical/document/user_manual/2f/77/25/0f/5c/38/48/80/DM00122015.pdf/files/DM00122015.pdf/jcr:content/translations/en.DM00122015.pdf
	 *https://community.st.com/s/question/0D50X00009XkaKtSAJ/recieve-more-then-1-byte-over-usart
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

//configure interrupt for uart
	//NVIC_EnableIRQ(USART1_IRQn);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//enable uart
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //interrupt for transmit empty
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//interrupt for receive not empty

//configure dma
	init_DMA();
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	//@TODO: IRQ Handler for dma
}

void init_DMA(void){
	//configure the dma
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; //source
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&DMA_BUFFER; //6 bytes global buffer
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->TDR);
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

}



//-----------------------------------------AUX FUNCTIONS -----------------------------------------------
void calibrate(void){
	//set the freq slow in one direction till the button interrupt
	while(!isCalibrated){

	}
}

void update_motor_control(unsigned int steps){
/*change freq and direction according to current position away from target_angle
 * will try and make current_steps==target_steps
 * target_steps is updated in the uart reading interrupt
 */

	//set frequency
	//note: freq = 48000000/psc/arr psc=999+1
	unsigned int error = abs(target_steps-steps);
	unsigned int arr;
	if(error<1){
		arr = 9599; //50 Hz
	}
	else{
		if(error>134){ //20 degrees: fastest
			arr = 47;//47;//479; //100 hz	//@TODO: tune eqn
		}
		else{
			arr = (int)(8000/error) - 1; //@TODO: tune eqn
		}
	}
	TIM14->ARR = arr; //set value to count up to

	//set direction @TODO: direction is opposite
	if(steps<target_steps){
		dir = 0;
		//GPIOB->ODR |= GPIO_ODR_1;
		GPIOB->ODR &= ~GPIO_ODR_1;
	}
	else{
		dir = 1;
		GPIOB->ODR |= GPIO_ODR_1;
		//GPIOB->ODR &= ~GPIO_ODR_1;
	}

}

void update_target_steps(uint16_t x_center){
	int dead_center = width/2;
	uint16_t x_error = abs(x_center-dead_center);
	uint16_t target;
	unsigned int steps = current_steps;

	if(x_error<25){ //small error
		if(x_center>dead_center){
			target = steps + 2*x_error; //@TODO: tune eqn
		}
		else{
			target = steps - 2*x_error; //@TODO: tune eqn
		}
	}
	else{ //large error
		if(x_center>dead_center){
			target = steps + 3*x_error; //@TODO: tune eqn
		}
		else{
			target = steps - 3*x_error; //@TODO: tune eqn
		}
	}

	//check target is within bounds
	if(target<1200 && target>0){
		target_steps = target;
	}//else invalid target
	else{
		target = 0;
	}

}


//----------------------------------------INTERRUPT HANDLERS--------------------------------------
void TIM14_IRQHandler(void){

	status = !status;

	if(status){
		if(dir){
			current_steps--;
		}
		else{
			current_steps++;
		}
		GPIOB->ODR |= GPIO_ODR_0; //set PB0 high
	}
	else{
		GPIOB->ODR &= ~GPIO_ODR_0; //set PB0 low

	}
	//clear the interrupt flag
	TIM14->SR &= ~TIM_SR_UIF;

}

void TIM15_IRQHandler(){ //@TODO
/* data needs to be sent on this interrupt
 * angle(current_steps) as 2 bytes
 * timestamp(microseconds) as 4 bytes (should give about 70 mins run time before overflow)
 * we must create data packet of 6 bytes to be sent via uart to jetson using dma
 **/


}

void TIM16_IRQHandler(){
	microseconds++;
}

void EXTI0_1_IRQHandler(void){
	if(!isCalibrated){
		isCalibrated = 1;
		current_steps = 0;
		//also set start time now
	}

	EXTI->PR |= EXTI_PR_PR0; //clear the interrupt
}

void USART1_IRQHandler(){
	if(USART_GetITStatus(USART1, USART_IT_RXNE)){//receive not empty
		//x_center = USART_ReceiveData(USART1);
		INPUT_BUFFER[input_buffer_index] = USART_ReceiveData(USART1);
		input_buffer_index++;
		if(input_buffer_index>=INPUT_BUFFER_MAX_LENGTH){
			input_buffer_index = 0;
			union{
				uint8_t b[2];
				unsigned int x;
			} bt;

			bt.b[1] = INPUT_BUFFER[0];
			bt.b[0] = INPUT_BUFFER[1];

			x_center = bt.x; //@TODO: remove x-center as global
			//int xc = bt.x;

			//now update target current_steps calculated from xcenter
			//update_target_steps(xc);
		}

	}

}

// ---------------------------END-------------------------------------------------
